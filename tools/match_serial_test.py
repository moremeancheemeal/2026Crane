#!/usr/bin/env python3
"""COM18取放流程串口助手。

输入三个MATCH码后，自动执行：
start -> 等待s -> SDONE -> 等待s -> SDONE -> 等待s -> SDONE -> MATCH
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ModuleNotFoundError:
    print("当前Python没有安装pyserial，无法访问COM18。")
    print(f"当前Python：{sys.executable}")
    print("请双击同目录下的“运行MATCH串口自动测试.bat”，它会自动选择正确的Python。")
    print(f'如需给当前Python安装依赖，请执行："{sys.executable}" -m pip install pyserial')
    raise SystemExit(3)


PORT = "COM18"
BAUD_RATE = 115200
WAIT_FOR_SAMPLE_SECONDS = 12.0
MATCH_AFTER_SDONE_DELAY_SECONDS = 0.1
SAMPLE_POSITIONS = (2, 3, 1)


def configure_console() -> None:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


def parse_match(text: str) -> tuple[int, int, int]:
    value = text.strip()
    if re.fullmatch(r"[1-5]{3}", value):
        numbers = tuple(int(char) for char in value)
    elif re.fullmatch(r"[1-5](?:[\s,，-]+[1-5]){2}", value):
        numbers = tuple(int(part) for part in re.split(r"[\s,，-]+", value))
    else:
        raise ValueError("请输入三个1～5的数字，例如：2,4,1 或 241")

    if len(set(numbers)) != 3:
        raise ValueError("三个MATCH数字必须互不相同")
    return numbers  # type: ignore[return-value]


def prompt_match() -> tuple[int, int, int]:
    print("MATCH字段顺序：a=采样位2，b=采样位3，c=采样位1")
    print("输入后脚本会立即发送 start，请确认机构处于可启动状态。")
    while True:
        try:
            return parse_match(input("请输入MATCH数字序列："))
        except ValueError as exc:
            print(f"输入错误：{exc}")


class SessionLog:
    def __init__(self, match: tuple[int, int, int]) -> None:
        log_dir = Path(__file__).resolve().parent / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"match_{''.join(map(str, match))}_{stamp}.log"
        self.file = self.path.open("w", encoding="utf-8", newline="\n")

    def write(self, direction: str, message: str) -> None:
        line = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] {direction:<4} {message}"
        print(line, flush=True)
        self.file.write(line + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def open_serial_port() -> serial.Serial:
    port = serial.Serial(
        port=None,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        write_timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    port.dtr = False
    port.rts = False
    port.port = PORT
    port.open()
    port.reset_input_buffer()
    port.reset_output_buffer()
    return port


def send_line(port: serial.Serial, logger: SessionLog, line: str) -> None:
    data = (line + "\n").encode("ascii")
    port.write(data)
    port.flush()
    logger.write("TX", repr(data.decode("ascii")))


def read_available_lines(port: serial.Serial, buffer: bytearray) -> list[str]:
    waiting = port.in_waiting
    chunk = port.read(waiting if waiting else 1)
    if chunk:
        buffer.extend(chunk)

    lines: list[str] = []
    while b"\n" in buffer:
        raw, _, rest = buffer.partition(b"\n")
        buffer[:] = rest
        lines.append(raw.rstrip(b"\r").decode("ascii", errors="replace"))
    return lines


def run_session(port: serial.Serial, match: tuple[int, int, int], logger: SessionLog) -> None:
    receive_buffer = bytearray()
    sample_count = 0
    deadline = time.monotonic() + WAIT_FOR_SAMPLE_SECONDS

    logger.write("INFO", f"串口={PORT}，波特率={BAUD_RATE}，MATCH={match[0]},{match[1]},{match[2]}")
    send_line(port, logger, "start")

    while sample_count < 3:
        for line in read_available_lines(port, receive_buffer):
            logger.write("RX", repr(line + "\n"))
            if line != "s":
                continue

            sample_count += 1
            position = SAMPLE_POSITIONS[sample_count - 1]
            logger.write("INFO", f"收到第{sample_count}次采样触发，对应取货位{position}")
            send_line(port, logger, "SDONE")
            deadline = time.monotonic() + WAIT_FOR_SAMPLE_SECONDS

            if sample_count == 3:
                time.sleep(MATCH_AFTER_SDONE_DELAY_SECONDS)
                send_line(port, logger, f"MATCH:{match[0]},{match[1]},{match[2]}")
                logger.write("INFO", "MATCH已发送；继续监视串口，按Ctrl+C退出")
                break

        if time.monotonic() >= deadline and sample_count < 3:
            expected_position = SAMPLE_POSITIONS[sample_count]
            raise TimeoutError(
                f"等待第{sample_count + 1}次s超时（预期取货位{expected_position}）。"
                "下位机可能已进入10秒视觉超时锁存，请复位后重试。"
            )

    while True:
        for line in read_available_lines(port, receive_buffer):
            logger.write("RX", repr(line + "\n"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="COM18自动取放串口测试：输入MATCH，自动收发start/s/SDONE/MATCH"
    )
    parser.add_argument(
        "match",
        nargs="?",
        help="三个互异的1～5数字，例如 2,4,1；不填写时进入交互输入",
    )
    return parser


def main() -> int:
    configure_console()
    args = build_parser().parse_args()
    try:
        match = parse_match(args.match) if args.match else prompt_match()
    except ValueError as exc:
        print(f"输入错误：{exc}")
        return 2

    logger = SessionLog(match)
    port: serial.Serial | None = None
    try:
        port = open_serial_port()
        run_session(port, match, logger)
    except KeyboardInterrupt:
        logger.write("INFO", "用户按Ctrl+C结束监视")
        return 0
    except TimeoutError as exc:
        logger.write("ERROR", str(exc))
        return 1
    except serial.SerialException as exc:
        logger.write("ERROR", f"无法使用{PORT}：{exc}。请关闭占用COM18的串口助手后重试。")
        return 1
    finally:
        if port is not None and port.is_open:
            port.close()
        logger.write("INFO", f"串口已关闭；日志：{logger.path}")
        logger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
