# ============================================================
#  main_maixcam.py — 主上位机 MaixCAM
#  WiFi AP热点 + 触摸屏按钮 + 自学习分类 + TCP服务端 + UART通信
#
#  流程:
#    1. 上电 → 启动WiFi AP → 初始化 → TCP监听
#    2. 下位机连上WiFi + TCP → 屏幕显示"START"按钮
#    3. 点击"START" → TCP发START给下位机 + 串口发"start"给STM32
#    4. 等待STM32发送"s" → 每次收到s进行30帧分类 → 3次后请求下位机结果
#    5. 下位机返回OCR结果 → 匹配 → 发送结果给STM32 → 结束
# ============================================================

import socket
import threading
from maix import camera, display, image, nn, sys, pinmap, uart, err, app
from maix import touchscreen
from maix import network
from maix import time as maix_time

# ======================== 配置 ========================
WIFI_SSID    = "maixcam"
WIFI_PASSWD  = "12345678"
WIFI_CHANNEL = 6

TCP_IP   = "0.0.0.0"
TCP_PORT = 9090
AP_IP    = "192.168.66.1"

UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200

CLASSIFIER_PATH = "/root/my_classes.bin"
MODEL_PATH      = "/root/models/mobilenet_v2_no_top.mud"

IDX_TO_NAME = {0: "yellow", 1: "green", 2: "white"}
REQUIRED    = 3

FRAMES_PER_CLASSIFY = 30
N_STEPS             = 5

# ======================== 初始化 WiFi AP ========================
print("[WIFI] 启动 AP 模式...")
wifi = network.wifi.Wifi()
wifi.start_ap(WIFI_SSID, WIFI_PASSWD, channel=WIFI_CHANNEL)
print(f"[WIFI] AP 启动成功, SSID: {WIFI_SSID}, AP IP: {AP_IP}")

# ======================== 初始化 UART ========================
try:
    ser = uart.UART(UART_PORT, UART_BAUD)
    uart_ok = True
    print(f"[UART] {UART_PORT} @ {UART_BAUD}")
except Exception as e:
    print(f"[UART] 失败: {e}")
    uart_ok = False

_uart_buf = ""
_uart_lock = threading.Lock()
_uart_lines = []


def _uart_on_received(serial, data):
    global _uart_buf
    try:
        text = data.decode("utf-8", errors="ignore")
        with _uart_lock:
            _uart_buf += text
            while "\n" in _uart_buf:
                line, _uart_buf = _uart_buf.split("\n", 1)
                line = line.strip()
                if line:
                    _uart_lines.append(line)
    except:
        pass


if uart_ok:
    ser.set_received_callback(_uart_on_received)

# ======================== 初始化触摸屏 ========================
touch = touchscreen.TouchScreen()

# ======================== 初始化分类器 ========================
clf = nn.SelfLearnClassifier(model=MODEL_PATH, dual_buff=True)
try:
    clf.load(CLASSIFIER_PATH)
    print("[INIT] 分类器已加载")
except Exception as e:
    print(f"[INIT] 加载失败: {e}")
    print("[INIT] 请先用官方「自学习分类器」App 训练并保存")

# ======================== 初始化摄像头 & 显示 ========================
cam  = camera.Camera(clf.input_width(), clf.input_height(), clf.input_format())
disp = display.Display()

IW = clf.input_width()
IH = clf.input_height()
DW = disp.width()
DH = disp.height()

print(f"[INIT] 图像:{IW}x{IH} 屏幕:{DW}x{DH}")

try:
    image.load_font("zh", "/maixapp/share/font/ppocr_keys_v1.ttf", size=20)
    image.set_default_font("zh")
except:
    pass

# ======================== UI 工具 ========================
BTN_W = int(IW * 0.5)
BTN_H = int(IH * 0.12)
BTN_X = (IW - BTN_W) // 2
BTN_Y = IH - BTN_H - 15


def hit_btn(tx, ty):
    ix = int(tx * IW / DW)
    iy = int(ty * IH / DH)
    return BTN_X <= ix <= BTN_X + BTN_W and BTN_Y <= iy <= BTN_Y + BTN_H


def draw_btn(img, text, color=image.COLOR_WHITE):
    img.draw_rect(BTN_X, BTN_Y, BTN_W, BTN_H, color, 2)
    tw = len(text) * 12
    tx = BTN_X + (BTN_W - tw) // 2
    ty = BTN_Y + (BTN_H - 16) // 2
    img.draw_string(tx, ty, text, color)


# ======================== UART 工具 ========================
def uart_read():
    with _uart_lock:
        if _uart_lines:
            return _uart_lines.pop(0)
    return None


def uart_write(msg):
    if not uart_ok:
        return
    try:
        ser.write_str(msg + "\n")
    except:
        pass


# ======================== TCP 状态 ========================
class State:
    def __init__(self):
        self._l = threading.Lock()
        self.running = True
        self.conn = None
        self.ocr  = None
        self.evt  = threading.Event()

    def set_conn(self, c):
        with self._l:
            self.conn = c

    def get_conn(self):
        with self._l:
            return self.conn


st = State()


# ======================== TCP 通信 ========================
def tcp_send(msg):
    c = st.get_conn()
    if not c:
        return False
    try:
        c.sendall((msg + "\n").encode())
        return True
    except:
        st.set_conn(None)
        return False


def _tcp_recv(conn):
    buf = ""
    conn.settimeout(1)
    while st.running:
        try:
            d = conn.recv(4096)
            if not d:
                print("[TCP] 下位机断开")
                st.set_conn(None)
                break
            buf += d.decode()
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    _handle_tcp(line)
        except socket.timeout:
            continue
        except:
            st.set_conn(None)
            break


def _handle_tcp(msg):
    if msg == "CONNECTED":
        print("[TCP] 下位机已连接")
    elif msg.startswith("RESULTS:"):
        parts = msg[8:].split(",")
        r = []
        for p in parts:
            try:
                r.append(int(p.strip()))
            except:
                r.append(None)
        st.ocr = r
        st.evt.set()
    elif msg == "SCANNING":
        st.ocr = None
        st.evt.set()
    elif msg.startswith("ACK:"):
        print(f"[TCP] 确认: {msg}")


def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_IP, TCP_PORT))
    srv.listen(1)
    srv.settimeout(1)
    print(f"[TCP] 监听 {TCP_IP}:{TCP_PORT}")
    while st.running:
        try:
            conn, addr = srv.accept()
            print(f"[TCP] 下位机 {addr} 上线")
            st.set_conn(conn)
            _tcp_recv(conn)
        except socket.timeout:
            continue
        except:
            pass
    srv.close()


# ======================== 匹配函数 ========================
def match_fn(classes, ocr):
    res = []
    for c in classes:
        pos = 0
        for i, v in enumerate(ocr):
            if v is not None and v == c:
                pos = i + 1
                break
        res.append(pos)
    return res


# ======================== 多帧投票分类 ========================
def vote_classify(frames=30):
    results = []
    for f in range(frames):
        frame = cam.read()
        try:
            res = clf.classify(frame)
            if res:
                idx, score = res[0]
                results.append(idx + 1)
            else:
                results.append(None)
        except:
            results.append(None)
        if (f + 1) % 10 == 0:
            print(f"  [分类] 第{f+1}/{frames}帧")
            frame.draw_string(10, 10, f"Classifying {f+1}/{frames}...", image.COLOR_YELLOW)
            disp.show(frame)

    valid = [v for v in results if v is not None]
    if not valid:
        return None, 0

    count = {}
    for v in valid:
        count[v] = count.get(v, 0) + 1
    winner = max(count, key=count.get)
    confidence = count[winner] / len(valid)

    return winner, confidence


# ======================== UI 状态常量 ========================
STEP_IDLE     = 0
STEP_WAIT_S   = 1
STEP_CLASSIFY = 2
STEP_MATCHING = 3
STEP_RESULT   = 4


# ======================== 启动 ========================
threading.Thread(target=tcp_server, daemon=True).start()
uart_write("READY")
print("[SYS] 上位机就绪")
print(f"[SYS] 请将下位机连接 WiFi: {WIFI_SSID} / {WIFI_PASSWD}")

step = STEP_IDLE
cls_list = []
cls_count = 0
match_result = None
ocr_result = None
status_msg = ""
prev_pressed = False

# ======================== 主循环 ========================
while st.running and not app.need_exit():

    try:
        tx, ty, pressed = touch.read()
    except:
        tx, ty, pressed = 0, 0, False

    pressed_edge = pressed and not prev_pressed
    prev_pressed = pressed

    uart_cmd = uart_read()
    img = cam.read()

    # ==================== STEP_IDLE ====================
    if step == STEP_IDLE:
        connected = st.get_conn() is not None

        if connected:
            img.draw_string(10, 10, "Slave: OK", image.COLOR_GREEN)
            draw_btn(img, "START", image.COLOR_GRAY)

            if pressed_edge and hit_btn(tx, ty):
                if tcp_send("START"):
                    uart_write("start")
                    cls_list = []
                    cls_count = 0
                    match_result = None
                    ocr_result = None
                    status_msg = ""
                    step = STEP_WAIT_S
                    print("[UI] START sent, waiting for STM32 's'")
                else:
                    status_msg = "TCP send fail!"
        else:
            img.draw_string(10, 10, "Slave: --", image.COLOR_RED)
            img.draw_string(10, 28, "Waiting...", image.COLOR_GRAY)

        img.draw_string(10, IH - 45, f"AP:{WIFI_SSID}", image.COLOR_YELLOW)
        if status_msg:
            img.draw_string(10, IH - 65, status_msg, image.COLOR_YELLOW)

    # ==================== STEP_WAIT_S ====================
    elif step == STEP_WAIT_S:
        img.draw_string(10, 10, f"Item {cls_count+1}/{REQUIRED}", image.COLOR_GRAY)
        img.draw_string(10, 28, "Waiting for 's' from STM32...", image.COLOR_YELLOW)

        for idx, c in enumerate(cls_list):
            name = IDX_TO_NAME.get(c - 1, "?")
            img.draw_string(10, 50 + idx * 18, f"#{idx+1}: {name}", image.COLOR_GREEN)

        if uart_cmd and uart_cmd.strip().lower() == "s":
            print(f"[UART] 收到 's', 开始分类第{cls_count+1}次")
            step = STEP_CLASSIFY

    # ==================== STEP_CLASSIFY ====================
    elif step == STEP_CLASSIFY:
        img.draw_string(10, 10, f"Classifying {cls_count+1}/{REQUIRED}...", image.COLOR_YELLOW)
        img.draw_string(10, 28, "Please wait...", image.COLOR_GRAY)
        disp.show(img)

        result, confidence = vote_classify(FRAMES_PER_CLASSIFY)

        if result is not None:
            cls_list.append(result)
            cls_count += 1
            name = IDX_TO_NAME.get(result - 1, "?")
            status_msg = f"#{cls_count}: {name} ({confidence:.1%})"
            print(f"[分类] 第{cls_count}次: class{result}({name}) 置信度={confidence:.1%}")

            uart_write("SDONE")

            if cls_count >= REQUIRED:
                step = STEP_MATCHING
                print("[分类] 3次完成, 进入匹配")
            else:
                step = STEP_WAIT_S
                print(f"[分类] 继续等待STM32发送下一个's'")
        else:
            status_msg = "Classify failed!"
            print("[分类] 识别失败，重新等待's'")
            step = STEP_WAIT_S

    # ==================== STEP_MATCHING ====================
    elif step == STEP_MATCHING:
        if not st.get_conn():
            ocr_result = None
            status_msg = "Slave disconnected!"
            step = STEP_RESULT
        else:
            got = False
            for attempt in range(30):
                if not st.running:
                    break

                img2 = cam.read()
                img2.draw_string(10, 10, "Matching...", image.COLOR_YELLOW)
                img2.draw_string(10, 30, f"Query {attempt+1}/30", image.COLOR_GRAY)
                for idx, c in enumerate(cls_list):
                    name = IDX_TO_NAME.get(c - 1, "?")
                    img2.draw_string(10, 50 + idx * 18, f"#{idx+1}: {name}", image.COLOR_GREEN)
                disp.show(img2)

                st.evt.clear()
                if not tcp_send("QUERY"):
                    status_msg = "TCP fail"
                    break

                if st.evt.wait(timeout=2):
                    if st.ocr is not None:
                        ocr_result = st.ocr
                        got = True
                        print(f"[匹配] 第{attempt+1}次QUERY拿到: {ocr_result}")
                        break
                    else:
                        print(f"[匹配] 第{attempt+1}次: 扫描中...")
                else:
                    print(f"[匹配] 第{attempt+1}次: 超时重试")

            if got and ocr_result is not None:
                if len(ocr_result) != N_STEPS:
                    status_msg = f"OCR len={len(ocr_result)} != {N_STEPS}"
                    print(f"[匹配] OCR结果长度错误: {ocr_result}")
                    match_result = None
                else:
                    m = match_fn(cls_list, ocr_result)
                    cls_s = ",".join(str(c) for c in cls_list)
                    ocr_s = ",".join(str(v) if v else "X" for v in ocr_result)
                    mat_s = ",".join(str(v) for v in m)

                    match_result = {"cls": cls_s, "ocr": ocr_s, "match": mat_s}
                    status_msg = f"MATCH:{mat_s}"
                    uart_write(f"MATCH:{mat_s}")

                    print(f"\n{'='*48}")
                    print(f"  CLS:   [{cls_s}]")
                    print(f"  OCR:   [{ocr_s}]")
                    print(f"  MATCH: [{mat_s}]")
                    print(f"{'='*48}\n")

            if match_result is None:
                match_result = None
                if not status_msg:
                    status_msg = "No OCR result"
                uart_write("ERR:NO_OCR")

            step = STEP_RESULT

    # ==================== STEP_RESULT ====================
    elif step == STEP_RESULT:
        if match_result:
            img.draw_string(10, 10, f"CLS: [{match_result['cls']}]", image.COLOR_GREEN)
            img.draw_string(10, 30, f"OCR: [{match_result['ocr']}]", image.COLOR_GREEN)
            img.draw_string(10, 50, f"MATCH: [{match_result['match']}]", image.COLOR_GRAY)
        else:
            img.draw_string(10, 10, "Match failed!", image.COLOR_RED)
            img.draw_string(10, 30, status_msg, image.COLOR_YELLOW)

        draw_btn(img, "RESET", image.COLOR_WHITE)

        if pressed_edge and hit_btn(tx, ty):
            step = STEP_IDLE
            cls_list = []
            cls_count = 0
            match_result = None
            ocr_result = None
            status_msg = ""
            print("[UI] Reset")

    # ==================== UART 备用命令 ====================
    if uart_cmd:
        cmd = uart_cmd.upper().strip()
        if cmd == "QUERY":
            uart_write("SENT:QUERY" if tcp_send("QUERY") else "ERR:NO_SLAVE")
        elif cmd == "START_SLAVE":
            uart_write("SENT:START" if tcp_send("START") else "ERR:NO_SLAVE")
        elif cmd == "RESET":
            step = STEP_IDLE
            cls_list = []
            cls_count = 0
            match_result = None
            ocr_result = None
            uart_write("RESET_OK")
        elif cmd == "STATUS":
            ok = "YES" if st.get_conn() else "NO"
            uart_write(f"CLS={cls_count} SLAVE={ok} STEP={step}")
        elif cmd == "PING":
            uart_write("PONG")

    disp.show(img)

# ======================== 退出清理 ========================
if uart_ok:
    ser.close()
print("[SYS] 已退出")
