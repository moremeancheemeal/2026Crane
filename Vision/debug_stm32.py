# ============================================================
#  debug_stm32.py — 上位机 + STM32 联调脚本
#  用途：不需要下位机MaixCAM，使用虚拟OCR结果，只和STM32联调
#
#  流程：
#    1. 点击START → 串口发"start"给STM32
#    2. 等待STM32发"s" → 30帧自学习分类 → 发"SDONE"
#    3. 重复3次
#    4. 使用虚拟OCR结果匹配 → 发"MATCH:..."
#
#  与 main_maixcam.py 的区别：
#    - 不需要WiFi AP和下位机
#    - 不需要TCP通信
#    - 匹配使用虚拟OCR结果
#    - 屏幕显示 [DEBUG] 标识
# ============================================================

import threading
from maix import camera, display, image, nn, sys, uart, app, touchscreen

# ======================== 配置 ========================
UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200

CLASSIFIER_PATH = "/root/my_classes.bin"
MODEL_PATH      = "/root/models/mobilenet_v2_no_top.mud"

IDX_TO_NAME = {0: "yellow", 1: "green", 2: "white"}
REQUIRED    = 3

FRAMES_PER_CLASSIFY = 30
N_STEPS             = 5

# 虚拟OCR结果（模拟下位机返回）
DEBUG_OCR_RESULT = [3, 1, 4, 2, 5]

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
uart_write("READY")
print("[SYS] 调试模式就绪（不需要下位机）")
print(f"[SYS] 虚拟OCR结果: {DEBUG_OCR_RESULT}")

step = STEP_IDLE
cls_list = []
cls_count = 0
match_result = None
status_msg = ""
prev_pressed = False

# ======================== 主循环 ========================
while not app.need_exit():

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
        img.draw_string(10, 10, "[DEBUG MODE]", image.COLOR_RED)
        img.draw_string(10, 30, f"OCR: {DEBUG_OCR_RESULT}", image.COLOR_GRAY)
        draw_btn(img, "START", image.COLOR_GRAY)

        if pressed_edge and hit_btn(tx, ty):
            uart_write("start")
            cls_list = []
            cls_count = 0
            match_result = None
            status_msg = ""
            step = STEP_WAIT_S
            print("[UI] START, waiting for 's'")

    # ==================== STEP_WAIT_S ====================
    elif step == STEP_WAIT_S:
        img.draw_string(10, 10, f"Item {cls_count+1}/{REQUIRED}", image.COLOR_GRAY)
        img.draw_string(10, 28, "Waiting for 's' from STM32...", image.COLOR_YELLOW)

        for idx, c in enumerate(cls_list):
            name = IDX_TO_NAME.get(c - 1, "?")
            img.draw_string(10, 50 + idx * 18, f"#{idx+1}: {name}", image.COLOR_GREEN)

        if uart_cmd and uart_cmd.strip().lower() == "s":
            print(f"[UART] 收到 's', 分类第{cls_count+1}次")
            step = STEP_CLASSIFY

    # ==================== STEP_CLASSIFY ====================
    elif step == STEP_CLASSIFY:
        img.draw_string(10, 10, f"Classifying {cls_count+1}/{REQUIRED}...", image.COLOR_YELLOW)
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
        else:
            status_msg = "Classify failed!"
            step = STEP_WAIT_S

    # ==================== STEP_MATCHING ====================
    elif step == STEP_MATCHING:
        ocr_result = DEBUG_OCR_RESULT
        print(f"[调试] 使用虚拟OCR结果: {ocr_result}")

        if len(ocr_result) == N_STEPS:
            m = match_fn(cls_list, ocr_result)
            cls_s = ",".join(str(c) for c in cls_list)
            ocr_s = ",".join(str(v) for v in ocr_result)
            mat_s = ",".join(str(v) for v in m)

            match_result = {"cls": cls_s, "ocr": ocr_s, "match": mat_s}
            status_msg = f"MATCH:{mat_s}"
            uart_write(f"MATCH:{mat_s}")

            print(f"\n{'='*48}")
            print(f"  CLS:   [{cls_s}]")
            print(f"  OCR:   [{ocr_s}]")
            print(f"  MATCH: [{mat_s}]")
            print(f"{'='*48}\n")
        else:
            match_result = None
            status_msg = "OCR result error"
            uart_write("ERR:NO_OCR")

        step = STEP_RESULT

    # ==================== STEP_RESULT ====================
    elif step == STEP_RESULT:
        img.draw_string(10, 10, "[DEBUG MODE]", image.COLOR_RED)

        if match_result:
            img.draw_string(10, 30, f"CLS: [{match_result['cls']}]", image.COLOR_GREEN)
            img.draw_string(10, 50, f"OCR: [{match_result['ocr']}]", image.COLOR_GREEN)
            img.draw_string(10, 70, f"MATCH: [{match_result['match']}]", image.COLOR_GRAY)
        else:
            img.draw_string(10, 30, "Match failed!", image.COLOR_RED)
            img.draw_string(10, 50, status_msg, image.COLOR_YELLOW)

        draw_btn(img, "RESET", image.COLOR_WHITE)

        if pressed_edge and hit_btn(tx, ty):
            step = STEP_IDLE
            cls_list = []
            cls_count = 0
            match_result = None
            status_msg = ""
            print("[UI] Reset")

    # ==================== UART 备用命令 ====================
    if uart_cmd:
        cmd = uart_cmd.upper().strip()
        if cmd == "RESET":
            step = STEP_IDLE
            cls_list = []
            cls_count = 0
            match_result = None
            uart_write("RESET_OK")
        elif cmd == "STATUS":
            uart_write(f"CLS={cls_count} STEP={step}")
        elif cmd == "PING":
            uart_write("PONG")

    disp.show(img)

# ======================== 退出 ========================
if uart_ok:
    ser.close()
print("[SYS] 已退出")
