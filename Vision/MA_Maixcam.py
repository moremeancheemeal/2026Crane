# ============================================================
# main_maixcam.py — 主上位机 MaixCAM (适配下位机 RESET 重扫)
# 流程：被动接收下位机 START → 通知 STM32 → 分类 3 次 → 直接匹配
# ============================================================
import socket
import threading
from maix import camera, display, image, nn, sys, pinmap, uart, err
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
print("[WIFI] 启动 AP...")
wifi = network.wifi.Wifi()
wifi.start_ap(WIFI_SSID, WIFI_PASSWD, channel=WIFI_CHANNEL)
print(f"[WIFI] AP: {WIFI_SSID} @ {AP_IP}")

# ======================== 初始化 UART ========================
pin_function = {"A16": "UART0_TX","A17": "UART0_RX"}
for pin, func in pin_function.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Pin {pin} fail")

try:
    ser = uart.UART(UART_PORT, UART_BAUD)
    uart_ok = True
    print(f"[UART] {UART_PORT} @ {UART_BAUD} OK")
except Exception as e:
    print(f"[UART] Fail: {e}")
    uart_ok = False

_uart_buf, _uart_lock, _uart_lines = "", threading.Lock(), []
def _uart_on_received(serial, data):
    global _uart_buf
    try:
        text = data.decode("utf-8", errors="ignore")
        with _uart_lock:
            _uart_buf += text
            while "\n" in _uart_buf:
                line, _uart_buf = _uart_buf.split("\n", 1)
                if line.strip(): _uart_lines.append(line.strip())
    except: pass

if uart_ok:
    ser.set_received_callback(_uart_on_received)

# ======================== 初始化其他 ========================
clf = nn.SelfLearnClassifier(model=MODEL_PATH, dual_buff=True)
try:
    clf.load(CLASSIFIER_PATH)
    print("[INIT] Classifier loaded")
except Exception as e:
    print(f"[INIT] Load fail: {e}")

cam = camera.Camera(clf.input_width(), clf.input_height(), clf.input_format())
disp = display.Display()
IW, IH = clf.input_width(), clf.input_height()
DW, DH = disp.width(), disp.height()
print(f"[INIT] Cam:{IW}x{IH} Disp:{DW}x{DH}")

try:
    image.load_font("zh", "/maixapp/share/font/ppocr_keys_v1.ttf", size=20)
    image.set_default_font("zh")
except: pass

# ======================== UART 工具 ========================
def uart_read():
    with _uart_lock:
        return _uart_lines.pop(0) if _uart_lines else None
def uart_write(msg):
    if uart_ok:
        try: ser.write_str(msg + "\n")
        except: pass

# ======================== TCP 状态 ========================
class State:
    def __init__(self):
        self._l = threading.Lock()
        self.running = True
        self.conn = None
        self.ocr = None
        self.evt = threading.Event()
        self.start_trigger = False
    def set_conn(self, c):
        with self._l: self.conn = c
    def get_conn(self):
        with self._l: return self.conn
st = State()

# ======================== TCP 通信 ========================
def tcp_send(msg):
    c = st.get_conn()
    if not c: return False
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
                print("[TCP] Disconnected")
                st.set_conn(None)
                break
            buf += d.decode()
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if line.strip(): _handle_tcp(line.strip())
        except socket.timeout: continue
        except: st.set_conn(None); break

def _handle_tcp(msg):
    if msg == "CONNECTED":
        print("[TCP] Slave connected")
    elif msg.startswith("RESULTS:"):
        parts = msg[8:].split(",")
        r = []
        for p in parts:
            try: r.append(int(p.strip()))
            except: r.append(None)
        st.ocr = r
        print(f"[TCP] Got results: {st.ocr}")
    elif msg == "START":
        print("[TCP] Got START from slave, trigger")
        with st._l:
            st.start_trigger = True
        st.evt.set()
    elif msg == "SCANNING":
        st.ocr = None
        st.evt.set()
    elif msg.startswith("ACK:"):
        print(f"[TCP] ACK: {msg}")

def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_IP, TCP_PORT))
    srv.listen(1)
    srv.settimeout(1)
    print(f"[TCP] Listen {TCP_IP}:{TCP_PORT}")
    while st.running:
        try:
            conn, addr = srv.accept()
            print(f"[TCP] Slave {addr} online")
            st.set_conn(conn)
            _tcp_recv(conn)
        except socket.timeout: continue
        except: pass
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

# ======================== 分类函数 ========================
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
            print(f"  [Classify] {f+1}/{frames}")
            frame.draw_string(10, 10, f"{f+1}/{frames}...", image.COLOR_YELLOW)
            disp.show(frame)
    valid = [v for v in results if v is not None]
    if not valid: return None, 0
    count = {}
    for v in valid: count[v] = count.get(v, 0) + 1
    winner = max(count, key=count.get)
    return winner, count[winner] / len(valid)

# ======================== 状态常量 ========================
STEP_IDLE, STEP_WAIT_S, STEP_CLASSIFY, STEP_MATCHING, STEP_RESULT = 0, 1, 2, 3, 4

# ======================== 启动 ========================
threading.Thread(target=tcp_server, daemon=True).start()
uart_write("READY")
print("[SYS] Master ready")

step = STEP_IDLE
cls_list, cls_count, match_result, ocr_result, status_msg = [], 0, None, None, ""

# ======================== 主循环 ========================
while st.running:
    uart_cmd = uart_read()
    img = cam.read()

    # === STEP_IDLE ===
    if step == STEP_IDLE:
        connected = st.get_conn() is not None
        if connected:
            img.draw_string(10, 10, "Slave: OK", image.COLOR_GREEN)
            img.draw_string(10, 50, "Waiting for Slave START...", image.COLOR_YELLOW)
            with st._l:
                if st.start_trigger:
                    st.start_trigger = False
                    cls_list, cls_count = [], 0
                    match_result, ocr_result, status_msg = None, None, ""
                    uart_write("start")
                    step = STEP_WAIT_S
                    print("[UI] Triggered by slave, notify STM32")
        else:
            img.draw_string(10, 10, "Slave: --", image.COLOR_RED)
            img.draw_string(10, 28, "Waiting...", image.COLOR_GRAY)
        img.draw_string(10, IH - 45, f"AP:{WIFI_SSID}", image.COLOR_YELLOW)
        if status_msg:
            img.draw_string(10, IH - 65, status_msg, image.COLOR_YELLOW)

    # === STEP_WAIT_S ===
    elif step == STEP_WAIT_S:
        img.draw_string(10, 10, f"Item {cls_count+1}/{REQUIRED}", image.COLOR_GRAY)
        img.draw_string(10, 28, "Waiting 's' from STM32...", image.COLOR_YELLOW)
        for idx, c in enumerate(cls_list):
            name = IDX_TO_NAME.get(c - 1, "?")
            img.draw_string(10, 50 + idx * 18, f"#{idx+1}: {name}", image.COLOR_GREEN)
        if uart_cmd and uart_cmd.strip().lower() == "s":
            print(f"[UART] Got 's', classify #{cls_count+1}")
            step = STEP_CLASSIFY

    # === STEP_CLASSIFY ===
    elif step == STEP_CLASSIFY:
        img.draw_string(10, 10, f"Classifying {cls_count+1}/{REQUIRED}...", image.COLOR_YELLOW)
        disp.show(img)
        result, confidence = vote_classify(FRAMES_PER_CLASSIFY)
        if result is not None:
            cls_list.append(result)
            cls_count += 1
            name = IDX_TO_NAME.get(result - 1, "?")
            status_msg = f"#{cls_count}: {name} ({confidence:.1%})"
            print(f"[Classify] #{cls_count}: class{result}({name})")
            uart_write("SDONE")
            if cls_count >= REQUIRED:
                step = STEP_MATCHING
                print("[Classify] 3 done, matching")
            else:
                step = STEP_WAIT_S
        else:
            status_msg = "Classify failed!"
            step = STEP_WAIT_S

    # === STEP_MATCHING ===
    elif step == STEP_MATCHING:
        img.draw_string(10, 10, "Matching...", image.COLOR_YELLOW)
        disp.show(img)
        ocr_result = st.ocr
        if ocr_result is None:
            status_msg = "No OCR from slave!"
            match_result = None
        elif len(ocr_result) != N_STEPS:
            status_msg = f"OCR len={len(ocr_result)} != {N_STEPS}"
            match_result = None
        else:
            m = match_fn(cls_list, ocr_result)
            cls_s = ",".join(str(c) for c in cls_list)
            ocr_s = ",".join(str(v) if v else "X" for v in ocr_result)
            mat_s = ",".join(str(v) for v in m)
            match_result = {"cls": cls_s, "ocr": ocr_s, "match": mat_s}
            status_msg = f"MATCH:{mat_s}"
            uart_write(f"MATCH:{mat_s}")
            print(f"\n{'='*48}\n  CLS:[{cls_s}]\n  OCR:[{ocr_s}]\n  MATCH:[{mat_s}]\n{'='*48}\n")
        if match_result is None and not status_msg:
            status_msg = "Match failed"
            uart_write("ERR:NO_OCR")
        step = STEP_RESULT

    # === STEP_RESULT ===
    elif step == STEP_RESULT:
        retriggered = False
        with st._l:
            if st.start_trigger:
                st.start_trigger = False
                retriggered = True

        if retriggered:
            cls_list, cls_count = [], 0
            match_result, ocr_result, status_msg = None, None, ""
            uart_write("start")
            step = STEP_WAIT_S
            print("[UI] Slave re-scanned, new cycle")
            img.draw_string(10, 10, "New scan from slave!", image.COLOR_YELLOW)
            img.draw_string(10, 30, "Restarting...", image.COLOR_YELLOW)
        else:
            if match_result:
                img.draw_string(10, 10, f"CLS:[{match_result['cls']}]", image.COLOR_GREEN)
                img.draw_string(10, 30, f"OCR:[{match_result['ocr']}]", image.COLOR_GREEN)
                img.draw_string(10, 50, f"MATCH:[{match_result['match']}]", image.COLOR_GRAY)
            else:
                img.draw_string(10, 10, "Match failed!", image.COLOR_RED)
                img.draw_string(10, 30, status_msg, image.COLOR_YELLOW)

    # === UART 命令 ===
    if uart_cmd:
        cmd = uart_cmd.upper().strip()
        if cmd == "PING":
            uart_write("PONG")

    disp.show(img)

# === 退出 ===
if uart_ok: ser.close()
print("[SYS] Exit")
sys.exit()