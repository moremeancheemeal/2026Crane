# ============================================================
# Ci_maixcam.py — 下位机 MaixCAM (直接初始化版)
# 1. 上电连网后直接初始化 YOLO + Camera + 显示按钮
# 2. 按 START 即扫描，按 RESET 可重新扫描
# 3. Camera 和 YOLO 全程保持，不重复初始化
# ============================================================
import socket
import threading
import sys
from itertools import permutations
from maix import camera, display, image, nn
from maix import network, touchscreen
from maix import time as maix_time
from maix import app

# ======================== 网络配置 ========================
WIFI_SSID   = "maixcam"
WIFI_PASSWD = "12345678"
UPPER_IP    = "192.168.66.1"
UPPER_PORT  = 9090
RECON_SEC   = 1

# ======================== YOLO 模型配置 ========================
YOLO_MODEL_PATH = "/root/Crane/Crane.mud"
CONF_TH = 0.45
IOU_TH  = 0.45

# ======================== 扫描参数 ========================
FRAMES_PER_SCAN = 5
result_str = None

POS_X_RANGES = [
    (240, 350),       # P1
    (445, 520),       # P2
    (630, 700),       # P3
    (800, 880),       # P4
    (1000, 1100),     # P5
]

POS_Y_RANGES = [
    (400, 520),       # P1
    (360, 460),       # P2
    (360, 460),       # P3
    (360, 460),       # P4
    (400, 520),       # P5
]

POS_LABELS = ["P1", "P2", "P3", "P4", "P5"]
TARGET_DIGITS = {"1", "2", "3", "4", "5"}

# ======================== 全局状态管理 ========================
class State:
    def __init__(self):
        self._l = threading.Lock()
        self.running = True
        self.connected = False

st = State()

# ======================== 1. 初始化基础外设 ========================
print("=" * 50)
print("[INIT] 初始化基础外设...")
print("=" * 50)
disp = display.Display()
touch = touchscreen.TouchScreen()

DW = 640
DH = 480
BTN_W = int(DW * 0.4)
BTN_H = int(DH * 0.15)
BTN_X = (DW - BTN_W) // 2
BTN_Y = DH - BTN_H - 80

# ======================== 2. 直接初始化 YOLO + Camera ========================
print("[INIT] 加载 YOLO 模型...")
detector = nn.YOLO11(YOLO_MODEL_PATH, dual_buff=True)
print("[INIT] YOLO 模型加载完成")

print("[INIT] 打开 Camera...")
cam = camera.Camera(1280, 720)
print("[INIT] Camera 已打开")

print("[INIT] 所有硬件就绪，等待网络...")

# ======================== 3. 智能连接 WiFi ========================
print("[WIFI] 检查系统网络状态...")
wifi = network.wifi.Wifi()
maix_time.sleep_ms(2000)

ip_ok = False
_local_ip = "0.0.0.0"
for _attempt in range(10):
    try:
        _s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _s.settimeout(0.3)
        _s.connect((UPPER_IP, 80))
        _local_ip = _s.getsockname()[0]
        _s.close()
        if _local_ip and _local_ip.startswith("192.168.66."):
            print(f"[WIFI] ✅ 系统已自动连接，IP: {_local_ip}")
            ip_ok = True
            break
    except:
        pass
    maix_time.sleep_ms(300)

if not ip_ok:
    print("[WIFI] 系统未自动连接，手动发起 connect...")
    wifi.connect(WIFI_SSID, WIFI_PASSWD, timeout=5)
    for _attempt in range(15):
        try:
            _s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _s.settimeout(0.5)
            _s.connect((UPPER_IP, 80))
            _local_ip = _s.getsockname()[0]
            _s.close()
            if _local_ip and _local_ip.startswith("192.168.66."):
                print(f"[WIFI] ✅ 手动连接成功，IP: {_local_ip}")
                ip_ok = True
                break
        except:
            pass
        maix_time.sleep_ms(500)

# ======================== TCP 客户端 ========================
class TCPClient:
    def __init__(self, ip, port):
        self.ip, self.port = ip, port
        self.sock = None
        self._l = threading.Lock()
        self._ok = False

    def _connect(self):
        self._close()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2)
            s.connect((self.ip, self.port))
            s.settimeout(1)
            self.sock = s
            self._ok = True
            st.connected = True
            self.send("CONNECTED")
            print(f"[TCP] ✅ 已连接上位机 {self.ip}:{self.port}")
        except Exception as e:
            self._ok = False
            st.connected = False

    def send(self, msg):
        if not self._ok:
            return False
        try:
            with self._l:
                self.sock.sendall((msg + "\n").encode())
            return True
        except:
            self._ok = False
            st.connected = False
            return False

    def loop(self):
        buf = ""
        while st.running:
            if not self._ok:
                self._connect()
                if not self._ok:
                    maix_time.sleep_ms(RECON_SEC * 1000)
                    continue
                buf = ""
            try:
                d = self.sock.recv(4096)
                if not d:
                    self._ok = False
                    st.connected = False
                    continue
                buf += d.decode()
                while "\n" in buf:
                    ln, buf = buf.split("\n", 1)
                    ln = ln.strip()
            except socket.timeout:
                continue
            except Exception:
                self._ok = False
                st.connected = False

    def _close(self):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
            self._ok = False

cli = TCPClient(UPPER_IP, UPPER_PORT)

# ======================== YOLO 识别辅助函数 ========================
def map_to_pos(cx, cy):
    """同时检查 X 和 Y 坐标，只有都在范围内才匹配到对应位置"""
    for i, (x_min, x_max) in enumerate(POS_X_RANGES):
        y_min, y_max = POS_Y_RANGES[i]
        if x_min <= cx < x_max and y_min <= cy < y_max:
            return i
    return None

def best_assignment(score_matrix):
    best_score, best_perm = -1, None
    for perm in permutations(range(5)):
        total = sum(score_matrix[i][perm[i]] for i in range(5))
        if total > best_score:
            best_score = total
            best_perm = perm
    return [p + 1 for p in best_perm] if best_perm else [None] * 5

# ======================== 核心扫描逻辑 ========================
def do_scan_and_send():
    global result_str
    print("\n  === 🚀 YOLO11 扫描开始 ===")

    total_score = [[0.0] * 5 for _ in range(5)]

    for f in range(FRAMES_PER_SCAN):
        try:
            img = cam.read()
            objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)
        except Exception as e:
            print(f"  [WARN] 推理异常: {e}")
            continue

        for obj in objs:
            label = detector.labels[obj.class_id]
            cx = obj.x + obj.w // 2
            cy = obj.y + obj.h // 2
            pos = map_to_pos(cx, cy)           # ← 同时检查 X 和 Y
            pos_name = POS_LABELS[pos] if pos is not None else "?"
            color = image.COLOR_GREEN if pos is not None else image.COLOR_RED

            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color, thickness=3)
            img.draw_string(obj.x, obj.y - 28,
                            f'{label} {obj.score:.2f} {pos_name}',
                            color=color, scale=2.5)

            if label in TARGET_DIGITS:
                digit = int(label)
                if pos is not None and obj.score > total_score[pos][digit - 1]:
                    total_score[pos][digit - 1] = obj.score

        img.draw_string(50, 50, f"Scanning {f + 1}/{FRAMES_PER_SCAN}...",
                        image.COLOR_YELLOW, 4)
        disp.show(img)

    result = best_assignment(total_score)
    result_str = ",".join(str(v) if v is not None else "X" for v in result)
    print(f"  🏆 最终结果: [{result_str}]")

    cli.send(f"RESULTS:{result_str}")
    cli.send("START")
    print(f"  === 扫描结束，已通知上位机 ===\n")

# ======================== 4. 启动 TCP + 主循环 ========================
threading.Thread(target=cli.loop, daemon=True).start()
print("[SYS] 下位机网络线程已启动，进入主循环...")

# 全局持久引用，防止 GC 回收
display_img = None

# 状态机: "WAIT" -> "SCAN" -> "DONE"
state = "WAIT"
prev_pressed = False
prev_state = None
prev_connected = None
need_redraw = True

while not app.need_exit():
    try:
        tx, ty, pressed = touch.read()
    except:
        tx, ty, pressed = 0, 0, False

    pressed_edge = pressed and not prev_pressed
    prev_pressed = pressed

    if st.connected != prev_connected:
        prev_connected = st.connected
        need_redraw = True

    if state != prev_state:
        prev_state = state
        need_redraw = True

    # ================== 状态机处理 ==================
    if state == "WAIT":
        if need_redraw:
            display_img = image.Image(DW, DH)
            display_img.draw_rect(0, 0, DW, DH,
                                  color=image.COLOR_BLACK, thickness=-1)

            if st.connected:
                display_img.draw_rect(BTN_X, BTN_Y, BTN_W, BTN_H,
                                      image.COLOR_GREEN, 5)
                display_img.draw_string(BTN_X + (BTN_W - 250) // 2,
                                        BTN_Y + 20, "START",
                                        image.COLOR_GREEN, scale=5)
                display_img.draw_string(50, 50, "Connected. Press START.",
                                        image.COLOR_GREEN, scale=3)
            else:
                display_img.draw_rect(BTN_X, BTN_Y, BTN_W, BTN_H,
                                      image.COLOR_GRAY, 3)
                display_img.draw_string(BTN_X + (BTN_W - 450) // 2,
                                        BTN_Y + 20, "WAITING...",
                                        image.COLOR_GRAY, scale=5)
                display_img.draw_string(50, 50, "Connecting Network...",
                                        image.COLOR_YELLOW, scale=3)

            disp.show(display_img)
            need_redraw = False

        if pressed_edge and st.connected:
            if BTN_X <= tx <= BTN_X + BTN_W and BTN_Y <= ty <= BTN_Y + BTN_H:
                print("[UI] 按下 START 按钮")
                state = "SCAN"

    elif state == "SCAN":
        do_scan_and_send()
        state = "DONE"

    elif state == "DONE":
        if need_redraw:
            display_img = image.Image(DW, DH)
            display_img.draw_rect(0, 0, DW, DH,
                                  color=image.COLOR_BLACK, thickness=-1)

            display_img.draw_string(10, 250, result_str,
                                    image.COLOR_GREEN, scale=6, thickness=2)

            disp.show(display_img)
            need_redraw = False

# 退出时释放资源
try:
    cam.close()
except:
    pass
cli._close()
print("[SYS] 程序已安全退出")
