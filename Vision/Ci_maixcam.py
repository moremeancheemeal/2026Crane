# ============================================================
#  Ci_maixcam.py — 下位机 MaixCAM (YOLO数字检测版)
#  连接上位机WiFi AP → 摄像头30帧YOLO检测 → 按坐标矩形区域插入 → TCP通信
#  只在收到上位机START时才扫描
#
#  识别逻辑：
#    1. 摄像头拍30帧，每帧YOLO检测数字（预期检测到4个，范围1~5）
#    2. 根据检测框中心点(X,Y)判断属于哪个位置（5个矩形区域）
#    3. 30帧投票，每个位置取出现次数最多的数字
#    4. 5个位置中没有检测到数字的那个，就是缺失的数字
#    5. 缺失数字自动填入空位
#    6. 最终结果如 [3,1,4,2,5]
# ============================================================

import socket
import threading
from maix import camera, display, image, nn, sys, app
from maix import network
from maix import time as maix_time

# ======================== 网络配置 ========================
WIFI_SSID   = "maixcam"
WIFI_PASSWD = "12345678"
UPPER_IP    = "192.168.66.1"
UPPER_PORT  = 9090
RECON_SEC   = 1

# ======================== YOLO模型配置 ========================
YOLO_MODEL_PATH = "/root/models/yolov5s.mud"  # ← 改成你的模型路径
YOLO_CONF_TH    = 0.5   # 置信度阈值
YOLO_IOU_TH     = 0.45  # NMS IoU阈值

# ======================== 扫描参数 ========================
FRAMES_PER_SCAN = 30    # 每次扫描拍多少帧

# ======================== 4个位置的坐标矩形范围 ========================
# 实际画面中只有4个数字，每个数字由一个矩形区域定位
# 格式：(x_min, x_max, y_min, y_max)
# 检测框中心点落在哪个矩形内，就属于哪个位置
# 第5个数字是推理出来的（1~5中没出现的那个），自动放到结果最后一位
# 单位是像素，请根据实际画面调试后填入
POS_RECTS = [
    # (x_min, x_max, y_min, y_max)
    (20, 80, 190, 220),     # 位置1  ← 调试后修改
    (100, 160, 190, 220),   # 位置2  ← 调试后修改
    (180, 240, 190, 220),   # 位置3  ← 调试后修改
    (260, 320, 190, 220),   # 位置4  ← 调试后修改
]


# ======================== 连接WiFi ========================
print("[WIFI] 连接上位机热点...")
wifi = network.wifi.Wifi()
wifi.disconnect()
maix_time.sleep_ms(500)
print("[WIFI] 已断开旧连接")
wifi.connect(WIFI_SSID, WIFI_PASSWD, timeout=10)
print("[WIFI] 已连接")


# ======================== YOLO模型初始化 ========================
print("[INIT] 加载YOLO模型...")
detector = nn.YOLOv5(model=YOLO_MODEL_PATH, dual_buff=True)
print(f"[INIT] YOLO模型已加载")
print(f"[INIT] 类别: {detector.labels}")

# ======================== 摄像头 & 显示初始化 ========================
cam  = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()

IW = cam.width()
IH = cam.height()

# 打印坐标范围
print(f"[INIT] 图像: {IW}x{IH}")
print(f"[INIT] 4个位置坐标矩形范围:")
for i, (x1, x2, y1, y2) in enumerate(POS_RECTS):
    print(f"  位置{i+1}: X=[{x1},{x2}) Y=[{y1},{y2})")
print(f"[INIT] 位置5: 自动推理（1~5中缺失的数字）")

# 加载字体
try:
    image.load_font("ppocr", "/maixapp/share/font/ppocr_keys_v1.ttf", size=20)
    image.set_default_font("ppocr")
except:
    pass


# ======================== 全局状态管理 ========================
class S:
    def __init__(self):
        self._l        = threading.Lock()
        self.running   = True
        self.trigger   = False
        self.busy      = False
        self.connected = False
        self.res       = [None] * 5  # 最终5个数字结果

    def begin(self):
        with self._l:
            self.res  = [None] * 5
            self.busy = True

    def end(self):
        with self._l:
            self.busy = False

    def put(self, i, v):
        with self._l:
            self.res[i] = v

    def to_str(self):
        with self._l:
            return ",".join(str(v) if v else "X" for v in self.res)

st = S()


# ======================== TCP客户端 ========================
class Cli:
    def __init__(self, ip, port):
        self.ip, self.port = ip, port
        self.sock = None
        self._l = threading.Lock()
        self._ok = False

    def _connect(self):
        self._close()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect((self.ip, self.port))
            s.settimeout(1)
            self.sock = s
            self._ok  = True
            st.connected = True
            self.send("CONNECTED")
            print(f"[TCP] 已连接 {self.ip}:{self.port}")
        except Exception as e:
            print(f"[TCP] 连接失败: {e}")
            self._ok = False

    def send(self, msg):
        if not self._ok:
            return False
        try:
            with self._l:
                self.sock.sendall((msg + "\n").encode())
            return True
        except:
            self._ok = False
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
                    if ln:
                        self._cmd(ln)
            except socket.timeout:
                continue
            except:
                self._ok = False
                st.connected = False

    def _cmd(self, c):
        print(f"[CMD] {c}")
        if c == "QUERY":
            if st.busy:
                self.send("SCANNING")
            else:
                self.send(f"RESULTS:{st.to_str()}")
        elif c == "START":
            st.trigger = True
            self.send("ACK:START")
        elif c == "PING":
            self.send("PONG")

    def _close(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None
            self._ok  = False

    def close(self):
        self._close()

cli = Cli(UPPER_IP, UPPER_PORT)


# ======================== YOLO数字检测 ========================
def detect_digits(img):
    """
    对一帧图像进行YOLO检测，返回检测到的数字列表和原始obj列表

    返回：(digits, objs)
        digits: list of (digit, x_center, y_center)  用于投票
        objs: 原始检测结果列表，用于绘制显示
    """
    digits = []
    objs = detector.detect(img, conf_th=YOLO_CONF_TH, iou_th=YOLO_IOU_TH)
    for obj in objs:
        digit = obj.class_id + 1  # class_id是0-based，转为1-based (1~5)
        x_center = obj.x + obj.w // 2  # 检测框中心X坐标
        y_center = obj.y + obj.h // 2  # 检测框中心Y坐标
        digits.append((digit, x_center, y_center))
    return digits, objs


def xy_to_position(x_center, y_center):
    """
    根据检测框中心点(X,Y)判断属于哪个位置（0~4）
    遍历5个矩形区域，找到包含该点的区域
    如果不在任何范围内，返回None（该检测结果被忽略）
    """
    for i, (x_min, x_max, y_min, y_max) in enumerate(POS_RECTS):
        if x_min <= x_center < x_max and y_min <= y_center < y_max:
            return i
    return None  # 不在任何位置范围内，忽略


def vote_and_infer(all_frames_results):
    """
    对30帧的检测结果进行投票，推理出最终5个数字

    逻辑：4个位置各检测1个数字，第5个数字是1~5中缺失的那个

    参数：
        all_frames_results: list of list，每帧的检测结果 [(digit, x, y), ...]

    返回：
        list: 5个数字的最终结果，如 [3, 1, 4, 2, 5]
               前4个是实际检测的，第5个是推理出来的
    """
    num_positions = len(POS_RECTS)  # 4个位置
    pos_votes = [{} for _ in range(num_positions)]

    for frame_results in all_frames_results:
        for digit, x_center, y_center in frame_results:
            pos = xy_to_position(x_center, y_center)
            if pos is not None and 1 <= digit <= 5:
                pos_votes[pos][digit] = pos_votes[pos].get(digit, 0) + 1

    # 每个位置取票数最高的数字
    pos_result = []
    for i in range(num_positions):
        if pos_votes[i]:
            pos_result.append(max(pos_votes[i], key=pos_votes[i].get))
        else:
            pos_result.append(None)

    # 找出缺失的数字（1~5中哪个没有出现在4个检测结果中）
    detected_digits = set(v for v in pos_result if v is not None)
    missing_digits = set(range(1, 6)) - detected_digits

    # 把缺失数字填入检测为空的位置
    if missing_digits:
        for i in range(num_positions):
            if pos_result[i] is None:
                pos_result[i] = missing_digits.pop()
                if not missing_digits:
                    break

    # 还有空位？填0标记异常
    for i in range(num_positions):
        if pos_result[i] is None:
            pos_result[i] = 0

    # ★ 第5个数字：推理出来的缺失数字（放到最后一位）
    # 再算一次，因为前面可能已经用掉了
    detected_now = set(pos_result)
    missing_now = set(range(1, 6)) - detected_now
    if missing_now:
        pos_result.append(missing_now.pop())
    else:
        # 极端情况：4个位置已经覆盖了1~5（不应该发生）
        pos_result.append(0)

    return pos_result  # 长度为5


# ======================== 扫描 ========================
def scan():
    """
    执行一轮扫描：拍30帧YOLO检测 → 投票 → 推理缺失数字

    时间估算：
        30帧 × 每帧约50ms ≈ 1.5秒
    """
    st.begin()
    print("\n  === YOLO扫描 ===")

    all_frames = []  # 存储每帧的检测结果

    for f in range(FRAMES_PER_SCAN):
        img = cam.read()
        digits, objs = detect_digits(img)
        all_frames.append(digits)

        # 在屏幕上显示检测结果（复用已有的objs，不重复detect）
        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, image.COLOR_RED, 2)
            label = f"{detector.labels[obj.class_id]}:{obj.score:.2f}"
            img.draw_string(obj.x, obj.y, label, image.COLOR_RED)

        # 绘制5个位置的矩形框
        for i, (x1, x2, y1, y2) in enumerate(POS_RECTS):
            img.draw_rect(x1, y1, x2 - x1, y2 - y1, image.COLOR_YELLOW, 1)
            img.draw_string(x1, y1 - 12, f"P{i+1}", image.COLOR_YELLOW)

        # 显示进度
        img.draw_string(10, 10, f"Scanning {f+1}/{FRAMES_PER_SCAN}...", image.COLOR_YELLOW)
        disp.show(img)

        if (f + 1) % 10 == 0:
            print(f"  [扫描] 第{f+1}/{FRAMES_PER_SCAN}帧")

    # 投票推理
    result = vote_and_infer(all_frames)

    # 存入全局状态
    for i, v in enumerate(result):
        st.put(i, v)

    # 最终结果显示
    final_img = cam.read()
    result_str = ",".join(str(v) for v in result)
    final_img.draw_string(10, 10, f"Result: [{result_str}]", image.COLOR_GREEN)

    # 显示每个位置的详细信息
    for i in range(len(POS_RECTS)):
        x1, x2, y1, y2 = POS_RECTS[i]
        final_img.draw_string(10, 40 + i * 20,
            f"Pos{i+1}: X[{x1},{x2}] Y[{y1},{y2}] = {result[i]}", image.COLOR_GRAY)
        final_img.draw_rect(x1, y1, x2 - x1, y2 - y1, image.COLOR_YELLOW, 1)
    # 第5个位置：推理出来的
    final_img.draw_string(10, 40 + len(POS_RECTS) * 20,
        f"Pos5: inferred = {result[4]}", image.COLOR_CYAN)

    disp.show(final_img)

    st.end()
    print(f"  完成: [{result_str}]\n")


# ======================== 主循环 ========================
threading.Thread(target=cli.loop, daemon=True).start()
print("[SYS] 下位机就绪，等待上位机连接...")

# ---- 第一步：死等TCP连接 ----
while st.running and not app.need_exit():
    if st.connected:
        break
    maix_time.sleep_ms(100)

# ---- 第二步：待命，只等上位机发START才扫描 ----
print("[SYS] 已就绪，等待上位机START指令...")
while st.running and not app.need_exit():
    while st.running and not app.need_exit():
        if st.trigger:
            st.trigger = False
            break
        maix_time.sleep_ms(100)

    if st.running and not app.need_exit():
        print("[SYS] 收到START，开始YOLO扫描...")
        scan()
        print("[SYS] 扫描完成，继续等待指令...")


# ======================== 程序退出清理 ========================
cli.close()
print("[SYS] 已退出")
