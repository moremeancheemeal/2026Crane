# ============================================================
#  lower_main.py — 下位机 MaixCAM
#  功能：舵机旋转5个位置 → 每个位置拍30帧OCR识别 → TCP连接上位机
#  【修改版】连接后扫描一遍，之后只在上位机发START时才再扫
# ============================================================

# ======================== 导入模块 ========================
import socket
import threading
from maix import camera, display, image, nn, pwm, pinmap, sys, err, app
from maix import time as maix_time

# ======================== 网络配置 ========================
UPPER_IP   = "192.168.1.100"
UPPER_PORT = 9090
RECON_SEC  = 1

# ======================== 扫描参数 ========================
N_STEPS         = 5      # 一共扫描几个位置
SETTLE_MS       = 500    # 舵机到位后等待时间(毫秒)，等画面稳定
FRAMES_PER_POS  = 30     # 每个位置拍多少帧再OCR，越多越准但越慢

# ======================== 舵机PWM参数 ========================
DUTY_MIN   = 2.5
DUTY_MAX   = 12.5
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180

# ======================== 5个位置的舵机角度(度) ========================
SERVO_DEG = [
    0,    # 位置1 舵机角度(度) ← 根据实际调试修改
    45,   # 位置2 舵机角度(度) ← 根据实际调试修改
    90,   # 位置3 舵机角度(度) ← 根据实际调试修改
    135,  # 位置4 舵机角度(度) ← 根据实际调试修改
    180,  # 位置5 舵机角度(度) ← 根据实际调试修改
]

# ======================== 调试模式开关 ========================
# -1  = 正常模式
# 0~4 = 只停在指定位置，方便调试
DEBUG_STEP = -1


# ======================== 舵机初始化 ========================

def deg_to_duty(deg):
    deg = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, deg))
    pct = (deg - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) * 100
    return (DUTY_MAX - DUTY_MIN) * pct / 100.0 + DUTY_MIN

dev = sys.device_id()
pn  = "A31" if dev == "maixcam2" else "A19"
pid = 7
err.check_raise(pinmap.set_pin_function(pn, f"PWM{pid}"), "pinmap failed")
servo = pwm.PWM(pid, freq=50, duty=deg_to_duty(0), enable=True)


# ======================== OCR模型 & 摄像头初始化 ========================
ocr  = nn.PP_OCR("/root/models/pp_ocr.mud")
cam  = camera.Camera(ocr.input_width(), ocr.input_height(), ocr.input_format())
disp = display.Display()
image.load_font("ppocr", "/maixapp/share/font/ppocr_keys_v1.ttf", size=20)
image.set_default_font("ppocr")


# ======================== 全局状态管理 ========================
class S:
    def __init__(self):
        self._l        = threading.Lock()
        self.running   = True
        self.trigger   = False
        self.busy      = False
        self.connected = False
        self.res       = [None] * N_STEPS

    def begin(self):
        with self._l:
            self.res  = [None] * N_STEPS
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


# ======================== OCR识别 ========================
def read_num(img):
    try:
        for obj in ocr.detect(img):
            for ch in obj.char_str().strip():
                if ch in "12345":
                    return int(ch)
    except:
        pass
    return None


# ======================== 多帧投票取最终结果 ========================
def vote_result(results):
    """
    从多帧识别结果中投票选出最可靠的那个数字

    参数：
        results : list — 每帧的识别结果，如 [1, 1, None, 1, 3, 1, None, ...]
    返回：
        int  — 投票选出的最终数字
        None — 全部帧都没识别到

    逻辑：
        1. 统计每个数字出现的次数（忽略None）
        2. 出现次数最多的那个就是最终结果
        3. 如果全部都是None，返回None
    """
    # 只看有效的识别结果
    valid = [v for v in results if v is not None]
    if not valid:
        return None

    # 统计每个数字出现次数
    count = {}
    for v in valid:
        count[v] = count.get(v, 0) + 1

    # 返回出现次数最多的数字
    return max(count, key=count.get)


# ======================== 扫描 ========================
def scan():
    """
    执行一轮完整扫描：舵机依次转到5个位置，每个位置拍FRAMES_PER_POS帧并OCR

    时间估算（默认参数）：
        每个位置 = 500ms稳定 + 30帧 × 每帧约33ms ≈ 1.5秒
        5个位置 ≈ 7~8秒
    """
    st.begin()
    print("\n  === 扫描 ===")

    # ---- 调试模式：只停在某一个位置，拍30帧 ----
    if 0 <= DEBUG_STEP < N_STEPS:
        i = DEBUG_STEP
        deg = SERVO_DEG[i]

        # 舵机转到目标角度
        servo.duty(deg_to_duty(deg))
        # 等舵机到位 + 画面稳定
        maix_time.sleep_ms(SETTLE_MS)

        # 拍FRAMES_PER_POS帧，每帧都做OCR
        frames_result = []
        for f in range(FRAMES_PER_POS):
            img = cam.read()
            v = read_num(img)
            frames_result.append(v)
            # 每10帧打印一次进度
            if (f + 1) % 10 == 0:
                print(f"  [DEBUG] pos{i+1} 第{f+1}/{FRAMES_PER_POS}帧")

        # 投票选出最终结果
        final = vote_result(frames_result)

        # 统计信息
        valid_count = sum(1 for v in frames_result if v is not None)
        st.put(i, final)

        # 屏幕显示
        t = str(final) if final else "X"
        c = image.COLOR_GREEN if final else image.COLOR_RED
        img.draw_string(10, 10, f"[DEBUG] pos{i+1} {deg}deg {valid_count}/{FRAMES_PER_POS} val={t}", c)
        disp.show(img)

        print(f"  [DEBUG] 位置{i+1} 角度={deg}° 有效帧={valid_count}/{FRAMES_PER_POS} 结果={t}")
        st.end()
        return

    # ---- 正常模式：5个位置依次扫描 ----
    for i in range(N_STEPS):
        if not st.running:
            break

        deg = SERVO_DEG[i]

        # 舵机转到目标角度
        servo.duty(deg_to_duty(deg))
        # 等舵机到位 + 画面稳定
        maix_time.sleep_ms(SETTLE_MS)

        # 拍FRAMES_PER_POS帧，每帧都做OCR
        frames_result = []
        for f in range(FRAMES_PER_POS):
            img = cam.read()
            v = read_num(img)
            frames_result.append(v)

        # 投票选出最终结果
        final = vote_result(frames_result)

        # 统计有效帧数
        valid_count = sum(1 for v in frames_result if v is not None)
        st.put(i, final)

        # 屏幕显示
        t = str(final) if final else "X"
        c = image.COLOR_GREEN if final else image.COLOR_RED
        img.draw_string(10, 10, f"{i+1}/5 {deg}deg {valid_count}/{FRAMES_PER_POS} val={t}", c)
        disp.show(img)

        print(f"  [{i+1}/5] 角度={deg}° 有效帧={valid_count}/{FRAMES_PER_POS} 结果={t}")

    st.end()
    print(f"  完成: [{st.to_str()}]\n")


# ======================== 主循环 ========================
threading.Thread(target=cli.loop, daemon=True).start()
print("[SYS] 下位机就绪，等待上位机连接...")

# ---- 第一步：死等TCP连接 ----
while st.running and not app.need_exit():
    if st.connected:
        break
    maix_time.sleep_ms(100)

# ---- 第二步：连上后自动扫描一遍 ----
if st.running and not app.need_exit():
    print("[SYS] TCP已连接，自动扫描第一遍...")
    scan()

# ---- 第三步：待命，只等上位机发START才再扫 ----
print("[SYS] 扫描完成，等待上位机指令...")
while st.running and not app.need_exit():
    while st.running and not app.need_exit():
        if st.trigger:
            st.trigger = False
            break
        maix_time.sleep_ms(100)

    if st.running and not app.need_exit():
        print("[SYS] 收到START，开始扫描...")
        scan()
        print("[SYS] 扫描完成，继续等待指令...")


# ======================== 程序退出清理 ========================
servo.duty(deg_to_duty(0))
servo.enable(False)
cli.close()
print("[SYS] 已退出")
