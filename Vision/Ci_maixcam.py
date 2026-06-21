# ============================================================
#  lower_main.py — 下位机 MaixCAM
#  舵机旋转 5 位 → OCR 识别 1~5 → TCP 客户端连接上位机
# ============================================================

import socket
import threading
from maix import camera, display, image, nn, pwm, pinmap, sys, err, app
from maix import time as maix_time

# ======================== 配置 ========================
UPPER_IP   = "192.168.1.100"   # ← 改成上位机实际IP
UPPER_PORT = 9090
RECON_SEC  = 5

N_STEPS    = 5
SETTLE_MS  = 500
DUTY_MIN   = 2.5
DUTY_MAX   = 12.5

# ======================== 舵机 ========================
def duty(pct):
    return (DUTY_MAX - DUTY_MIN) * pct / 100.0 + DUTY_MIN

dev = sys.device_id()
pn  = "A31" if dev == "maixcam2" else "A19"
pid = 7
err.check_raise(pinmap.set_pin_function(pn, f"PWM{pid}"), "pinmap failed")
servo = pwm.PWM(pid, freq=50, duty=duty(0), enable=True)

# ======================== OCR & 摄像头 ========================
ocr  = nn.PP_OCR("/root/models/pp_ocr.mud")
cam  = camera.Camera(ocr.input_width(), ocr.input_height(), ocr.input_format())
disp = display.Display()
image.load_font("ppocr", "/maixapp/share/font/ppocr_keys_v1.ttf", size=20)
image.set_default_font("ppocr")

# ======================== 状态 ========================
class S:
    def __init__(self):
        self._l      = threading.Lock()
        self.running = True
        self.trigger = False
        self.busy    = False
        self.res     = [None] * N_STEPS

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

# ======================== TCP 客户端 ========================
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

# ======================== OCR ========================
def read_num(img):
    try:
        for obj in ocr.detect(img):
            for ch in obj.char_str().strip():
                if ch in "12345":
                    return int(ch)
    except:
        pass
    return None

# ======================== 扫描 ========================
def scan():
    st.begin()
    print("\n  === 扫描 ===")
    for i in range(N_STEPS):
        if not st.running:
            break
        pct = int(i * 100 / (N_STEPS - 1))
        servo.duty(duty(pct))
        maix_time.sleep_ms(SETTLE_MS)

        img = cam.read()
        v = read_num(img)
        st.put(i, v)

        t = str(v) if v else "X"
        c = image.COLOR_GREEN if v else image.COLOR_RED
        img.draw_string(10, 10, f"{i+1}/5 = {t}", c)
        disp.show(img)
        print(f"  [{i+1}/5] {t}")

    st.end()
    print(f"  完成: [{st.to_str()}]\n")

# ======================== 主循环 ========================
threading.Thread(target=cli.loop, daemon=True).start()
print("[SYS] 下位机就绪")

while st.running and not app.need_exit():
    scan()
    for _ in range(20):
        if not st.running or app.need_exit() or st.trigger:
            break
        maix_time.sleep_ms(100)

servo.duty(duty(0))
servo.enable(False)
cli.close()
print("[SYS] 已退出")