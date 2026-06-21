# ============================================================
#  upper_main.py — 主上位机 MaixCAM
#  加载官方App训练的自学习分类器 + TCP服务端 + UART0通信
#
#  映射(按官方App创建顺序):
#    class1 (idx=0) → 1 = 黄(yellow)
#    class2 (idx=1) → 2 = 绿(green)
#    class3 (idx=2) → 3 = 白(white)
# ============================================================

import socket
import threading
from maix import camera, display, image, nn, sys, pinmap, uart, err, app
from maix import time as maix_time

# ======================== 配置 ========================
UART_PORT       = "/dev/ttyS0"         # UART0, A16=TX A17=RX (系统自动配置)
UART_BAUD       = 115200
TCP_IP          = "0.0.0.0"
TCP_PORT        = 9090
CLASSIFIER_PATH = "/root/my_classes.bin"
MODEL_PATH      = "/root/models/mobilenet_v2_no_top.mud"

# 按创建顺序映射: index+1 → 含义
#   idx 0 → 1 = 黄
#   idx 1 → 2 = 绿
#   idx 2 → 3 = 白
IDX_TO_NAME = {0: "yellow(黄)", 1: "green(绿)", 2: "white(白)"}
REQUIRED    = 3   # 累计 3 次后匹配

# ======================== UART ========================
try:
    ser = uart.UART(UART_PORT, UART_BAUD)
    uart_ok = True
    print(f"[UART] {UART_PORT} @ {UART_BAUD}")
except Exception as e:
    print(f"[UART] 失败: {e}")
    uart_ok = False

# ======================== 分类器 & 摄像头 ========================
clf = nn.SelfLearnClassifier(model=MODEL_PATH, dual_buff=True)
try:
    clf.load(CLASSIFIER_PATH)
    print("[INIT] 分类器已加载")
except Exception as e:
    print(f"[INIT] 加载失败: {e}")
    print("[INIT] 请先用官方「自学习分类器」App 训练并保存")

cam  = camera.Camera(clf.input_width(), clf.input_height(), clf.input_format())
disp = display.Display()

# ======================== UART 工具 ========================
_uart_buf = ""

def uart_read():
    global _uart_buf
    if not uart_ok:
        return None
    data = ser.read()
    if data:
        _uart_buf += data.decode("utf-8", errors="ignore")
    while "\n" in _uart_buf:
        line, _uart_buf = _uart_buf.split("\n", 1)
        line = line.strip()
        if line:
            return line
    return None

def uart_write(msg):
    if not uart_ok:
        return
    try:
        ser.write((msg + "\n").encode())
    except:
        pass

# ======================== 状态 ========================
class State:
    def __init__(self):
        self._l = threading.Lock()
        self.running = True
        self.conn    = None          # 下位机连接
        self.cls_list = []           # 3次分类结果
        self.ocr      = None         # 下位机返回的OCR结果
        self.evt      = threading.Event()  # 等OCR结果
        self.match    = None

    def reset(self):
        with self._l:
            self.cls_list = []
            self.ocr = None
            self.match = None

    def add(self, c):
        with self._l:
            self.cls_list.append(c)
            return len(self.cls_list)

    def get_cls(self):
        with self._l:
            return list(self.cls_list)

    def set_conn(self, c):
        with self._l:
            self.conn = c

    def get_conn(self):
        with self._l:
            return self.conn

st = State()

# ======================== TCP ========================
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
            try: r.append(int(p.strip()))
            except: r.append(None)
        st.ocr = r
        st.evt.set()
    elif msg == "SCANNING":
        st.ocr = None

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

# ======================== 匹配 ========================
def match_fn(classes, ocr):
    """
    classes: [2, 1, 3]        — 3次分类编号
    ocr:     [3, 1, 5, 2, 4]  — 5个位置的OCR编号
    返回:    [4, 2, 1]        — 每个分类在OCR第几位找到(1-indexed, 0=未找到)
    """
    res = []
    for c in classes:
        pos = 0
        for i, v in enumerate(ocr):
            if v is not None and v == c:
                pos = i + 1
                break
        res.append(pos)
    return res

# ======================== 启动 ========================
threading.Thread(target=tcp_server, daemon=True).start()

uart_write("READY")
print("[SYS] 上位机就绪")
print(f"[SYS] 分类映射: {IDX_TO_NAME}")
print("[SYS] 等待 UART 命令: RECOGNIZE | QUERY | RESET | STATUS")

while st.running and not app.need_exit():
    cmd = uart_read()
    if not cmd:
        maix_time.sleep_ms(20)
        continue

    cmd = cmd.upper().strip()

    # ========== RECOGNIZE ==========
    if cmd == "RECOGNIZE":
        img = cam.read()
        try:
            idx, score = clf.classify(img)
            cls_num = idx + 1          # idx→num
        except Exception as e:
            uart_write(f"ERR:{e}")
            continue

        name = IDX_TO_NAME.get(idx, "?")
        cnt  = st.add(cls_num)
        uart_write(f"CLASS:{cls_num}")
        print(f"[识别] #{cnt}: {cls_num}({name}) conf={score:.3f}")

        img.draw_string(10, 10, f"#{cnt} {cls_num}:{name}", image.COLOR_GREEN)
        disp.show(img)

        # 满 3 次 → 查询下位机 → 匹配
        if cnt >= REQUIRED:
            uart_write("MATCHING...")
            print("[匹配] 向下位机查询 OCR 结果...")

            if not st.get_conn():
                uart_write("ERR:NO_SLAVE")
                st.reset()
                continue

            st.evt.clear()
            if not tcp_send("QUERY"):
                uart_write("ERR:TCP_FAIL")
                st.reset()
                continue

            if not st.evt.wait(timeout=15):
                uart_write("ERR:TIMEOUT")
                st.reset()
                continue

            if st.ocr is None:
                uart_write("ERR:NO_OCR")
                st.reset()
                continue

            cls = st.get_cls()
            ocr = st.ocr
            m   = match_fn(cls, ocr)

            # 输出
            cls_s = ",".join(str(c) for c in cls)
            ocr_s = ",".join(str(v) if v else "X" for v in ocr)
            mat_s = ",".join(str(v) for v in m)
            nam_s = ",".join(IDX_TO_NAME.get(c-1, "?") for c in cls)

            uart_write(f"MATCH:{mat_s}")

            print(f"\n{'='*48}")
            print(f"  分类:  [{cls_s}]  = {nam_s}")
            print(f"  OCR:   [{ocr_s}]")
            print(f"  匹配:  [{mat_s}]  (0=未找到)")
            print(f"{'='*48}\n")

            img = cam.read()
            img.draw_string(10, 10, f"MATCH: {mat_s}", image.COLOR_GREEN)
            disp.show(img)

            st.reset()

    # ========== QUERY ==========
    elif cmd == "QUERY":
        uart_write("SENT:QUERY" if tcp_send("QUERY") else "ERR:NO_SLAVE")

    # ========== START_SLAVE ==========
    elif cmd == "START_SLAVE":
        uart_write("SENT:START" if tcp_send("START") else "ERR:NO_SLAVE")

    # ========== RESET ==========
    elif cmd == "RESET":
        st.reset()
        uart_write("RESET_OK")

    # ========== STATUS ==========
    elif cmd == "STATUS":
        n = len(st.get_cls())
        ok = "YES" if st.get_conn() else "NO"
        uart_write(f"CLS={n} SLAVE={ok}")

    # ========== PING ==========
    elif cmd == "PING":
        uart_write("PONG")

    else:
        uart_write(f"UNKNOWN:{cmd}")

if uart_ok:
    ser.close()
print("[SYS] 已退出")