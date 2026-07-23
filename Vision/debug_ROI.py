from maix import camera, display, image, nn, app

detector = nn.YOLO11(model="/root/models/yolo11n.mud", dual_buff=True)
disp = display.Display()

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

ROI_COLORS = [
    image.COLOR_RED,
    image.COLOR_GREEN,
    image.COLOR_BLUE,
    image.COLOR_YELLOW,
    image.COLOR_BLACK,
]

while not app.need_exit():
    img = image.load("/root/test.jpg")

    # ---- YOLO 检测 ----
    objs = detector.detect(img, conf_th=0.1, iou_th=0.45)

    # ---- 画 5 个 ROI 框 ----
    for i in range(5):
        x_min, x_max = POS_X_RANGES[i]
        y_min, y_max = POS_Y_RANGES[i]
        w = x_max - x_min
        h = y_max - y_min
        color = ROI_COLORS[i]

        # 画矩形框（粗线）
        img.draw_rect(x_min, y_min, w, h, color=color, thickness=3)

        # 在框左上角标位置名
        img.draw_string(x_min + 5, y_min + 5, POS_LABELS[i],
                        color=color, scale=3)

    # ---- 画检测结果 ----
    # for obj in objs:
    #     label = detector.labels[obj.class_id]
    #     cx = obj.x + obj.w // 2
    #     cy = obj.y + obj.h // 2

    #     # 判断是否在某个 ROI 内
    #     in_roi = "?"
    #     for i in range(5):
    #         x_min, x_max = POS_X_RANGES[i]
    #         y_min, y_max = POS_Y_RANGES[i]
    #         if x_min <= cx < x_max and y_min <= cy < y_max:
    #             in_roi = POS_LABELS[i]
    #             break

    #     color = image.COLOR_GREEN if in_roi != "?" else image.COLOR_RED
    #     img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color, thickness=3)
    #     msg = f'{label} {obj.score:.2f} {in_roi}'
    #     img.draw_string(obj.x, obj.y - 28, msg, color=color, scale=2.5)

    disp.show(img)
