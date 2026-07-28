# 02 - 图像处理与 AI 视觉推理指南

本模块涵盖 MaixPy v4 中的经典图像算法、色块/二维码识别、神经网络 AI 模型（YOLO、Classifier、人脸识别等）推理。

---

## 1. 基础图像绘制与 OpenMV 式算法

### 色块寻找 (Find Blobs)
```python
from maix import camera, display, image, app

cam = camera.Camera(320, 240)
disp = display.Display()
# LAB 阈值: [L_min, L_max, A_min, A_max, B_min, B_max]
red_threshold = [0, 80, 30, 100, 15, 127]

while not app.need_exit():
    img = cam.read()
    blobs = img.find_blobs([red_threshold], pixels_threshold=100)
    for b in blobs:
        img.draw_rect(b.x(), b.y(), b.w(), b.h(), color=image.COLOR_RED)
    disp.show(img)
```

### 二维码识别 (Find QRCodes)
```python
from maix import camera, display, image, app

cam = camera.Camera(640, 480)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    qrcodes = img.find_qrcodes()
    for qr in qrcodes:
        img.draw_rect(qr.x(), qr.y(), qr.w(), qr.h(), color=image.COLOR_GREEN)
        img.draw_string(qr.x(), qr.y() - 15, qr.payload(), color=image.COLOR_GREEN)
    disp.show(img)
```

---

## 2. AI 目标检测 (YOLOv5 / YOLOv8 / YOLO11)

MaixPy v4 原生内置 YOLO 硬件 NPU 加速库，只需 10~15 行代码即可实现极速目标检测。

```python
from maix import camera, display, image, nn, app

# 加载 MUD 模型文件，并启用双缓冲加速
detector = nn.YOLOv5(model="/root/models/yolov5s.mud", dual_buff=True)
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
    for obj in objs:
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
        msg = f"{detector.labels[obj.class_id]}: {obj.score:.2f}"
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)
    disp.show(img)
```

*(注意: YOLOv8 / YOLO11 替换 `nn.YOLOv8` 或 `nn.YOLO11` 即可)*

---

## 3. AI 图像分类 (Classifier)

```python
from maix import camera, display, image, nn, app

classifier = nn.Classifier(model="/root/models/mobilenetv2.mud")
cam = camera.Camera(classifier.input_width(), classifier.input_height(), classifier.input_format())
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    res = classifier.classify(img)
    max_idx, max_prob = res[0]
    msg = f"{classifier.labels[max_idx]}: {max_prob:.2f}"
    img.draw_string(10, 10, msg, color=image.COLOR_GREEN, scale=1.5)
    disp.show(img)
```

---

## 4. 人脸检测与关键点 (Face Detector)

```python
from maix import camera, display, image, nn, app

face_detector = nn.FaceDetector(model="/root/models/retinaface.mud")
cam = camera.Camera(face_detector.input_width(), face_detector.input_height(), face_detector.input_format())
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    faces = face_detector.detect(img)
    for f in faces:
        img.draw_rect(f.x, f.y, f.w, f.h, color=image.COLOR_BLUE)
        for pt in f.points:
            img.draw_circle(pt[0], pt[1], 2, color=image.COLOR_GREEN, thickness=-1)
    disp.show(img)
```
