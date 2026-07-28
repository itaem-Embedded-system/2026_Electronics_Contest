# 01 - 应用基础与架构指南

本模块包含 MAIXcam (MaixPy v4) 应用开发的核心基础：生命周期控制、摄像头输入、屏幕显示、触摸屏交互以及 App 打包发布规范。

## 0. 验证优先级与复现入口

在当前项目里，建议所有应用验证都按这个顺序进行：

1. **先跑安全基础例程**：优先使用已验证的 7 个 `examples/basic/` 基础示例，确认 Python 运行、SSH 下发和 UART `API_VERIFY` 回传闭环正常。
2. **再做静态/导入预检**：确认目标示例所需的 `maix` 模块、模型路径、设备节点和参数签名。
3. **最后人工验证硬件能力**：相机、显示、触摸、音频、网络、GPIO、PWM、UART、SPI、I2C 等都应在用户明确确认条件后再实际运行。

说明：

- 当前已知观测闭环是 **SSH `root@10.0.0.121` + MaixCAM + UART0 `/dev/ttyS0` 115200 + PC `COM8`**。
- 很多应用示例依赖相机、显示或人工交互，**未自动执行不代表 API 不可用**。

---

## 1. 生命机制与退出控制

所有 MaixPy v4 应用必须通过 `app.need_exit()` 控制运行逻辑，以便安全响应系统信号或用户退出指令。

```python
from maix import app, time

while not app.need_exit():
    # 应用核心逻辑
    time.sleep_ms(10)
```

要在触摸或满足条件时主动退出应用：
```python
app.set_exit_flag(True)
```

### 高精度时间与延时 (`maix.time`)
```python
from maix import time

t_start = time.ticks_s()       # 系统启动至今秒数
t_ms = time.time_ms()          # 毫秒时间戳
time.sleep_ms(200)             # 非阻塞/毫秒级延时

# 差值计算与耗时统计
elapsed_ms = time.ticks_diff(time.ticks_s(), t_start) * 1000
now = time.now()               # 当前系统日期时间
print(now.strftime("%Y-%m-%d %H:%M:%S"))
```

---

## 2. 摄像头 (Camera) 与 屏幕 (Display) 基础

### 极简预览循环

```python
from maix import camera, display, app

# 初始化摄像头与显示屏
cam = camera.Camera(640, 480)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    disp.show(img)
```

### 摄像头参数常用设置
```python
# 获取传感器支持的最佳帧长宽
cam = camera.Camera(640, 480)
print(f"Cam size: {cam.width()}x{cam.height()}")

# 翻转/镜像 (如果画面方向不对)
cam.hmirror(True)
cam.vflip(True)
```

---

## 3. 触摸屏 (TouchScreen) 交互

```python
from maix import touchscreen, display, image, app

disp = display.Display()
ts = touchscreen.TouchScreen()

while not app.need_exit():
    x, y, pressed = ts.read()
    img = image.Image(disp.width(), disp.height())
    if pressed:
        img.draw_circle(x, y, 15, color=image.COLOR_RED, thickness=-1)
        img.draw_string(10, 10, f"Touch: ({x}, {y})", color=image.COLOR_WHITE)
    disp.show(img)
```

---

## 4. MAIXcam 应用打包规范 (`app.yaml`)

对于打包为可在 MAIXcam 屏幕菜单中运行的 App，目录结构与 `app.yaml` 约定如下：

### 目录结构
```text
my_app/
├── app.yaml
├── icon.png        (分辨率推荐 80x80)
└── main.py
```

### `app.yaml` 配置规范示例
```yaml
id: com.example.myapp
name: My MAIXcam App
version: 1.0.0
icon: icon.png
exec: python3 main.py
author: AuthorName
description: A short description of the application
```
