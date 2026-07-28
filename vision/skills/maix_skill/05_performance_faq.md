# 05 - 性能调优与验证 FAQ

本模块除了性能优化，也用于统一回答“为什么很多例程没有自动执行”“如何判断 API 是否真的有问题”这类高频问题。

## 1. 先看验证口径，不要先下不可用结论

### Q1: 为什么报告里只有 7 项“已验证”，其余 199 项都是“受限验证”？

- **原因**：当前验证策略是安全优先。只有 7 个基础例程被纳入安全白名单并完成了真实执行闭环，其余 199 个示例大多只做了静态分析、导入预检或板端预检。
- **结论**：`受限验证` 不等于 API 不可用，只表示这轮没有在自动化流程中执行其顶层副作用。
- **当前闭环**：已经确认 **SSH `root@10.0.0.121` + MaixCAM + UART0 `/dev/ttyS0` 115200 + PC `COM8`** 可观测 `API_VERIFY` 状态。

### Q2: 为什么“没跑”不能直接当成“坏了”？

- **原因**：许多例程依赖模型、网络、接线、屏幕、相机、音频设备或人工交互；自动化流程在没有明确授权时故意不运行它们。
- **正确表述**：应写成“未自动执行”“待人工验证”或“需确认资源前置条件”，而不是“API 不可用”。

### Q3: 推荐的验证优先级是什么？

- **步骤 1**：先跑 `examples/basic/` 中已验证的安全基础例程，确认 Python、SSH 下发和 UART 回传链路正常。
- **步骤 2**：再做静态/导入预检，确认模块导入、模型路径、设备节点和参数签名。
- **步骤 3**：最后在用户明确确认后执行 GPIO/PWM/SPI/UART、网络、相机、显示、音频等人工硬件验证。

## 2. 性能与帧率优化

### Q4: 如何提升 AI 推理帧率？

- **建议**：模型初始化优先使用 `dual_buff=True`。
- **作用**：让 NPU 推理与 CPU 前后处理并行，通常能显著改善吞吐。

```python
detector = nn.YOLOv5(model="/root/models/yolov5s.mud", dual_buff=True)
```

### Q5: 如何减少图像缩放带来的 CPU 开销？

- **建议**：让相机分辨率直接匹配模型输入尺寸。

```python
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
```

### Q6: 为什么循环里总建议 `sleep`？

- **原因**：纯轮询循环会让单核 CPU 长时间满载，进而影响 UI、触摸、串口或系统响应。
- **建议**：外设轮询用 `time.sleep_ms(1)`，流媒体等待可用 `time.sleep(1)`。

## 3. 多机型适配

### Q7: 为什么示例里经常先判断 `sys.device_id()`？

- **原因**：不同机型的串口号、I2C 总线号、PWM 通道和引脚映射不完全一致。
- **示例**：

```python
from maix import sys

device = sys.device_id()
if device == "maixcam2":
    uart_dev = "/dev/ttyS4"
else:
    uart_dev = "/dev/ttyS0"
print(f"Running on {device}, using UART {uart_dev}")
```

## 4. 常见 API 调用误区

### Q8: `AttributeError: module 'maix._maix.network' has no attribute 'WLAN'`

- **原因**：MaixCAM 这里不应套用旧式 `network.WLAN(...)` 写法。
- **正确方式**：按仓库示例使用 **`network.wifi.Wifi().connect(...)`**。

```python
from maix import err, network

w = network.wifi.Wifi()
e = w.connect("SSID_NAME", "PASSWORD", wait=True, timeout=60)
err.check_raise(e, "connect wifi failed")
```

### Q9: RTSP / WebRTC 为什么不能写成 `write(img)` 风格？

- **原因**：当前仓库真实示例采用的是“实例化服务端后绑定相机”的模式，而不是手工循环送帧。
- **正确方式**：
  - RTSP：**`rtsp.Rtsp(); bind_camera(cam)`**
  - WebRTC：**`webrtc.WebRTC(); bind_camera(cam)`**

### Q10: `PWM` 或 `I2C` 初始化报 `TypeError: incompatible constructor arguments`

- **原因**：调用签名与传统 MicroPython 写法不同。
- **正确方式**：

```python
p = pwm.PWM(0, freq=50, duty=7.5, enable=True)
bus = i2c.I2C(0, i2c.Mode.MASTER)
```

补充说明：

- `pwm.PWM()` 第一个参数是整数 ID，不是 `"PWM0"`。
- `i2c.I2C()` 需要显式给出模式，常用为 `i2c.Mode.MASTER`。

### Q11: UART 推荐怎么初始化？

- **正确方式**：按仓库示例使用 **`uart.UART(device, 115200)`**，其中 `device` 直接写 Linux 设备节点，如 `/dev/ttyS0`。

## 5. 图像与显示常见坑

### Q12: `AttributeError: module 'maix._maix.image' has no attribute 'COLOR_CYAN'`

- **原因**：不同固件的预定义颜色常量集合可能不同。
- **建议**：优先使用 `image.Color.from_rgb(r, g, b)`。

### Q13: `TypeError: draw_circle(): incompatible function arguments`

- **原因**：误用了 `fill=True` 之类的非 MaixPy 参数。
- **正确方式**：实心圆使用 `thickness=-1`。

### Q14: 程序运行一段时间后才在目标出现时崩溃

- **原因**：只测试了 `target is None` 分支，没有覆盖“检测到目标后进入绘制逻辑”的路径。
- **建议**：视觉程序必须同时验证未命中和命中两条路径。

### Q15: `Trigger signal, code:SIGSEGV(11)!`

- **原因**：常见情况是把不适合直接显示的图像格式送进了 `display.Display().show(...)`。
- **建议**：显示路径保持稳定格式，算法需要灰度图时可在内存中单独转换副本。

## 6. 日志与排障

### Q16: `isp setNoiseProfile: Noise profile get fail` 是不是崩溃根因？

- **通常不是**：这类信息更像底层 ISP 警告，不应先于 Python Traceback 下结论。
- **排查顺序**：先看日志结尾的 Python 异常类型、文件名和行号，再判断驱动日志是否相关。

### Q17: 如何快速判断问题出在验证流程、资源还是 API 本身？

- **看执行层级**：先确认是“静态分析失败”“导入失败”“顶层执行失败”还是“人工硬件未做”。
- **看资源依赖**：模型、网络、相机、显示、音频、外设接线缺失时，不要先怀疑 API。
- **看真实示例**：优先用 `examples/` 里的调用签名对照，避免因文档写法过时而误判为接口失效。

