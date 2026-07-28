---
name: maixcam-py-dev
description: Expert skill for MaixPy v4 on MaixCAM. Invoke when editing or generating MaixPy apps involving vision, peripherals, networking, streaming, validation workflow, or deployment guidance.
---

# MAIXcam (MaixPy v4) 开发 Skill 指南

本 Skill 用于指导 AI 编程助手在 Sipeed **MaixCAM / MaixPy v4** 平台上编写、审阅和解释示例代码与应用文档。所有建议都应优先对齐工作区内 `examples/` 的真实调用方式，以及当前已掌握的板端验证证据。

## 当前验证闭环

- 当前已实测闭环是：**SSH `root@10.0.0.121` + MaixCAM + UART0 `/dev/ttyS0` 115200 + PC `COM8`**。
- 板端状态会通过 UART0 输出，PC 侧可观测到 `API_VERIFY|START|...` 与 `API_VERIFY|RESULT|...`，说明 `API_VERIFY` 状态回传链路有效。
- 该闭环证明了“远程下发 + 板端执行/预检 + 串口回传”的验证路径可用，但**不等于所有硬件/网络/视觉能力都已被逐项人工实测**。

## 验证结论口径

- 当前结论必须明确区分：**已验证 7 项基础例程**，**受限验证 199 项**。
- `已验证 7 项` 指的是安全白名单内的基础示例已经在板端完成实际执行闭环。
- `受限验证 199 项` 指的是已完成静态分析、导入预检或板端预检，但因网络、模型、外设接线、相机/显示占用、系统副作用或人工交互要求而**没有自动执行顶层示例**。
- **不能把“未执行”或“受限验证”等同于 API 不可用**。当前证据只能说明“本轮未自动执行/未人工完成硬件复现”，不能反推 API 失效。

## 默认安全边界

除非用户明确确认，否则默认**禁止自动运行**以下示例或等价操作：

- 系统安装、运行时安装、卸载、写系统配置、切换 USB 模式。
- 电源管理、关机、重启、WDT、可能影响系统状态的持久化配置。
- GPIO / PWM / SPI / UART 等会向外设主动输出电平、时钟、波形或总线数据的驱动示例。
- 外部网络连接、对外服务监听、向第三方地址推流、上传数据。
- 长时间占用相机、显示、音频设备的流媒体或预览示例。
- 需要人工接线、手工按键、触摸、扫码、目标物配合或其他人工交互的示例。

## 核心开发原则

1. **以 `examples/` 为准**：API 名称、构造参数和典型流程优先与真实示例保持一致，不自行发明旧版或 MicroPython 风格写法。
2. **主循环可退出**：可持续运行的应用优先使用 `while not app.need_exit():`，避免无退出路径的死循环。
3. **示例先极简**：优先用最小可复现代码演示单个 API 闭环，先通路，再叠加模型、网络或 UI。
4. **避免高占用**：轮询循环、串口监听或等待状态时加入 `time.sleep_ms(1)` 或 `time.sleep(1)`。
5. **视觉与流媒体分离判断**：`camera/display/rtsp/webrtc` 能否导入，不代表当前环境已具备可安全自动运行的相机/显示/推流条件。
6. **异常优先看 Traceback**：先定位 Python 致命异常，再判断是否与底层驱动警告相关。
7. **不要改写 `examples/` 原文件**：Skill 文档可引用和解释示例，但不应擅自替换仓库原始例程。

## 验证优先级与复现流程

建议所有后续验证都按以下顺序执行：

1. **先跑安全基础例程**：优先使用 `examples/basic/` 中已纳入白名单的 7 个基础示例，确认 Python 运行、SSH 下发和 UART 回传链路正常。
2. **再做静态/导入预检**：检查目标示例依赖的 `maix` 模块、模型路径、设备节点、网络目标和顶层副作用，避免一上来直接跑高风险示例。
3. **最后做人工硬件验证**：对 UART、GPIO、PWM、I2C、SPI、相机、显示、网络推流等场景，先确认接线、供电、目标设备、权限和退出方式，再由操作者手工执行。
4. **结论分层记录**：把“已执行闭环”“导入通过”“静态可解析”“待人工确认”分开写，不混成单一“可用/不可用”二分结论。

## 模块化指南索引

| 模块文件 | 主题内容 | 核心 API 关键字 |
| :--- | :--- | :--- |
| [01_app_basics.md](./01_app_basics.md) | **应用基础与架构** | `app.need_exit()`, `camera.Camera`, `display.Display`, `touchscreen.TouchScreen`, `app.yaml` |
| [02_vision_ai.md](./02_vision_ai.md) | **图像处理与 AI 视觉** | `image.Image`, `nn.Classifier`, `nn.YOLOv5`, `nn.YOLO11`, `nn.FaceDetector`, `mllm` |
| [03_peripherals.md](./03_peripherals.md) | **片上外设与通信** | `pinmap.set_pin_function`, `uart.UART(device, 115200)`, `gpio.GPIO`, `pwm.PWM`, `i2c.I2C` |
| [04_network_stream.md](./04_network_stream.md) | **网络与音视频流** | `network.wifi.Wifi`, `rtsp.Rtsp`, `webrtc.WebRTC`, `audio.Recorder`, `audio.Player` |
| [05_performance_faq.md](./05_performance_faq.md) | **性能调优与验证 FAQ** | `dual_buff=True`, 验证分层, `sys.device_id()`, 调试技巧 |

## 快速 Hello MaixCAM 模版

```python
from maix import app, camera, display, image, time

cam = camera.Camera(640, 480)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    img.draw_string(10, 10, "Hello MaixCAM!", color=image.Color.from_rgb(0, 255, 0), scale=1.5)
    disp.show(img)
    time.sleep_ms(1)
```
