# 04 - 网络通信与音视频流指南

本模块说明 MaixCAM 上 Wi-Fi、HTTP/Socket、RTSP、WebRTC、音频等场景的推荐写法。这里必须特别注意：**网络连接、对外服务、推流、长时间占用相机/显示/音频设备** 都属于默认禁止自动执行的高风险示例，除非用户明确确认。

## 1. 默认安全边界

除非用户明确确认网络目标、带宽环境、相机占用和退出方式，否则默认不要自动运行：

- Wi-Fi 配网、AP 模式、外部网络连接。
- HTTP/MQTT/WebSocket/Socket 对外请求或服务监听。
- RTSP、RTMP、WebRTC、HTTP JPEG 推流。
- 长时间占用 `camera.Camera()`、`display.Display()`、`audio.Recorder()`、`audio.Player()` 的流媒体例程。

当前这类示例大多属于**受限验证**，原因是它们依赖网络目标、音视频资源或人工确认，不是因为 API 已被证明不可用。

## 2. Wi-Fi 调用方式

### 与 `examples/network/wifi_connect.py` 一致的写法

```python
from maix import err, network

# 连接指定 Wi-Fi
w = network.wifi.Wifi()
e = w.connect("SSID_NAME", "PASSWORD", wait=True, timeout=60)
err.check_raise(e, "connect wifi failed")
print("Connect success, got ip:", w.get_ip())
```

关键点：

- 推荐写法是 **`network.wifi.Wifi().connect(...)`**。
- 不要再写成 `network.WLAN(...)` 或 `network.wifi.connect(...)` 这种与仓库示例不一致的形式。
- 即便 API 写法明确，是否自动执行仍取决于是否获得了用户对外部网络操作的明确授权。

## 3. HTTP、Socket 与其他网络示例

HTTP、Socket、MQTT、WebSocket 例程在当前验证体系中通常只适合做以下两类确认：

- 静态确认：检查导入、URL/主机/端口参数和异常处理方式。
- 人工复现：由操作者确认目标地址、服务端、认证信息和网络出口后，再手工执行。

请避免把“没有自动请求外部地址”写成“HTTP/Socket API 不可用”。当前正确表述应是：**本轮未自动执行，待人工确认网络目标后复现。**

## 4. RTSP 推流

### 与 `examples/vision/streaming/rtsp.py` 一致的写法

```python
from maix import audio, camera, image, rtsp, time

AUDIO_ENABLE = True
audio_recorder = None

cam = camera.Camera(640, 480, image.Format.FMT_YVU420SP)
server = rtsp.Rtsp()
server.bind_camera(cam)

if AUDIO_ENABLE:
    audio_recorder = audio.Recorder()
    server.bind_audio_recorder(audio_recorder)

server.start()
print(server.get_url())

while True:
    time.sleep(1)
```

关键点：

- 推荐构造方式是 **`rtsp.Rtsp(); bind_camera(cam)`**。
- 不要写成旧式的 `rtsp.RTSP(...)`，也不要假设必须手工 `write(img)`。
- 实际示例使用 `image.Format.FMT_YVU420SP` 作为相机格式，更符合推流链路需要。
- RTSP 属于外部网络服务，同时还会持续占用相机和可选音频设备，默认不自动运行。

## 5. WebRTC 推流

### 与 `examples/vision/streaming/webrtc.py` 一致的写法

```python
from maix import app, camera, image, time, webrtc

cam = camera.Camera(640, 480, image.Format.FMT_YVU420SP, fps=30)
server = webrtc.WebRTC()
server.bind_camera(cam)
server.start()

print(server.get_url())

while not app.need_exit():
    time.sleep(1)
```

关键点：

- 推荐构造方式是 **`webrtc.WebRTC(); bind_camera(cam)`**。
- 不要写成 `webrtc.WebRTC(width, height)` 再循环 `write(img)` 的旧风格。
- WebRTC 同样属于网络服务 + 长时间占用相机资源，默认不自动执行。

## 6. 其他流媒体说明

`http.JpegStreamer`、`rtmp.Rtmp`、`uvc.UvcServer`、`uvc.UvcStreamer` 等 API 也应按同样原则处理：

- 先对齐仓库真实示例调用方式。
- 再区分“可静态解释”和“已安全执行”。
- 未自动跑推流，不等于这些 API 不可用，只是当前安全策略下未进入实际推流阶段。

## 7. 音频录制与播放

音频示例通常依赖麦克风、扬声器或流媒体链路，默认也不自动运行。可以把它们作为“调用说明”保留：

```python
from maix import audio

player = audio.Player()
recorder = audio.Recorder()
```

说明：

- `audio.Player()`、`audio.Recorder()` 的存在可用于静态解释或导入预检。
- 真正播放/录制前，应先确认设备占用、目标文件、采样率与退出路径。

## 8. 网络与流媒体验证建议

建议按下面顺序复现：

1. 先确认 SSH + UART `API_VERIFY` 闭环正常，避免把网络问题和板端执行问题混在一起。
2. 再做静态/导入预检，确认 `network`、`rtsp`、`webrtc`、`audio` 模块可导入。
3. 最后在用户明确确认网络目标、设备占用和人工观察方式后，执行单个流媒体例程。

这样可以避免把“当前没跑网络/推流”误写成“网络/流媒体 API 已失效”。
