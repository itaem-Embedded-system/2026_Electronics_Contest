# 03 - 片上外设与通信协议指南

本模块聚焦 MaixCAM 上的 UART、GPIO、PWM、I2C、SPI 及应用层通信协议。文档示例优先对齐工作区 `examples/peripheral/` 与 `examples/protocol/` 的真实调用方式，并明确区分“已验证闭环”和“待人工硬件验证”。

## 1. 当前已知外设验证闭环

- 当前已实测闭环为：**SSH `root@10.0.0.121` -> MaixCAM -> UART0 `/dev/ttyS0` 115200 -> PC `COM8`**。
- 已能在 PC 串口侧观测 `API_VERIFY|START|...` 与 `API_VERIFY|RESULT|...`，说明 UART0 回传链路和验证状态输出有效。
- 这只能证明 **UART0 观测链路已打通**，不等于 GPIO、PWM、I2C、SPI 或其他外设都已经过同等级人工接线验证。

## 2. 默认安全边界

除非用户明确确认接线、供电和目标设备状态，否则默认**不要自动运行**以下外设示例：

- GPIO 输出、电平翻转、板载 LED 驱动。
- PWM 波形输出、舵机/电机/蜂鸣器等执行器控制。
- SPI、UART、I2C 向外部设备主动发包或写寄存器。
- 任何会造成外部硬件动作、总线争用或电气风险的顶层例程。

推荐流程是：**先看静态调用与引脚映射，再做导入预检，最后由操作者人工上板验证。**

## 3. Pinmap 引脚映射

初始化外设前，先完成引脚复用。以 MaixCAM 上最常用的 UART0 为例：

```python
from maix import err, pinmap

err.check_raise(pinmap.set_pin_function("A16", "UART0_TX"), "set A16 to UART0_TX failed")
err.check_raise(pinmap.set_pin_function("A17", "UART0_RX"), "set A17 to UART0_RX failed")
```

说明：

- MaixCAM 上 UART0 常见对应引脚是 `A16` / `A17`。
- Linux 设备节点通常是 `/dev/ttyS0`。
- 若使用不同机型，需先根据 `sys.device_id()` 选择正确引脚和设备节点。

## 4. UART 调用方式

### 与 `examples/peripheral/uart/comm_uart.py` 一致的写法

```python
from maix import app, err, pinmap, sys, time, uart

device_id = sys.device_id()
if device_id == "maixcam2":
    pin_function = {
        "A21": "UART4_TX",
        "A22": "UART4_RX",
    }
    device = "/dev/ttyS4"
else:
    pin_function = {
        "A16": "UART0_TX",
        "A17": "UART0_RX",
    }
    device = "/dev/ttyS0"

for pin, func in pin_function.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin {pin} to {func}")

serial_dev = uart.UART(device, 115200)
serial_dev.write(b"hello 1\r\n")
serial_dev.write_str("hello 2\r\n")

while not app.need_exit():
    data = serial_dev.read()
    if data:
        print("Received:", data)
        serial_dev.write(data)
    time.sleep_ms(1)
```

关键点：

- 推荐构造方式是 **`uart.UART(device, 115200)`**。
- `device` 应直接使用 Linux 串口节点，如 `/dev/ttyS0`。
- `write()` 发送 `bytes`，`write_str()` 发送字符串。
- 当前项目里的 `API_VERIFY` 闭环，就是通过这一类 UART0 输出链路完成状态观测。

## 5. GPIO 使用说明

GPIO 例程通常需要真实连线或板载 LED 对应关系，默认不要自动执行输出型代码。仅说明典型写法：

```python
from maix import gpio, pinmap

pinmap.set_pin_function("A14", "GPIOA14")
led = gpio.GPIO("GPIOA14", gpio.Mode.OUT)
led.value(1)
```

注意：

- GPIO 输出本质上会驱动外部电平，必须先确认该引脚没有连接敏感设备。
- 若只是确认 API 存在，优先做静态分析或导入预检，不直接翻转引脚。

## 6. PWM 调用方式

### 与 `examples/peripheral/pwm/pwm_led.py` 一致的写法

```python
from maix import err, pinmap, pwm, sys, time

device_id = sys.device_id()
if device_id == "maixcam2":
    pin_name = "B25"
    pwm_id = 6
else:
    pin_name = "A18"
    pwm_id = 6

err.check_raise(pinmap.set_pin_function(pin_name, f"PWM{pwm_id}"), "set pinmap failed")

out = pwm.PWM(pwm_id, freq=100000, duty=0, enable=True)

for duty in range(0, 100, 10):
    out.duty(duty)
    time.sleep_ms(100)
```

关键点：

- 推荐构造方式是 **`pwm.PWM(0, freq=..., duty=..., enable=True)`** 这一类签名。
- 第一个参数是 **整数 ID**，不是 `"PWM0"` 这样的字符串。
- `enable=True` 可以在构造时直接启用，无需假设一定存在单独 `enable()` 步骤。
- PWM 属于主动输出，默认不自动执行。

## 7. I2C 调用方式

### 与 `examples/peripheral/i2c/i2c_master.py` 一致的写法

```python
from maix import err, i2c, pinmap, sys

device_id = sys.device_id()
if device_id == "maixcam2":
    scl_pin_name = "A1"
    scl_i2c_name = "I2C6_SCL"
    sda_pin_name = "A0"
    sda_i2c_name = "I2C6_SDA"
    i2c_id = 6
else:
    scl_pin_name = "A15"
    scl_i2c_name = "I2C5_SCL"
    sda_pin_name = "A27"
    sda_i2c_name = "I2C5_SDA"
    i2c_id = 5

err.check_raise(pinmap.set_pin_function(scl_pin_name, scl_i2c_name), "set scl pin failed")
err.check_raise(pinmap.set_pin_function(sda_pin_name, sda_i2c_name), "set sda pin failed")

bus = i2c.I2C(i2c_id, i2c.Mode.MASTER)
print(bus.scan())
```

关键点：

- 推荐构造方式是 **`i2c.I2C(0, i2c.Mode.MASTER)`** 这一类签名。
- 真实例程中常根据机型动态选择 `i2c_id`，MaixCAM 未必固定是 `0`。
- 如果文档只想展示 API 形式，可以写 `i2c.I2C(0, i2c.Mode.MASTER)`；如果要给可运行示例，优先参考仓库里的真实引脚与 `i2c_id` 选择逻辑。
- `scan()` 属于相对安全的探测方式，但仍需确认总线接线和电压兼容。

## 8. SPI 与其他总线

SPI、Modbus、自定义协议等都属于**默认需要人工确认**的范畴。原因不是这些 API 不可用，而是：

- 本轮验证没有对这些总线做自动闭环执行。
- 许多例程会主动发包、写寄存器或依赖外部从设备响应。
- “未自动执行”仅表示当前安全策略下保守处理，**不能解释为 API 不存在或不可用**。

## 9. `maix.comm.CommProtocol` 与协议类

对于上层协议打包，可参考 `examples/protocol/comm_protocol.py`：

```python
from maix import app, comm

p = comm.CommProtocol(buff_size=1024)

while not app.need_exit():
    msg = p.get_msg()
    if msg:
        print("got message")
```

说明：

- 这类协议封装适合建立在 UART/TCP 等通道之上。
- 若底层通信链路本身未人工确认，协议层也只能视为“静态/导入可参考”，不能宣称已完成功能闭环。
