# 2026_Electronics_Contest

本仓库用于维护 2026 年电子设计竞赛项目资料，覆盖通信协议、引脚定义、硬件、视觉、软件、机械结构、整机联调和最终交付文件。

## 目录结构

```text
NUEDC-Project/
├── README.md
├── .gitignore
├── communication_protocol/       # 通信协议
│   ├── README.md
│   ├── protocol_specification.md  # 协议帧格式、字段和校验规则
│   ├── command_table.md           # 命令编号、参数和响应定义
│   ├── error_codes.md             # 错误码及处理说明
│   ├── source/                    # 协议相关源码
│   └── tools/                     # 协议调试、生成和验证工具
├── pinout/                       # 唯一引脚表
│   ├── README.md
│   ├── pinout.md                  # MCU 及模块引脚分配
│   └── connector_pinout.md        # 接口、排针、插座定义
├── hardware/                     # 硬件资料
│   ├── README.md
│   ├── schematic/                 # 原理图
│   ├── pcb/                       # PCB 设计文件
│   ├── libraries/                 # 符号库、封装库等硬件库
│   ├── datasheets/                # 芯片、模块和器件数据手册
│   ├── bom/                       # 硬件物料清单
│   ├── production/                # Gerber、坐标、贴片和生产文件
│   └── test/                      # 硬件测试资料
├── vision/                       # 视觉模块
│   ├── README.md
│   ├── src/                       # 视觉算法和应用源码
│   ├── calibration/               # 相机和坐标系标定文件
│   ├── models/                    # 视觉模型文件
│   ├── datasets/                  # 训练、验证和测试数据集
│   ├── scripts/                   # 训练、转换、标定和调试脚本
│   └── tests/                     # 视觉测试用例和测试资料
├── software/                     # 软件工程
│   ├── README.md
│   ├── embedded/                  # 嵌入式软件
│   │   ├── Application/           # 应用层代码
│   │   ├── BSP/                   # 板级支持包
│   │   ├── Drivers/               # 外设和模块驱动
│   │   ├── Algorithms/            # 控制、滤波、规划等算法
│   │   ├── Tasks/                 # 任务调度、线程和业务任务
│   │   ├── Protocol/              # 嵌入式端协议解析和封装
│   │   ├── Config/                # 嵌入式配置文件
│   │   └── Tests/                 # 嵌入式软件测试
│   ├── host_computer/             # 上位机软件
│   └── tools/                     # 软件构建、烧录和调试工具
├── mechanical/                   # 机械结构
│   ├── README.md
│   ├── cad/                       # 机械 CAD 源文件
│   ├── drawings/                  # 加工和装配工程图
│   ├── 3d_print/                  # 3D 打印模型和说明
│   ├── machining/                 # 机加工资料
│   ├── assembly/                  # 装配说明和检查记录
│   └── bom/                       # 机械物料清单
├── docs/                         # 项目方案、设计说明和过程文档
├── integration_tests/            # 整机联调方案、记录和问题追踪
└── deliverables/                 # 比赛最终提交材料和交付清单
```

## 目录作用说明

| 目录 | 作用 |
| --- | --- |
| `communication_protocol/` | 维护设备之间、上位机与下位机之间的通信协议定义和配套工具。 |
| `pinout/` | 作为项目唯一引脚表，集中维护 MCU、模块和接口引脚分配，避免硬件、软件、机械文档不一致。 |
| `hardware/` | 存放硬件设计、生产、物料和测试相关资料。 |
| `vision/` | 存放视觉识别、标定、模型、数据集、脚本和测试资料。 |
| `software/` | 存放嵌入式软件、上位机软件和软件开发工具。 |
| `mechanical/` | 存放机械结构设计、加工、装配和机械 BOM 资料。 |
| `docs/` | 存放项目级文档，例如方案设计、技术说明、会议记录和阶段总结。 |
| `integration_tests/` | 存放整机联调相关测试方案、测试记录、问题追踪和验证结果。 |
| `deliverables/` | 存放比赛提交用的最终材料、演示资料和交付清单。 |

## 维护约定

- 每个目录保留一个 `README.md`，说明该目录当前用途和文件放置规则。
- 引脚定义统一维护在 `pinout/`，其他文档如需引用引脚，应以该目录内容为准。
- 通信协议统一维护在 `communication_protocol/`，软件和上位机实现应与协议文档同步更新。
- 面向比赛提交的最终版本资料统一放入 `deliverables/`。