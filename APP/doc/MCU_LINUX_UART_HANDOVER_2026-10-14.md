# MCU ↔ Linux 串口链路改造 —— 设计与交接文档

| 项 | 内容 |
|---|---|
| 日期 | 2026-10-14 |
| 目标工程 | `C:\Users\Lenovo\Desktop\H785_Test\Lwip_Ping_Test\APP` |
| 背景 | 主板不再搭载 WiFi 模组，R900PNR 移到 Linux 板；MCU 与遥控器的一切无线通信经由 Linux 串口透传 |
| 当前阶段 | 设计已评审，**尚未动一行代码** |
| 本文用途 | 若更换会话，新会话读本文即可完整接手 |

---

## 0. 立即要做的事（新会话照着做）

1. 读本文全文，特别是第 3、4 章的**依赖关系**和第 6 章的**待决问题**
2. 先回答第 6 章的 2 个待决问题
3. 按第 5 章的 M0 / M1 顺序，**一次只做一小步**，每步产出可观测结果后停下来等用户确认
4. **不要在未经确认的情况下批量改文件**

---

## 1. 背景与范围

### 1.1 改造前后

```
【改造前】
  遥控器(AP) ⇄ 无线 ⇄ R900PNR 模组 ⇄ SPI ⇄ MCU(N32H785)
  MCU 自己跑泰芯 hgic SDIO-SPI 驱动

【改造后】
  遥控器(AP) ⇄ 无线 ⇄ R900PNR 模组 ⇄ Linux 板 ⇄ 串口(UART) ⇄ MCU(N32H785)
  主板不再有 WiFi 模组；R900PNR_CMD/ 整套 SPI 驱动作废
```

### 1.2 各方职责

| 角色 | 职责 |
|---|---|
| 遥控器(AP) | 组帧/解帧，指令下发与应答，超时重发（后续版本） |
| Linux 板 | 视频流发送；**串口数据纯透传，不修改、不封装遥控器数据** |
| MCU(STA/船控) | 收发本协议帧，执行指令，回 ACK；日志也从这个串口发给 Linux |

### 1.3 本阶段（M0/M1）范围

- **做**：USART1 双向 DMA 收发通道、帧解析、命令分发、心跳收发闭环、日志走同口
- **不做**：等待 ACK 状态机、超时重发、连接管理、序号匹配逻辑
- **保留不变**：OTA 的 `"UPDATE"` ASCII 命令检测（Linux 下发该字样后照旧升级）

---

## 2. 协议规格（来源：`遥控器-无人船命令下发数据格式.doc`）

### 2.1 帧结构

| 偏移 | 字段 | 长度 | 说明 |
|---|---|---|---|
| 0 | HEAD | 2 | 固定 `AA 55`，不参与 CRC |
| 2 | CMD | 1 | 高 2 位标志 + 低 6 位指令码 |
| 3 | LEN | 2 | DATA 字节数，**小端** |
| 5 | DATA | N | N = LEN，0 ~ 1024 |
| 5+N | CRC16 | 2 | **小端**（先低字节） |
| 7+N | TAIL | 2 | 固定 `0D 0A` |

**总长度 = 9 + LEN**；最小帧 9 字节，最大帧 1033 字节。

### 2.2 CMD 位定义

| 位 | 名称 | 含义 |
|---|---|---|
| bit7 | ACK | 0=请求或主动上报；1=应答 |
| bit6 | ERR | 0=正常；1=错误 |
| bit5~0 | CODE | 指令码，0x00 ~ 0x3F |

```c
CODE = CMD & 0x3F
ACK  = (CMD & 0x80) >> 7
ERR  = (CMD & 0x40) >> 6
```

| ACK | ERR | 含义 |
|---|---|---|
| 0 | 0 | 请求指令或主动上报 |
| 1 | 0 | 正常应答 |
| 1 | 1 | 错误应答 |
| 0 | 1 | 保留 |

### 2.3 指令码 CODE

| CODE | 名称 | 说明 |
|---|---|---|
| 0x00 | 心跳 | 保持连接或检测在线 |
| 0x01 | 设备信息 | 读设备型号、版本、序列号 |
| 0x02 | 读参数 | |
| 0x03 | 写参数 | |
| 0x04 | 数据上报 | 设备主动上传 |
| 0x05 | 控制指令 | 控制设备动作 |
| 0x06 | 固件升级 | |
| 0x07 | 时间同步 | |
| 0x08 | 日志读取 | |
| 0x09 | 恢复出厂 | |
| 0x0A~0x3F | 保留 | 未定义 CMD 收到后回错误应答 0x03 |

### 2.4 CRC16

| 参数 | 值 |
|---|---|
| 类型 | CRC-16/MODBUS |
| 多项式 | 0x8005（反射 0xA001） |
| 初值 | 0xFFFF |
| 输入反射 / 输出反射 | 是 / 是 |
| 结果异或 | 0x0000 |
| 计算范围 | **CMD + LEN + DATA**（不含 HEAD / CRC 自身 / TAIL） |
| 发送字节序 | 低字节在前 |

**已用脚本验证**：两个文档示例帧算出的 CRC 与文档值完全一致，算法确认无误。

### 2.5 错误码（ERR=1 时 DATA 首字节）

`0x01` CRC错误 · `0x02` 长度错误 · `0x03` 不支持的命令 · `0x04` 参数错误 ·
`0x05` 设备忙 · `0x06` 权限不足 · `0x07` 执行失败 · `0x08` 超时 · `0x09` 数据格式错误

### 2.6 解帧规则（协议第 8 节）

1. 缓冲区搜索 `AA 55` 定位帧起始
2. 至少再读 5 字节获得 CMD 和 LEN
3. 算总长 `9 + LEN`；不足则等更多数据
4. 够长则查 TAIL 是否 `0D 0A`
5. 提取 CRC16，计算 `CMD+LEN+DATA` 的 CRC
6. 校验通过 → 提取 CMD 和 DATA 交业务处理
7. 处理完从缓冲区移除该帧，**继续解析剩余字节（必须支持粘包）**

**关键约束**：DATA 中允许出现 `AA 55`、`0D 0A` 等任意字节，
接收方必须依据 **LEN 和 CRC16** 解析，**不得依据包尾截断**。

### 2.7 本次需要的帧（已实算）

| 用途 | 帧（hex） | CRC |
|---|---|---|
| 心跳请求 | `AA 55 00 00 00 71 C0 0D 0A` | 0xC071 |
| 心跳应答 | `AA 55 80 00 00 70 28 0D 0A` | 0x2870 |
| 错误应答（不支持的命令） | `AA 55 C0 01 00 03 2D E5 0D 0A` | 0xE52D |

---

## 3. 现有架构（已吃透，勿改）

### 3.1 目录分层

```
APP/
├─ src/            main.c · n32h7xx_it.c        流程编排，不含业务
├─ Mcal/           inc/ + src/                  MCAL层：硬件抽象，只碰寄存器/外设
│    Mcal_InitSum.c        统一初始化入口（按顺序拉起所有MCAL模块）
│    Mcal_TimerTask.c/h    两级时基 + 协作式调度器
│    Mcal_LedGpio_Init.c / Mcal_KeyGpio_Init.c
│    Mcal_SensI2C.c / Mcal_SdSpi.c / Mcal_Esc_*.c
├─ User_App/       inc/ + src/                  APP层：业务逻辑（App_*_Handle）
│    App_TaskServer_Handle.c   任务的注册与内容
│    App_Sensor_Handle.c / App_Fusion_Handle.c / App_OTA_Handle.c / ...
├─ bsp/            log.c 等
└─ R900PNR_CMD/    泰芯厂商驱动（本次作废）
```

### 3.2 MCAL 与 APP 的边界（重要）

- MCAL **不许知道** APP 要干什么
- `Mcal_TimerTask.c` 里**不注册任何具体任务**，只提供 `Task_Create()`
- 具体"哪个周期跑什么活"写在 APP 的 `App_Task_Init()` 里
- 需要 APP 参与的地方用**函数指针**挂钩（如 `Mcal_SampleTick_SetHandler()`），
  MCAL 不 include APP 的头文件

### 3.3 两级时基

| | 1ms 协作式调度器 | 2kHz 硬件采样节拍 |
|---|---|---|
| 载体 | SysTick 中断只做 `TimeBase++` | BTIM1 中断 |
| 执行 | `Task_Handle_Cycle()` 在主循环里跑 | 在中断里直接跑 |
| 周期 | 5 个固定档位 10/20/50/200/300ms | 2000Hz ÷16 = 125Hz |
| 定位 | 允许抖动的工作（打印、按键消抖、协议解析） | 需要精确间隔的工作（传感器采样） |

**选型原则**：需要精确间隔的活儿**绝不能**挂在主循环 slot 上——
主循环里还跑着射频、网络、视频，slot 量到的是它们的调度，不是自己的周期。

### 3.4 主循环骨架

```c
main():
    App_VectorTableInit();   /* VTOR + SysTick 槽位重定向 */
    时钟 / Cache / MPU
    SysTick_Config();        /* 1ms */
    Mcal_InitSum();          /* 按顺序拉起所有 MCAL */
    App_Task_Init();         /* 注册 5 个周期任务，必须在 TimerTask_Init 之后 */
    while (1) { <轮询>; Task_Handle_Cycle(); }
```

### 3.5 代码风格约定（务必遵守）

- 每个函数上方一块注释：`@name` / `@brief`(一句话作用) / `@param` / `@retval`，
  必要时加 `@note` 说明**为什么这么做**（尤其是踩过的坑）
- 函数内部**少量中文注释，每句不超过 10 字**
- 大段设计理由写在 `.h` 的宏定义旁或函数注释的 `@note` 里，不塞进函数体
- **变量命名**：驼峰 + 下划线，每词首字母大写，例：`A_Big_Apple`
- **全局变量不带 `g_` 前缀**
- 变量名保持全工程一致

---

## 4. 设计方案

### 4.1 文件变更

**新增**

| 文件 | 作用 |
|---|---|
| `Mcal/inc/Mcal_Uart.h` + `src/Mcal_Uart.c` | USART1 双向 DMA + IDLE 中断驱动 |
| `Mcal/inc/Mcal_Uart_Cfg.h` | 引脚/波特率/DMA通道/缓冲区大小 宏定义 |
| `User_App/inc/App_RcProto_Handle.h` + `src/App_RcProto_Handle.c` | 帧解析/组帧/CRC16/粘包处理 |
| `User_App/inc/App_RcCmd_Handle.h` + `src/App_RcCmd_Handle.c` | 各 CMD 的处理函数 |

**修改**

| 文件 | 改动 |
|---|---|
| `src/main.c` | 删 R900PNR 初始化与 WiFi 配置；改为 `Mcal_InitSum()` + `App_Task_Init()`；主循环只留 `Task_Handle_Cycle()` |
| `inc/main.h` | 摘掉 `n32h7xx_eth.h`、`n32eth_cfg.h`、`bsp_eth.h`，以及 `DEMO_BOARD`/`PHY_USE_*` 的 `#error` 块 |
| `src/n32h7xx_it.c` | 摘掉 `#include "bsp_eth.h"`，删除 `ETH_GLOBAL_IRQHANDLER` |
| `bsp/src/log.c` | `printf` 底层改为 `Mcal_Uart` DMA 发送 |

**删除 / 移出编译**（见 4.3 依赖关系）

### 4.2 MCAL 对外接口

```c
/* Mcal_Uart.h —— 只搬字节，不懂协议 */
void     Mcal_Uart_Init(void);
int32_t  Mcal_Uart_Send(uint8_t *Data, uint32_t Len);      /* 异步DMA，忙返回 -1 */
uint8_t  Mcal_Uart_RxReady(void);                          /* 有数据待取? */
uint32_t Mcal_Uart_GetData(uint8_t *Dst, uint32_t MaxLen); /* 取走，返回实际长度 */
```

### 4.3 数据结构（DMA 相关用结构体封装）

```c
/* RX: 硬件 DMA 直接写这块，不经过 CPU */
typedef struct
{
    uint8_t   *DmaBuf;       /* 接收暂存，2048 字节 */
    uint16_t   DmaLastPos;   /* 上次搬走的位置 */
    uint16_t   FifoLen;      /* Fifo 里待取长度 */
    uint8_t    Fifo[256];    /* 交给 APP 的 Fifo */
} Uart_Rx_t;

/* TX: 日志和协议帧共用同一条 DMA 通道 */
typedef struct
{
    uint8_t   *Buf;          /* 发送缓冲，1088 字节 */
    uint32_t   Len;          /* 本次发送长度 */
    uint8_t    Busy;         /* 1 = DMA 正在发 */
} Uart_Tx_t;
```

### 4.4 数据流

```
【收】USART1 ──DMA──> Uart_Rx.DmaBuf[2048]
                        │  IDLE中断 / 已收 ≥64 字节 / 接近满
                        ↓  按 DMA 剩余计数算出新增长度，搬进 Uart_Rx.Fifo[256]
                        ↓  置 RxFlag
      10ms任务槽: Mcal_Uart_GetData() ──> APP解析(搜 AA55 → 校验 LEN → CRC → TAIL)
                        ↓
                CMD 分发 → 执行 → 组 ACK → Mcal_Uart_Send()

【发】日志 / 协议帧 ──> Uart_Tx.Buf[1088] ──DMA──> USART1 ──> Linux
                       Busy=1 期间：日志丢弃，协议帧返回 -1
```

### 4.5 关键设计点

1. **必须用 DMA + IDLE 中断**，不能只用"接收完成中断"。
   变长帧场景下 DMA 只知道收了多少字节，不知道对方说完了；
   USART IDLE 中断在总线空闲一个字符时间后触发，此时
   `2048 - DMA剩余计数` 就是本帧长度。

2. **TX 缓冲区必须 ≥1033 字节**。协议最大帧 = 9 + 1024 = 1033。
   日志用不到这么多，但同一个缓冲要兼顾协议帧，故定 **1088**。

3. **日志走同一根 TX 线，无环缓冲**。DMA 忙时新日志直接丢弃——
   日志是非关键信息，不能为了它让主循环阻塞。
   **严禁**回到 `fputc` 里 `while(TXC==RESET)` 逐字节死等的旧写法：
   115200 下一个字节 87µs，50 字符的日志会占住主循环 4.3ms。

4. **日志与帧的区分靠 Linux 侧解析，MCU 侧不加前缀**。
   日志是 ASCII 文本行，协议帧有 `AA 55` 包头 —— 天然可辨。

5. **`log.c` 留在 bsp/，只换底层**。`log.c` 调 `Mcal_Uart_Send()`，
   绝不 include APP 头文件，避免 MCAL→APP 的反向依赖。

6. **MCU 复位后主动发一次心跳请求**，作为链路可用的最早信号，
   也让 Linux 侧能立即校验组帧正确性。

7. **`BTIM1` 125Hz 采样节拍本次不移**（当前工程无传感器），
   只移 `Mcal_TimerTask` 的 1ms 调度部分。

### 4.6 TX 缓冲优化（可选）

`printf` 若逐字符调 `Mcal_Uart_Send()`，会频繁丢弃。
建议在 `log.c` 内加一个 `static uint8_t LogBuf[256]`，
攒到 `\n` 或缓冲满时**一次性** DMA 发出。这不算环缓冲，只是行缓冲。

### 4.7 任务槽分配

```c
Task_10ms_ID   →  串口收包解析 + 命令分发        ← 本次新增
Task_20ms_ID   →  OTA_UpdateCheck_Handle()        ← 从 main 循环搬过来
Task_50ms_ID   →  预留（心跳定时发送，M2）
Task_200ms_ID  →  预留（LED）
Task_300ms_ID  →  预留
```

`Task_Timer_MaxNum = 5`，槽位 ID 定义在 `Mcal_TimerTask.h`。

### 4.8 废弃 SPI 链路的处理方式

采用 **Keil 工程 Remove File + 磁盘重命名**，不使用 `/**/` 块注释：
`hgic_sdspi_v2.c` 内含数十处 `/* */`，整文件外包一层会编译失败。

### 4.9 待删文件清单（**含依赖关系，勿漏**）

**Keil 工程 Remove（文件保留在磁盘）**

| 组 | 移除项 |
|---|---|
| `Mcal` | `r900pnr_spi_init.c` |
| `User_App` | `App_R900PNR_Handle.c` |
| `R900PNR_CMD` | `hgic_cmd.c` `hgic_raw.c` `hgic_sdspi_v2.c` `r900pnr_spi_api.c`（整组） |
| `STARTUP` | `startup_n32h73x_76x.s`（本就排除编译） |
| `FWLB` | `n32h7xx_eth.c` |
| `LwIP\*` | 全部 9 组 41 个文件 |
| `ETH\bsp` | `bsp_eth.c` `ethernetif.c` |
| `BSP` | `delay.c` |

**磁盘删除**

`lwip-2.2.0/`（整目录）、`bsp/src/bsp_sdram.c`、`bsp/inc/bsp_sdram.h`、
`bsp/src/delay.c`、`bsp/inc/delay.h`、`bsp/bsp_eth.c`、`bsp/bsp_eth.h`、
`bsp/ethernetif.c`、`bsp/ethernetif.h`、`bsp/lwipopts.h`、`bsp/sys_arch.c`、`bsp/arch/`

**依赖关系（必须同步改，否则编译不过）**

| 被删文件 | 谁在依赖它 | 怎么改 |
|---|---|---|
| `bsp_eth.h` / `n32eth_cfg.h` | `inc/main.h`（第 73、75 行） | 摘掉 include |
| `bsp_eth.h` | `src/n32h7xx_it.c`（第 55 行） | 摘掉 include，删 `ETH_GLOBAL_IRQHANDLER` |
| `DEMO_BOARD` / `PHY_USE_*` | `inc/main.h`（第 78~108 行） | 删掉这些 `#error` 校验块 |
| `delay.h` | `inc/main.h`（第 72 行） | 摘掉 include。`delay.c` 只有 `systick_delay_ms()`（会破坏 SysTick），唯一调用方都在删，无人再用 |

**保留不动**

`bsp/inc/log.h`、`bsp/src/log.c`（只改底层）、`bsp/src/print_remap.c`（未参与编译）、
`bsp/inc/n32h7xx_eval.h`、`Hardware/App_VectorTable.c`、`Hardware/App_OTA_Handle.c`

### 4.10 M1 最小闭环验证项

| # | 操作 | 期望结果 |
|---|---|---|
| 1 | MCU 上电 | Linux 侧收到 ASCII 日志 |
| 2 | MCU 主动发心跳 | Linux 侧字节完全等于 `AA 55 00 00 00 71 C0 0D 0A` |
| 3 | Linux 下发心跳 | MCU 回 `AA 55 80 00 00 70 28 0D 0A` |
| 4 | Linux 下发未知 CMD(0x0A) | MCU 回 `AA 55 C0 01 00 03 2D E5 0D 0A` |
| 5 | Linux 连发 3 帧粘包 | MCU 全部正确解析，不丢帧 |

**第 2 条是黄金验证点**：Linux 侧收到的字节若与协议文档示例帧完全一致，
说明组帧与 CRC 百分百正确。

---

## 5. 实施顺序

### M0 —— 地基（先把架子搭起来）

1. 移植 `Mcal/inc/Mcal_TimerTask.h` + `src/Mcal_TimerTask.c`（只移 1ms 调度部分，暂不移 BTIM1）
2. 移植 `Mcal/inc/Mcal_InitSum.h` + `src/Mcal_InitSum.c`（按本工程实际模块精简）
3. 新建 `User_App/inc/App_TaskServer_Handle.h` + `src/App_TaskServer_Handle.c`（先放空任务槽）
4. 改 `main.c` 走新骨架
5. **验证**：两个 LED 或日志节拍能看出 `Task_Handle_Cycle()` 在跑

### M1 —— 串口链路

6. 新建 `Mcal_Uart_Cfg.h` + `Mcal_Uart.h/.c`（GPIO + USART1 + DDR DMA + IDLE 中断）
7. 改 `bsp/src/log.c` 底层为 `Mcal_Uart_Send()`
8. **验证**：Linux 侧收到日志，且 MCU 不再阻塞
9. 新建 `App_RcProto_Handle.h/.c`（CRC16 + 组帧 + 解帧 + 粘包）
10. **验证**：先用**已知帧**单独测 CRC 和解析（可以只在 MCU 内部自测，不接 Linux）
11. 新建 `App_RcCmd_Handle.h/.c`（心跳 + 未知命令错误应答）
12. 接到 `Task_10ms_Handle()`
13. **验证**：跑完 4.10 的五条验证项

### M2 及以后（本次不做）

序号匹配、超时重发、各业务 CMD（设备信息/读参数/写参数/控制/上报）、连接管理

---

## 6. 待决问题（新会话必须先问）

| # | 问题 | 说明 |
|---|---|---|
| 1 | APP 层文件夹用 `User_App/` 还是 `Hardware/`？ | 本工程现用 `User_App/`，**建议沿用 `User_App/`** |
| 2 | `BTIM1` 125Hz 采样节拍本次移不移？ | 本工程有传感器，**可能需要一起移**；需确认 |

## 7. 已确认的决策（勿再重复询问）

| # | 决策 |
|---|---|
| 1 | 日志与协议数据**共用 USART1**，日志也从这里发给 Linux；开发期日志多，后期收敛 |
| 2 | **TX 也用 DMA**，一个数组即可，**不做环缓冲**；缓冲区定 **1088** 字节 |
| 3 | 日志是**只发不收**的原始 ASCII，不加前缀；命令帧自带包头包尾，天然可辨 |
| 4 | 接收溢出判定用 **DMA + USART IDLE 中断** |
| 5 | 协议解析挂在 **10ms 任务槽** |
| 6 | 波特率**初始值 115200**，在 `Mcal_Uart_Cfg.h` 里用宏定义，便于后续更改 |
| 7 | **序号位采用方案 S1**：加在 `DATA[0]`，LEN 相应 +1。但**M1 先不带序号**，M2 再加 |
| 8 | **OTA 保留不变**：Linux 下发 `UPDATE` 字样后照旧升级 |
| 9 | 本次**不做**等待 ACK 状态机、超时、重发 |
| 10 | UI 变量命名：驼峰+下划线+首字母大写，全局变量不带 `g_` 前缀 |
| 11 | 废弃 SPI 链路用 **Keil Remove File** 方式，不用 `/**/` 块注释 |

---

## 8. 环境备忘（会踩的坑）

### 8.1 文件是加密存储的，且加密是进程级白名单

```powershell
# pwsh.exe 读到的是密文（头 8 字节 88 7D 1C xx 1A 00 05 00）
# python.exe 能透明解密，读到明文
```

**读源码请用 python**：

```powershell
python -c "import sys; sys.stdout.reconfigure(encoding='utf-8'); print(open(r'绝对路径', encoding='utf-8', errors='replace').read())"
```

批量读多个文件时写个小脚本，比逐个调用快。

### 8.2 写入的加密行为

实测：用 `pwsh` 新建的文件**不会**被加密，是明文。
Keil 对此无感知（它读加密文件同样看到明文），**编译不受影响**。
但若公司 DLP 有"源码必须加密存储"的合规要求，需自行确认。

### 8.3 `.doc` 文件的解析

`遥控器-无人船命令下发数据格式.doc` 是 OLE2（老版 Word），
Word COM 在沙箱里不可用。用 python 直接扫 UTF-16LE 文本段即可，
表格内容在另一段，扫描时要允许跳过小的非文本间隙再合并。

### 8.4 其他

- 工程是 **git 仓库**（`Lwip_Ping_Test/.git`），改坏可 `git checkout` 回滚
- `startup_n32h73x_76x.s` 在 Keil 工程组里但**被排除编译**，实际链接的是 `startup_n32h78x_cm7.o`
- `n32h7xx_it.c` 里的 `SysTick_Handler` 被 SDK 链接在**向量表槽位 250**，
  而槽位 15（SysTick 异常）指向空壳 `N32SysTick_Handler`。
  `App_VectorTableInit()` 在 `main()` 最开头把槽位 15 改写到真 handler。
  **这个函数必须在任何延时/超时逻辑之前调用**，否则 `TimeBase` 永远不动
- 现有 `USART1_IRQHandler` 在 `bsp/src/log.c`，是强定义。新的 USART1 驱动接管它不会冲突

---

## 9. 参考工程对照

| 路径 | 用途 |
|---|---|
| `Lwip_Ping_Test/APP` | **本工程**（要被改造的） |
| `Lwip_Ping_Test/BOOTLOADER` | 配套 Bootloader 工程 |
| `Desktop/H785_SEND_TEST/APP` | 古老的 R900PNR WiFi 测速备份工程，**与本次改造无关**，仅可作参考 |

`H785_SEND_TEST/APP/readme.txt` 里有 R900PNR SPI 接线的完整记录
（PA4=CS / PA5=SCK / PA6=MISO / PA7=MOSI / PA8=INT），本次作废但留档。

---

*文档结束*