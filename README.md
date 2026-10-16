# N32H785-APP-BOOTLOADER

基于国民技术 **N32H785**（Cortex-M7 @ 600 MHz）的 **Bootloader + APP** 完整工程，
Bootloader 在官方 SDK `IAP_Uart` 例程基础上移植修复。

| 目录 | 内容 | Keil 工程 |
|---|---|---|
| **`BOOTLOADER/`** | 一级引导：XMODEM-CRC 接收固件、双 Bank 交替写入、CRC 校验后跳转 | `BOOTLOADER/Bootloader/MDK-ARM/Bootloader.uvprojx` |
| **`APP/`** | 业务应用：运行在 ITCM，经 USART1 与 Linux 主控通信 | `APP/MDK-ARM/LWIP_PING.uvprojx` |

两个工程**各自独立编译**。**烧录顺序：先 Bootloader，再 APP。**

---

## 防变砖设计

- **双 Bank 交替升级** —— 新固件写另一个 Bank，**不碰正在运行的 Bank**
  - BankA = `880K`，BankB = `1024K`
- **计数器 EOT 提交** —— 只有收完最后一包才写计数器，半成品没有"凭证"
- **CRC 拒跳** —— 跳转前校验镜像 CRC32，坏的**不执行**
- **60 秒超时自恢复** —— 升级等待期无数据自动复位运行旧版
- **等待期抗脏数据** —— 误发的命令字节不会让 bootloader 卡死

---

## 触发升级的两种方式

**方式一：APP 运行中触发**（推荐，不需按键/复位）

串口发送 ASCII 字符串 **`UPDATE`** + 回车换行：

```
UPDATE\r\n
```

APP 依次执行：打印提示 → 等发送完成 → 写 OTA 标志 → `SCB_CleanDCache()` → 软复位。

**方式二：复位后 5 秒内长按 WKUP(PA0) ≥ 2 秒**（调试/生产备用）

之后 Bootloader 打印 `C` 请求传输，用 **Tera Term / SecureCRT 的 XMODEM-1K** 发送
`APP/MDK-ARM/N32H78x/bin/LWIP_PING.bin`。

---

## Flash 空间规划（2MB 版本）

```
0x15000000 ┌──────────────────────────────┐
           │ Bootloader (16K，镜像须<12K)   │  Flash 直接执行
0x15003000 ├──────────────────────────────┤
           │ 计数器/CRC 扇区 (4K)           │  0x15003FF8: APP CRC32
           │                               │  0x15003FFC: BankB 计数器 (u16)
0x15004000 │                               │  0x15003FFE: BankA 计数器 (u16)
           │ BankA (880K)                  │  APP 镜像槽 1
0x150E0000 │                               │
           │ BankB (1024K)                 │  APP 镜像槽 2
0x151DFFFF └──────────────────────────────┘
```

## RAM / TCM 规划

```
0x00000000 ┌──────────────────────────────┐
           │ ITCM 1MB                      │  APP 完整镜像（boot 拷入，断电即失）
0x000FFFFF └──────────────────────────────┘
0x24000000 ┌──────────────────────────────┐
           │ AXI SRAM1 128K                │  boot 向量表 (0x400) / RW + ZI
0x2401F000 ├──────────────────────────────┤
           │ UART DMA 接收缓冲 (1152 B)     │  MPU 非缓存区，见下
0x2401FF00 │ OTA 升级标志 (4 B)             │  魔数 0x4F544100 ("OTA\0")
0x2401FFFF └──────────────────────────────┘
```

**APP 镜像链接在 ITCM (`0x00000000`)**，由 Bootloader 搬运后执行，600 MHz 零等待。

---

## 目录结构

```
BOOTLOADER/
├── Bootloader/
│   ├── src/          main.c、system_n32h7xx.c（PWR/TCM）、uart.c、n32h7xx_it.c
│   ├── upgrade/      IAP.c（双 Bank 选库/跳转）、xmodem.c、crc.c
│   ├── inc/          flash.h（空间布局定义）
│   └── MDK-ARM/      Bootloader.uvprojx
├── bsp/              按键、串口控制台、延时
└── firmware/         N32H7xx 标准外设库 + CMSIS

APP/
├── src/              main.c（主循环 + 时钟/MPU 配置）、n32h7xx_it.c
├── Mcal/             硬件抽象层
├── User_App/         应用层
├── bsp/              控制台日志 (log.c)、LED
├── inc/              main.h 等公共头
├── firmware/         N32H7xx 标准外设库 + CMSIS
├── doc/              协议规格与实施交接文档
└── MDK-ARM/          LWIP_PING.uvprojx
```

### 参与编译的模块

**APP 侧**（`LWIP_PING.uvprojx`，33 个编译单元）：

| 层 | 模块 |
|---|---|
| **入口** | `main.c`、`n32h7xx_it.c` |
| **MCAL** | `Mcal_Uart`（串口链路）、`Mcal_TimerTask`（调度器 + 采样节拍）、`Mcal_InitSum`、`Mcal_SensI2C`、`Mcal_LedGpio_Init`、`Mcal_KeyGpio_Init` |
| **APP** | `App_VectorTable`（ITCM 向量表重定位）、`App_TaskServer_Handle`（任务槽）、`App_Cmd_Handle`（帧层）、`App_RcCmd_Handle`（命令层）、`App_OTA_Handle`、`App_Sensor_Handle`、`App_SensorCalib_Handle`、`App_Fusion_Handle`、`App_FusionLog_Handle` |
| **BSP** | `log.c` |
| **SDK** | `misc`、`n32h7xx_{rcc,gpio,usart,dma,dmamux,tim,spi,exti,pwr,eth,dcmu}`、`system_n32h7xx`、`startup_n32h78x_cm7.s` |

> **不在编译的模块**：视频/SD/无线链路（`App_VideoFile_Handle`、`App_VideoTx_Handle`、
> `App_R900PNR_Handle`、`App_SdCard_Handle`、`Mcal_SdSpi`、`r900pnr_spi_init`）与
> EtherCAT 预研（`Mcal_Esc_*`、`App_Cia402`、`App_Esc_Esm`）。
> 文件仍在磁盘上，但 `.uvprojx` 未引用，也不参与构建。

---

## 串口链路（`Mcal_Uart`）

USART1 是**控制台日志 + 与 Linux 主控的协议**共用的一条口。

| 项 | 值 |
|---|---|
| 引脚 | **PA9 (TX, AF7) / PA10 (RX, AF5)** |
| 波特率 | **115200 8N1** |
| 接收 | **DMA1 通道 0 + DMAMUX 通道 0 + USART 空闲中断** |
| 发送 | 轮询（等 `TXDE`，有界自旋，不会死等） |

### 接收接口：一个标志 + 一个数组 + 一个长度

驱动把一整批收到的数据放进缓冲区，记录长度，然后置标志。**应用直接读那个缓冲区** ——
没有"取数"接口，因为驱动在标志被清掉之前不会再碰缓冲区，**清标志就是消费**。

```c
/* 驱动对外只有这几个名字 */
extern volatile uint8_t  Mcal_Uart_RxFlag;   /* 1 = 有一批完整数据 */
extern volatile uint16_t Mcal_Uart_RxLen;    /* 它的长度 */
extern const uint8_t    *Mcal_Uart_RxBuf;    /* 它就在这儿，直接读 */
extern volatile uint32_t Mcal_Uart_RxDrop;   /* 被丢弃的字节数（诊断） */

/* 应用的标准用法 */
if (Mcal_Uart_RxFlag != 0u)
{
    Mcal_Uart_RxFlag = 0u;                   /* 消费掉这一批 */
    ... 读 Mcal_Uart_RxBuf[0 .. Mcal_Uart_RxLen - 1] ...
}
```

> ⚠ **`Mcal_Uart_RxFlag` 必须在任何可能 `return` 的校验之前清掉。**
> ISR 里有"上一批未消费就丢弃新数据"的保护，所以如果一批格式错误的数据让标志永远为 1，
> 驱动会**从此拒绝所有新数据**，接收路径**永久锁死**，而且标志一直显示"有数据待处理"。

### 切换到轮询模式

`APP/Mcal/inc/Mcal_Uart_Cfg.h`：

```c
#define UART_LINK_RX_USE_DMA    (1)   /* 改成 0 = 切回轮询接收 */
```

两种模式对外接口完全一致，**上层代码不需要改**。

---

## 通信协议

### 帧格式（**变长**）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0 | 2 | **包头** | `AA 55` |
| 2 | 1 | **命令** | `bit7` = ACK，`bit6` = ERR，`bit5:0` = CODE |
| 3 | 2 | **长度** | `LEN`，小端，= `DATA` 的字节数（0 .. 1024） |
| 5 | 1 | **序号** | `DATA[0]`，**请求和应答都带** |
| 6 | `LEN-1` | **数据** | `DATA[1..]` |
| `5+LEN` | 2 | **CRC16** | 小端 |
| `7+LEN` | 2 | **包尾** | `0D 0A` |

**总长 = 9 + LEN**，最大 **1033** 字节。

### CRC16

| 参数 | 值 |
|---|---|
| 类型 | **CRC-16/MODBUS** |
| 多项式 | `0x8005`（反射 `0xA001`） |
| 初值 | `0xFFFF` |
| 输入/输出反射 | 是 / 是 |
| 结果异或 | `0x0000` |
| **计算范围** | **`CMD + LEN + DATA`**（不含包头、CRC 自身、包尾） |
| 字节序 | 低字节在前 |

### 命令码

| CODE | 名称 | 状态 |
|---|---|---|
| `0x00` | 心跳 | ✅ 已实现 |
| `0x01` | 设备信息 | 待实现 |
| `0x02` | 读参数 | 待实现 |
| `0x03` | 写参数 | 待实现 |
| `0x04` | 数据上报 | 待实现 |
| `0x05` | 控制指令 | 待实现 |
| `0x06` | 固件升级 | 待实现（当前 OTA 走 ASCII `UPDATE`） |
| `0x07` | 时间同步 | 待实现 |
| `0x08` | 日志读取 | 待实现 |
| `0x09` | 恢复出厂 | 待实现 |
| 其它 | 保留 | 回 `0xC0` + 错误码 `0x03` |

### 错误码（`ERR=1` 时 `DATA` 首字节）

`0x01` CRC 错 · `0x02` 长度错 · `0x03` 不支持的命令 · `0x04` 参数错
`0x05` 设备忙 · `0x06` 权限不足 · `0x07` 执行失败 · `0x08` 超时 · `0x09` 数据格式错

### 处理流程（`app_uart_handler`）

```
每 10 ms 任务槽被调用一次
│
├─ Mcal_Uart_RxFlag == 0 ?  →  返回（没有完整的一帧）
│
├─ 包头不是 AA 55 ?
│    └─ 当作 OTA 的 ASCII 命令，逐字节喂 ota_cmd_feed()，消费，返回
│       （协议帧【不】喂给它，避免 DATA 里恰好出现 "UPDATE" 被误触发）
│
├─ 是帧 → 【先消费标志】，再做后面的校验
│
├─ 校验 LEN ≤ 1024
├─ 校验 Mcal_Uart_RxLen == 9 + LEN   （必须正好一帧）
├─ 校验包尾 == 0D 0A
├─ 校验 CRC16
│
└─ App_RcCmd_Handle(帧)   →  取命令/序号/数据，分发，组应答发送
```

**校验不过的帧静默丢弃**（不应答）—— 对端无法区分"帧损坏"和"帧没到"，
而**用更多流量去应答噪声，只会让链路更吵**。

### 已验证的帧（实测通过）

| 方向 | 字节 |
|---|---|
| 请求（心跳，无序号） | `AA 55 00 00 00 71 C0 0D 0A` |
| 应答 | `AA 55 80 00 00 70 28 0D 0A` |
| 请求（心跳，序号 `0x5A`） | `AA 55 00 01 00 5A D1 DF 0D 0A` |
| 应答 | `AA 55 80 01 00 5A F8 1F 0D 0A` |
| 请求（未知命令 `0x0A`） | `AA 55 0A 00 00 51 C2 0D 0A` |
| 应答（ERR，错误码 `0x03`） | `AA 55 C0 01 00 03 2D E5 0D 0A` |
| 请求（未知命令 + 序号 `0x01`） | `AA 55 0A 01 00 01 93 FC 0D 0A` |
| 应答 | `AA 55 C0 02 00 01 03 64 38 0D 0A` |

> ⚠ 用 `echo` 发帧会**多带一个换行**，`RxLen` 比 `Total` 大 2 → 帧被丢弃。
> 请用 `printf`，或关闭终端的行尾附加。

---

## 构建

**工具链**：Keil MDK-ARM，**ARM Compiler 5.06u5**（`uAC6 = 0`）
**调试器**：CMSIS-DAP

### ⚠ 编译后必须做的一步

**中断向量表 235 个槽位中有 226 个仍指向启动文件的弱符号（一个 `B .` 死循环）。**
`NVIC_EnableIRQ` 对链接器**不可见**，所以**漏定义中断服务函数不会报错**，
只会在第一次中断时**整机死机**（无输出、LED 停摆）。
交接文档里记录过一次真实事故：`USART1_IRQHandler` 没定义，板子直接卡死。

**每次编译后确认 `USART1_IRQHandler` 来自我们自己的目标文件：**

```powershell
Select-String -Path APP\MDK-ARM\N32H78x\Listings\LWIP_PING.map -Pattern 'USART1_IRQHandler'
```

**必须解析到 `mcal_uart.o(...)` 且代码长度非 0。**
若解析到 `startup_*.o(...)` 且长度为 **0** —— **不要烧录**。

---

## 已知注意事项

| # | 事项 |
|---|---|
| **1** | **`DMA_GetTransferredNumber()` 返回已搬运字节数**（不是剩余）。SDK 的函数名与返回值说明都如此，本板实测也是（10 字节帧 → 返回 10）。**不要用缓冲大小去减它** —— 供应商 bootloader 那样写，但那是它自己的语义，照搬会拿到满缓冲。 |
| **2** | **周期性日志与协议帧抢同一条发送路径**，而发送是**同步阻塞**的：日志行可能被插进正在发送的帧里，对端 CRC 失败。用 `APP/bsp/inc/log_cfg.h` 的 `LOG_PERIODIC_ENABLE` 关掉周期性输出（**默认已关**）。**根治办法是给发送加队列，尚未实现。** |
| **3** | 驱动**一次只保留一批数据**。上一批未被消费时，新到的一批**丢弃并计入 `Mcal_Uart_RxDrop`**。一问一答的协议不会触发；若将来要连续收帧，需改为双缓冲。 |
| **4** | 工程源码是 **DLP 加密存储**的。**Python 能透明解密读写，PowerShell 的 `Get-Content`/`Set-Content` 会读到密文并写坏文件。** 改文件请用编辑器或 Python。 |
| **5** | **不要相信 Keil 的 `[3S] build <日期> <时间>` 字符串** —— 它只在 `App_TaskServer_Handle.c` 重编译时才更新。**判断构建新鲜度看 `.o` / `.map` 的时间戳。** |
| **6** | Keil 工程文件名是 `LWIP_PING.uvprojx`（历史遗留，**LwIP 已不在编译范围内**）。 |
| **7** | 采样节拍 BTIM1 运行在 **125 Hz**（不是 2 kHz）。 |

---

## 里程碑

| 阶段 | 内容 | 状态 |
|---|---|---|
| **M0** | 地基：移出视频链路、保留传感器与姿态融合、BTIM1 采样节拍 | ✅ |
| **M1** | 串口链路：DMA + 空闲中断收发、帧解析、心跳闭环、日志走同口 | ✅ |
| **M2** | 完整协议：各业务 CMD（设备信息/读参数/写参数/控制/上报）、序号匹配、超时重发、连接管理 | 待做 |

### M1 验收结果

| # | 验收项 | 状态 |
|---|---|---|
| 1 | 上电后 Linux 侧收到 ASCII 日志 | ✅ |
| 2 | MCU 主动发心跳 | ⬜ 未实现 |
| 3 | Linux 下发心跳 → MCU 应答（字节与规格一致） | ✅ |
| 4 | Linux 下发未知 CMD → MCU 回错误应答 | ✅ |
| 5 | Linux 连续发送多帧 → 全部正确解析 | ✅ |
| 6 | 坏帧（CRC 错）后仍能正常接收下一帧 | ✅ |

---

## 相关文档

- `APP/doc/MCU_LINUX_UART_HANDOVER_2026-10-14.md` —— 协议规格、M0/M1 范围、决策记录
- `APP/doc/UART_DMA_HANDOVER_2026-10-15.md` —— DMA + 空闲中断的实施与踩坑记录
