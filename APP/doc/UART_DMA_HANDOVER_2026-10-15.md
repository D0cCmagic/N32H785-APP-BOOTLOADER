# N32H785 USART1 + DMA 交接文档

**日期**：2026-10-15
**工程**：`C:\Users\Lenovo\Desktop\H785_Test\Lwip_Ping_Test\APP`（Keil MDK，目标 `N32H78x`）
**核心文件**：`Mcal/src/Mcal_Uart.c`、`Mcal/inc/Mcal_Uart_Cfg.h`、`User_App/src/App_TaskServer_Handle.c`

---

## 一、当前状态

| 方向 | 机制 | 状态 |
|---|---|---|
| **发送** | DMA（`DMA1 CH0`，`M2P_DMA`） | ✅ **已验证工作**（`done=5`、日志稳定输出） |
| **接收** | DMA 循环模式（`DMA1 CH1`，`P2M_DMA`）+ 空闲中断分帧 | ⚠️ **DMA 搬数据正常，但空闲中断到不了 CPU** |

**唯一的阻塞点**：USART1 的中断**进不了 CPU**。

---

## 二、接收侧的确定事实（实测数据，可直接采信）

| 项 | 实测值 | 说明 |
|---|---|---|
| DMA 搬运 | ✅ **正常** | `[3S] rx moved=931`，剩余计数 `CTRL` 高位在变化 |
| 接收通道 | ✅ 运行中 | `CHEN = 0x00000002` |
| 循环模式 | ✅ 生效 | `CHxCFG` 高位 `0x80000004`（`ADR` 位 bit31 = 1） |
| USART 空闲中断使能 | ✅ 已置位 | `CTRL1 = 0x00000107`，bit8 = `IDLEIEN` |
| USART DMA 接收请求 | ✅ 已置位 | `CTRL3 = 0x00000030`，bit5 = `DMARXEN` |
| 接收器使能 | ✅ 已置位 | `CTRL1` bit1 = `RXEN` |
| DMAMUX 路由 | ✅ 正确 | `CHxCTRL[1] = 0x85`（`DMAMUX1_REQUEST_USART1_RX`） |
| **空闲中断进 CPU** | ❌ **`raw = 0`** | ISR 从未执行过（ISR 入口计数一直是 0） |
| **NVIC 使能位** | ❌ **`ISER3` 读回 0** | 中断 116（USART1）的使能位始终是 0 |

### 关键矛盾
- 外设侧：标志会置位、中断使能已开、DMA 请求已开
- NVIC 侧：`NVIC->ISER[3]` 的 bit20 读回 **0**，即使直接写 ISER 也一样
- 向量表：`USART1_IRQn = 116`，启动文件里 `USART1_IRQHandler` 位于向量表索引 132（116+16），**数值吻合**

**结论：问题在"NVIC 中断分发"环节，不在外设配置。**

---

## 三、已被排除的可能（不要重复验证）

1. **DMA 引擎本身** —— 内存到内存自检通过（`Mcal_Uart_DmaSelfTest` 返回 1）
2. **数据寄存器地址** —— `USARTx->DAT` 偏移 `0x10`，与 `USART1_BASE + 0x10 = 0x4000C410` 一致
3. **DMAMUX 请求号** —— `0x85`(RX) / `0x86`(TX) 与 SDK 宏一致，bootloader 用同值可工作
4. **接收通道配置** —— 逐字段与 `IAP_Uart/Bootloader/src/uart.c` 对照过，仅块长度不同
5. **GPIO 引脚** —— PA9(TX, AF7) / PA10(RX, AF5)，bootloader 用同样配置能收 xmodem 固件
6. **DMA 时钟 / DMAMUX 时钟** —— 代码里明确使能，寄存器可读可写
7. **MPU / Cache** —— 缓冲位于非缓存区 `0x2401F000`（`buf at 2401F000 want 2401F000` 已核对）
8. **时钟树 / 波特率** —— 阻塞发送正常、xmodem 校验通过
9. **编译器/加密文件问题** —— 已用"源码指纹"（`__DATE__`/`__TIME__` + 跨头文件常量）验证构建新鲜

---

## 四、发送侧踩过的 5 个坑（已修复，保留记录）

| # | 问题 | 现象 | 修法 |
|---|---|---|---|
| 1 | `BlkTfrSize` 通过 `DMA_ChannelInit()` 进不了寄存器 | BTS = 2（SDK 默认值），每块立即"完成" | 初始化后用 `DMA_SetChannelBlockSize()` 显式补写 |
| 2 | 握手接口同样进不了寄存器 | `DSTPER`/`SRCPER` 保持 0 | 直接写 `CHxCFG` 的 bit43-45 / bit39-41 |
| 3 | **USART1 的发送请求在握手接口 `IF_0`**（不是 `IF_1`） | 通道使能着、地址长度都对，就是不搬 | `UART_LINK_TX_HS_IF = 0`（实测出来的，手册无此映射表） |
| 4 | bootloader 遗留的 NVIC 中断使能 | 发送完成 → 中断跑飞进 `Default_Handler` | 初始化时 `NVIC_DisableIRQ(DMA1_Channel0_IRQn)` |
| 5 | 厂家库 `DMA_ChannelCmd(DISABLE)` 是**无界死等** | 取消阻塞，整机卡死 | 封装 `Mcal_Uart_ChannelDisable()`：先清 `ADR/ASR` 再禁用，等待加界 |

### 第 3 条怎么测出来的
用"接口实验"：依次把 `CHxCFG` 的 `DSTPER` 设为 0/1/2/3，每次启动一次真实传输，读通道剩余计数。结果 `if0=1(BTS 8) if1=0 if2=0 if3=0` → **USART1 的请求在 IF_0**。
（`UART_LINK_TX_HS_IF` 现为 0，代码里保留了 `Mcal_Uart_ProbeTxInterface` 的说明注释）

### 第 1、2 条的根因
SDK 的 `DMA_ChannelInit()` 里：
- `WRITE_REG(CH[ChNum].CTRL, DMA_ChInitParam->ChCtrl)` —— 结构体 `BlkTfrSize` 未进寄存器
- 握手接口只在 `TfrTypeFlowCtrl == P2M_DMA || P2P_DMA` 时才写 **目标侧**，`M2P_DMA` 时写 **源侧**，两者**错开**，所以发送通道（`M2P_DMA`）的握手配置完全没被库碰过

---

## 五、接收侧的原方案（模板给出，尚未成功）

用户提供的模板要点（**其中"循环模式"这一条是决定性的**）：

1. **DMA 必须用循环模式**（`DMA_MODE_CIRCULAR`），不能用单块模式 —— 单块模式下通道搬完一块自动停止，剩余计数冻结
   - N32 标准库里对应 `DMA_CH_TRANSFER_TYPE_MULTI_BLOCK_SRCADR_CONTIGUOUS_DSTADR_RELOAD`（SDK 为它置 `ADR` 位）
2. **IDLE 标志清除**：读 `ISR`（即 `STS`）再读 `RD`（即 `DAT`），**顺序不能颠倒，写寄存器清不掉**
3. 修改 DMA 计数器前**必须先 Disable 通道**
4. 中断内只置标志，解析放主循环

**已按此实现**，DMA 搬运立即正常（`moved=931`），**只剩中断进不来这一个问题**。

### ⚠️ 重要陷阱
`Mcal_Uart_ChannelDisable()` 会清 `ADR` 位 —— **而 `ADR` 是循环模式的开关**。接收通道的任何禁用操作**不能**走这个封装，必须用库原始调用 `DMA_ChannelCmd(DISABLE)`。

---

## 六、建议的下一步（按可能性排序）

1. **查 `NVIC->ISER[3]` 为什么写不进**
   - 断点在 `Mcal_Uart_Init()` 的 `NVIC_EnableIRQ(USART1_IRQn)` 处
   - 单步后读 `*(unsigned int *)0xE000E110`（ISER3，bit20 应为 1）
   - 读 `*(unsigned int *)0xE000ED08`（SCB->VTOR，应为 `0x00000000`）
   - 若写入无效 → 试直接写 `NVIC->ISER[3] = 0x00100000`
   - **注意**：`NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)` 在 `bsp/bsp_eth.c:394` 被调用，确认它不覆盖使能位

2. **双核外设分配**（模板"踩坑清单"第 1 条）
   - N32H785 是 **M7 + M4 双核**，部分外设可分配给任一核
   - 模板明确提示："一定要在系统配置里把 USART1、DMA 分配到你正在跑程序的内核"
   - **这条我从未验证过** —— 但 bootloader 的 DMA 接收工作正常、发送 DMA 也在工作，所以可能性中等

3. **改用 DMA 完成中断**（官方例程验证过的路径）
   - 例程 `projects/n32h7xx_EVAL/examples/USART/DMA` 用 `DMA1_Channel0_IRQHandler` + `DMA_GetChannelIntTfrStatus` 分帧
   - 它是定长环回，变长帧需要配合"超长块"或软件超时

4. **纯软件空闲判定**（无中断，一定能工作）
   - 循环模式 + 每 1ms 采样 DMA 剩余计数
   - 计数在一段时间（如 10ms）内不变 → 帧结束
   - **这个方案不依赖任何中断**，代码里已有 `TimeBase`（在 `src/n32h7xx_it.c` 的 `SysTick_Handler` 里自增，1ms 精度）

---

## 七、工程环境要点

- **Keil 工程文件被 DLP 加密**（文件头 `88 7D 1C`）：PowerShell `Get-Content` 读到密文，需要能解密的工具或直接在 Keil 里改
- **烧录方式**：Keil 编译出 `MDK-ARM/N32H78x/bin/LWIP_PING.bin` → 通过 bootloader 的 xmodem 通道写入（上电时看到 `crc32 calculating`）
- **Keil 直接下载不可用**：工程链接在 `0x00000000`（ITCM），Flash 算法只覆盖 `0x15000000`，所以 `F8` 下载必然失败；调试需设 `Utilities → Update Target before Debugging` 为不勾选
- **调试器**：CMSIS-DAP（Keil 自带）

### 关键地址速查
| 名称 | 地址 |
|---|---|
| `USART1` 基址 | `0x4000C400` |
| `USART1->STS` | `0x4000C40C` |
| `USART1->DAT` | `0x4000C410` |
| `DMA1` 基址 | `0x40046800` |
| `DMA1->CHEN` | `0x40046BA0` |
| `DMA1->CH[0].CTRL` | `0x40046818`（高32位 = 剩余计数） |
| `DMA1->CH[1].CTRL` | `0x40046858`（高32位 = 接收剩余计数） |
| `DMAMUX1->CHxCTRL[0]` | `0x40046484` |
| NVIC `ISER3` | `0xE000E110` |
| `SCB->VTOR` | `0xE000ED08` |

### 有用的参考
- **官方 USART+DMA 例程**：`N32H785/Nations.N32H7xx_Library.1.2.0/.../projects/n32h7xx_EVAL/examples/USART/DMA`
  - 注意：它用 `USARTy=USART3` 发送、`USARTz=UART9` 接收，**不是 USART1**，通道/接口号不能直接照搬
- **IAP bootloader（USART1 + DMA 接收，已验证可用）**：`.../applications/IAP_Uart/Bootloader/src/uart.c` 与 `src/n32h7xx_it.c`
- **用户手册**：`C:\Users\Lenovo\Desktop\N32H785\CN_UM_N32H7xx_Series_User_Manual_V1.2.0.pdf`
  - DMA 章节 §15.4/15.5；`CHEN` 位说明在流号 1022 附近；`CHxCFG` 字段在 968/969
  - 手册**没有**给出"握手接口号 ↔ 外设"的映射表（这是第 3 个坑只能实测的原因）
- **YIDENG_REMOTE（AT32F011，空闲中断接收参考）**：`C:\Users\Lenovo\Desktop\Yideng_Remote/.../hardward/Src/Mcal_BlueTooth_Usrart.c`

---

## 八、当前代码里需要清理的临时探针

接收成功后应删除（都标了 `BRING-UP ONLY`）：

- `Mcal_Uart_DumpRegs()`、`Mcal_Uart_PutStringBlocking()`
- `Mcal_Uart_DmaSelfTest()` + `Mcal_Uart_GetTestResult()`
- `Mcal_Uart_GetRxTransferred/GetChen/GetIsrHits/GetIsrDiag/GetTxCounters`
- `Mcal_LinkSelfReport()`（`Mcal_InitSum.c`）、`Mcal_LogPathReport()`
- `App_TaskServer_Handle.c` 里的 3 秒探针块（`[3S]` 那几行）
- `APP_LINK_ECHO_ENABLE` / `App_LinkEcho`

**保留**：`Mcal_Uart_ChannelDisable()`（发送侧必需）、初始化里的所有修复逻辑、`UART_LINK_TX_HS_IF = 0`

---

## 九、未提交的改动（非本次工作）

- `MDK-ARM/LWIP_PING.uvoptx`
- `R900PNR_CMD/hgic_cmd.c`（BOM 被去掉）
- `User_App/src/App_Fusion_Handle.c`（加了中文注释）
