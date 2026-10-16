# R900PNR 接收端（AP 角色）移植说明

给需要把本 WiFi 接收链路并入自己工程的同事。目标平台同为 **N32H785（Cortex-M7）**。

| 项 | 内容 |
|---|---|
| 你要做的事 | 把下面的文件加进你的 OTG 工程，移植 3 个平台文件，在你的主循环里调 1 个轮询函数 |
| 你要移植的代码量 | **约 330 行，3 个文件** |
| 你需要改的 API 调用 | **4 个函数** |
| 参考实现 | 本仓库 `APP/`，`APP_WIFI_ROLE` 设为 `APP_ROLE_AP` |

---

## 1. 数据流是什么样的

```
STA 板（发送端）                         你的板子（接收端）
  SD 卡 H.265 文件
      ↓ 读卡、切 1400 字节/帧
      ↓ 每帧带 12 字节头（magic/seq/crc32）
      ↓ hgic_raw_send()       ──── 无线 ────→   R900PNR 模块
                                                      ↓ 你在主循环里轮询
                                                App_R900PnrPoll()
                                                      ↓ 校验、剥掉 12 字节头
                                                32 KB 环形缓冲
                                                      ↓ App_VideoRxRead()
                                                你的 OTG 代码 → 手机
```

**你拿到的是纯 H.265 字节流**，按到达顺序、帧头已剥掉。你不需要理解 12 字节头的格式。

数据率约 **500 KB/s**（4000 Kbps）。

---

## 2. 你要移植的部分

这是你和本工程**唯一**的硬件耦合点。同款 MCU，所以大概率只是改引脚。

### 2.1 三个文件

| 文件 | 行数 | 内容 |
|---|---|---|
| `Mcal/inc/r900pnr_spi_cfg.h` | 118 | 外设 / 引脚 / 中断号 / SPI 模式，**全部宏化** |
| `Mcal/src/r900pnr_spi_init.c` | 142 | `R900PNR_SPI_Init()`：GPIO 复用、片选 GPIO、INT 的 EXTI、SPI 初始化 |
| `R900PNR_CMD/r900pnr_spi_api.c` | 188 | 6 个回调 + **内部 static 的 `R900PNR_SPI_BatchXfer()`**（真正收发字节的地方）+ EXTI 中断服务函数 |

> 最底层的 SPI 收发 `R900PNR_SPI_BatchXfer()` 是 `r900pnr_spi_api.c` 里的 **static 函数**（约 25 行），
> 所以不需要再多一个文件——它就是你要重写的那段「拿一个字节、发一个字节」。

### 2.2 你要实现的 7 个函数

声明都在 `R900PNR_CMD/r900pnr_spi_api.h`：

```c
/* 开机调一次 */
int  R900PNR_SPI_Init(void);

/* 片选：enable=1 拉低选中，enable=0 拉高释放 */
void spidrv_cs(void *priv, char enable);

/* 全双工收发。注意 wdata 和 rdata 可能是同一指针，见下方警告 */
void spidrv_write_read(void *priv, unsigned char *wdata,
                       unsigned char *rdata, unsigned int len);

/* 只发（丢弃收到的字节） */
void spidrv_write(void *priv, unsigned char *data, unsigned int len, char dma_flag);

/* 只收（发 0xFF 产生时钟） */
void spidrv_read (void *priv, unsigned char *data, unsigned int len, char dma_flag);

/* 硬件 CRC：本模块不用，直接 return 0 */
int  spidrv_hw_crc(void *priv, unsigned char *data, unsigned int len, char flag);

/* INT 引脚下降沿中断服务函数 */
void R900PNR_SPI_INT_IRQHandler(void);
```

另外需要两个全局量：

```c
volatile uint8_t g_r900pnr_spi_rx_flag;   /* 中断里置 1，主循环清除 */
volatile uint8_t g_dma_tx_done;           /* 本工程未使用，定义成 0 即可 */
volatile uint8_t g_dma_rx_done;           /* 同上 */
```

> `g_dma_tx_done` / `g_dma_rx_done` 在本工程里**只在头文件里声明过，没有任何代码读取**，
> 是原厂驱动的遗留物。定义成 0 就行。

**`priv` 参数一律忽略**，本工程传的是 0。

> ⚠ **`spidrv_write_read()` 的 `wdata` 和 `rdata` 可能是同一个指针。**
> 原厂 `hgic_sdspi_v2.c` 有两处这么用（`sd_send_command` 里 `sd_mgr.status` 既发又收，
> `sd_setup_data_message` 里 `temp` 既发又收）。**先读 `tx[i]` 再写 `rx[i]`，同一 buffer 才是安全的。**
> 这也是下面「不要用 DMA」的直接原因。

### 2.3 配置头逐项

`r900pnr_spi_cfg.h` 里你要动的：

| 组 | 宏 | 本工程取值 | 说明 |
|---|---|---|---|
| SPI 外设 | `R900PNR_SPIx` / `_CLK` / `_APBx_CLK_CMD` / `_IRQn` / `_IRQHandler` | `SPI1` 等 | 换成你空闲的 SPI |
| 三根信号线 | `..._SCK/MISO/MOSI_` `PIN/PORT/CLK/AF` | PA5/PA6/PA7，AF4/AF4/AF5 | 换引脚时注意复用号 |
| 片选 | `R900PNR_SPI_CS_PIN/PORT/CLK` | PA4，软件控制 | 普通 GPIO 输出 |
| 中断 | `R900PNR_SPI_INT_PIN/PORT/CLK` | PA8 | 模块拉低表示有数据 |
| | `R900PNR_SPI_INT_EXTI_LINE` / `_EXTI_GPIO` | `EXTI_LINE8` / `EXTI_GPIOA_Pin_8` | 跟随引脚 |
| | `R900PNR_SPI_INT_IRQn` / `_IRQHandler` | `EXTI9_5_IRQn` / `EXTI9_5_IRQHandler` | 见第 4 节冲突 |
| SPI 参数 | `R900PNR_SPI_CPOL` / `_CPHA` | **低 / 第一边沿（Mode 0）** | ⚠ **不能改**，模块要求 |
| | `DATA_SIZE` / `FIRST_BIT` / `NSS` | 8 位 / MSB / 软件 NSS | ⚠ 不能改 |
| 波特率 | `R900PNR_SPI_BAUDRATE_PRES` | `SPI_BR_PRESCALER_4` | 相对 APB2；可调 |
| 缓冲 | `RX_BUF_SIZE` / `TX_BUF_SIZE` | 2048 | ⚠ **不能小于模块单帧最大长度** |

**可以整段删掉的**：第 5 节的 DMA 宏（`R900PNR_DMAx*`、`R900PNR_DMA_TX/RX_*`、`R900PNR_SPI_DR_ADDR`）
和第 6 节的调试串口宏（`R900PNR_LOG_*`）——本工程都没有用到。

### 2.4 ⚠ 不要加 DMA

原厂驱动会给大块数据传 `DMA_TRANS`，看起来像必须上 DMA。**不是。** 本工程完全忽略这个参数：

```c
/* R900PNR_CMD/r900pnr_spi_api.c */
/* 全部使用 CPU 批量轮询, 忽略 dma_flag 参数 */
void spidrv_read(void *priv, unsigned char *data, unsigned int len, char dma_flag)
{
    (void)priv;
    (void)dma_flag;          /* ← 忽略 */
    ...
}
```

**原因**（原文在 `Mcal/src/r900pnr_spi_init.c` 顶部）：

> `hgic_sdspi_v2.c` **TX 和 RX 共用同一个缓冲区**，所以 **DMA 配合 D-Cache 会把栈变量写坏**。
> 因此改用 CPU 轮询。

也就是说：**DMA 不是「没做」，是「不能做」**，除非你先关掉 D-Cache 或改成非缓存内存。
`r900pnr_spi_api.h` 里那句 `dma_flag=1: DMA, dma_flag=0: 批量轮询` 的注释是**过时的**，与实现不符。

**这一条能省掉你大半的移植工作。**

---

## 3. 文件清单

### 3.1 直接拿去用，不要改

**原厂驱动（`R900PNR_CMD/`）8 个：**

```
hgic.h              hgic_bootdl.h       hgic_raw.h      hgic_raw.c
hgic_cmd.c          hgic_sdspi.h        hgic_sdspi_v2.c r900pnr_spi_api.h
```

> ⚠ **`hgic_sdspi_*` 名字里的 "sdio" 不是 SD 卡。** 它是「用 SPI 线跑 SDIO 协议」的协议层，
> 是模块通信的必需部分。**不要因为名字里带 SD 就删掉。**

**应用层 5 个：**

```
User_App/inc/App_VideoRx_Handle.h     ← 你唯一需要包含的头文件
User_App/inc/App_R900PNR_Handle.h
User_App/src/App_R900PNR_Handle.c
User_App/inc/App_Crc32.h
User_App/src/App_Crc32.c
```

`App_VideoRx_Handle.h` 只 `#include <stdint.h>`，不引任何 R900PNR 内部头。
`App_Crc32.c` 是纯算法，不依赖任何硬件。

### 3.2 不要拿

| 不要拿 | 原因 |
|---|---|
| `src/main.c` | 用你自己的 `main()` |
| `bsp/log.c`、`bsp/delay.c` | 用你自己的打印和延时；你只需**提供 `log_info` / `log_error`**（见 5.3） |
| `App_SdCard_Handle.*`、`App_VideoFile_Handle.*`、`Mcal_SdSpi*` | **SD 卡，接收端没有卡**。已经拆干净了，接收路径一行都不引用 |
| `App_OTA_Handle.*`、`App_VectorTable.*` | 那是本工程的 bootloader/IAP 方案 |
| lwIP、以太网、SDRAM | 与本链路无关 |

---

## 4. 集成到已有工程：符号冲突清单

合进现有工程，**头号风险是符号重复**。按严重度排：

| 我方符号 | 定义处 | 风险 | 怎么办 |
|---|---|---|---|
| **`EXTI9_5_IRQHandler`** | 由 `R900PNR_SPI_INT_IRQHandler` **宏映射**得到 | 你工程很可能已占用 EXTI9_5 | 改配置头，换到空闲的 EXTI 线 |
| **`SPI1_IRQHandler`** | 由 `R900PNR_SPIx_IRQHandler` 宏映射 | 可能已占用 | 换一个空闲的 SPI |
| `sys_now()` | `src/main.c` | 你几乎必然有自己的 ms 时基 | **只保留一个**，接收代码依赖它 |
| `USART1_IRQHandler`、`fputc()` | `bsp/log.c` | 你有自己的串口和 printf | 不要拿 log.c |
| `delay_ms()` | `App_R900PNR_Handle.c` | 名字太通用 | 真撞了就改名 |
| `main()` | `src/main.c` | 必然重复 | 用你自己的 |

**好消息**：因为 `r900pnr_spi_cfg.h` 把外设、引脚、EXTI 线、中断号**全部宏化**了，
把模块挪到你工程没占用的引脚上**只需要改这一个头文件**，驱动代码一行不用动。

---

## 5. 你要写的胶水

### 5.1 开机（一次）

```c
r900pnr_wifi_module_init();      /* SPI 初始化 + 模块 SDIO 握手（含重试）+ 稳定延时 */
App_WiFiConfigure();             /* 下发 SSID/信道/功率，并启动 AP */
```

`r900pnr_wifi_module_init()` 的返回值：

| 返回 | 含义 |
|---|---|
| `0` | 正常 |
| `-1` | SPI 初始化失败——查你的 `R900PNR_SPI_Init()` |
| `-2` | 模块 SDIO 握手失败（重试后仍失败）——查接线、供电、片选、INT 引脚 |

SDIO 握手失败最常见的原因是**模块没上电或没复位干净**，其次才是 SPI 参数不对。

> `APP_WIFI_ROLE` 必须是 `APP_ROLE_AP`（在 `App_R900PNR_Handle.h`）。
> 编错角色会得到发送端固件，收不到任何东西。

### 5.2 主循环

```c
uint8_t buf[4096];

while (1)
{
    App_R900PnrPoll();                                  /* 必须够快，见第 6 节 */

    uint32_t n = App_VideoRxRead(buf, sizeof(buf));
    if (n > 0)
    {
        /* 把 buf[0..n-1] 送你的 OTG 通道 */
    }

    /* 你其他的事 */
}
```

### 5.3 你必须提供的

**一个能用的 `printf`**，重定向到你的串口。这不是可选项：

- **原厂驱动内部大量直接调用 `printf`**（`hgic_raw.c` 里 20 多处：连接、断连、
  配对、扫描、以及 `STRONG BGRSSI` / `WRONG PASSWORD` / `TEMPERATURE OVERTOP`
  等异常事件），没有 `printf` 这些全都看不到
- 接收路径通过 `log.h` 的宏，最终也是打到 `printf`

```c
int fputc(int ch, FILE *f);      /* 重定向到你的串口（需 --microlib 或等价配置） */
uint32_t sys_now(void);          /* 毫秒时基，单调递增 */
```

`log.h` 是**纯宏头**——`log_info(...)` 就是 `printf(__VA_ARGS__)`，**不是函数**，
可以直接用我们的。它带两个开关：

| 宏 | 作用 |
|---|---|
| `LOG_LEVEL` | 默认 `LOG_DEBUG`；调到 `LOG_ERROR` 可大幅减少日志量、省中断开销 |
| `LOG_ENABLE 0` | 把 `log_info` / `log_error` 全部编译成空 —— **但 `printf` 仍然必需**，原厂驱动在用 |

`log_init()` 接收路径**不调用**，不需要实现。

---

## 6. 时序要求（最容易踩的地方）

| 调用 | 最低频率 | 违反的后果 |
|---|---|---|
| **`App_R900PnrPoll()`** | **几百 µs ~ 几 ms 一次** | 模块内部 SDIO 缓冲溢出，**帧丢在空中**，`App_VideoRxRead` 再也拿不到 |
| **`App_VideoRxRead()`** | **< 60 ms 一次** | 32 KB 环形缓冲写满并覆盖最旧数据，`DroppedBytes` 增长 |

**`App_R900PnrPoll()` 是重中之重。** OTG/USB 传输常见的写法是阻塞几十毫秒，那样一定丢帧。
建议：

- OTG 传输做成**非阻塞 / 分段**
- 或者把 `App_R900PnrPoll()` 放进一个**定时器中断**里

违例的典型现象：`[RX STAT]` 里 `int=0 rd=0 pkts=0`，但 `poll` 计数正常在涨——
说明主循环在转，只是没转到这里。

---

## 7. 对外 API 参考

```c
/* 当前可读字节数 */
uint32_t App_VideoRxAvailable(void);

/* 取走最旧的字节。返回实际取走的字节数；返回 0 表示此刻没数据，不是错误。
 * 永不阻塞、永不失败。 */
uint32_t App_VideoRxRead(uint8_t *pBuf, uint32_t MaxLen);

/* 运行计数器 */
typedef struct {
    uint32_t TotalBytes;    /* 已到达的载荷字节 */
    uint32_t ReadBytes;     /* 已被你取走的字节 */
    uint32_t DroppedBytes;  /* 因缓冲满而丢弃的字节 */
} VideoRxStats_t;
void App_VideoRxGetStats(VideoRxStats_t *pStats);
```

**三个计数的恒等式**（`DroppedBytes` 长期为 0 时）：

```
TotalBytes - ReadBytes == App_VideoRxAvailable()
```

`DroppedBytes` **持续增长**说明你调 `App_VideoRxRead()` 不够勤——**这是消费端的问题，
不是链路问题，调射频没用**。它和「帧丢在空中」是两回事，必须分开看。

---

## 8. 板上诊断

`App_RxStatsTick()` 每秒打一行：

```
[RX STAT] pkts=359 vid=357 oth=2 lost=0 bad=0 short=0 rate=3998 Kbps seq=1991916
          win=1000 ms poll=1398410 int=360 rd=360 vidT=1405 lostT=0 stall=0 aprb=2
          rtot=1950140 rread=1950140 rdrop=0 ravail=0
```

| 字段 | 含义 | 正常值 |
|---|---|---|
| `rate` | 有效视频码率 | ≈ 目标码率（4000 Kbps） |
| `vid` / `pkts` | 每秒收到的视频帧 / 总数据帧 | ≈ 356 / ≈ 358 |
| `bad` | CRC 校验失败的帧 | **0** |
| `int` / `rd` | SDIO 中断次数 / 读到数据的次数 | 与 `vid` 同量级 |
| `poll` | 轮询循环转了多少次 | 稳定在百万量级 |
| `rtot` | 累计写入环形缓冲的字节 | ~500 KB/s 增长 |
| **`rread`** | **累计被取走的字节** | **应紧跟 `rtot`** |
| **`rdrop`** | **因缓冲满丢弃的字节** | **0** |
| `ravail` | 当前缓冲内字节 | 0 ~ 32768 之间波动 |

**三个最常见的故障签名：**

| 现象 | 原因 |
|---|---|
| `int=0 rd=0 pkts=0`，但 `poll` 在涨 | 主循环没转到 `App_R900PnrPoll()`——见第 6 节 |
| `rread=0`，`rdrop` 持续增长 | 你没调 `App_VideoRxRead()`，或调得太慢 |
| `bad > 0` 持续 | 链路质量差，或对端发的东西不对 |

> **`lostT` 会大幅高估丢包（最多约 16 倍），不要引用它。** 判断真实丢包用
> `seq + 1 - vidT`。这是本工程已经踩过的坑。

---

## 9. 帧格式（调试时可能用得上）

你拿到的字节流里**已经不含这段**，但排错时可能需要认出来：

```
偏移 0    magic     uint16   0x4E56       小端
偏移 2    seq       uint32   帧序号，逐帧 +1
偏移 6    flags     uint8    bit0 = 末帧（循环播放，恒为 0）
偏移 7    reserved  uint8
偏移 8    crc32     uint32   载荷的 CRC32
偏移 12   payload   1388 字节
```

整帧 **1400 字节**，`App_VideoRxRead()` 交给你的是剥掉前 12 字节之后的 1388 字节载荷。
`crc32` 由发送端填、接收端校验，不匹配会计入 `bad=`。

**小端字节序**（Cortex-M 天然如此）。

---

## 10. 资源占用（参考）

| 项 | 数值 | 说明 |
|---|---|---|
| Flash | 约 20 KB 量级 | 大头是原厂驱动 |
| **RAM** | **32 KB（环形缓冲）** + 约 14 KB（栈 8 KB + 其他） | 环形缓冲是唯一的大块 |

如果 RAM 紧张，**环形缓冲大小是唯一的调节旋钮**——但要记住：**调小它，第 6 节那个
60 ms 的约束会同步收紧**（60 ms ≈ 缓冲大小 ÷ 500 KB/s）。

---

## 附：本工程里可对照的调用点

- 接收处理与环形缓冲：`User_App/src/App_R900PNR_Handle.c`，搜 `App_RingWrite` / `App_VideoRxRead`
- 初始化与主循环：`APP/src/main.c`
- 平台移植样板：`Mcal/src/r900pnr_spi_init.c`、`R900PNR_CMD/r900pnr_spi_api.c`
