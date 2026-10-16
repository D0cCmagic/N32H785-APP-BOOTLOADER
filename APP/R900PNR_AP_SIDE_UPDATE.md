# R900PNR AP 端 —— 本轮变更与接口

**日期** 2026-09-28
**对应发送端** 15 fps / GOP 5 / 约 2 Mbps
**配套** 完整移植说明见同目录 `R900PNR_AP_RECEIVER_HANDOVER.md`（9-23 版）。
本文只讲**这一轮的改动**，以及那份说明里已经**过期**的三处。两者冲突时以本文为准。

---

## 1. 必须改的只有一个宏

`App_R900PNR_Handle.h`：

```c
#define APP_WIFI_BSS_BW     (4u)      /* 原来是 8u */
```

**发送端和接收端必须是同一个值。** 发送端现在是 4。两端带宽不一致时，STA
**连信标都收不到**，现场表现是：

```
[TX STAT] ... conn=0 rssi=0 tbr=0 ... air=0 prb=0
CMD:53, RESP:OK[0]          <- 扫描结果为空
Disconnected!               <- 每秒一次
```

注意这不是"命令被拒"——`CMD:34`（SET_BSS_BW）会正常回 `OK`。模块接受了这个设置，
只是对面不在同一个带宽上。

这一处对两个角色都生效：频率范围 / 带宽 / 信道表三行代码在角色分支**之外**，
AP 走的是同一段，改完重新编译烧录即可。

### 不要顺手改信道表

```c
#define APP_WIFI_CHAN_LIST    {9080u, 9160u, 9240u}
```

调用处把首项、末项和条数**写死**了：

```c
ret = hgic_raw_set_freq_range(chan_list[0], chan_list[2], APP_WIFI_BSS_BW);
ret = hgic_raw_set_chan_list(chan_list, 3);
```

**加减信道不会让频率范围跟着你的表走**，只会让范围缩小或不变。保持三项。

---

## 2. 你要调的 API

三个函数，都在 `App_VideoRx_Handle.h`：

```c
uint32_t App_VideoRxAvailable(void);
uint32_t App_VideoRxRead(uint8_t *pBuf, uint32_t MaxLen);
void     App_VideoRxGetStats(VideoRxStats_t *pStats);
```

- `App_VideoRxRead()` 返回实际取走的字节数。**返回 0 表示此刻没数据，不是错误**；
  永不阻塞、永不失败，不给你回压的机会。
- 取出来的字节**已经剥掉 12 字节帧头**，就是发送端卡上那份 H.265 裸流，
  可以直接送解码 / 送 OTG。

两个时序约束，一个是你的、一个不完全是：

| 谁 | 周期 | 超了会怎样 |
|---|---|---|
| `App_R900PnrPoll()` | 几百 µs ~ 几 ms | 模块 SDIO 缓冲溢出，**空中的帧直接丢**（`int=0 rd=0`） |
| `App_VideoRxRead()` | **< 130 ms** | 32 KB 环形缓冲覆盖最旧数据，`DroppedBytes` 增长 |

第二条要跟着你的 USB/OTG 传输节奏走。因为 `App_VideoRxRead()` 不阻塞，把它放在
**每次搬完一块就调一次**的位置，不要挂在"每秒一次"的统计里。

---

## 3. 相对 9-23 那版说明，三处已经过期

| 位置 | 旧 | 现 |
|---|---|---|
| §2.2 / §7 / §8 / §10 的数据率 | 500 KB/s (4000 Kbps) | **250 KB/s (2000 Kbps)** |
| §6 / §2.2 的环形缓冲约束 | < 60 ms | **< 130 ms** |
| §7 的 `VideoRxStats_t` | 3 个字段 | **5 个字段**，多了 `LostFrames` / `BadCrcFrames` |

`< 130 ms` 和旧的 `< 60 ms` 是同一个式子（缓冲 ÷ 速率），只是速率减半：
`32768 ÷ 250 KB/s ≈ 130 ms`。缓冲大小没变，**码率降了一半，所以你手上的时间宽裕了一倍**。

`VideoRxStats_t` 现值：

```c
typedef struct {
    uint32_t TotalBytes;    /* 已到达并写入环形的载荷字节 */
    uint32_t ReadBytes;     /* 已被你取走的字节 */
    uint32_t DroppedBytes;  /* 因缓冲满被覆盖的字节 */
    uint32_t LostFrames;    /* 编号帧从未到达 */
    uint32_t BadCrcFrames;  /* 到达了但 CRC 不符，被拒收 */
} VideoRxStats_t;
```

后两个是新增的，**你手上那块 AP 板的固件里还没有**——它早于这次改动，
会把损坏帧**照收不误**地交给你。两者的区别很重要：

- `LostFrames` —— 帧根本没到。
- `BadCrcFrames` —— 帧到了但内容坏了，被拒收。

两者都在交给你的字节流里留下同样大小的洞，而**一个洞的代价约等于一个 GOP 的画面**。
所以这两个数要连着 `TotalBytes` 一起看，不要只看"有没有数据在流"。

计数恒等式（`DroppedBytes` 不再增长时成立）：

```
TotalBytes - ReadBytes == App_VideoRxAvailable()
```

对不上说明有字节被覆盖掉了，先解决取数节奏，不要怀疑链路。

---

## 4. 怎么算调对了

**发送端** `[TX STAT]` 一分钟内应该稳定成：

```
[TX STAT] pkts=... rate=2000 Kbps fail=0 seq=... conn=1 rssi=<非0> air=... prb=2
```

- `conn=1` —— 关联成功。
- **`prb` 从 0 变成 2** —— 收到你的 AP 广播探针（每 500 ms 一个）。
  这是链路通的最早信号，比 `conn` 更早出现，**先看它**。

**接收端** `[RX STAT]` 每行比上一行看两个增量：

- `Δseq` 应约等于本窗口的 `vid`。**不要看 `lostT` 那个字段——它最多高估 16 倍。**
- `stall=0`，且 `rdrop` 不再增长（说明你在取数）。

以上都对了，说明链路和数据通路都通了。有任何一项不对，**发一行 `[RX STAT]` 过来**，
对着上面两项核一遍就能定位是链路丢包还是消费端取数太慢。
