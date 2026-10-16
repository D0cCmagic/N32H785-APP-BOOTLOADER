# R900PNR 接收端补丁 —— 拦截损坏帧 + 丢包计数

给把本 WiFi 接收链路接入 OTG 工程的同学。基于你手上那份
`R900PNR_AP_RECEIVER_HANDOVER.md` 描述的同一套代码。

| 项 | 内容 |
|---|---|
| 替换的文件 | **2 个**：`App_VideoRx_Handle.h`、`App_R900PNR_Handle.c` |
| 改动量 | +58 / −17 行，全部在 AP 分支内 |
| 你要改的调用代码 | **0 处**。接口是纯追加，你现有代码照常编译 |
| 改动性质 | 1 个行为修复 + 2 个新增计数 |
| 基线 | 2026-09-18 交接的那一版 |

---

## 1. 为什么改

### 1.1 损坏的帧之前会直接交到解码器手上（行为修复）

接收路径原来是这三行：

```c
App_RxCountLoss(vid_data, vid_len);
App_RxCheckCrc(vid_data, vid_len);          /* 只把 Rx_Bad_Crc 加一 */
App_RingWrite(&vid_data[VID_FRAME_HDR_LEN], VID_FRAME_PAYLOAD_LEN);  /* 无条件写入 */
```

`App_RxCheckCrc()` 原来返回 `void`。CRC 校验失败时它只把计数器加一，**结果被丢弃**，
损坏的 1388 字节载荷照样写进环形缓冲，由 `App_VideoRxRead()` 交给你送上位机。

于是上位机的 H.265 解码器拿到一段**已知是坏的**字节：它无从分辨，只能照着解，
画面烂到下一个关键帧为止。**丢掉它反而更好** —— 缺一块是确定的损伤，
坏字节是不确定的损伤，后者可能让解码器在错误的位置重新同步。

改后：

```c
if (App_RxCheckCrc(vid_data, vid_len) != 0u)
{
    App_RingWrite(&vid_data[VID_FRAME_HDR_LEN], VID_FRAME_PAYLOAD_LEN);
}
```

`App_RxCheckCrc()` 由 `void` 改为返回 `uint8_t`：`1` = 载荷完好、可以转发，`0` = 拒绝。

**行为差异只在 CRC 不匹配时出现。** 走到这个函数时 `len` 必然等于 1400、
magic 必然正确（前面的 `App_RxIsVideo()` 已经筛过），所以函数里那两个提前返回
实际上是死路径。链路干净时（`bad = 0`）这个改动是彻底 no-op。

### 1.2 丢包现在能从 API 读出来（新增计数）

`VideoRxStats_t` 原来只有三个字段，能看出"环形缓冲被覆盖"，
但**看不出帧是丢在空中的**，也看不出有多少帧因为 CRC 被拒。
这两项之前只出现在串口日志里，现场排查不方便 —— 现在它们在 API 里。

```c
typedef struct
{
    uint32_t TotalBytes;    /* payload bytes received and made readable */
    uint32_t ReadBytes;     /* payload bytes the consumer has taken away */
    uint32_t DroppedBytes;  /* bytes lost because the buffer was full     */
    uint32_t LostFrames;    /* 新增：编号了但从没到达的帧                    */
    uint32_t BadCrcFrames;  /* 新增：到了但被打坏、已被拒收的帧                */
} VideoRxStats_t;
```

---

## 2. 怎么替换

只需要这两个文件，**整文件替换**即可：

```
User_App/inc/App_VideoRx_Handle.h
User_App/src/App_R900PNR_Handle.c
```

### ⚠ 必须成对替换

| 只换一个 | 后果 |
|---|---|
| 只换 `.c`，不换 `.h` | `pStats->LostFrames` 找不到字段 → **编译报错** |
| 只换 `.h`，不换 `.c` | 能编过，但两个新字段**恒为 0**（静默错误，更危险） |

如果你改过这两个文件、不便整文件覆盖，用随附的 `changes.diff`。
改动集中在 5 处：

| 位置 | 改动 |
|---|---|
| `App_VideoRx_Handle.h` | `VideoRxStats_t` 追加两个字段 |
| `App_R900PNR_Handle.c` / `App_RxCheckCrc()` | `void` → `uint8_t`，返回载荷是否可用 |
| `App_R900PNR_Handle.c` / `App_RxDataHandler()` | 只有 CRC 通过才 `App_RingWrite()` |
| `App_R900PNR_Handle.c` / `App_VideoRxGetStats()` | 填充两个新字段 |
| `App_R900PNR_Handle.c` 文件作用域 | 新增 `Rx_Seq_Max`、`Rx_Bad_Total` 两个累计量 |

---

## 3. 验证

### 3.1 不回归（先做这个，最重要）

正常跑一遍，确认画面和之前一样。

如果你这条链路是干净的，串口 `[RX STAT]` 那行的 `bad` 会一直是 0 ——
那就证明这个改动的行为分支**根本没被触发过**，不可能造成回归。

### 3.2 新计数可用

在你自己的代码里周期读一次、打印增量：

```c
VideoRxStats_t st;
App_VideoRxGetStats(&st);
printf("tot=%u rd=%u drop=%u lostF=%u badCrcF=%u\r\n",
       st.TotalBytes, st.ReadBytes, st.DroppedBytes,
       st.LostFrames, st.BadCrcFrames);
```

正常链路下的期望值：

| 字段 | 期望 |
|---|---|
| `tot` | 以约 500 KB/s 增长 |
| `rd` | 紧跟 `tot`，差值等于还剩在缓冲里的字节 |
| `drop` | **0** |
| `lostF` | **长时间停在 0** |
| `badCrcF` | **长时间停在 0** |

### 3.3 可选：把"拦截损坏帧"这条路径真跑一遍

想让故障注入一下，可以让**发送端**（STA 板）把 `App_R900PNR_Handle.h` 里的
`VID_CRC_SELF_TEST` 从 `0u` 改成 `1u`。它会让每一帧的 CRC 都必然对不上。

预期对照：

| | 改前（旧代码） | 改后（本补丁） |
|---|---|---|
| `badCrcF` | 每帧都涨 | 每帧都涨（**不变**） |
| `tot` | **持续增长** | **停在 0 不动** |
| 上位机 | 收到一条从头烂到尾的流 | **一个字节都收不到** |

判据是 `badCrcF` 两边必须一样、只有 `tot` 变了。`badCrcF` 若也变了，说明改动有问题。

演练完把 `VID_CRC_SELF_TEST` 改回 `0u`。

---

## 4. 三个计数怎么读

| 现象 | 结论 | 该动哪里 |
|---|---|---|
| `DroppedBytes` 增长 | 环形缓冲被覆盖 | **消费端调 `App_VideoRxRead()` 太慢**，不是链路问题，调射频没用 |
| `LostFrames` 增长，`DroppedBytes` 不动 | 帧**丢在空中** | 射频 / 距离 / 速率 |
| `BadCrcFrames` 增长 | 帧到了但被打坏，**已被本补丁拒收** | 查 SDIO/SPI 链路质量 |
| 三个都是 0，画面仍然花 | 丢在接收端**下游** | OTG 传输，或上位机自己的缓冲 |

### ⚠ 不要用 `lostT`

串口 `[RX STAT]` 那行的 `lostT` 字段**会大幅高估丢包，最多约 16 倍**。
这是本工程已经踩过的坑，实测记录：

```
seq=16831  vidT=16832  lostT=0     ← 一帧没丢
seq=17186  vidT=17187  lostT=17    ← 一帧没丢，却记了 17 次
```

偏差最大到过 `seq=47864 vidT=47790` 差 75，而 `lostT=1190`。
**判断真实丢包请用 `seq + 1 - vidT`。**

### `LostFrames` 用的就是这个可靠算法

```c
lost = (最高 seq + 1) - 实际收到的帧数
```

- `seq` 是发送端逐帧递增的编号，从第 0 帧起
- "实际收到的帧数"计的是**所有 magic 和长度都正确的帧，包括 CRC 不合格的那些** ——
  因为它们确实到达了，只是坏了

所以 `LostFrames` 只统计**压根没来**的帧，和 `BadCrcFrames` 严格互补，不重不漏。

**重传造成的重复投递**会让"实际帧数 > 最高 seq + 1"，代码里有防下溢判断，
这时夹到 0，**不会回绕成 43 亿**。这一条已经用 8 个用例验证过（含全空、
全到、丢中间、只收到一帧、重复投递两种边界）。

---

## 5. 随附文件

```
App_VideoRx_Handle.h     改后完整文件
App_R900PNR_Handle.c     改后完整文件
changes.diff             同上两文件的 unified diff
README.md                本文件
```

有问题直接找我。
