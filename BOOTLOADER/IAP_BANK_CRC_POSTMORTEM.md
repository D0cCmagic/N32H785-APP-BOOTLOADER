# IAP 升级后 APP CRC 校验失败 —— 故障报告

| 项 | 内容 |
|---|---|
| 日期 | 2026-09-15 |
| 严重度 | 高 —— 板子变砖,升级通道完全不可用 |
| 影响范围 | 所有 **从未升级过** 的板子(全新板、只烧过 bootloader + 首次 IAP 的板) |
| 修复提交 | `3a935a5` fix: stop the IAP bootloader from validating the wrong bank |
| 涉及文件 | `BOOTLOADER/Bootloader/upgrade/IAP.c`(唯一逻辑改动) |

---

## 1. 结论先行

**根因:boot 计数器/CRC 扇区(`0x15003000`)在全新板子上从未被初始化,里面残留着一段"很久以前烧在 flash 基址 `0x15000000` 的旧 APP 镜像"的 ASCII 字节。IAP 的 EOT 阶段对整扇区做读-改-写,把这些字节当成 Bank 计数器继承了下来。**

于是两个计数器是 `0x4150` 和 `0x204C`(不是计数器,是两个 ASCII 字符片段)。而 `check_bank()` 的**升级分支和启动分支是镜像逻辑**,只在两计数器相差 ≤1 时才自洽。`0x4150 > 0x204C` 在这个错位状态下导致:

- **升级路径** 选中 **BankB** → APP 被写进 BankB
- **启动路径** 选中 **BankA** → bootloader 去校验 BankA 里那段陈旧的镜像残片

两条路径**永久分歧**。APP 本身完全正常(传完立刻就能跑),但复位后 bootloader 永远在 CRC 校验一块根本不该看的区域。

**与 APP 代码零关系。** APP 一行没改。

---

## 2. 现象

烧完 bootloader,走一次 IAP 传输,XMODEM 全程 ACK、`Firmware updated!`,并且**当场跳进 APP 跑得好好的**。但一复位就变成死循环:

```
************************ N32h7xx uart IAP ************************
 5 --- 4 --- 3 --- 2 --- 1 ---
 app start_addr: 0x15004000, app end_addr: 0x150dffff

Wait a minute, crc32 calculating ......
Error, APP CRC check err, app start_addr: 0x15004000, app end_addr: 0x150dffff
Wait a minute, crc32 calculating ......
Error, APP CRC check err, app start_addr: 0x15004000, app end_addr: 0x150dffff
...
```

**关键矛盾点:** 传完之后**能跑**,复位之后**不能跑**。说明 APP 镜像本身是好的、CRC 也是好的 —— 问题出在"bootloader 复位后去看的那块区域"和"APP 实际所在的区域"不是同一块。

---

## 3. 背景设计(理解根因的前提)

### 3.1 Flash 布局(2MB 版本)

```
0x15000000 ┌──────────────────────────────┐
           │ Bootloader (16K)              │  须 <12K
0x15003000 ├──────────────────────────────┤
           │ 计数器 / CRC 扇区 (4K)         │  ← BOOT_LSAST_SECTOR
           │   0x15003FF8  APP CRC32       │
           │   0x15003FFC  (bankA<<16)|bankB │
0x15004000 ├──────────────────────────────┤
           │ BankA (880K)                  │  APP 镜像槽 1
0x150E0000 ├──────────────────────────────┤
           │ BankB (1024K)                 │  APP 镜像槽 2
0x151DFFFF └──────────────────────────────┘
```

### 3.2 两个计数器的物理位置

`flash.h` 里的定义:

| 宏 | 地址 | 含义 |
|---|---|---|
| `BANKA_UPDATE_CNT_ADDR` | `0x15003FFE` | BankA 升级次数,`uint16` |
| `BANKB_UPDATE_CNT_ADDR` | `0x15003FFC` | BankB 升级次数,`uint16` |

两者拼成 `0x15003FFC` 处的**一个 32 位字**:高 16 位是 BankA,低 16 位是 BankB。EOT 阶段写入方式:

```c
flash_page_backup[FLASH_SECTOR_SIZE/4-1] = (bankA_cnt << 16) | bankB_cnt;
```

### 3.3 `check_bank()` 的镜像逻辑

```c
flash_bank check_bank(uint8_t bank_select)
{
    flash_bank bank_use = FLASH_BANKA;

    if(bank_select >= 4)   /* 升级路径:选"小"的那个 */
    {
        if      (a == b)                 bank_use = FLASH_BANKA;
        else if (a != 0xFFFF && b == 0xFFFF) bank_use = FLASH_BANKB;
        else if (a > b)                  bank_use = FLASH_BANKB;
    }
    else                   /* 启动路径:选"大"的那个 */
    {
        if      (a == b)                 bank_use = FLASH_BANKB;
        else if (a != 0xFFFF && b == 0xFFFF) bank_use = FLASH_BANKA;
        else if (a > b)                  bank_use = FLASH_BANKA;
    }
    return bank_use;
}
```

设计意图:**升级时写"当前没在跑"的那个 bank,启动时读"最后一次写"的那个 bank**。

这个设计**只在两个计数器相差 0 或 1 时才成立** —— 因为每次升级只给其中一个计数器 +1。合法状态全集:

| 状态 | 相差 | 升级选 | 启动选 | 自洽 |
|---|---|---|---|---|
| `a == b` | 0 | BankA | BankB | ✅ |
| `a == b + 1` | 1 | BankB | BankA | ✅ |
| **其他任意值** | **≥2 或回绕** | **?** | **?** | ❌ **分歧** |

**`check_bank()` 本身没有任何合法性检查。** 它是这套方案里最脆弱的一环,而后面的故障正好命中这一点。

### 3.4 EOT 阶段对整扇区做读-改-写

`xmodem.c` 的 `X_EOT` 分支:

```c
/* 1. 先把整个 4K 扇区原样读进内存 */
for(int i=0; i<FLASH_SECTOR_SIZE/4; i++)
    flash_page_backup[i] = *((uint32_t *)(BOOT_LSAST_SECTOR + i*4));

/* 2. 只改两个字段:计数器 + APP CRC */
flash_page_backup[1023] = (bankA_cnt<<16) | bankB_cnt;
flash_page_backup[1022] = CRC32(...);

/* 3. 擦掉整扇区,再把 4K 缓冲区整体写回 */
SMU_EraseFlash(BOOT_LSAST_SECTOR);
flash_write(BOOT_LSAST_SECTOR, &flash_page_backup[0], FLASH_SECTOR_SIZE);
```

**这是根因成立的物理前提:** 扇区里除这两个字段以外的字节,**既没被清零、也没被校验,而是原封不动地被继承下去**。如果这片区域一开始是干净的(全 `0xFF`),什么问题都不会有。

---

## 4. 排查过程

### 步骤 1 —— 先分清"传输坏了"还是"校验坏了"

`Firmware updated!` + 传输完当场能跑 ⇒ 镜像写入了、也正确 ⇒ **问题不在传输,在校验**。这一刀把范围砍掉一半。

### 步骤 2 —— 排除"存和取不是同一个地址"

CRC 存入点与读取点必须一致,否则永远对不上。逐字核对:

- 存:`flash_page_backup[1022]` → 扇区内偏移 `1022*4 = 4088 = 0xFF8` → `0x15003000 + 0xFF8 = 0x15003FF8`
- 取:`*((uint32_t *)(BOOT_END_ADDR + 1 - 8))` = `0x15003FFF + 1 - 8` = `0x15003FF8`

**一致。排除。**

### 步骤 3 —— 排除"擦除粒度把 bootloader 自己擦了"

若 `SMU_EraseFlash` 实际按 256 字节擦,`0x15003000` 会连带把 bootloader 尾部一起擦掉。实测 bootloader 镜像 10456 字节(`0x28D8`),结束于约 `0x150028D7`,**不含** `0x15003000`。且厂商文档明确 4KB 粒度。**排除。**

### 步骤 4 —— 排除"APP 自己在写 flash"

APP 若也持有 flash 写函数,可能在跑起来之后覆盖了 trailer。在 APP 的 map 文件里搜 `n32h7xx_smu`,**命中 0 次**;也没有直接使用裸 ROM 入口 `0x1fff7b81` / `0x1fff7c81` / `ER_FLASH` / `WR_FLASH`。**排除。**

### 步骤 5 —— 加仪表,把 bootloader 眼里的世界打出来

以上都是"排除法",到此必须**直接观察**。在三个关键点插入打印:

- `main.c` 开机时,直接按地址读 trailer 和 BankA 头部
- `IAP.c` 的 `check_app_crc()` 里,打印算出来的和读出来的 CRC
- `xmodem.c` 的 EOT 里,打印写进去的 CRC 和写完后回读的值

读回结果(**这是整个排查的转折点**):

```
DBG last=13d2a57d cnt=4150204c app0=4f575353 app4=0a0d4452
```

### 步骤 6 —— 逐字节解码那 4 个十六进制数

打印出来的是 32 位字,**按小端展开成字节,再查 ASCII**:

| 打印值 | 地址 | 小端字节序列 | ASCII |
|---|---|---|---|
| `app0=4f575353` | `0x15004000` | `53 53 57 4F` | `S S W O` |
| `app4=0a0d4452` | `0x15004004` | `52 44 0D 0A` | `R D ␍ ␊` |
| `cnt=4150204c` | `0x15003FFC` | `4C 20 50 41` | `L ␠ P A` |

拼起来,从 `0x15003FFC` 往后的原始字节是:

```
0x15003FFC:  4C 20 50 41 53 53 57 4F 52 44 0D 0A
             L␠ P  A  S  S  W  O  R  D  ␍  ␊
```

**是一段可读的 ASCII 字符串:`"L PASSWORD\r\n"`。**

**flash 里不该有字符串。** 这条线索基本就锁定答案了。

### 步骤 7 —— 反推这段字符串从哪来

APP 的 `LWIP_PING.bin` 里 hexdump 偏移 `0x3FF0`:

```
00003ff0: 493a 2564 0d0a 0000 5752 4f4e 4720 5041  I:%d....WRONG PA
00004000: 5353 574f 5244 0d0a 0000 0000 5445 4d50  SSWORD......TEMP
```

即 `"WRONG PASSWORD\r\n"` 位于镜像偏移 `0x3FF8` 附近。而板子返回的 10 个字节在 flash `0x15003FFE`,内容是 `"PASSWORD\r\n"` —— **与镜像偏移 `0x3FFE` 起的 10 个字节逐字节相同**。

```
0x15003FFE - 0x3FFE = 0x15000000   ← 镜像基址
```

**结论:BankA 里躺着的,是一份很久以前直接烧在 flash 基址 `0x15000000` 的旧 APP 镜像**(那个年代还没有 bootloader)。这段字符串来自 APP 里 `hgic_raw.c` 的 `printf("WRONG PASSWORD\r\n")`(泰芯原厂驱动,未修改)。

**bootloader 镜像只有 10456 字节,从未覆盖过 `0x15003000` 之后 —— 于是这段陈旧字节在 trailer 扇区里一直存活到现在。**

### 步骤 8 —— 把"残片"翻译成"计数器",复现分歧

trailer 扇区的布局与那段陈旧 ASCII 恰好重叠:

| 地址 | 本应存放 | 实际残留 |
|---|---|---|
| `0x15003FF8` | APP CRC32 | 被 EOT 覆盖(本案例 `0x13D2A57D`) |
| `0x15003FFC` | **BankB 计数器** | 被 EOT 覆盖,值为 `0x204C` |
| `0x15003FFE` | **BankA 计数器** | **未被覆盖** = ASCII `"PA"` = `0x4150` |

EOT 老老实实地把 `0x15003FFE` 上的 `'P','A'` 读成了 BankA 计数器 `0x4150`,把 `0x15003FFC` 上那两字节读成 BankB 计数器并 +1,然后写回去。

代入 §3.3 的真值表,`a = 0x4150`,`b = 0x204C`,`a > b`:

| 路径 | `bank_select` | 第 3 分支 `a > b` | 选定 |
|---|---|---|---|
| 升级 | `≥ 4` | 真 | **BankB** |
| 启动 | `< 4` | 真 | **BankA** |

**永久分歧,且每次复位都复现。**

### 步骤 9 —— 在硬件上验证分歧

长按 WKUP 进入升级模式,打印:

```
app start_addr: 0x150e0000   ← BankB(升级路径)
```

普通复位,打印:

```
app start_addr: 0x15004000   ← BankA(启动路径)
```

**同一块板、同一个 trailer,两条路径选了不同的 bank —— 分歧假设在硬件上被直接证实。** 而且 `0x150e0000` 恰好证明 APP 真的被写进了 BankB(所以传完能跑),bootloader 却去校验 BankA 的陈旧残片(所以复位报 CRC 错)。

---

## 5. 修复

唯一改动在 `check_update()` 内、`check_bank()` 调用之前:

```c
#ifdef BANK_AB_UPDATA

    /* Each update raises exactly one counter, so a pair that came from a real
       history always differs by 0 or 1. Any other gap means the trailer was
       never initialized and holds arbitrary bytes, which sends the update path
       and the boot path to different banks. Restart the pair from the erased
       state before choosing the bank. */
    if (update_flag >= 4)
    {
        uint16_t Bank_A_Cnt = *((uint16_t *)BANKA_UPDATE_CNT_ADDR);
        uint16_t Bank_B_Cnt = *((uint16_t *)BANKB_UPDATE_CNT_ADDR);
        uint16_t Count_Diff = (uint16_t)(Bank_A_Cnt - Bank_B_Cnt);

        if (Count_Diff > 1u)
        {
            printf("\r\n Bank counters invalid (%04x/%04x), reset the trailer\r\n",
                   Bank_A_Cnt, Bank_B_Cnt);
            if (FLASH_SUCCESS != SMU_EraseFlash(BOOT_LSAST_SECTOR))
            {
                printf("\r\nERASE FAILED\r\n");
            }
        }
    }

    flash_bank bank_use = check_bank(update_flag);
```

### 5.1 为什么判据是 `Count_Diff > 1u`

由 §3.3,合法历史只能产生 `a - b ∈ {0, 1}`。**任何大于 1 的间隔(含回绕)都等价于"这对计数器对不上一段真实历史"** —— 而从 §3.3 的真值表可知,这种状态下两个分支本来就在选不同 bank,板子**已经是坏的**。

所以这个擦除**不可能破坏一块本来能正常启动的板子**。这是本次修复最关键的安全论证。

### 5.2 为什么要用 `update_flag >= 4` 门控

擦除会连 **APP CRC** 一起清掉。如果无条件擦:

> 一块"计数器是残渣、但 BankA 里刚好躺着能跑的好 APP"的板子,复位一次就被我搞成 CRC 错。

所以只在**用户主动升级时**擦。普通复位(`update_flag == 0`)永远不碰 trailer。

### 5.3 修复后的状态迁移

擦除后两个计数器都是 `0xFFFF`(差 0),进入合法状态。以本板为例:

| 步骤 | BankA | BankB | 差 | 升级选 | 启动选 | APP 实际在 |
|---|---|---|---|---|---|---|
| 修复前(残渣) | `0x4150` | `0x204C` | — | BankB | BankA | **分歧** |
| 擦除后 | `0xFFFF` | `0xFFFF` | 0 | — | — | — |
| 第一次 IAP 后 | `0x0000` | `0xFFFF` | 1 | BankB | **BankA** | BankA ✅ |
| 第二次 IAP 后 | `0x0000` | `0x0000` | 0 | BankA | **BankB** | BankB ✅ |
| 第三次 IAP 后 | `0x0001` | `0x0000` | 1 | BankB | **BankA** | BankA ✅ |
| … | | | | | | 此后永久交替 ✅ |

**注意第一轮是特殊的**:擦除后 `b = 0xFFFF` 表示 BankB 从未写过,`check_bank` 的第 2 分支(`a != 0xFFFF && b == 0xFFFF`)专门处理这种"有一边是空的"情况,语义是"B 空着 → 写 B / 跑 A"。之后差值就只在 `{0, 1}` 之间循环了。

---

## 6. 硬件验证

保留了三处临时仪表,重烧 bootloader 后走一次完整 IAP + 复位。

**复位后的决定性打印:**

```
************************ N32h7xx uart IAP ************************
DBG last=5df24259 cnt=0000ffff app0=240082d0 app4=00000401
 5 --- 4 --- 3 --- 2 --- 1 ---
 app start_addr: 0x15004000, app end_addr: 0x150dffff

Wait a minute, crc32 calculating ......
DBG calc=5df24259 stored=5df24259 region=15004000-150dffff
SdCard Init fali ,ret = -2
[ROLE] STA sender (R900PNR rate test)
[R900PNR] SPI init success
[R900PNR] SDIO init success
[CFG] set_mode(sta) = 15 (try 1)
...
CMD:54, RESP:OK[0]
```

逐字段判读:

| 字段 | 值 | 判据 |
|---|---|---|
| `cnt` | `0000ffff` | 计数器合法了(BankA=0x0000 / BankB=0xFFFF,差 1)✅ |
| `app0` | `240082d0` | BankA 开头是合法 MSP(AXI SRAM 量级)✅ |
| `app4` | `00000401` | ITCM 复位向量,奇地址 ✅ |
| `calc` | `5df24259` | 本次算出的 CRC |
| `stored` | `5df24259` | trailer 里的 CRC —— **相等,校验通过** ✅ |
| `region` | `15004000-150dffff` | 校验区域是 BankA,且 APP 确实在 BankA ✅ |

随后 `[ROLE] STA sender` → `SDIO init success` → 一串 `RESP:OK` 证明 **APP 已正常运行**。`SdCard Init fali ,ret = -2` 是预期的(SD 卡未插)。

**验证通过后,三处临时 `DBG` 打印已全部移除**,仅保留修复本身的 `Bank counters invalid` 提示。

---

## 7. 排查方法论小结

| 做法 | 价值 |
|---|---|
| **先二分"传输 vs 校验"** | "传完能跑、复位不能跑"一句话把范围砍半 |
| **打印原始十六进制,而不是打印"结论"** | 全部打印的都是裸 `%08x`,没有一处是"我以为的语义"。`app0=4f575353` 这串数字本身没意义,**小端展开成 ASCII 才有意义** |
| **把 flash 当作可以被反推的证据** | 通过字符串逐字节比对反推出镜像基址 `0x15000000`,直接把"这段字节是谁留下的"变成事实而非猜测 |
| **用改造过的仪表覆盖时间轴两端** | EOT 写入点 + boot 读取点**同时**打点,才看得出"写进去的和读出来的其实是同一个值,只是读写区域错了" |
| **硬件上直接验证分歧假设** | 长按 WKUP 打出 `0x150e0000`,普通复位打出 `0x15004000` —— 一次对照实验直接证死 |

**最大的教训:"能跑"和"能启动"是两件独立的事。** 本次故障全程,APP 镜像一直是好的、CRC 也一直是正确的 —— 坏的只是 bootloader 判断"该校验哪里"的那一步。

---

## 8. 遗留风险

1. **`check_bank()` 自身仍无合法性检查。** 本次修复是在**调用点上游**加了一次性自愈,而不是让 `check_bank()` 变健壮。任何绕过 `check_update()` 的调用路径(目前没有)都不会受保护。属于"够用但不彻底",已评估,暂不动。

2. **其他全新板子仍会遇到同样的残渣条件。** 只要是"曾经在 `0x15000000` 烧过 APP、之后才引入 bootloader"的板子,trailer 里就有陈旧字节。**修复后会在第一次 WKUP 升级时自动完成自愈**,无需人工干预 —— 但第一次升级时应当能看到 `Bank counters invalid (xxxx/xxxx), reset the trailer` 这行提示,看到即正常。

3. **`BankA/BankB 容量不对称`(880K vs 1024K)是既有设计约束**,与本次故障无关,但升级 BankA 时 APP 体积上限更小,勿混淆。

---

## 9. 相关文件

| 文件 | 作用 |
|---|---|
| `BOOTLOADER/Bootloader/upgrade/IAP.c` | `check_bank()`、`check_update()`、`check_app_crc()`、`flash_jump_to_app()` —— 本次唯一逻辑改动处 |
| `BOOTLOADER/Bootloader/upgrade/xmodem.c` | `X_EOT` 分支:计数器递增 + CRC 计算 + trailer 读-改-写 |
| `BOOTLOADER/Bootloader/inc/flash.h` | Flash 布局、`BOOT_LSAST_SECTOR`、两个计数器地址 |
| `BOOTLOADER/Bootloader/src/main.c` | 复位后的 CRC 校验死循环 |
| `README.md` | Flash 空间规划、三级防变砖设计说明 |
