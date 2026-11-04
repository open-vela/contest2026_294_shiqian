# 声音反馈：ES8388 喇叭（已打通）与蜂鸣器音调（已实现）

## 结论先行

| 方案 | 状态 | 结论 |
|---|---|---|
| **ES8388 + SAI1 + GPDMA1** 播放 WAV / 单音 | ✅ **已上板** | 15 s / 2.6 MB WAV 实测 **99.86%～100.06%** 时长；块边界饿死 6～7 / 162；播放结束**无嘶声、无爆音** |
| 蜂鸣器音调反馈 | ✅ 已上板 | PD3 无源蜂鸣器 + TIM3 中断翻转 |

**收尾固件**：`nuttx.bin` = 995,896 B，`_ebss = 0x341e1188`，`sha256[:16] = 33a10b2813c7c06e`

## 一、硬件与时钟

| 项 | 值 |
|---|---|
| 信号 | `PB0=FS_A` / `PB2=SD_A` / `PC2=SCK_A` / `PE2=MCLK_A`（SAI1，AF6） |
| 音频时钟 | HSE → PLL2（M6/N192/PDIV1=4）→ VCO 1536 MHz → IC7 /17 = 22.588 MHz |
| 内核时钟选择 | `RCC_CCIPR7.SAI1SEL = 2`（ic7_ck），**不写这一位整条链路白建** |
| MCLK / 采样率 | `MCKDIV=2` → MCLK 11.294 MHz，Fs **44.117 kHz**（N6 的 MCKDIV 没有 +1） |
| codec 控制 | **硬件 I2C4**，`PE13=SCL` / `PE14=SDA`（AF4），地址 `0x10` |
| 功放 | 外置 `U3 = MD8002A`；`SHUTDOWN`/`CE` 都是电容到地硬使能，**无软件控制脚** |

## 二、打通过程中修掉的 8 个真 bug

| # | 现象 | 根因 | 判据 |
|---|---|---|---|
| 1 | 完全无声 | `RCC_CCIPR7.SAI1SEL` 从未写过，内核时钟停在 pclk2 | `divenr.IC7EN=1` 但一个帧都不出 |
| 2 | codec 配不上 | ES8388 必须在 SAI 启动**前**配置，SAI 一跑就永久 NACK | `ab ES8388 -> ee (ret -6)` |
| 3 | I2C 假成功 | `stm32n6_i2c_transfer` 的 `errout` 返回粘滞 `priv->error`，NACK 被当成功 | 预置 `0xee` 读回还是 `0xee` |
| 4 | 节流正确但无声 | GPDMA 通道缺安全属性 → RIF 丢弃对安全外设 SAI1 的写，**请求线照常节流** | `ctr1` 回读 `0002000a` → 补 `SSEC/DSEC` 后 `8002800a` |
| 5 | 只有第一块等待 | 清标志写 `CSR`（只读），必须写 `CFCR` | 15 s 音频 1.1 s 放完 |
| 6 | 速率恰好 2 倍慢 | 半字宽，SAI 的 DMA 请求按帧发；必须**字宽**（一 beat 一帧） | 800 ms → 1600 ms |
| 7 | DMA 按总线速度狂灌 | 缺 `CTR2.PFREQ`（外设流控） | `cbr1` 几微秒归零而 FIFO 仍满 |
| 8 | WAV 密集"鼓点" | `dma_wait` 忙轮询 93 ms/块占满 CPU，读卡线程被饿死 → 吞吐 159 KB/s < 需求 176.4 | `loop spent 1200 ms waiting for the reader` |

## 三、三个关键设计

**1. 槽位环 + 读卡线程**：8 × 16 KB 环形缓冲，`wavrd` 线程用 `open/read` 预读；播放前先垫满整个环（745 ms），避免起始爆读与播放抢 CPU。

**2. 协作式让步**（`#8` 的修法）：`dma_wait` 每 32 次轮询 `sched_yield()`、**剩余字节 ≤256 时改为紧轮询**；`sdmmc_read_block` 等卡循环每 64 次让步；读卡线程优先级 **等于** 播放线程（`sched_yield` 只切同/高优先级，这是前提）。
效果：读卡吞吐 159 → **316 KB/s**，等待 reader 的时间降到 **0**。

**3. `quiet()` + 静音保活线程**：
- `stm32n6_sai_output_quiet()`：释放 DMA、清 `DMAEN`、`CR2.FFLUSH`、塞 8 个零字，**保留 `SAIEN`** —— codec 一直有时钟，不会进"无时钟空闲态"（那才是嘶声的来源）
- 只留 8 个零不够（FIFO 181 µs 后饥饿，饥饿态输出的是垃圾帧 → **细碎爆音**），所以加低优先级线程 `saiidle` 持续喂 8 KB 零块（46 ms 一轮），并用一把锁与播放线程互斥
- 播放结束**先塞一块零再交接**，保证 FIFO 不断供

## 四、已知边界（如实记录）

| 项 | 现状 | 原因 |
|---|---|---|
| codec 静音 | 写不进去 | SAI 跑过之后 ES8388 永久 NACK I2C，`0x04=0x00` 无法写入；因此改用"持续喂零"代替静音 |
| 空闲 CPU 占用 | 略增 | `saiidle` 每 46 ms 忙轮询一次 + 8 KB 缓存清理；优先级很低，主业务可抢占，但空闲期它吃满剩余 CPU |
| 按上下键有轻微杂音 | 硬件 | 播放后模拟链路处于常开状态，UART 突发活动耦合进功放；上电未播放时无此现象 |
| HyperRAM 预载 | **已关闭**（`AUDIO_CLIP_ENABLE 0`） | 两个候选窗口都"不保留写入"，且它自己要 1 秒总线，会把保活线程的传输拖超时 |
| 链表环形 DMA | 未做 | N6 的 GPDMA 支持 `CLLR.LA/ULL`，可让零数据由 DMA 自循环，彻底去掉保活线程与锁 |

## 五、可复现的操作

```text
nsh> eye_cam audio tone 440 800                       # 单音，自带 RAM 数据
nsh> eye_cam audio /mnt/sdcard/tone_440_15s.wav       # 44.1 kHz / 16 bit / 立体声
```

判定：`audio: played 15000 ms of audio in ~15000 ms (100.xx%)`；且不出现
`ic7_ck did not clock the block` / `no dma completion` / `codec refused the mute` 之外的异常。

## 六、蜂鸣器（已实现）

### 硬件

```
PD3 ──R66(1K)──▶ Q7(三极管基极) ──▶ 蜂鸣器 ── VCC3.3
```

- 三极管驱动，PD3 高 = 导通
- **无源**（需要方波）：用 `eye_cam tone 400` 与 `tone 3000` 对比听感确认，音高明显不同
  （有源蜂鸣器则无法变调 —— 用耳朵判定最可靠）

### 实现（`board/atk-dnn647/src/stm32n6_buzzer.c`）

| 接口 | 说明 |
|---|---|
| `stm32n6_buzzer_beep(ms)` | 纯 GPIO 开关（"咔"一声） |
| `stm32n6_buzzer_beeps(n, on, gap)` | 短促连响 |
| `stm32n6_buzzer_tone(hz, ms)` | 阻塞式发音（自检用） |
| `stm32n6_buzzer_tone_async(hz)` | **非阻塞**持续音：TIM3 更新中断每半个周期翻转 PD3 |
| `stm32n6_buzzer_tone_stop()` | 停音并拉低引脚（空闲调用安全） |

要点：

- **不能用忙等发持续音**：眼控循环本身要判断"动作是否还在保持"，忙等会把判决饿死
- **TIM3 在本工程空闲**（TIM2=`/dev/timer0`、TIM5=`/dev/oneshot`），1 MHz 时基，
  `ARR = 500000/hz - 1`，更新中断频率 = 音调 ×2
- 板级文件需要 `#include <arch/board/board.h>` 才能取到 `STM32_APB1_TIM_FREQUENCY`

### 眼控音调策略

| 事件 | 声音 |
|---|---|
| 二级页按住某动作 | 该动作音调**连续**响（直到动作触发或视线变化） |
| 菜单移动 / 进入 / 返回等动作完成 | **一声短音**（90 ms，同音调） |
| 闭眼 | 784 Hz (G5) |
| 左看 | 587 Hz (D5) |
| 右看 | 988 Hz (B5) |
| 其它 | 660 Hz (E5) |

## 七、后续可做

- **链表环形 DMA**（`CLLR.LA` + `ULL`）：零数据由 GPDMA 自循环，可删掉 `saiidle` 线程、锁与忙轮询，空闲占用归零
- **codec 静音**：若能查明"SAI 跑过即 NACK"的原因（引脚不冲突已排除），可恢复真正的输出关断
- 蜂鸣器可进一步做"语音化"节奏（例如进入成功 = 上行两音）
