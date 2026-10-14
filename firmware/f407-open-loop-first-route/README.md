# PBRoBot 开环第一路线

日期：2026-10-09

## 2026-10-12 当前烧录候选：过减速带后单段横移

现场结论：旧 MS901M 已损坏，新陀螺仪尚未到货；过减速带会引入较大误差，因此当前暂停
减速带及其之前路线的联测。下一轮由人工把车放在“已经通过减速带”的标准位置，固件复位后
只执行一次 `-X` 横移，随后走完整的零速、STOP 和四机失能链。

```text
活动路线：PostBump
vx=-100 rpm, vy=0
hold=500, K=250
路线点数=1
IMU/航向控制=关闭
视觉纠偏=尚未接入，横移完成即停车
```

同时保留但不在本轮执行的完整路线候选中，斜移参数已更新为
`vx=-40 rpm, vy=+40 rpm, hold=25, K=50`。按速度积分估算，其距离约为原 `21 rpm`
版本的 1.9 倍；该估算不是实车距离验收。

路线编排语义固定为：普通运动使用 `MOVE`；每个赛道 corner 显式插入直线视觉纠偏；
减速带前的纠偏是一个显式的非 corner 例外；小球、抓取、放置等属于任务动作，不命名为
“纠偏”。视觉尚不可用或纠偏失败时必须停车，不得静默跳过并继续下一运动段。

本候选已完成 CubeIDE 工具链强制全量构建：

```text
text=22192
data=16
bss=2416
dec=24624
ELF SHA-256=1863FCF97841434255258F320DD5F7FE1FC7CABB2D4B9E77F28F17D0D3FE2D93
```

当前只证明源码、链接结果和活动 `PostBump` 符号进入 ELF；尚未烧录、Flash Verify 或实车验证。

## 基线

本工程复制自已通过实车无 IMU 动作的 `PBRoBot_NoIMU_20261009`。复制前关键源码与项目归档
`artifacts/chassis-work/20261009-f407-no-imu-sequence-rebuild` 的 SHA-256 已逐项比对一致。

可信回退映像：

- ELF：`219CE1BD0A3B0901D16D55AFBEF686C4BFA131FF23F36C5C0CACBFB594972037`
- HEX：`A1350014E58AB14F99BA3967B3E69F35DC8E11A33FC35CCDE2346BCC7B4F86D1`

## 本版本职责

- 新增 `MOTOR_TEST_OPEN_LOOP_FIRST_ROUTE` 独立模式。
- 复用 `Chassis_Path_FirstRoute` 的三段参数和正弦加减速。
- 路线平移目标通过独立开环适配器生成，固定 `w=0`。
- 开环适配器拒绝任何 `Delta_Angle != 0` 的路线点，禁止无反馈转向静默进入路线。
- 保留四机配置预检、逐帧 ACK、2 ms 路线调度、调度逾期失败、分段停车和最终失能。
- 原 `MOTOR_TEST_FIRST_ROUTE` 的 IMU 健康门保持不变。

## 当前编译开关

```text
VISION_LINK_TEST_ACTIVE         = 0
IMU_ONLY_TEST_ACTIVE            = 0
CHASSIS_HEADING_TURN_TEST       = 0
CHASSIS_HEADING_LOW_SPEED_TEST  = 0
CHASSIS_ROUTE_THIRD_ONLY        = 0
MOTOR_TEST_MODE                 = MOTOR_TEST_OPEN_LOOP_FIRST_ROUTE
```

## 证据边界

源码和构建通过不等于路线实车通过。烧录时保持电机 12V 关闭；只有 Flash Verify 成功、12V
稳定后重新复位，才建立新的路线运行会话。完整路线连续成功三次后才能升级当前验收状态。

## 构建结果

使用 STM32CubeIDE 1.19.0 自带 GNU Tools for STM32 13.3.rel1，在英文工作路径
`D:\STM32\PBRoBot_OpenLoop_FirstRoute_20261009` 执行强制全量构建。

```text
text=20360
data=16
bss=2304
dec=22680
```

- ELF SHA-256：`837CF1295DC187CE911379F1CC6FE6751B6299CC3309F9F3A897DC980E11EBF4`
- HEX SHA-256：`E178DF8B04DD04C0590BD8CF41C4FEFBF94C3406FDE9AADA2907787DA99CC530`

构建与静态核验：

- 全量编译、链接通过，无编译器警告。
- ELF 反汇编确认 `MotorTest_Init` 将 `motor_test_diag.mode` 写为 `11`。
- 最终映像保留 `Chassis_Path_FirstRoute` 与 `Chassis_PathPreparePointOpenLoop`。
- `ChassisHeading_ArmCurrent`、第三段单测路线和 `Vision_Init` 位于链接器丢弃区，没有装入最终运行映像。
- `MOTOR_TEST_MODE=9/10/11` 三个分支分别通过语法回归检查。
- 与可信基线相比，仅 `motor_test.h`、`chassis_heading.h`、`chassis_path.h`、
  `motor_test.c`、`chassis_path.c`、`main.c` 六个预定文件发生变化。

## 烧录结果

2026-10-09 使用 Horco CMSIS-DAP v0.2、序列号 `132765404768`，从隔离英文路径直接烧录
上述 ELF。OpenOCD 明确返回：

```text
Programming Finished
Verified OK
Resetting Target
```

烧录时电机 12V 保持关闭。当前只确认芯片 Flash 与本版本 ELF一致；三段路线、距离、越障和
最终停车仍须实车验收。

## 2026-10-10 Safe Start V1

### 修改原因

旧固件在复位后只要通信预检恢复，就会自动进入三段路线。2026-10-10 的被动监听已经同时
捕获四机失能 ACK、四机 33 字节配置回包、速度 ACK、同步 ACK、STOP ACK 和最终失能 ACK，
证明本次会话中的 F407 TX、MCE、X42S 回包、F407 RX 与运动命令链均能工作。此前“四机全无
回包”因此归入间歇性供电／线束／接触／复位会话问题，不能再把“链路恢复”当成“允许运动”。

### 新的安全状态机

- 上电或复位后先执行只含失能与读配置的预检，不发送使能、速度或同步运动命令。
- 启动预检最多尝试 3 次，必须连续通过 2 次才进入 `MOTOR_STAGE_WAIT_ARM`（数值 18）。
- 进入 `WAIT_ARM` 后继续保持失能；复位、掉线恢复和重新上电都不会自动开始路线。
- 只有先写入 `ARM1`（`0x41524D31`），再在 5 秒窗口内单独写入 `STRT`
  （`0x53545254`），才会触发最后一次只读预检；该次通过后才允许运动。
- `STRT` 无有效 ARM、错误密钥或 ARM 超时均不运动，并记录拒绝计数。
- ARM 和 START 密钥由固件消费后立即清零；复位会清除全部授权。
- 路线速度、三段参数、轮序映射、运行期逐帧 ACK、停车和最终失能逻辑未改。

本次 ELF 中 `motor_test_control` 地址为 `0x200001F4`，字段偏移如下：

```text
+0   arm_key
+4   start_key
+8   arm_state
+12  arm_events
+16  start_events
+20  rejected_start_events
```

ARM 和 START 必须分两次写入：先写 ARM，确认固件已消费，再写 START。禁止同时预置两个密钥；
ARM 处理会主动清除陈旧 START，避免一次调试器写入意外放行运动。

### 构建、映像与烧录证据

在英文构建路径 `D:\STM32\PBRoBot_OpenLoop_FirstRoute_20261009` 使用 STM32CubeIDE
GNU Tools for STM32 13.3.rel1 强制全量构建：

```text
text=21560
data=16
bss=2400
dec=23976
```

- 全量编译、链接通过，无编译器警告。
- `MOTOR_TEST_MODE=0/9/10/11` 分支均通过 `-Wall -Wextra -Werror` 编译检查。
- ELF SHA-256：`9B33C1DCDF5F0AB32A2D1A63BA96AB7CA795175ABE2CCEB79DDB794E79E1653C`
- HEX SHA-256：`AF12D27A88B5D7B383759EF2F0E89C9526E9BC06D46203B84EEBE5F91EFE878C`
- BIN SHA-256：`AFCD2E0149634D12BD5BC1D39FA5A176263BBB942097D73E55E1A0B0012DE93A`
- 由 HEX 还原的 BIN 与 ELF 直接导出的 BIN 逐字节一致。
- Horco CMSIS-DAP `132765404768` 烧录返回 `Programming Finished`、`Verified OK`、
  `Resetting Target`，随后又对芯片执行独立 `verify_image`，21576 字节一致。

### 当前验收边界

烧录和独立校验均在电机 12V 关闭时完成。断电 RAM 证据显示预检恰好重试 3 次后进入
`FAILED`，ARM/START 控制字段全部为 0，说明无电机电源时不会保留运动授权。

仍未完成的实车安全门是：12V 稳定后复位，确认底盘全程不动且 RAM 进入 `WAIT_ARM=18`，
并确认四机失能／配置掩码均为 `0x0F`。在该门通过前，不得写入 ARM 或 START；路线本身仍未
因本次改造而获得新的实车通过结论。

### 2026-10-10 实车安全联锁验收

12V 稳定、USB-TTL 监听器已移除、F407—MCE 通信线与 DAP 保持连接时完成以下验收：

- 12V 上电后底盘无移动、无轮组抖动。
- 人工复位 F407 后底盘仍无移动、无轮组抖动。
- RAM 进入 `WAIT_ARM=18`，`config_ok_mask=0x0F`、`disable_ok_mask=0x0F`，启动预检
  2 次连续通过，瞬态失败为 0，故障码为 0。
- ARM/START 初始密钥、状态和事件均为 0。
- 只写入 ARM 后等待超过 5 秒，`arm_events` 从 0 增至 1，授权自动回到 DISARMED，
  `start_events` 仍为 0，底盘无移动、无轮组抖动。
- ARM 已失效后单独写入 START，`rejected_start_events` 从 0 增至 1，`start_events`
  仍为 0，状态保持 `WAIT_ARM=18`，底盘无移动、无轮组抖动。

结论：Safe Start V1 的“复位不启动、ARM 超时失效、无 ARM 的 START 拒绝”已获得 RAM 与
实物双重证据。尚未验收的是有效 ARM→START 后的最终预检、路线运动、停车与最终失能。

### 2026-10-10 首次受控启动结果

在四轮架空、用户可立即断开 12V 的条件下执行一次受控 ARM→START：

- 写 ARM 前状态严格满足 `WAIT_ARM=18`、四机掩码 `0x0F`、零故障、零待处理密钥。
- ARM 被固件独立消费，`arm_state=1`、`arm_events` 加 1 后才单独写 START。
- 最终运动前预检在第一帧 `ID1 / F3` 失能请求处得到零字节，故障为
  `MOTOR_FAILURE_NO_REPLY`；`last_tx=STEP_MOTOR_TX_OK`，接收长度、溢出与 UART 错误均为 0。
- 安全门未增加 `start_events`，未进入运动阶段，实车没有获得路线授权。
- 随后由 DAP 发出 `SYSRESETREQ`，启动预检 3 次仍全部在 `ID1 / F3` 得到零字节；
  最终 `FAILED=15`，所有 ARM/START 字段为 0，底盘保持不动。

结论：间歇性“全无回包”已在新固件下再次复现，并被最终预检可靠拦截。软件已证明 UART
发送函数完成，但 F407 RX 未收到任何字节且没有 UART 硬件错误；故障范围缩小为 PD8 实际
波形／连接、MCE 命令接收与回传、PD9 实际连接／输入这段物理链。仍需在故障发生时进行
外部被动监听，才能区分“MCE 没有回包”和“已有回包但 F407 没收到”。

## 2026-10-11 运行期回包调度修复与当前停点

### 故障重新分层

本轮确认“无回包”包含不同故障，不能再合并为一个原因：

- 启动预检从 `ID1/F3` 起连续得到零字节：慢速单命令路径已有 10 ms 间隔，不能由路线 2 ms 槽位解释；范围仍是供电、PD8/MCE/PD9、接触或运行会话。
- 路线已经运动约 2 秒后，随机在某个 ID/F6 得到零字节：主因是运行期代码把名义发送槽位当作完整 ACK 截止时间，并在下一次 TX 前清空共享 RX；物理链的少量迟到或偶发丢包会被放大为整条路线终止。
- USB-TTL 监听为 `NO_BYTES`、但 F407 RAM 同时取得预检回复：监听支路自身不稳定，不得覆盖主控端原始诊断。
- `CMSIS-DAP command CMD_INFO failed`：属于 DAP USB/HID 路径，不属于电机 UART 回包。

原始 2 ms 代码只要下一槽位读取到 `drain.length==0` 就立即失败；没有迟到窗口，也没有重试。路线每 10 ms 名义上包含四个需 ACK 的 F6 事务，长任务的数百或数千次请求会显著放大低概率异常，因此短预检通过不能外推为长路线稳定。

### 最小修复

- 开放环路线等待完整 F6 ACK 最长 `MOTOR_TEST_ROUTE_REPLY_TIMEOUT_MS=8U`；未完成前不发送下一帧，不用下一次 TX 清除未决 RX。
- 迟到但合法的 ACK 会把下一槽位顺延，不追赶发送。
- 只有回复仍为零字节，且 F6 同目标在同步前可幂等覆盖时，才重发相同目标；`MOTOR_TEST_ROUTE_REPLY_MAX_RETRIES=2U`。
- 无效帧、UART error、设备拒绝或重试耗尽仍立即进入安全收尾。
- ID1→ID2→ID3→ID4→Sync 顺序、路线参数、轮位映射、STOP/失能和航向模式严格调度均未改变。

### 实机证据

1. 原始 2 ms 版本：四轮运动约 2 秒后在 `ROUTE_RUNNING / ID1 / F6` 记录 `NO_REPLY`，随后安全停车和失能。
2. 8 ms 等待版本：第一段完整跑完，第二段已经开始；之后在 `wheel_index=1 / ID2 / F6` 再次记录零回复并安全停车。该变化证明原 2 ms 判死会误伤迟到 ACK，同时表明仍存在偶发整帧丢失。
3. 最新 8 ms＋两次有界重发版本已部署，但三段实机结果尚未取得；不得写成问题已经解决。

### 当前源码与固件身份

三处 `Core/Inc/motor_test.h` 与 `Core/Src/motor_test.c` 已核对一致：项目归档、本机旧工作区和实际构建工程 `D:\STM32\PBRoBot_OpenLoop_FirstRoute_20261009`。

- `motor_test.h` SHA-256：`BC8CF301DC34BD7B8D1A7457D0CBEA01C5240C85A87BCE67C730768C9E4D2AAE`
- `motor_test.c` SHA-256：`60FDD79742301D6E5BDD7D659D09BA13C7D923FF5F1F41184BA0BF947AC46B7E`
- ELF SHA-256：`3221ABCCFD61D7C91A9308FB9491F803A7E00325AB203838E8460B32A0371A3D`
- HEX SHA-256：`E0A6F0762A559E6E244F37FC1354F6B22F389685EFDC4CC3C18794815EE4017C`
- BIN SHA-256：`D3F9CA13E8439C013071E2F16A620E634BD61489B24A8C1FD5A11CA16675DB4F`
- 装载映像：`LOAD_IMAGE_MATCH=True`
- 模式 0/9/10/11：`-Wall -Wextra -Werror` 通过
- OpenOCD：`Programming Finished`、`Verified OK`
- 独立 SWD 映像抽样：32/32 匹配

### 当前停点与唯一下一闸门

最新固件准备做架空三段终验时，Horco CMSIS-DAP 暂时卡死；ARM/START 尚未写入。用户最后收到的现场动作是关闭 12V、只重插电脑端 DAP USB、其他接线不动，再打开 12V。该动作尚未回传完成结果，不能假定当前动力状态。

DAP 恢复后：先只复位并等待预检；RAM 必须进入 `mode=11 / WAIT_ARM=18`，`first_failure=0`，四机配置和失能掩码均为 `0x0F`，连续预检 2 次且 key 清零。随后在四轮架空、运动区清空时执行一次受控 ARM→START，运行期间不得 halt 或轮询 CPU。终态目标为 `COMPLETE=14`、`completed_rounds=3`、`first_failure=0`、`cleanup_failure_mask=0`、`disable_ok_mask=0x0F`。架空通过后直接进入落地原参数首跑；此前不继续调整路线参数。

## 2026-10-11 实机失败与快速开发模式纠偏

### 本轮已发生的实机事实

- 实际烧录并运行的仍是 ELF `3221ABCCFD61D7C91A9308FB9491F803A7E00325AB203838E8460B32A0371A3D`；烧录后独立 SWD 抽样 32/32 匹配，RAM 为 `mode=11`，四机配置和失能预检均为 `0x0F`。
- 一次有效 `ARM→START` 后，第一段完成；第二段 `wheel_index=1` 开始后在 `ID1 / 0xF6` 得到 `MOTOR_FAILURE_INVALID_REPLY`，最终 `stage=FAILED`、`completed_rounds=1`、`cleanup_failure_mask=0x0F`。用户实际观察为车轮“突然不动”。这证明停止动作在实物上发生，但四机清理 ACK 未被确认，不能写成可靠停车链已经通过。
- 用户随后重新开关电源。复位后只读状态重新为 `mode=11 / WAIT_ARM=18`、`first_failure=0`、四机掩码 `0x0F`、连续预检通过；不自动运动是旧 Safe Start 行为，不是固件未运行。
- 原失败的 `failure_reply` 原始字节未在复位前导出，复位后已不可恢复。因此不能用“可能是回显或杂字节”作为事实，也不能据此放宽 ACK 成功判定。
- 中间候选 ELF `9391D93580394BAC857F3E90AC49E752297FE581ED4D65DB6A114B735C1078D8` 曾把回复窗口改为 20 ms、重试改为 3 次并放宽 ACK 扫描；它只有构建证据，没有烧录/实机证据，现已撤回，禁止使用。

### 当前快速开发候选

活动源码现采用以下边界：

- `MOTOR_TEST_MODE=MOTOR_TEST_OPEN_LOOP_FIRST_ROUTE`，即模式 11。
- `MOTOR_TEST_OPEN_LOOP_FAST_DEV_AUTORUN=1`：下载或复位后保留 3 秒明显延时，完成一次成功预检后自动跑一次，不再依赖 DAP 写 `ARM→START`。
- ACK 成功判定保持原严格协议；`MOTOR_TEST_ROUTE_REPLY_TIMEOUT_MS=8U`、`MOTOR_TEST_ROUTE_REPLY_MAX_RETRIES=2U`、清理超时 60 ms。
- 对同一幂等缓冲目标 `F6`，零字节、残帧或无效帧均允许有界重试；首次非空异常窗口锁存在 `failure_reply`，不得因后续重试覆盖。
- `tools/read_motor_safety_state.py` 已补 `last_reply_bytes` 与 `failure_reply_bytes`，下次失败必须先读取并保存，安全需要时先断 12V，但不得先 Reset、重新烧录或重新开关逻辑电源。

新候选已完成 CubeIDE 工具链强制全量构建，无编译错误：

- ELF：`D:\STM32\PBRoBot_OpenLoop_FirstRoute_20261009\Debug\PBRoBot.elf`
- 时间：`2026-10-11 16:32:40`
- 尺寸：`text=21792, data=16, bss=2408, dec=24216`
- SHA-256：`8444830C8E713B2F1017A617FED2EAADA712F7C4BC3823B6AC362779B5E74FEC`
- 证据边界：仅源码和构建通过；尚未烧录、Flash Verify或实车。

### 下一位 Agent 的唯一执行路径

1. 不再调整路线速度、时长、映射或 ACK 成功语义；先使用固定入口 `PBRoBot F407 OpenLoop Wireless DAP` 烧录上述 ELF。烧录配置必须继续指向该精确路径；默认烧录器仍是 Horco `PBRoBot F407 Wireless DAP`，不是 ST-LINK。
2. **该固件复位后会在约 3 秒＋预检时间后自动运动。** 用户点击 Run 前必须已经把车轮架空，或将小车放到空旷跑图起点并握住可立即关闭的 12V 电机开关；烧录过程中不再追加 SWD ARM。
3. 以本轮 OpenOCD 日志中的 `Programming Finished`、`Verified OK`、`Resetting Target` 作为下载回执；复位后不要为了追加 SWD 抽样而重新附加、halt 或轮询正在运行的 2 ms 路线调度，直接观察本轮三段动作。
4. 若三段完成，读取终态；目标为 `COMPLETE=14`、`completed_rounds=3`、`first_failure=0`、`cleanup_failure_mask=0`、`disable_ok_mask=0x0F`。同一固件无需再改即可直接落地原参数跑图。
5. 若中途突然停止，危险运动先关 12V；否则保持供电与会话。第一动作是运行 `python tools/read_motor_safety_state.py` 保存 `failure_reply_bytes`，不得先复位或重烧。根据原始字节判断回显、残帧、设备拒绝或事务错配，只改这一层。

旧章节中的 `WAIT_ARM→ARM→START` 是 Safe Start V1 历史验收流程，已被本节快速开发模式取代；不得把它作为当前下一步。
