# Vision 测试身份回执

> 提交时刻：2026-10-23
>
> 本文件是本次 Git 提交的固定测试基线，不是动态项目真源。现场的当前动作、供电状态与首个失败证据仍以项目 `00-项目管理/任务看板.md` 为准。

## 这次提交锁定的通讯闭包

| 项目 | 文件或身份 | SHA-256 |
| --- | --- | --- |
| F407 TEXT TRACK 源码 | `f407-vision-transport/` | 与冻结候选源逐字同哈希 |
| K230 现场 TRACK 脚本 | `k230/line_track_uart.py` | `36BBFF06F906017E054532471CCC7DF583655595C892C0774C4FE813EFC45C97` |
| K230 运输层基线 | `k230/uart_link_test.py` | `627297D6B00068552E76E8F7EF73D13741AAE0DF270D1BAAEB27238C4636D857` |
| F407 被动测试映像（不提交） | `PBRoBot-text-track-passive.hex` | `288D998A78F003977FCF49BFC9A19D2E6C9ABC8E70EEA4E59020B02467CA2007` |
| 协议合同 | `F407-K230-TEXT-TRACK-协议合同.md` | 见本提交 |
| 脚本门禁 | `tools/check_k230_track_protocol.py` | 见本提交 |

## 冻结接口

```text
F407 USART2 PD5/TX → K230 UART2 GPIO12/RX
K230 UART2 GPIO11/TX → F407 USART2 PD6/RX
GND 共地；3.3 V；115200/8N1

F407 → K230: MODE,TRACK\r\n
K230 → F407: ACK,TRACK\r\n
K230 → F407: TRACK,<seq>,<dx>,0,<quality>\r\n | TRACK,<seq>,LOST\r\n
```

UART5（PC12/PD2）、电机、机械臂不属于本次通讯测试。

## 提交时的证据边界

- F407 冻结被动映像曾完成 Programming / Verify / Reset，随后板端向量读回匹配该映像；这不是本提交再次烧录的证据。
- K230 的当前脚本身份已由哈希确认；用户报告单独发送 `MODE,TRACK` 时 K230 可以接收并回传。
- `VisionUart_Send()` 的软件队列成功不等于 PD5 已出现物理字节。
- 本提交不宣称 F407↔K230 T1 已通过。后续 PASS 必须同时取得 ACK 增长、`link_active=1`、序号前进及协议/UART错误为0。
- 现场异常首先按被动、RX-only 的前向观察定位；禁止把调试文本写入 F407 UART。

## 提交前可复现检查

```powershell
python tools/check_k230_track_protocol.py --source k230/line_track_uart.py
```

`uart_link_test.py` 的主动探测行为只用于运输层定位，不适用也不需要通过 TRACK 视觉脚本门禁。
