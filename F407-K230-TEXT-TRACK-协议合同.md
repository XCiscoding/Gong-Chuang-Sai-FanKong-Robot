# F407 ↔ K230 TEXT TRACK 协议合同

这是接入 F407 被动 TEXT TRACK 镜像的 K230 脚本唯一通讯合同。视觉算法可以替换；本文件规定的引脚、串口、握手和帧不能各写一套。

## 1. 物理层（冻结）

| 方向 | 引脚 | 参数 |
| --- | --- | --- |
| F407 → K230 | F407 USART2 `PD5/TX` → K230 UART2 `GPIO12/RX` | 3.3 V、115200、8N1 |
| K230 → F407 | K230 UART2 `GPIO11/TX` → F407 USART2 `PD6/RX` | 3.3 V、115200、8N1 |
| 共同参考 | GND ↔ GND | 不接 5 V 信号 |

UART5（`PC12/PD2`）不属于本协议，不能作为本协议的回执或控制通道。

## 2. 会话状态机（冻结）

```text
K230 WAIT
  -- 收到 MODE,TRACK\r\n --> 发 ACK,TRACK\r\n --> ACTIVE
ACTIVE
  -- 每次观测 --> TRACK 有效帧 或 TRACK LOST 帧
```

冻结 F407 镜像实际发出 `MODE,TRACK\r\n`（hex：`4D 4F 44 45 2C 54 52 41 43 4B 0D 0A`）。K230 必须忽略终止的 `\r` 并以 `\n` 分帧；其当前实现也可兼容只带LF的诊断发送端，但该兼容性不改变正式帧必须使用CRLF的合同。

## 3. 允许写入 F407 的业务帧

K230 只可以借 `send_line()` 向 UART 写入下面三种 ASCII 行；每行必须为 CRLF 结尾：

```text
ACK,TRACK\r\n
TRACK,<seq>,<dx>,0,<quality>\r\n
TRACK,<seq>,LOST\r\n
```

- `seq`：`1..65535` 后可回绕到 `0`；每次 ACTIVE 会话从 `0` 重新开始。
- `dx`：有符号整数。它的真实左右语义属于 T2 画面验收，T1 不凭旧记录冻结符号。
- `quality`：整数；当前 F407 的 `Vision_IsFresh()` 只把 `100` 当作可用于后续控制的稳定观测，`50`可用于诊断但不构成控制资格。
- `TRACK,<seq>,LOST`：未检测到、跳变拒绝或观测无效时使用；绝不能拿上一帧 `dx` 冒充有效观测。

`print()`、异常堆栈、hex/ASCII 抓包只允许去 CanMV IDE/PC 调试输出，**禁止**写入 F407 UART。原始调试字节不是协议帧，会被 F407 计为坏行或导致溢出。

## 4. 不允许的变体

- 不得改成 B3/5B 二进制协议、JSON、中文、`VISION,...`或自定义调试行。
- 不得改 UART 号、GPIO、波特率、校验、停止位或逻辑电平。
- 不得在未收到 `MODE,TRACK` 前主动发送 ACK/TRACK（`uart_link_test.py` 的单向探测例外只用于运输层定位，不能用于视觉脚本）。
- 不得让算法模块直接生成 `uart.write()`；业务帧必须集中经过 `send_line()`。

## 5. 接入门

任何新的 K230 TRACK 脚本，在上传 CanMV 前先运行：

```powershell
python tools/check_k230_track_protocol.py --source <新脚本路径>
```

该检查只验证源码合同，不能证明现场接线、F407 PD5 物理波形、相机语义或底盘闭环。现场 T1 仍必须取得：`ACK >= 1`、`link_active = 1`、序号前进、协议/UART错误为0。
