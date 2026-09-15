# 雨轩视觉程序：单文件版

压缩包里只需要上传一个文件：`main.py`。

先在 CanMV IDE 打开 `main.py`，保持顶部默认值：

```python
RUN_MODE = "LOCAL"
MODE = "PREVIEW"
UART_ENABLED = False
```

运行后先确认能看到 GC2093 实时画面。画面颠倒或镜像时，改 `HMIRROR`、`VFLIP`。

确认取图后，再改顶部 `MODE` 测功能：

| MODE | 做什么 |
|---|---|
| PREVIEW | 只显示相机画面 |
| QR | 读取三位比赛任务码 |
| BOMB | 识别第一位指定颜色的排爆物 |
| TARGET | 识别第二位指定颜色靶的中心 |
| RESCUE | 识别第三位指定的白色救援物 |
| IDLE | 暂停识别 |

单项调试时填 `TASK = "211"`。第一位是排爆物颜色，第二位是靶颜色，第三位是救援物形状。`1/2/3` 分别对应红绿蓝；救援物对应圆柱、圆台、腰鼓。

白色救援物需要从**侧面**看、完整入镜、背景干净。程序无法明确区分时会输出 `UNKNOWN`，这时先调云台角度和 `WHITE_LAB`，不要强用结果。

以后接 F407 时，把 `RUN_MODE` 改为 `REMOTE`，确认实际串口 GPIO 后填写 TX/RX，并把 `UART_ENABLED` 改为 `True`。主控发送带 CRC 的指令切模式，例如：

```text
$V1,CMD,1,QR*EC8F
```

程序返回 `ACK` 并输出 `OBS`。只有 `status=VALID` 是连续多帧确认的有效坐标；`NONE`、`UNKNOWN`、`SETTLING`、`SLOW`、`IDLE` 都不能用于控制动作。

这份程序还没有经过 K230D、真实颜色阈值、三种道具、F407 串口、激光偏差和整车流程的实测。
