# ZiMing｜Vision

这个分支只保存 STM32F407 侧的视觉传输层：文本协议、UART 收发、观测新鲜度与协议回归用例。
它不包含 K230 的识别算法（见 `YuXuan/Vision`），也不包含电机路线、机械臂或固件映像。

## 当前模块

- `f407-vision-transport/`：`MODE,TRACK` / `ACK,TRACK`、`TRACK` / `LOST` 文本协议的 F407 适配代码与 C 主机回归测试。
- `f407-vision-transport/EVIDENCE.md`：原候选的构建、协议测试和物理验收边界。

## 边界

该快照证明协议实现与当时的构建/主机测试，不替代当前 K230 画面、目标识别稳定性或视觉驱动底盘的实车验收。固件映像是可再生成物，故意不提交。

## 仓库分支约定

- `YuXuan/Vision`：K230 识别与相机侧代码。
- `ZiMing/Vision`：F407 视觉传输与运动侧接口（当前分支）。
- `ZiMing/Motor`：X42S 电机、底盘运动和停车安全链。
- `ZiMing/Arm`：机械臂模块获得独立源码后建立。
- `main`：仅收录跨模块实车验证完成的集成版本。
