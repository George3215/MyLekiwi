# MyLekiwi

MyLekiwi 是一套面向 LeKiwi / SO-101 的单帧桌面抓取闭环。Jetson 只负责腕相机采集与飞特舵机执行，RTX 4090 负责 GroundingDINO、SAM 2、Depth Anything V2、AnyGrasp、手眼变换和 LeRobot IK。

当前默认提示词是 `black object.`。机械臂到达观察姿态后只拍一帧；开始运动后不会再次观察或重新规划。

## 两台机器如何分工

MyLekiwi 不是把所有程序都放在同一台机器上运行，而是明确分成 4090 工作站和 Jetson 两端：

| 机器 | 运行内容 | 连接的硬件 | 不负责什么 |
|---|---|---|---|
| RTX 4090 工作站 | GroundingDINO、SAM 2、Depth Anything V2、AnyGrasp、手眼变换、SO-101 IK、安全候选筛选、可视化 | NVIDIA GPU；通过 SSH 访问 Jetson | 不直接连接舵机，不执行机械臂动作 |
| Jetson | 返回观察姿态、读取 1–5 关节和夹爪、腕相机单帧采集、读取 `plan.json`、飞特舵机执行和运行时 floor guard | 腕相机、`/dev/ttyACM0` 飞特总线、实体 LeKiwi | 不加载 DINO、SAM2、Depth Anything 或 AnyGrasp |

推荐目录如下：

```text
4090:   ~/MyLekiwi                 # 完整仓库、模型、AnyGrasp、可视化和计划
Jetson: ~/MyLekiwi                 # 仅运行时源码、配置、URDF 和碰撞 hull
Jetson: ~/lerobot/.venv/bin/python # 已安装 LeRobot + Feetech 的 Python
```

一次性部署时，4090 会向 Jetson 同步运行时代码和配置。每次抓取时只交换两个运行数据文件：

```text
Jetson -- wrist.png --> 4090
Jetson <-- plan.json  -- 4090
```

模型权重、AnyGrasp SDK、点云和可视化不会传到 Jetson；舵机命令也不会从 4090 直接发送。

## 信息流

```text
Jetson wrist RGB (one PNG)
        │
        ├──────────────┐
        ▼              ▼
 GroundingDINO   Depth Anything V2
        │ bbox         │ relative depth
        ▼              │
      SAM 2            │
        │ mask         │
        └──────┬───────┘
               ▼
      masked metric point cloud
               ▼
            AnyGrasp
               │ candidate centers
               ▼
 mask/width/table filters
               ▼
 hand-eye + vertical SO-101 IK
               ▼
 joint limits + full-gripper floor guard
               ▼
       plan.json (one JSON to Jetson)
```

AnyGrasp 的输入是目标 mask 内的米制相机坐标点云 `points[N,3]`、对应 RGB `colors[N,3]` 和空间边界 `lims[6]`。输出候选包含 `score`、`translation`、`rotation_matrix`、`width`、`height`、`depth` 和 `object_id`。

本实现只采用最高分安全候选的 `translation` 作为最终中心，丢弃 AnyGrasp 的旋转。SO-101 工具被强制竖直向下，并绕竖直轴搜索 yaw。运动顺序固定为：

```text
above = center + [x_offset, y_offset, approach_clearance]
down  = center + [x_offset, y_offset, 0]
grasp = center
lift  = center + [0, 0, lift]
```

因此 `x_offset_m`/`y_offset_m` 只用于单边夹爪的临时避让：先在偏置位置下降，再在抓取高度水平移动回 AnyGrasp 中心，最后闭合夹爪。

## 仓库结构

```text
configs/lekiwi.yaml                         统一运行参数
configs/calibration/lab_unit/               当前实验样机标定
integrations/anygrasp_worker.py             AnyGrasp 隔离进程适配器
mylekiwi/capture_wrist_frame.py             Jetson 单帧采集
mylekiwi/plan_grasp.py                      4090 感知与 IK 规划
mylekiwi/execute_grasp.py                   Jetson 只读预检/真机执行
mylekiwi/return_to_observation.py           安全返回观察姿态
mylekiwi/gripper_geometry.py                完整夹爪地面检查
robot/so101_kin_only.urdf                   SO-101 运动学模型
robot/gripper_collision_hull.npz            夹爪碰撞包络
scripts/sync_jetson.sh                      一次性同步代码与配置
scripts/run_grasp_pipeline.sh               一帧 PNG -> 一个 plan.json
```

## 4090 环境准备

4090 端需要：

- Python 3.12；
- CUDA 可用的 PyTorch；
- LeRobot 和 `placo-dep`，用于 `lerobot.model.kinematics.RobotKinematics`；
- Transformers 5.4–5.5；
- GroundingDINO、SAM 2、Depth Anything V2 本地权重；
- 单独获得许可的 AnyGrasp SDK、checkpoint 和专用 Python 环境；
- 能通过 SSH 登录 Jetson。

全新安装可以在 4090 上运行：

```bash
git clone https://github.com/George3215/MyLekiwi.git
cd MyLekiwi
uv sync
```

如果已有 LeRobot 环境，也可以不新建 MyLekiwi `.venv`，运行闭环时通过 `MYLEKIWI_PYTHON=/path/to/lerobot/.venv/bin/python` 指定解释器。

三套 Hugging Face 模型只下载到 4090，而且只需下载一次：

```bash
mkdir -p models
hf download IDEA-Research/grounding-dino-tiny --local-dir models/grounding-dino-tiny
hf download depth-anything/Depth-Anything-V2-Small-hf --local-dir models/Depth-Anything-V2-Small-hf
hf download facebook/sam2-hiera-small --local-dir models/sam2-hiera-small
```

运行时已设置 `HF_HUB_OFFLINE=1` 和 `TRANSFORMERS_OFFLINE=1`，不会重复联网下载权重。

AnyGrasp SDK、模型、二进制扩展和许可证不随本仓库发布。请按其许可单独安装，并设置：

```bash
export ANYGRASP_ROOT=/path/to/anygrasp_sdk/grasp_detection
export ANYGRASP_PYTHON=/path/to/anygrasp/python
```

`ANYGRASP_ROOT` 内需要能导入 `gsnet`，默认 checkpoint 是 `log/checkpoint_detection.tar`。

先在 4090 检查两个 Python 环境：

```bash
uv run python -c 'import torch; print("planner CUDA:", torch.cuda.is_available())'
$ANYGRASP_PYTHON -c 'import torch; print("AnyGrasp CUDA:", torch.cuda.is_available())'
```

两行都应输出 `True`。AnyGrasp 使用单独 Python 是因为其二进制 SDK、CUDA 和 Python 版本约束可能与主规划环境不同。

## Jetson 环境准备

Jetson 不需要下载任何感知模型，也不需要安装 AnyGrasp。Jetson 只需要：

- Python 3.12；
- 已安装 `feetech` 支持的 LeRobot 环境；
- 腕相机；
- 飞特舵机总线，默认 `/dev/ttyACM0`；
- 4090 可以通过 SSH 登录的用户账号。

本仓库默认假设 Jetson 已有：

```text
~/lerobot/.venv/bin/python
```

可以先在 Jetson 检查依赖，但不要连接或移动机械臂：

```bash
cd ~/lerobot
.venv/bin/python -c 'from lerobot.motors.feetech import FeetechMotorsBus; print("Jetson LeRobot/Feetech OK")'
```

如果 Jetson 的 Python 不在默认位置，运行 4090 脚本时设置：

```bash
export MYLEKIWI_JETSON_PYTHON=/absolute/path/to/python
```

## 当前样机标定

`configs/calibration/lab_unit/` 来自一台实体 LeKiwi，仅用于复现实验，不是 SO-101 通用标定。更换机械臂、舵机校准、相机、相机安装位姿、桌面高度或夹爪结构后，必须重新标定。

特别注意：当前手眼结果的 base frame 是未物理锚定的运动学 gauge；Depth Anything V2 使用当前场景拟合的仿射米制映射。不能把这些文件复制到另一台机器人后直接真机执行。

主要可调参数都在 [`configs/lekiwi.yaml`](configs/lekiwi.yaml)：

- `grasp.x_offset_m`、`grasp.y_offset_m`：仅作用于 `above/down` 的 Base-XY 临时避让；当前为 `30 mm / 50 mm`。
- `grasp.tcp_height_m`：最终抓取 TCP 相对桌面的高度。
- `grasp.floor_margin_m`、`execution.runtime_floor_reserve_m`：规划与运行时地面安全余量。
- `execution.arm_speed_raw`：飞特位置模式中的速度寄存器目标，不是轨迹插值次数。
- `execution.settle_timeout_s`、各 `*_tolerance_deg`：到位等待时间和关节到位容差。

## 双机部署：只需做一次

以下命令在 **4090** 上运行。Jetson 地址只通过环境变量传入，不会写入仓库：

```bash
cd ~/MyLekiwi
MYLEKIWI_JETSON=user@jetson-ip ./scripts/sync_jetson.sh
```

它在 Jetson 创建 `~/MyLekiwi`，并同步以下运行时文件：

```text
mylekiwi/*.py
configs/
robot/
```

它不会复制 4090 模型、AnyGrasp SDK、照片、旧计划或日志。源码更新后重新运行该命令即可。

## 每次抓取的完整顺序

下面严格标明每条命令在哪台机器运行。

### 第 1 步：Jetson 返回观察姿态

先在 **Jetson** 做零写入检查：

```bash
cd ~/MyLekiwi
../lerobot/.venv/bin/python -m mylekiwi.return_to_observation --read-only
```

确认当前路径、关节范围和 tool-Z 检查通过后，仍在 **Jetson** 执行复位：

```bash
cd ~/MyLekiwi
../lerobot/.venv/bin/python -m mylekiwi.return_to_observation --execute
```

该动作把机械臂移动到配置中的观察姿态，并把夹爪打开到 80%。

### 第 2 步：4090 触发单帧采集并生成计划

下面的命令只在 **4090** 运行：

```bash
cd ~/MyLekiwi
export ANYGRASP_ROOT=/path/to/anygrasp_sdk/grasp_detection
export ANYGRASP_PYTHON=/path/to/anygrasp/python
MYLEKIWI_JETSON=user@jetson-ip ./scripts/run_grasp_pipeline.sh
```

这个脚本自动完成：

1. 4090 通过 SSH 在 Jetson 调用 `mylekiwi.capture_wrist_frame`；
2. Jetson 只读同一时刻的 1–5 关节角和夹爪位置，并拍摄一张腕相机 PNG；
3. 只把这一张带关节 metadata 的 `wrist.png` 传到 4090；
4. 4090 运行 DINO、SAM2、Depth Anything、AnyGrasp、手眼变换、IK 和安全筛选；
5. 4090 保存所有可视化；
6. 只把最终 `plan.json` 传回 Jetson。

从拍照完成到整个机械臂动作结束，系统不再读取相机，也不重新规划。

如果 MyLekiwi 自己没有 `.venv`，脚本会尝试相邻 LeRobot 的 `.venv`；也可以在 4090 显式设置 `MYLEKIWI_PYTHON=/path/to/python`。

4090 的输入、输出和可视化位于：

```text
outputs/captures/latest/wrist.png
outputs/plans/latest/plan.json
outputs/plans/latest/rgb.png
outputs/plans/latest/groundingdino.png
outputs/plans/latest/sam2.png
outputs/plans/latest/depth.png
outputs/plans/latest/anygrasp.png
outputs/plans/latest/pipeline.png
```

传回 Jetson 的唯一规划文件位于：

```text
~/MyLekiwi/outputs/plans/latest/plan.json
```

### 第 3 步：Jetson 只读检查 plan.json

回到 **Jetson**，先做零写入预检：

```bash
cd ~/MyLekiwi
../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --read-only --allow-low-score
```

`READ_ONLY_OK` 只说明计划格式、当前观察姿态、关节范围和 floor guard 等只读条件通过，不代表已经抓取成功。

### 第 4 步：Jetson 真机执行

操作者确认 `plan.json`、现场净空、夹爪状态和急停手段后，才在 **Jetson** 显式执行：

```bash
cd ~/MyLekiwi
../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --execute --allow-low-score
```

只有这一步和第 1 步的复位 `--execute` 会写舵机。4090 上的规划命令不会直接写电机。

### 异常停在抓取点附近

执行失败并停在 grasp 附近时，不要直接重跑完整计划。先检查恢复条件：

```bash
cd ~/MyLekiwi
../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --read-only --resume-grasp --allow-low-score
```

确认后才在 Jetson 运行：

```bash
cd ~/MyLekiwi
../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --execute --resume-grasp --allow-low-score
```

恢复模式只继续执行：闭夹爪、停留、抬升、返回观察姿态、打开夹爪。

## 安全边界

- 所有舵机写入入口都要求显式 `--execute`；`--read-only` 不写电机。
- 计划器检查 IK 残差、关节范围、最大关节运动、腕旋转限制和完整夹爪沿路径的最低 Z。
- 执行器发送每段的单个关节位置目标，用 `Goal_Velocity` 限速，并持续读取编码器和夹爪最低 Z；它不会在运动中重新拍照。
- 任意 Python 异常或键盘中断会停止后续阶段，并把 Goal Position 设为当前读数以保持当前位置。
- 当前没有实时负载/电流/堵转检测、完整三维环境碰撞检测或抓取成功检测。夹住物体后，夹爪可能无法到达 `gripper_close`，从而被严格到位检查报告为失败。
- `5°` 的关节容差可能对应厘米级 TCP 偏差；它是减少误停的运行折中，不代表抓取精度。

请始终在现场监督真机，预留机械急停或断电手段，并在每次执行前检查最新 `plan.json` 与 `--read-only` 输出。

## License

本仓库使用 Apache-2.0。AnyGrasp 相关资产不包含在此许可证或仓库中，须遵守其单独许可。
