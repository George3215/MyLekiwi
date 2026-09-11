# MyLekiwi

MyLekiwi 是一套面向 LeKiwi / SO-101 的单帧桌面抓取闭环。Jetson 只负责腕相机采集与飞特舵机执行，RTX 4090 负责 GroundingDINO、SAM 2、Depth Anything V2、AnyGrasp、手眼变换和 LeRobot IK。

当前默认提示词是 `black object.`。机械臂到达观察姿态后只拍一帧；开始运动后不会再次观察或重新规划。

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

## 环境准备

Python 需要 3.12。规划器依赖带 `placo-dep` 的 LeRobot，Jetson 依赖带 `feetech` 的 LeRobot。新环境可运行 `uv sync`；若已有 LeRobot 工作区，也可直接复用它的 `.venv`。

在 4090 下载三套 Hugging Face 模型一次即可：

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

## 当前样机标定

`configs/calibration/lab_unit/` 来自一台实体 LeKiwi，仅用于复现实验，不是 SO-101 通用标定。更换机械臂、舵机校准、相机、相机安装位姿、桌面高度或夹爪结构后，必须重新标定。

特别注意：当前手眼结果的 base frame 是未物理锚定的运动学 gauge；Depth Anything V2 使用当前场景拟合的仿射米制映射。不能把这些文件复制到另一台机器人后直接真机执行。

主要可调参数都在 [`configs/lekiwi.yaml`](configs/lekiwi.yaml)：

- `grasp.x_offset_m`、`grasp.y_offset_m`：仅作用于 `above/down` 的 Base-XY 临时避让；当前为 `30 mm / 50 mm`。
- `grasp.tcp_height_m`：最终抓取 TCP 相对桌面的高度。
- `grasp.floor_margin_m`、`execution.runtime_floor_reserve_m`：规划与运行时地面安全余量。
- `execution.arm_speed_raw`：飞特位置模式中的速度寄存器目标，不是轨迹插值次数。
- `execution.settle_timeout_s`、各 `*_tolerance_deg`：到位等待时间和关节到位容差。

## 使用

以下命令中的地址只通过环境变量传入，不会写入仓库。先做一次代码和配置同步：

```bash
MYLEKIWI_JETSON=user@jetson-ip ./scripts/sync_jetson.sh
```

1. 在 Jetson 只读检查并返回观察姿态：

```bash
cd ~/MyLekiwi && ../lerobot/.venv/bin/python -m mylekiwi.return_to_observation --read-only
cd ~/MyLekiwi && ../lerobot/.venv/bin/python -m mylekiwi.return_to_observation --execute
```

2. 在 4090 运行单帧采集、完整可视化、规划，并只把一个 JSON 传回 Jetson：

```bash
MYLEKIWI_JETSON=user@jetson-ip ./scripts/run_grasp_pipeline.sh
```

如果 MyLekiwi 自己没有 `.venv`，脚本会优先尝试相邻 LeRobot 的 `.venv`；也可显式设置 `MYLEKIWI_PYTHON=/path/to/python`。4090 的可视化保存在：

```text
outputs/plans/latest/rgb.png
outputs/plans/latest/groundingdino.png
outputs/plans/latest/sam2.png
outputs/plans/latest/depth.png
outputs/plans/latest/anygrasp.png
outputs/plans/latest/pipeline.png
```

3. 在 Jetson 做零写入预检：

```bash
cd ~/MyLekiwi && ../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --read-only --allow-low-score
```

4. 操作者确认计划、机器人状态和现场安全后，才显式执行：

```bash
cd ~/MyLekiwi && ../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --execute --allow-low-score
```

执行失败并停在 grasp 附近时，不要直接重跑完整计划。先检查恢复条件：

```bash
cd ~/MyLekiwi && ../lerobot/.venv/bin/python -m mylekiwi.execute_grasp --read-only --resume-grasp --allow-low-score
```

确认后才可将 `--read-only` 改成 `--execute`，继续闭夹爪、停留、抬升和回观察姿态。

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
