# GMR PKL -- SONIC

根据 sonic protocol V1 进行修改，要求是输入IsaacLab 顺序的29个关节位置和速度

GMR 输出为 Mujoco 顺序，需要转换为 IsacLab 顺序

IsaacLab 顺序的29个关节为：

```text
0  LeftHipPitch
1  LeftHipRoll
2  LeftHipYaw
3  LeftKnee
4  LeftAnklePitch
5  LeftAnkleRoll
6  RightHipPitch
7  RightHipRoll
8  RightHipYaw
9  RightKnee
10 RightAnklePitch
11 RightAnkleRoll
12 WaistYaw
13 WaistRoll
14 WaistPitch
15 LeftShoulderPitch
16 LeftShoulderRoll
17 LeftShoulderYaw
18 LeftElbow
19 LeftWristRoll
20 LeftWristPitch
21 LeftWristYaw
22 RightShoulderPitch
23 RightShoulderRoll
24 RightShoulderYaw
25 RightElbow
26 RightWristRoll
27 RightWristPitch
28 RightWristYaw
```

- 左腿 6 个
- 右腿 6 个
- 腰部 3 个
- 左臂 7 个
- 右臂 7 个

## Install 

将 pkl_stream_server.py 和 inspect_pkl.py 放入 gear_sonic/scripts 目录下



## 可选检查

提前检查pkl文件是否规范

```bash
cd /home/qc/GR00T-WholeBodyControl
source .venv_sim/bin/activate
python gear_sonic/scripts/inspect_pkl.py /path/to/your_motion.pkl
```

## 终端 1

```bash
cd /home/qc/GR00T-WholeBodyControl
source .venv_sim/bin/activate
python gear_sonic/scripts/run_sim_loop.py
```

## 终端 2

```bash
cd /home/qc/GR00T-WholeBodyControl/gear_sonic_deploy
bash deploy.sh sim --input-type zmq --zmq-host localhost
```

## 终端 3

```bash
cd /home/qc/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pkl_stream_server.py \
  --pkl_file /path/to/your_motion.pkl \
  --port 5556 \
  --target_fps 50 \
  --num_frames_to_send 10 \
  --loop
```

## 多动作字典

```bash
cd /home/qc/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pkl_stream_server.py \
  --pkl_file /path/to/motions_dict.pkl \
  --motion_name motion_1 \
  --port 5556 \
  --target_fps 50
```

## 按键顺序

1. 终端 2 按 `]`
2. MuJoCo 窗口按 `9`
3. 终端 2 按 `ENTER`

## 按键

- `]`
- `9`
- `ENTER`
- `O`


## 真机

可以直接上真机，参考 nvidia sonic 官方文档

终端2输入命令改为：

```bash
cd /home/qc/GR00T-WholeBodyControl/gear_sonic_deploy
bash deploy.sh real --input-type zmq --zmq-host localhost
```