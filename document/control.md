## 函数

```python
# 定义关节控制函数
def control_joints(joint_names, positions):
    for joint_name, position in zip(joint_names, positions):
        joint_id = mujoco.mj_name2id(physics.model.ptr, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        physics.data.qpos[joint_id] = position
```

速度数据

freejoint的qpos一共有七个数，其中，qpos[0~2]是freejoint自身坐标系原点在世界坐标系的位置，qpos[3~6]是自身坐标系相对于世界坐标系的四元数；qvel一共有六个数，其中，qvel[0~2]是freejoint速度在世界坐标系下的表示，而qvel[3~5]是freejoint角速度在自身坐标系下的表示（之前自己一直认为角速度也是在世界坐标系下的表示）

`physics.data.qpos[:]`：**位置**（position）。`qpos` 表示物理引擎中物体的广义位置向量（包括位置和关节角度）。通过给 `qpos` 赋值，可以直接设置物体在环境中的位置或姿态（如坐标、角度等）。

`physics.data.qvel[:]`：**速度**（velocity）。`qvel` 表示物体的广义速度向量，包括线速度和角速度。赋值给 `qvel` 会设置物体的速度状态，影响其随时间的动态行为。

`physics.data.ctrl[:]`：**控制量**（control）。`ctrl` 通常用于设置控制输入，比如机器人关节的驱动力矩或力。这一行代码赋值的是控制信号，影响仿真中机器人的运动。