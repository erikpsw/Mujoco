## 安装

[Windows系统下安装mujoco环境的教程【原创】_windows安装mujoco-CSDN博客](https://blog.csdn.net/qq_54900679/article/details/140907420

Python安装3.2.0版本

```python
pip install mujoco==3.2.0
pip install dm-control==1.0.21
pip install mujoco-python-viewer
```

## 模型

先修改URDF

修改前

```xml
<?xml version="1.0" encoding="utf-8"?>
<!-- This URDF was automatically created by SolidWorks to URDF Exporter! Originally created by Stephen Brawner (brawner@gmail.com) 
     Commit Version: 1.6.0-1-g15f4949  Build Version: 1.6.7594.29634
     For more information, please see http://wiki.ros.org/sw_urdf_exporter -->
<robot
  name="mini_mec_six_arm">  

  <link
    name="base_link">
    <inertial>
      <origin
        xyz="-0.0126930979005275 0.00191328701604972 0.138027242309266"
        rpy="0 0 0" />
```

修改后

```xml
<?xml version="1.0" encoding="utf-8"?>
<!-- This URDF was automatically created by SolidWorks to URDF Exporter! Originally created by Stephen Brawner (brawner@gmail.com) 
     Commit Version: 1.6.0-1-g15f4949  Build Version: 1.6.7594.29634
     For more information, please see http://wiki.ros.org/sw_urdf_exporter -->
<robot
  name="mini_mec_six_arm">  
  <mujoco>
        <compiler 
        meshdir="./" 
        balanceinertia="true" 
        discardvisual="false" />
  </mujoco>
  <link
    name="base_link">
    <inertial>
      <origin
        xyz="-0.0126930979005275 0.00191328701604972 0.138027242309266"
        rpy="0 0 0" />
```

增加

```xml
  <mujoco>
        <compiler 
        meshdir="./" 
        balanceinertia="true" 
        discardvisual="false" />
  </mujoco>
```

在安装路径的bin目录 进入cmd

```bash
compile \path\to\mini_mec_six_arm.urdf \path\to\mini_mec_six_arm.xml
```

### 控制

add actuator to control

```XMl
 <actuator>
    <position name="joint1_actuator" joint="joint1" kp="1000" kv="10"/>
    <position name="joint2_actuator" joint="joint2" kp="1000" kv="10"/>
    <position name="joint3_actuator" joint="joint3" kp="1000" kv="10"/>
    <position name="joint4_actuator" joint="joint4" kp="1000" kv="10"/>
    <position name="joint5_actuator" joint="joint5" kp="10"/>
    <position name="joint6_actuator" joint="joint6" kp="10"/>
    <velocity name="left_rear_wheel_joint_actuator" joint="left_rear_wheel_joint" />
    <velocity name="right_rear_wheel_joint_actuator" joint="right_rear_wheel_joint" />
    <velocity name="left_front_wheel_joint_actuator" joint="left_front_wheel_joint" />
    <velocity name="right_front_wheel_joint_actuator" joint="right_front_wheel_joint"/>
    
  </actuator>
```



## 可视化

### 方案1（废弃）

[Mujoco（dm-control渲染问题） - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/632973860)

导入`dm_render.py`

```python
from dm_render import DMviewer

physics = mujoco.Physics.from_xml_path(model_path)
# 渲染模型
viewer = DMviewer(physics)

# 持续运行模拟
while physics.time() < 10:
    physics.step()   # 进行模拟步
    viewer.render()  # 渲染场景
```

### 方案2

```python
import mujoco_viewer
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
# 创建渲染器
viewer = mujoco_viewer.MujocoViewer(model, data)
while True:
    mujoco.mj_step(model, data)
    viewer.render()
```

