## 1. 地形生成

```bash
python3 legged_robot_description/go2_description/scripts/generate_go2_terrain_scene.py \
  --terrain mixed \ 
  --seed 7  # gap pit flat rough stairs stairs-high mixed
```
会默认生成到
```bash
/home/ws/projects/legged_ros2/legged_robot_description/go2_description/mjcf/scene_terrain.xml
/home/ws/projects/unitree_mujoco/unitree_robots/go2/scene_terrain.xml
```

## 2. 宿主机启动 MuJoCo
```bash
cd ~/projects/unitree_mujoco/simulate/build
./unitree_mujoco -s scene_terrain.xml
```

确认 `~/projects/unitree_mujoco/simulate/config.yaml` 是：

```yaml
domain_id: 1
interface: "lo"
```

## 3：进入 Docker 并编译
```bash
cd /home/ws/projects/legged_ros2
xhost +local:docker
docker/run.sh

cd /root/legged_ws
source /root/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1

rm -rf build/legged_ros2_control install/legged_ros2_control
rm -rf build/go2_description install/go2_description
rm -rf build/legged_rl_controller install/legged_rl_controller

colcon build --packages-select \
  legged_ros2_control \
  go2_description \
  legged_rl_controller \
  --symlink-install

source install/setup.bash
```

**先确认 MuJoCo 状态已进 ROS**
```bash
ros2 topic list | grep -E 'lowstate|lowcmd|sport|wireless'
ros2 topic echo /lowstate --once
```

如果 `/lowstate` 没输出，先不要启动 RL。检查当前 shell：

```bash
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
```

`ROS_DOMAIN_ID` 必须是 `1`。

## 4.启动 sim-to-sim
```bash
cd /root/legged_ws
source /root/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1
source install/setup.bash

ros2 launch go2_description bringup_rl.launch.py \
  use_rviz:=true \
  use_goal_to_cmd_vel:=true \
  use_heightmap_publisher:=true
```

或若想使用观测中带target_pos项的策略的话：
```bash
ros2 launch go2_description bringup_rl.launch.py \
  use_rviz:=true \
  use_goal_to_cmd_vel:=true \
  use_heightmap_publisher:=true \
  onnx_model_path:=/root/legged_ws/install/go2_description/share/go2_description/config/rl_policy_target_pos/policy.onnx \
  io_descriptors_path:=/root/legged_ws/install/go2_description/share/go2_description/config/rl_policy_target_pos/IO_descriptors.yaml
  ```

## 5. 切换控制状态
```bash
docker exec -it legged-ros2-humble bash
cd /root/legged_ws
source /root/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1
source install/setup.bash

ros2 topic echo /joint_states --once
ros2 control list_controllers
```

`/joint_states.position` 不应全是 `0.0`。

**控制状态切换命令**

站立：

```bash
ros2 control switch_controllers \
  --deactivate sit_static_controller rl_controller \
  --activate stand_static_controller
```

切到 RL 控制：

```bash
ros2 control switch_controllers \
  --deactivate stand_static_controller sit_static_controller \
  --activate rl_controller
```

坐下：

```bash
ros2 control switch_controllers \
  --deactivate stand_static_controller rl_controller \
  --activate sit_static_controller \
  --best-effort
```

停止三个控制器：

```bash
ros2 control switch_controllers \
  --deactivate stand_static_controller sit_static_controller rl_controller \
  --best-effort
```

## 6. 发送目标点

RViz 里把目标点发到 `base` frame，先点很近的目标，例如前方 `0.3 ~ 0.6 m`。也可以用命令测试：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

监控位置指令转速度指令：

```bash
ros2 topic echo /rl_cmd_vel
```