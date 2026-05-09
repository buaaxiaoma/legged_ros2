import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="go2_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value="rl.yaml",
            description="Controller configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "main_loop_config",
            default_value="rl.yaml",
            description="Main loop configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_lowlevel_write",
            default_value="true",
            description="Enable low-level command writing, useful in debugging or testing scenarios. \
                        If set to true, the robot will receive low-level commands from the controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lowstate_topic",
            default_value="/lowstate",
            description="Low-level motor state topic. Unitree MuJoCo may expose this as /rt/lowstate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lowcmd_topic",
            default_value="/lowcmd",
            description="Low-level motor command topic. Unitree MuJoCo may expose this as /rt/lowcmd.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("go2_description"),
                    "config",
                    "rl_policy",
                    "policy.onnx",
                ]
            ),
            description="Path to ONNX policy model for RL controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "io_descriptors_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("go2_description"),
                    "config",
                    "rl_policy",
                    "IO_descriptors.yaml",
                ]
            ),
            description="Path to IO descriptors YAML for RL controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mujoco_scene_xml_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("go2_description"), "mjcf", "scene_terrain.xml"]
            ),
            description="MuJoCo scene XML used by the terrain heightmap publisher.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_heightmap_publisher",
            default_value="false",
            description="Publish /heightmap from the generated MuJoCo terrain scene.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rqt_cm",
            default_value="false",
            description="Start rqt_controller_manager automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_goal_to_cmd_vel",
            default_value="false",
            description="Start RViz goal-to-cmd_vel helper for position-tracking policies.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controller_config = LaunchConfiguration("controller_config")
    main_loop_config = LaunchConfiguration("main_loop_config")
    enable_lowlevel_write = LaunchConfiguration("enable_lowlevel_write")
    lowstate_topic = LaunchConfiguration("lowstate_topic")
    lowcmd_topic = LaunchConfiguration("lowcmd_topic")
    onnx_model_path = LaunchConfiguration("onnx_model_path")
    io_descriptors_path = LaunchConfiguration("io_descriptors_path")
    mujoco_scene_xml_path = LaunchConfiguration("mujoco_scene_xml_path")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")
    use_goal_to_cmd_vel = LaunchConfiguration("use_goal_to_cmd_vel")
    use_heightmap_publisher = LaunchConfiguration("use_heightmap_publisher")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "enable_sim:=",
            "false",
            " ",
            "enable_lowlevel_write:=",
            enable_lowlevel_write,
            " ",
            "lowstate_topic:=",
            lowstate_topic,
            " ",
            "lowcmd_topic:=",
            lowcmd_topic,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "ros2_control",
            controller_config,
        ]
    )

    main_loop_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "main_loop",
            main_loop_config,
        ]
    )

    rl_controller_params = {
        "onnx_model_path": onnx_model_path,
        "io_descriptors_path": io_descriptors_path,
    }

    main_loop_node = Node(
        package="legged_ros2_control",
        executable="go2_main_loop",
        parameters=[
            controller_config_path,
            robot_description,
            main_loop_config_path,
            rl_controller_params,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # -----------------------------------------------------------------------
    # RVIZ
    # -----------------------------------------------------------------------
    pkg_share = get_package_share_directory("go2_description")
    rviz_config_file = os.path.join(pkg_share, "rviz2", "go2.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )

    goal_to_cmd_vel_node = Node(
        package="legged_ros2_control",
        executable="goal_to_cmd_vel",
        name="goal_to_cmd_vel",
        output="both",
        parameters=[
            {
                "world_frame": "odom",
                "base_frame": "base",
                "goal_pose_topic": "/goal_pose",
                "clicked_point_topic": "/clicked_point",
                "cmd_vel_topic": "/rl_cmd_vel",
                "target_pos_topic": "/rl_target_pos_b",
                "sport_mode_state_topic": "/sportmodestate",
                "use_tf_pose": True,
                "use_sport_mode_state_pose": True,
                "prefer_sport_mode_state_pose": True,
                "update_rate": 50.0,
                "velocity_control_stiffness": 1.0,
                "heading_control_stiffness": 1.5,
                "only_positive_lin_vel_x": True,
                "max_lin_vel_x": 2.0,
                "max_lin_vel_y": 0.0,
                "max_ang_vel_z": 1.5,
                "target_dis_threshold": 0.1,
                "target_slowdown_distance": 0.4,
                "enable_soft_target_slowdown": True,
                "enable_heading_speed_gate": True,
                "heading_speed_gate_min": 0.25,
                "disallow_reverse_target_component": True,
                "lin_vel_threshold": 0.02,
                "ang_vel_threshold": 0.1,
                "max_linear_cmd_step": 0.05,
                "max_angular_cmd_step": 0.08,
                "command_smoothing_factor": 0.1,
            }
        ],
        condition=IfCondition(use_goal_to_cmd_vel),
    )

    terrain_heightmap_script = PathJoinSubstitution(
        [FindPackageShare(description_package), "scripts", "terrain_heightmap_publisher.py"]
    )

    terrain_heightmap_publisher_node = ExecuteProcess(
        cmd=[
            "python3",
            terrain_heightmap_script,
            "--ros-args",
            "-r",
            "__node:=terrain_heightmap_publisher",
            "-p",
            ["scene_xml_path:=", mujoco_scene_xml_path],
        ],
        output="both",
        condition=IfCondition(use_heightmap_publisher),
    )

    # -----------------------------------------------------------------------
    # Spawners
    # -----------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_controller", "-c", "/controller_manager", "--inactive"],
    )

    stand_static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["stand_static_controller", "-c", "/controller_manager", "--inactive"],
    )

    sit_static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sit_static_controller", "-c", "/controller_manager", "--inactive"],
    )

    delay_after_stand_static_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stand_static_controller_spawner,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    delay_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                imu_state_broadcaster_spawner,
            ],
        )
    )

    delay_after_imu_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_state_broadcaster_spawner,
            on_exit=[
                rl_controller_spawner,
            ],
        )
    )

    delay_after_rl_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rl_controller_spawner,
            on_exit=[
                sit_static_controller_spawner,
            ],
        )
    )

    delay_after_sit_static_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=sit_static_controller_spawner,
            on_exit=[
                rviz_node,
                rqt_controller_manager,
                goal_to_cmd_vel_node,
                terrain_heightmap_publisher_node,
            ],
        )
    )

    nodes = [
        main_loop_node,
        robot_state_pub_node,
        stand_static_controller_spawner,
        delay_after_stand_static_controller_spawner,
        delay_after_joint_state_broadcaster_spawner,
        delay_after_imu_state_broadcaster_spawner,
        delay_after_rl_controller_spawner,
        delay_after_sit_static_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
