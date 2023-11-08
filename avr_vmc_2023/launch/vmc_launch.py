from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------- Diagnostic Aggregator ----------
    # diagnostic_aggregator = Node(
    #         package='diagnostic_aggregator',
    #         executable='aggregator_node',
    #         output='screen',
    #         parameters=[PathJoinSubstitution([
    #             FindPackageShare('avr_vmc_2023'),
    #             'launch',
    #             'diagnostic_analyzers.yaml'
    #         ])]
    # )

    # ---------- Status Lights ----------
    # status_lights_node = Node(
    #         package='avr_vmc_2023_status_lights',
    #         executable='status_lights_node'
    # )

    # ---------- PCC ----------
    pcc_uros_agent = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=[
                'serial',
                '--baudrate',
                '115200',
                '--dev',
                '/dev/ttyUSB0'
            ]
    )
    pcc_enable_servos = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call '
                '/servo/enable std_srvs/srv/SetBool ',
                '\"{data: true}\"'
            ]],
            shell=True
    )

    # ---------- PX4 ----------
    px4_uros_agent = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=[
                'serial',
                '--baudrate',
                '115200',
                '--dev',
                '/dev/ttyTHS1'
            ]
    )

    # ---------- ZED ----------
    zed_wrapper = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                FindPackageShare('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_model': 'zedm',
                'config_path': PathJoinSubstitution([
                        FindPackageShare('avr_vmc_2023'),
                        'config',
                        'zed_config.yaml'
                ])
            }.items()
    )

    # ---------- CSI ----------
    csi_driver = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                FindPackageShare('avr_vmc_2023_csi_driver'),
                '/launch/csi_driver_launch.py'
            ]),
            launch_arguments={
                'framerate': 30,
                'height': 720,  # ToDo: Recalibrate for 1640x1232
                'width': 1280,
                'info_file': PathJoinSubstitution([
                    FindPackageShare('avr_vmc_2023'),
                    'config',
                    'csi.yaml'
                ])
            }.items()
    )

    # ---------- BDU ----------
    bdu_trigger = Node(
            package='avr_vmc_2023_bdu',
            executable='bdu_trigger_node',
            parameters=[
                {
                    'hold_duration': 0.75,
                    'max_value': 255,
                    'min_value': 0,
                    'servo_num': 0
                }
            ]
    )

    return LaunchDescription([
        # diagnostic_aggregator,
        # status_lights_node,
        pcc_uros_agent,
        px4_uros_agent,
        bdu_trigger,
        zed_wrapper,
        csi_driver,
        RegisterEventHandler(
            OnProcessStart(
                target_action=pcc_uros_agent,
                on_start=[
                    LogInfo(msg='Enabling servos'),
                    pcc_enable_servos
                ]
            )
        )
    ])
