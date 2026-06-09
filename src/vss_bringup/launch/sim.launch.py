from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ── Argumentos configuráveis na linha de comando ──────────────────────
    # Uso: ros2 launch vss_bringup sim.launch.py team_color:=yellow attack_dir:=-1
    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value='blue',
        description='Cor do time: blue ou yellow'
    )
    attack_dir_arg = DeclareLaunchArgument(
        'attack_dir',
        default_value='1',
        description='Direção de ataque: 1 = positivo X, -1 = negativo X'
    )
    hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='60.0',
        description='Frequência do loop de controle (Hz)'
    )

    team_color  = LaunchConfiguration('team_color')
    attack_dir  = LaunchConfiguration('attack_dir')
    control_hz  = LaunchConfiguration('control_hz')

    # ── Arquivo de parâmetros ─────────────────────────────────────────────
    params_file = PathJoinSubstitution([
        FindPackageShare('vss_bringup'),
        'config', 'vss_params.yaml'
    ])

    # ── Nó 1: FIRASim Bridge ──────────────────────────────────────────────
    # Ponte UDP Protobuf <-> ROS 2
    # Recebe: FIRASim 224.0.0.1:10002 e VSSRef 224.0.0.1:10003
    # Publica: /ball_raw, /robot_raw_0/1/2, /game_state
    # Assina:  /robot_commands → envia para FIRASim 127.0.0.1:20011
    fira_bridge_node = Node(
        package='vss_fira_bridge',
        executable='fira_bridge_node',
        name='fira_bridge',
        parameters=[
            params_file,
            {
                'team_color':    team_color,
                'vision_ip':     '224.0.0.1',
                'vision_port':   10002,
                'referee_ip':    '224.0.0.1',
                'referee_port':  10003,
                'command_ip':    '127.0.0.1',
                'command_port':  20011,
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # ── Nó 2: EKF Vision ─────────────────────────────────────────────────
    # Filtra bola e estima velocidade
    # Assina:  /ball_raw
    # Publica: /ball_state (filtrado + predito)
    ekf_node = Node(
        package='vss_vision',
        executable='ekf_node',
        name='ekf',
        parameters=[
            params_file,
            {
                'field_length':  1.50,
                'field_width':   1.30,
                'ball_friction': 0.05,
                'wall_restitution': 0.8,
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    # ── Nó 3: Strategy ───────────────────────────────────────────────────
    # Roda VSSPlaybook a 60Hz
    # Assina:  /ball_state, /robot_raw_0/1/2, /game_state
    # Publica: /robot_commands
    # Delay de 0.5s para garantir que bridge e EKF já subiram
    strategy_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='vss_strategy',
                executable='strategy_node',
                name='strategy',
                parameters=[
                    params_file,
                    {
                        'team_color':  team_color,
                        'attack_dir':  attack_dir,
                        'control_hz':  control_hz,
                        'wheel_base':  0.075,
                    }
                ],
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        # Argumentos
        team_color_arg,
        attack_dir_arg,
        hz_arg,

        # Info de inicialização
        LogInfo(msg='=== VSS_AI iniciando ==='),
        LogInfo(msg=['Time: ', team_color, ' | Attack dir: ', attack_dir]),

        # Nós (bridge e EKF sobem juntos, strategy com delay)
        fira_bridge_node,
        ekf_node,
        strategy_node,
    ])
