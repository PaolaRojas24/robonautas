from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
#############################################grupo0
    sp_node0 = Node(
        name="sp_gen",
        package='motor_control_argos',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="grupo0"
    )

    ctrl_node0 = Node(
        name="ctrl",
        package='motor_control_argos',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="grupo0",
        parameters=[{
            'gain_Kp': 0.33,
            'gain_Ki': 0.8,
            'gain_Kd': 0.035,
            'sample_time': 0.1
        }]
        
    )

    motorsys_node0 = Node(
        name="motor_sys",
        package='motor_control_argos',
        executable='motor_sys',
        emulate_tty=True,
        output='screen',
        namespace="grupo0",
        parameters=[{
            'sample_time': 0.02,
            'sys_gain_K': 1.75,
            'sys_tau_T': 0.5,
            'initial_conditions': 0.0,
        }]
    )
#d###########################grupo1
    sp_node1= Node(
        name="sp_gen",
        package='motor_control_argos',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="grupo1"
    )

    ctrl_node1 = Node(
        name="ctrl",
        package='motor_control_argos',
        executable='controller',
        emulate_tty=True,
        output='screen', 
        namespace="grupo1",
        parameters=[{
            'gain_Kp': 0.33,
            'gain_Ki': 0.8,
            'gain_Kd': 0.035,
            'sample_time': 0.1
        }]
    )
    

    motorsys_node1 = Node(
        name="motor_sys",
        package='motor_control_argos',
        executable='motor_sys',
        emulate_tty=True,
        output='screen',
        namespace="grupo1",
        parameters=[{
            'sample_time': 0.02,
            'sys_gain_K': 1.75,
            'sys_tau_T': 0.5,
            'initial_conditions': 0.0,
        }]
    )
    ######################################################grupo2
    sp_node2= Node(
        name="sp_gen",
        package='motor_control_argos',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="grupo2"
    )

    ctrl_node2 = Node(
        name="ctrl",
        package='motor_control_argos',
        executable='controller',
        emulate_tty=True,
        output='screen', 
        namespace="grupo2",
        parameters=[{
            'gain_Kp': 0.33,
            'gain_Ki': 0.8,
            'gain_Kd': 0.035,
            'sample_time': 0.1
        }]
    )
    

    motorsys_node2 = Node(
        name="motor_sys",
        package='motor_control_argos',
        executable='motor_sys',
        emulate_tty=True,
        output='screen',
        namespace="grupo2",
        parameters=[{
            'sample_time': 0.02,
            'sys_gain_K': 1.75,
            'sys_tau_T': 0.5,
            'initial_conditions': 0.0,
        }]
    )
    
    
 ################################################################################################### 
    plot_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['/grupo0/motor_input_u_argos/data','/grupo0/set_point_argos/data','/grupo0/motor_output_y_argos/data']
                    )
    graph_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_graph', 'rqt_graph'],
        output='screen'
    )

    reconf_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
        output='screen'
    )

    return LaunchDescription([motorsys_node0, sp_node0, ctrl_node0,motorsys_node1, sp_node1, ctrl_node1,motorsys_node2, sp_node2, ctrl_node2,graph_node,plot_node,reconf_node,])