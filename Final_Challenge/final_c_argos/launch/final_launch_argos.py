from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #Get the address od the YAML File
    config = os.path.join(
        get_package_share_directory('final_c_argos'),
                                    'config',
                                    'params.yaml',        
    )

    input_node= Node(
        name="input",
        package='final_c_argos',
        executable='input',
        emulate_tty=True,
        output='screen',
        parameters=[config],
    )
    plot_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['/set_point_argos/data','/motor_output_argos/data','/pwm_argos/data'],
                    )
    graph_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_graph', 'rqt_graph'],
        output='screen'
    )

    reconf_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
        output='screen'
    )

    return LaunchDescription([ input_node,plot_node,graph_node,reconf_node])