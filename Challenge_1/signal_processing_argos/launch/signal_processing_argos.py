from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    signal_gen_node=Node(package='signal_processing_argos',
                         executable='signal_generator',
                         output='screen'
                         )
    signal_proc_node=Node(package='signal_processing_argos',
                         executable='process',
                         output='screen'
                         )
    rqt_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['/signal_argos/data','/proc_signal_argos/data']
                    )
    l_d = LaunchDescription([signal_gen_node,signal_proc_node,rqt_node])


    
    return l_d