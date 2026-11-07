"""One world; fleet:=single|hybrid|five; control:=false for teleoperation."""
from pathlib import Path
import os
from datetime import datetime
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def setup(context):
    from lissajous_path.core import Scenario, Lissajous
    cfg=yaml.safe_load(Path(LaunchConfiguration('scenario').perform(context)).read_text())
    fleet=LaunchConfiguration('fleet').perform(context)
    if fleet not in ('single','hybrid','five'):raise ValueError('fleet must be single, hybrid or five')
    physical=[2] if fleet=='single' else [2,3,4] if fleet=='hybrid' else [1,2,3,4,5]
    control=LaunchConfiguration('control').perform(context).lower()=='true'
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    c=Scenario(**{k:tuple(cfg[k]) if k=='order' else cfg[k] for k in ('A','B','a','b','omega','order','p')})
    path=Lissajous(c)
    world=str(Path(share('ackermann_gazebo'))/'worlds/patrol.sdf')
    plugin_path=str(Path(get_package_prefix('gz_ros2_control'))/'lib')
    resource_path=str(Path(share('ackermann_description')).parent)
    actions=[ExecuteProcess(cmd=['ign','gazebo','-r',*([] if gui else ['-s']),world],output='screen',
        additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':plugin_path+':'+os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',''),
                        'IGN_GAZEBO_RESOURCE_PATH':resource_path+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')})]
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],output='screen'))
    common={'use_sim_time':True,**cfg}
    for i in physical:
        ns=f'robot_{i}';phase=c.phase(i)
        xy=path.xy(phase) if fleet!='single' or control else [0.,0.]
        yaw=float(path.heading(phase)) if fleet!='single' or control else 0.
        # Fortress plugins share one process: all namespaces must see the same
        # parameter file rather than overwrite the process-wide ROS arguments.
        controllers=str(Path(share('ackermann_gazebo'))/'config/controllers.yaml')
        urdf=xacro.process_file(str(Path(share('ackermann_description'))/'urdf/vehicle.urdf.xacro'),
            mappings={'robot_name':ns,'controllers_file':controllers}).toxml()
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',
            arguments=['--x',str(float(xy[0])),'--y',str(float(xy[1])),'--z','0',
                       '--yaw',str(yaw),'--frame-id','world','--child-frame-id',f'{ns}/odom']))
        actions.append(Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=ns,
            parameters=[{'robot_description':urdf,'use_sim_time':True}],output='screen'))
        spawn=Node(package='ros_gz_sim',executable='create',namespace=ns,
            arguments=['-world','patrol','-topic',f'/{ns}/robot_description','-name',ns,
                       '-x',str(float(xy[0])),'-y',str(float(xy[1])),'-z','0.005','-Y',str(yaw)],output='screen')
        joint=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','-c',f'/{ns}/controller_manager','--controller-manager-timeout','60'],output='screen')
        steering=Node(package='controller_manager',executable='spawner',arguments=['ackermann_steering_controller','-c',f'/{ns}/controller_manager','--param-file',controllers,'--controller-manager-timeout','60'],output='screen')
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=[joint])),
                        RegisterEventHandler(OnProcessExit(target_action=joint,on_exit=[steering])),spawn])
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',namespace=ns,
            arguments=[f'/model/{ns}/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                       f'/model/{ns}/contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'],
            remappings=[(f'/model/{ns}/ground_truth',f'/{ns}/ground_truth'),
                        (f'/model/{ns}/contacts',f'/{ns}/contacts')],output='screen'))
        if control:
            # Use the venv interpreter explicitly; ROS entrypoints otherwise use /usr/bin/python3.
            python=LaunchConfiguration('python').perform(context)
            actions.append(ExecuteProcess(cmd=[python,'-m','ackermann_mpcc.run',
                '--ros-args','-r',f'__ns:=/{ns}','--params-file',
                LaunchConfiguration('node_params').perform(context),
                *sum((['-p',f'{k}:={yaml.safe_dump(list(getattr(c,k)) if k=="order" else getattr(c,k),default_flow_style=True).strip().removesuffix("...").strip()}']
                      for k in ('A','B','a','b','omega','order','p')),[])],output='screen',
                additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}))
    if control:
        for i in c.order:
            actions.append(Node(package='swarm_coordination',executable='agent',namespace=f'robot_{i}',
                parameters=[common,{'agent_id':i,'virtual':i not in physical}],output='screen'))
        actions.append(Node(package='patrol_evaluation',executable='evaluate',
            parameters=[common,{'physical_ids':physical,
                'output_dir':LaunchConfiguration('output_dir').perform(context)}],output='screen'))
        if LaunchConfiguration('record').perform(context).lower()=='true':
            topics=['/clock','/patrol/start','/patrol/finished','/tf','/tf_static']
            for i in c.order:
                topics.extend(f'/robot_{i}/{suffix}' for suffix in ('phase','phase_reference','ground_truth','joint_states','control_status','contacts','ackermann_steering_controller/odometry'))
            actions.append(ExecuteProcess(cmd=['ros2','bag','record','-o',str(Path(LaunchConfiguration('output_dir').perform(context))/'bag'),*topics],output='screen'))
    if LaunchConfiguration('rviz').perform(context).lower()=='true':
        # Fit the selected scenario and include only robots with physical models.
        view=yaml.safe_load((Path(share('multi_robot_bringup'))/'rviz/patrol.rviz').read_text())
        manager=view['Visualization Manager']
        manager['Views']['Current']['Scale']=650.0/(2.4*max(c.A,c.B))
        manager['Views']['Saved']=[{
            'Class':'rviz_default_plugins/Orbit','Name':f'Robot {i} close view',
            'Target Frame':f'robot_{i}/base_footprint','Distance':2.5,
            'Pitch':0.65,'Yaw':0.8,'Near Clip Distance':0.01,
            'Focal Point':{'X':0.14,'Y':0.0,'Z':0.1}} for i in physical]
        if fleet=='single' and not control:
            manager['Views']['Current']={**manager['Views']['Saved'][0],
                                         'Name':'Current View','Distance':1.15}
            # Teleoperation has no evaluation node / patrol markers.
            manager['Displays']=[d for d in manager['Displays'] if d['Name']!='Patrol']
        for i in physical:
            manager['Displays'].append({
                'Class':'rviz_default_plugins/RobotModel','Name':f'Robot {i}',
                'Enabled':True,'Value':True,'Description Source':'Topic',
                'Description Topic':{'Value':f'/robot_{i}/robot_description',
                    'Durability Policy':'Transient Local','Reliability Policy':'Reliable','Depth':1},
                'Visual Enabled':True,'Collision Enabled':False})
        output=Path(LaunchConfiguration('output_dir').perform(context))
        output.mkdir(parents=True,exist_ok=True)
        rviz_config=output/'patrol.rviz'
        rviz_config.write_text(yaml.safe_dump(view,sort_keys=False))
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(rviz_config)],parameters=[{'use_sim_time':True}]))
    return actions


def generate_launch_description():
    root=Path(share('multi_robot_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('fleet',default_value='hybrid'),
        DeclareLaunchArgument('control',default_value='true'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('record',default_value='false'),
        DeclareLaunchArgument('python',default_value=str(Path.cwd()/'.venv/bin/python')),
        DeclareLaunchArgument('scenario',default_value=str(root/'config/feasible.yaml')),
        DeclareLaunchArgument('node_params',default_value=str(root/'config/mpcc.yaml')),
        DeclareLaunchArgument('output_dir',default_value=str(Path.cwd()/'patrol_runs'/('interactive_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:6]))),
        OpaqueFunction(function=setup)])
