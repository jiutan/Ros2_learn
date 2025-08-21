import launch
import launch_ros
import launch.launch_description_sources
# 通过 功能包名字 获取 share 目录
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    '''
        通过 参数 传递，来实现 条件 功能组件
    '''
    # 1. 创建 action对象，声明 launch 参数
    action_declare_startup_rqt = launch.actions.DeclareLaunchArgument('startup_rqt',default_value='False')
    # 2. 创建一个 参数传递 对象，使用该对象 加上 条件 即可控制
    startup_rqt = launch.substitutions.LaunchConfiguration('startup_rqt')

    ''' 
        动作1 - 启动 其他 launch文件
    '''
    # 1. 创建 路径对象，使用 数组形式 拼接 launch文件 的 路径
    #      参数：（1）使用 get_package_share_directory('功能包') 方法 寻找 功能包 的路径：turtlesim 功能包下 
    #          （2）'文件夹'：  launch 文件夹 下
    #          （3） 'launch文件名.launch.py'： 的 multisim.launch.py launch文件
    multisim_launch_path = [get_package_share_directory('turtlesim'),'/launch','/multisim.launch.py']
    # 2. 创建 action对象：通过 launch文件路径 寻找 launch文件
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(multisim_launch_path)
    )
    
    '''
        动作2 - 打印 数据
    '''
    # 创建 action 对象：打印 内容
    #    方法： launch.actions.LogInfo(msg=str(内容))
    action_log_info = launch.actions.LogInfo(msg=str(multisim_launch_path))

    '''
        动作3 - 执行 进程，其实 就是 执行一个 命令行：ros2 topic list
    '''
    # 创建 action 对象：执行 命令行语句
    #   方法：launch.actions.ExecuteProcess(cmd=['ros2','命令',''])
    action_topic_list = launch.actions.ExecuteProcess(
        # 创建 条件 组件对象
        # if startup_rqt:   
        #    run rqt
        condition = launch.conditions.IfCondition(startup_rqt),

        cmd=['rqt']                         # 执行ros2 topic list
    )

    '''
        动作4 - 组织 多个动作 （按顺序） 放到 一组中 实现
        方法：launch.actions.GroupAction([动作1,动作2,...])
        
        动作5 - 使用 定时器 间隔 执行 不同的 动作
        方法： launch.actions.TimerAction(period=时间,actions=[动作])
        作用：在程序 第时间s 后 ，执行 action动作
    '''
    # 创建 action对象
    #   方法：
    action_group = launch.actions.GroupAction([
        # 组织 多个动作 的 先后顺序
        
        # 使用 定时器 间隔 执行 动作
        launch.actions.TimerAction(period=2.0,actions=[action_include_launch]),     # 第2s后，启动 action_include_launch
        launch.actions.TimerAction(period=4.0,actions=[action_topic_list])          # 第4s后，启动 action_topic_list

    ])

    return launch.LaunchDescription([
        # 这两个 动作 会 一起 执行
        action_declare_startup_rqt,
        action_log_info,
        action_group

    ])