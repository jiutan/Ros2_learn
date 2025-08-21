import launch
import launch_ros

'''
    函数 作用：产生 launch 描述
    
    实现： 当使用 ros2 launch 功能包名 文件名.launch.py 时，就会 运行 generate_launch_description()函数 执行 多个节点
'''
def generate_launch_description():
    
    '''
        使用 launch 来 传递 参数
        实现：  先 声明 动作对象， 再能 传递 参数
    '''
    # 1. 先 创建并声明 launch.actions 动作/对象 ：声明 DeclareLaunchArgument(参数) 方法
    #       参数：      （1）给参数起一个别名           （2）默认值:default_value = '值' （必须是 字符串格式）
    #    后，在 return 中 加入
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument('launch_args_bg',default_value='150')
    

    '''
        使用 launch 运行 多个节点
    '''
    # 1. 创建 launch_ros.actions 对象,并 传入参数
    # 一次 运行 3个 节点/动作
    action_node_turtlesim_node =  launch_ros.actions.Node(
    # 传入的参数为：
        # (1) 功能包 的名字
        package = 'turtlesim',   
        # (2) 可执行文件 的 名字        
        executable= 'turtlesim_node',                   # 小海龟 节点
        
    # 2. 把 launch 的 参数 手动传递 给 某个节点
    #  使用 launch.substitutions.LaunchConfiguration() 作用：把 launch中的 参数 转换成 节点中用的参数
        parameters= [{'background_g':launch.substitutions.LaunchConfiguration('launch_args_bg')}],

        # (3) 日志 输出 的 目的地  
        output = 'screen',                              # screen：输出 至 屏幕          


    )

    action_node_patrol_clien =  launch_ros.actions.Node(
        package = 'demo_cpp_service',    
        executable= 'patrol_client',                    # 巡逻 客户端 节点 
        output = 'log',                                 # log：输出 至 日志文家          
    )
    action_node_turtle_control =  launch_ros.actions.Node(
            package = 'demo_cpp_service',   
            executable= 'turtle_control',               # 小海龟 控制 服务端 节点
            output = 'both',                            # both：两者 都输出            
        )

    # 2. 返回 LaunchDescription 对象，其 参数为一个 数组
    return launch.LaunchDescription([
        # 数组 内容为：actions动作
        action_declare_arg_background_g,
        action_node_turtlesim_node ,                    # 启动 小海龟
        action_node_patrol_clien ,                      # 启动 巡逻 客户端
        action_node_turtle_control                      # 启动 控制 服务端
    ])