import launch
import launch_ros

'''
    函数 作用：产生 launch 描述
    
    实现： 当使用 ros2 launch 时，就会 运行 generate_launch_description()函数 执行 多个节点
'''
def generate_launch_description():
    # 1. 创建 actions 对象,并 传入参数
    # 一次 运行 3个 节点/动作
    action_node_turtlesim_node =  launch_ros.actions.Node(
    # 传入的参数为：
        # (1) 功能包 的名字
        package = 'turtlesim',   
        # (2) 可执行文件 的 名字        
        executable= 'turtlesim_node',                   # 小海龟 节点
        # (3) 日志 输出 的 目的地  
        output = 'screen',                              # screen：输出 至 屏幕            
    )

    action_node_patrol_clien =  launch_ros.actions.Node(
    # 传入的参数为：
        # (1) 功能包 的名字
        package = 'demo_cpp_service',   
        # (2) 可执行文件 的 名字        
        executable= 'patrol_client',                    # 巡逻 客户端 节点
        # (3) 日志 输出 的 目的地  
        output = 'log',                                 # log：输出 至 日志文家          
    )

    action_node_turtle_control =  launch_ros.actions.Node(
        # 传入的参数为：
            # (1) 功能包 的名字
            package = 'demo_cpp_service',   
            # (2) 可执行文件 的 名字        
            executable= 'turtle_control',               # 小海龟 控制 服务端 节点
            # (3) 日志 输出 的 目的地  
            output = 'both',                            # both：两者 都输出            
        )

    # 2. 返回 LaunchDescription 对象，其 参数为一个 数组
    return launch.LaunchDescription([
        # 数组 内容为：actions动作
        action_node_turtlesim_node ,                    # 启动 小海龟
        action_node_patrol_clien ,                      # 启动 巡逻 客户端
        action_node_turtle_control                      # 启动 控制 服务端
    ])