# ROS2 学习过程中 遇到的模板
# 以后 可以 直接 复制粘贴 使用
## C++
### 一、 CMakeLists
### 二、 参数 服务
#### 1. 更新 其他节点 参数K 的 模板
```c
void update_server_param_k(double k){
    // 1. 创建 一个 参数服务 的对象（包含：  name 与 value 两个 参数）
    auto param = rcl_interfaces::msg::Parameter();
    // 2. 给 服务对象 name 赋值
    //      string name 与  ParameterValue value（复合消息接口）
    param.name = "k";
    // 3. 创建 value参数对象 ，其为ParameterValue消息接口 类型 
    auto param_value = rcl_interfaces::msg::ParameterValue(); 
    //  分别给 ParameterValue对象 的 参数 赋值
    // 优点：使用 消息接口 标准化
    param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;    // 类型为 double消息接口 类型 
    // 由于 是 double类型，可 直接赋值 double_value
    param_value.double_value = k;
    // 4. 将 服务对象 的 value 赋值 param_value
    param.value = param_value;
    // 5. 请求 更新/设置参数 并 将 返回的 结果 进行 处理
    auto response = this->call_set_parameter(param);
    // 判断 返回结果
    if(response == NULL)        // response 为 指针
    {
        RCLCPP_INFO(this->get_logger(),"参数 更新 失败！");
        return;
    }
    // 将返回的结果 进行 遍历处理
    // 其 response内容为 ：SetParametersResult[]数组，其 对象为 results
    //      内部参数为：bool successful 与  string reason
    for(auto result : response -> results){
        if(result.successful == false){
            // successful   是否成功
            // reason       失败的原因（注意使用：c_str() 转换为 C语言下的 字符串类型）
            RCLCPP_INFO(this->get_logger(),"参数 更新 失败！其原因为:%s",result.reason.c_str()); 
        }else{
             RCLCPP_INFO(this->get_logger(),"参数 更新 成功！"); 
        }
    }
}
```
## Python