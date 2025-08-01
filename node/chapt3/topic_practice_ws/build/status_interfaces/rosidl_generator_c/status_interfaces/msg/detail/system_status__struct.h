// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from status_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#ifndef STATUS_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__STRUCT_H_
#define STATUS_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'host_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SystemStatus in the package status_interfaces.
/**
  * 在 下面 写 需要 的 消息 信息
 */
typedef struct status_interfaces__msg__SystemStatus
{
  /// 记录 时间辍
  builtin_interfaces__msg__Time stamp;
  /// string 为 ros2 中 内置的 字符串类型
  /// 系统 主机 的 名字
  rosidl_runtime_c__String host_name;
  /// float32 为 ros2 中 内置的 字符串类型
  /// CPU 使用率
  float cpu_percent;
  /// 内存 使用率
  float memory_percent;
  /// 内存 总大小
  float memory_total;
  /// 剩余 内存大小
  float memory_available;
  /// 网络 数据 发送 总量  1 MB = 8MB
  double net_sent;
  /// 网络 数据 接收 总量
  double net_recv;
} status_interfaces__msg__SystemStatus;

// Struct for a sequence of status_interfaces__msg__SystemStatus.
typedef struct status_interfaces__msg__SystemStatus__Sequence
{
  status_interfaces__msg__SystemStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} status_interfaces__msg__SystemStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STATUS_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__STRUCT_H_
