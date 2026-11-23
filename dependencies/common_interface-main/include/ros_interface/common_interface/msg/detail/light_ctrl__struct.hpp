// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/LightCtrl.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__LightCtrl __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__LightCtrl __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LightCtrl_
{
  using Type = LightCtrl_<ContainerAllocator>;

  explicit LightCtrl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cutter = 0;
      this->map = 0;
      this->wifi = 0;
      this->lock = 0;
      this->ble = 0;
      this->signal_4g = 0;
      this->cmd = 0;
      this->data_show = 0;
    }
  }

  explicit LightCtrl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cutter = 0;
      this->map = 0;
      this->wifi = 0;
      this->lock = 0;
      this->ble = 0;
      this->signal_4g = 0;
      this->cmd = 0;
      this->data_show = 0;
    }
  }

  // field types and members
  using _cutter_type =
    uint8_t;
  _cutter_type cutter;
  using _map_type =
    uint8_t;
  _map_type map;
  using _wifi_type =
    uint8_t;
  _wifi_type wifi;
  using _lock_type =
    uint8_t;
  _lock_type lock;
  using _ble_type =
    uint8_t;
  _ble_type ble;
  using _signal_4g_type =
    uint8_t;
  _signal_4g_type signal_4g;
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _data_show_type =
    uint16_t;
  _data_show_type data_show;

  // setters for named parameter idiom
  Type & set__cutter(
    const uint8_t & _arg)
  {
    this->cutter = _arg;
    return *this;
  }
  Type & set__map(
    const uint8_t & _arg)
  {
    this->map = _arg;
    return *this;
  }
  Type & set__wifi(
    const uint8_t & _arg)
  {
    this->wifi = _arg;
    return *this;
  }
  Type & set__lock(
    const uint8_t & _arg)
  {
    this->lock = _arg;
    return *this;
  }
  Type & set__ble(
    const uint8_t & _arg)
  {
    this->ble = _arg;
    return *this;
  }
  Type & set__signal_4g(
    const uint8_t & _arg)
  {
    this->signal_4g = _arg;
    return *this;
  }
  Type & set__cmd(
    const uint8_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__data_show(
    const uint16_t & _arg)
  {
    this->data_show = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::LightCtrl_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::LightCtrl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::LightCtrl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::LightCtrl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::LightCtrl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::LightCtrl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::LightCtrl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::LightCtrl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::LightCtrl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::LightCtrl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__LightCtrl
    std::shared_ptr<common_interface::msg::LightCtrl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__LightCtrl
    std::shared_ptr<common_interface::msg::LightCtrl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LightCtrl_ & other) const
  {
    if (this->cutter != other.cutter) {
      return false;
    }
    if (this->map != other.map) {
      return false;
    }
    if (this->wifi != other.wifi) {
      return false;
    }
    if (this->lock != other.lock) {
      return false;
    }
    if (this->ble != other.ble) {
      return false;
    }
    if (this->signal_4g != other.signal_4g) {
      return false;
    }
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->data_show != other.data_show) {
      return false;
    }
    return true;
  }
  bool operator!=(const LightCtrl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LightCtrl_

// alias to use template instance with default allocator
using LightCtrl =
  common_interface::msg::LightCtrl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__LIGHT_CTRL__STRUCT_HPP_
