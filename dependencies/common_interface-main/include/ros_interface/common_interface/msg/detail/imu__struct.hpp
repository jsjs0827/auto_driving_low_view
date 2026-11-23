// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__IMU__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__IMU__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__Imu __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__Imu __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Imu_
{
  using Type = Imu_<ContainerAllocator>;

  explicit Imu_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->roll = 0;
      this->pitch = 0;
      this->yaw = 0;
      this->gyro_x = 0;
      this->gyro_y = 0;
      this->gyro_z = 0;
      this->accel_x = 0;
      this->accel_y = 0;
      this->accel_z = 0;
      this->temperature = 0;
    }
  }

  explicit Imu_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->roll = 0;
      this->pitch = 0;
      this->yaw = 0;
      this->gyro_x = 0;
      this->gyro_y = 0;
      this->gyro_z = 0;
      this->accel_x = 0;
      this->accel_y = 0;
      this->accel_z = 0;
      this->temperature = 0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _roll_type =
    int16_t;
  _roll_type roll;
  using _pitch_type =
    int16_t;
  _pitch_type pitch;
  using _yaw_type =
    int16_t;
  _yaw_type yaw;
  using _gyro_x_type =
    int16_t;
  _gyro_x_type gyro_x;
  using _gyro_y_type =
    int16_t;
  _gyro_y_type gyro_y;
  using _gyro_z_type =
    int16_t;
  _gyro_z_type gyro_z;
  using _accel_x_type =
    int16_t;
  _accel_x_type accel_x;
  using _accel_y_type =
    int16_t;
  _accel_y_type accel_y;
  using _accel_z_type =
    int16_t;
  _accel_z_type accel_z;
  using _temperature_type =
    int16_t;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__roll(
    const int16_t & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const int16_t & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const int16_t & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__gyro_x(
    const int16_t & _arg)
  {
    this->gyro_x = _arg;
    return *this;
  }
  Type & set__gyro_y(
    const int16_t & _arg)
  {
    this->gyro_y = _arg;
    return *this;
  }
  Type & set__gyro_z(
    const int16_t & _arg)
  {
    this->gyro_z = _arg;
    return *this;
  }
  Type & set__accel_x(
    const int16_t & _arg)
  {
    this->accel_x = _arg;
    return *this;
  }
  Type & set__accel_y(
    const int16_t & _arg)
  {
    this->accel_y = _arg;
    return *this;
  }
  Type & set__accel_z(
    const int16_t & _arg)
  {
    this->accel_z = _arg;
    return *this;
  }
  Type & set__temperature(
    const int16_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::Imu_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::Imu_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::Imu_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::Imu_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Imu_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Imu_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::Imu_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::Imu_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::Imu_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::Imu_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__Imu
    std::shared_ptr<common_interface::msg::Imu_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__Imu
    std::shared_ptr<common_interface::msg::Imu_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Imu_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->gyro_x != other.gyro_x) {
      return false;
    }
    if (this->gyro_y != other.gyro_y) {
      return false;
    }
    if (this->gyro_z != other.gyro_z) {
      return false;
    }
    if (this->accel_x != other.accel_x) {
      return false;
    }
    if (this->accel_y != other.accel_y) {
      return false;
    }
    if (this->accel_z != other.accel_z) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const Imu_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Imu_

// alias to use template instance with default allocator
using Imu =
  common_interface::msg::Imu_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__IMU__STRUCT_HPP_
