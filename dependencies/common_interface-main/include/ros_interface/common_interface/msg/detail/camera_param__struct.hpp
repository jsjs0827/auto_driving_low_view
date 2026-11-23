// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:msg/CameraParam.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__STRUCT_HPP_
#define COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__msg__CameraParam __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__msg__CameraParam __declspec(deprecated)
#endif

namespace common_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraParam_
{
  using Type = CameraParam_<ContainerAllocator>;

  explicit CameraParam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->cx = 0.0f;
      this->cy = 0.0f;
      std::fill<typename std::array<float, 6>::iterator, float>(this->radial_distortion_k.begin(), this->radial_distortion_k.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->tangential_distortion_p.begin(), this->tangential_distortion_p.end(), 0.0f);
      this->bxf = 0.0;
      this->base_line = 0.0;
      this->width = 0l;
      this->height = 0l;
      this->dsp_max_value = 0l;
    }
  }

  explicit CameraParam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : radial_distortion_k(_alloc),
    tangential_distortion_p(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->cx = 0.0f;
      this->cy = 0.0f;
      std::fill<typename std::array<float, 6>::iterator, float>(this->radial_distortion_k.begin(), this->radial_distortion_k.end(), 0.0f);
      std::fill<typename std::array<float, 2>::iterator, float>(this->tangential_distortion_p.begin(), this->tangential_distortion_p.end(), 0.0f);
      this->bxf = 0.0;
      this->base_line = 0.0;
      this->width = 0l;
      this->height = 0l;
      this->dsp_max_value = 0l;
    }
  }

  // field types and members
  using _fx_type =
    float;
  _fx_type fx;
  using _fy_type =
    float;
  _fy_type fy;
  using _cx_type =
    float;
  _cx_type cx;
  using _cy_type =
    float;
  _cy_type cy;
  using _radial_distortion_k_type =
    std::array<float, 6>;
  _radial_distortion_k_type radial_distortion_k;
  using _tangential_distortion_p_type =
    std::array<float, 2>;
  _tangential_distortion_p_type tangential_distortion_p;
  using _bxf_type =
    double;
  _bxf_type bxf;
  using _base_line_type =
    double;
  _base_line_type base_line;
  using _width_type =
    int32_t;
  _width_type width;
  using _height_type =
    int32_t;
  _height_type height;
  using _dsp_max_value_type =
    int32_t;
  _dsp_max_value_type dsp_max_value;

  // setters for named parameter idiom
  Type & set__fx(
    const float & _arg)
  {
    this->fx = _arg;
    return *this;
  }
  Type & set__fy(
    const float & _arg)
  {
    this->fy = _arg;
    return *this;
  }
  Type & set__cx(
    const float & _arg)
  {
    this->cx = _arg;
    return *this;
  }
  Type & set__cy(
    const float & _arg)
  {
    this->cy = _arg;
    return *this;
  }
  Type & set__radial_distortion_k(
    const std::array<float, 6> & _arg)
  {
    this->radial_distortion_k = _arg;
    return *this;
  }
  Type & set__tangential_distortion_p(
    const std::array<float, 2> & _arg)
  {
    this->tangential_distortion_p = _arg;
    return *this;
  }
  Type & set__bxf(
    const double & _arg)
  {
    this->bxf = _arg;
    return *this;
  }
  Type & set__base_line(
    const double & _arg)
  {
    this->base_line = _arg;
    return *this;
  }
  Type & set__width(
    const int32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__dsp_max_value(
    const int32_t & _arg)
  {
    this->dsp_max_value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common_interface::msg::CameraParam_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::msg::CameraParam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::msg::CameraParam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::msg::CameraParam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::CameraParam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::CameraParam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::msg::CameraParam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::msg::CameraParam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::msg::CameraParam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::msg::CameraParam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__msg__CameraParam
    std::shared_ptr<common_interface::msg::CameraParam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__msg__CameraParam
    std::shared_ptr<common_interface::msg::CameraParam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraParam_ & other) const
  {
    if (this->fx != other.fx) {
      return false;
    }
    if (this->fy != other.fy) {
      return false;
    }
    if (this->cx != other.cx) {
      return false;
    }
    if (this->cy != other.cy) {
      return false;
    }
    if (this->radial_distortion_k != other.radial_distortion_k) {
      return false;
    }
    if (this->tangential_distortion_p != other.tangential_distortion_p) {
      return false;
    }
    if (this->bxf != other.bxf) {
      return false;
    }
    if (this->base_line != other.base_line) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->dsp_max_value != other.dsp_max_value) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraParam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraParam_

// alias to use template instance with default allocator
using CameraParam =
  common_interface::msg::CameraParam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common_interface

#endif  // COMMON_INTERFACE__MSG__DETAIL__CAMERA_PARAM__STRUCT_HPP_
