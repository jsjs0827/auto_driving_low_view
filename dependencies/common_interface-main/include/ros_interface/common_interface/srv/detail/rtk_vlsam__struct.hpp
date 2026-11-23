// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common_interface:srv/RTKVlsam.idl
// generated code does not contain a copyright notice

#ifndef COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__STRUCT_HPP_
#define COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__RTKVlsam_Request __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__RTKVlsam_Request __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RTKVlsam_Request_
{
  using Type = RTKVlsam_Request_<ContainerAllocator>;

  explicit RTKVlsam_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
      this->has_dock = false;
      this->origin_latitude = 0.0;
      this->origin_longitude = 0.0;
      this->origin_altitude = 0.0;
      this->map2enu_tx = 0.0;
      this->map2enu_ty = 0.0;
      this->map2enu_tz = 0.0;
      this->map2enu_qx = 0.0;
      this->map2enu_qy = 0.0;
      this->map2enu_qz = 0.0;
      this->map2enu_qw = 0.0;
    }
  }

  explicit RTKVlsam_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
      this->has_dock = false;
      this->origin_latitude = 0.0;
      this->origin_longitude = 0.0;
      this->origin_altitude = 0.0;
      this->map2enu_tx = 0.0;
      this->map2enu_ty = 0.0;
      this->map2enu_tz = 0.0;
      this->map2enu_qx = 0.0;
      this->map2enu_qy = 0.0;
      this->map2enu_qz = 0.0;
      this->map2enu_qw = 0.0;
    }
  }

  // field types and members
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _has_dock_type =
    bool;
  _has_dock_type has_dock;
  using _origin_latitude_type =
    double;
  _origin_latitude_type origin_latitude;
  using _origin_longitude_type =
    double;
  _origin_longitude_type origin_longitude;
  using _origin_altitude_type =
    double;
  _origin_altitude_type origin_altitude;
  using _map2enu_tx_type =
    double;
  _map2enu_tx_type map2enu_tx;
  using _map2enu_ty_type =
    double;
  _map2enu_ty_type map2enu_ty;
  using _map2enu_tz_type =
    double;
  _map2enu_tz_type map2enu_tz;
  using _map2enu_qx_type =
    double;
  _map2enu_qx_type map2enu_qx;
  using _map2enu_qy_type =
    double;
  _map2enu_qy_type map2enu_qy;
  using _map2enu_qz_type =
    double;
  _map2enu_qz_type map2enu_qz;
  using _map2enu_qw_type =
    double;
  _map2enu_qw_type map2enu_qw;

  // setters for named parameter idiom
  Type & set__cmd(
    const uint8_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__has_dock(
    const bool & _arg)
  {
    this->has_dock = _arg;
    return *this;
  }
  Type & set__origin_latitude(
    const double & _arg)
  {
    this->origin_latitude = _arg;
    return *this;
  }
  Type & set__origin_longitude(
    const double & _arg)
  {
    this->origin_longitude = _arg;
    return *this;
  }
  Type & set__origin_altitude(
    const double & _arg)
  {
    this->origin_altitude = _arg;
    return *this;
  }
  Type & set__map2enu_tx(
    const double & _arg)
  {
    this->map2enu_tx = _arg;
    return *this;
  }
  Type & set__map2enu_ty(
    const double & _arg)
  {
    this->map2enu_ty = _arg;
    return *this;
  }
  Type & set__map2enu_tz(
    const double & _arg)
  {
    this->map2enu_tz = _arg;
    return *this;
  }
  Type & set__map2enu_qx(
    const double & _arg)
  {
    this->map2enu_qx = _arg;
    return *this;
  }
  Type & set__map2enu_qy(
    const double & _arg)
  {
    this->map2enu_qy = _arg;
    return *this;
  }
  Type & set__map2enu_qz(
    const double & _arg)
  {
    this->map2enu_qz = _arg;
    return *this;
  }
  Type & set__map2enu_qw(
    const double & _arg)
  {
    this->map2enu_qw = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CMD_START =
    1u;
  static constexpr uint8_t CMD_STOP_AT_RETURN_ANCHOR =
    2u;
  static constexpr uint8_t CMD_MEMORIZE_BASE_STATION =
    3u;
  static constexpr uint8_t CMD_RESET =
    4u;

  // pointer types
  using RawPtr =
    common_interface::srv::RTKVlsam_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::RTKVlsam_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::RTKVlsam_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::RTKVlsam_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__RTKVlsam_Request
    std::shared_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__RTKVlsam_Request
    std::shared_ptr<common_interface::srv::RTKVlsam_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RTKVlsam_Request_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->has_dock != other.has_dock) {
      return false;
    }
    if (this->origin_latitude != other.origin_latitude) {
      return false;
    }
    if (this->origin_longitude != other.origin_longitude) {
      return false;
    }
    if (this->origin_altitude != other.origin_altitude) {
      return false;
    }
    if (this->map2enu_tx != other.map2enu_tx) {
      return false;
    }
    if (this->map2enu_ty != other.map2enu_ty) {
      return false;
    }
    if (this->map2enu_tz != other.map2enu_tz) {
      return false;
    }
    if (this->map2enu_qx != other.map2enu_qx) {
      return false;
    }
    if (this->map2enu_qy != other.map2enu_qy) {
      return false;
    }
    if (this->map2enu_qz != other.map2enu_qz) {
      return false;
    }
    if (this->map2enu_qw != other.map2enu_qw) {
      return false;
    }
    return true;
  }
  bool operator!=(const RTKVlsam_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RTKVlsam_Request_

// alias to use template instance with default allocator
using RTKVlsam_Request =
  common_interface::srv::RTKVlsam_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKVlsam_Request_<ContainerAllocator>::CMD_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKVlsam_Request_<ContainerAllocator>::CMD_STOP_AT_RETURN_ANCHOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKVlsam_Request_<ContainerAllocator>::CMD_MEMORIZE_BASE_STATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKVlsam_Request_<ContainerAllocator>::CMD_RESET;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace common_interface


#ifndef _WIN32
# define DEPRECATED__common_interface__srv__RTKVlsam_Response __attribute__((deprecated))
#else
# define DEPRECATED__common_interface__srv__RTKVlsam_Response __declspec(deprecated)
#endif

namespace common_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RTKVlsam_Response_
{
  using Type = RTKVlsam_Response_<ContainerAllocator>;

  explicit RTKVlsam_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->session_id = "";
      this->error_code = 0;
      this->message = "";
    }
  }

  explicit RTKVlsam_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : session_id(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->session_id = "";
      this->error_code = 0;
      this->message = "";
    }
  }

  // field types and members
  using _result_type =
    uint8_t;
  _result_type result;
  using _session_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _session_id_type session_id;
  using _error_code_type =
    uint16_t;
  _error_code_type error_code;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__session_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->session_id = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint16_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t RESULT_REJECTED =
    0u;
  static constexpr uint8_t RESULT_ACCEPTED =
    1u;
  static constexpr uint16_t ERROR_NONE =
    0u;

  // pointer types
  using RawPtr =
    common_interface::srv::RTKVlsam_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const common_interface::srv::RTKVlsam_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::RTKVlsam_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common_interface::srv::RTKVlsam_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common_interface__srv__RTKVlsam_Response
    std::shared_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common_interface__srv__RTKVlsam_Response
    std::shared_ptr<common_interface::srv::RTKVlsam_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RTKVlsam_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RTKVlsam_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RTKVlsam_Response_

// alias to use template instance with default allocator
using RTKVlsam_Response =
  common_interface::srv::RTKVlsam_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKVlsam_Response_<ContainerAllocator>::RESULT_REJECTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKVlsam_Response_<ContainerAllocator>::RESULT_ACCEPTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t RTKVlsam_Response_<ContainerAllocator>::ERROR_NONE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace common_interface

namespace common_interface
{

namespace srv
{

struct RTKVlsam
{
  using Request = common_interface::srv::RTKVlsam_Request;
  using Response = common_interface::srv::RTKVlsam_Response;
};

}  // namespace srv

}  // namespace common_interface

#endif  // COMMON_INTERFACE__SRV__DETAIL__RTK_VLSAM__STRUCT_HPP_
