// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from solution_interfaces:srv/Assign.idl
// generated code does not contain a copyright notice

#ifndef SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__TRAITS_HPP_
#define SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "solution_interfaces/srv/detail/assign__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace solution_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Assign_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: drone_name
  {
    out << "drone_name: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_name, out);
    out << ", ";
  }

  // member: role
  {
    out << "role: ";
    rosidl_generator_traits::value_to_yaml(msg.role, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Assign_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drone_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drone_name: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_name, out);
    out << "\n";
  }

  // member: role
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "role: ";
    rosidl_generator_traits::value_to_yaml(msg.role, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Assign_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace solution_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use solution_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const solution_interfaces::srv::Assign_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  solution_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use solution_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const solution_interfaces::srv::Assign_Request & msg)
{
  return solution_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<solution_interfaces::srv::Assign_Request>()
{
  return "solution_interfaces::srv::Assign_Request";
}

template<>
inline const char * name<solution_interfaces::srv::Assign_Request>()
{
  return "solution_interfaces/srv/Assign_Request";
}

template<>
struct has_fixed_size<solution_interfaces::srv::Assign_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<solution_interfaces::srv::Assign_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<solution_interfaces::srv::Assign_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace solution_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Assign_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Assign_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Assign_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace solution_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use solution_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const solution_interfaces::srv::Assign_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  solution_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use solution_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const solution_interfaces::srv::Assign_Response & msg)
{
  return solution_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<solution_interfaces::srv::Assign_Response>()
{
  return "solution_interfaces::srv::Assign_Response";
}

template<>
inline const char * name<solution_interfaces::srv::Assign_Response>()
{
  return "solution_interfaces/srv/Assign_Response";
}

template<>
struct has_fixed_size<solution_interfaces::srv::Assign_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<solution_interfaces::srv::Assign_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<solution_interfaces::srv::Assign_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<solution_interfaces::srv::Assign>()
{
  return "solution_interfaces::srv::Assign";
}

template<>
inline const char * name<solution_interfaces::srv::Assign>()
{
  return "solution_interfaces/srv/Assign";
}

template<>
struct has_fixed_size<solution_interfaces::srv::Assign>
  : std::integral_constant<
    bool,
    has_fixed_size<solution_interfaces::srv::Assign_Request>::value &&
    has_fixed_size<solution_interfaces::srv::Assign_Response>::value
  >
{
};

template<>
struct has_bounded_size<solution_interfaces::srv::Assign>
  : std::integral_constant<
    bool,
    has_bounded_size<solution_interfaces::srv::Assign_Request>::value &&
    has_bounded_size<solution_interfaces::srv::Assign_Response>::value
  >
{
};

template<>
struct is_service<solution_interfaces::srv::Assign>
  : std::true_type
{
};

template<>
struct is_service_request<solution_interfaces::srv::Assign_Request>
  : std::true_type
{
};

template<>
struct is_service_response<solution_interfaces::srv::Assign_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__TRAITS_HPP_
