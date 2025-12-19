// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from solution_interfaces:srv/Assign.idl
// generated code does not contain a copyright notice

#ifndef SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__BUILDER_HPP_
#define SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "solution_interfaces/srv/detail/assign__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace solution_interfaces
{

namespace srv
{

namespace builder
{

class Init_Assign_Request_role
{
public:
  explicit Init_Assign_Request_role(::solution_interfaces::srv::Assign_Request & msg)
  : msg_(msg)
  {}
  ::solution_interfaces::srv::Assign_Request role(::solution_interfaces::srv::Assign_Request::_role_type arg)
  {
    msg_.role = std::move(arg);
    return std::move(msg_);
  }

private:
  ::solution_interfaces::srv::Assign_Request msg_;
};

class Init_Assign_Request_drone_name
{
public:
  Init_Assign_Request_drone_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Assign_Request_role drone_name(::solution_interfaces::srv::Assign_Request::_drone_name_type arg)
  {
    msg_.drone_name = std::move(arg);
    return Init_Assign_Request_role(msg_);
  }

private:
  ::solution_interfaces::srv::Assign_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::solution_interfaces::srv::Assign_Request>()
{
  return solution_interfaces::srv::builder::Init_Assign_Request_drone_name();
}

}  // namespace solution_interfaces


namespace solution_interfaces
{

namespace srv
{

namespace builder
{

class Init_Assign_Response_success
{
public:
  Init_Assign_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::solution_interfaces::srv::Assign_Response success(::solution_interfaces::srv::Assign_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::solution_interfaces::srv::Assign_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::solution_interfaces::srv::Assign_Response>()
{
  return solution_interfaces::srv::builder::Init_Assign_Response_success();
}

}  // namespace solution_interfaces

#endif  // SOLUTION_INTERFACES__SRV__DETAIL__ASSIGN__BUILDER_HPP_
