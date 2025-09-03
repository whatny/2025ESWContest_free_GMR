#ifndef dynamixel_control_HPP_
#define dynamixel_control_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

class ReadWriteNode : public rclcpp::Node
{
public:
  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  int present_position;
};

#endif  // dynamixel_control_HPP_
