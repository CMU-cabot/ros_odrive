/*******************************************************************************
 * Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

const double VEL_GAIN_DEFAULT            = 0.16;
const double VEL_INTEGRATOR_GAIN_DEFAULT = 0.33;

enum CmdId : uint32_t {
  kRxSdo = 0x004,
  kTxSdo = 0x005,
  kSetVelGains = 0x01b,
};

enum NodeId : uint32_t {
  kMotorLeft  = 0x00,
  kMotorRight = 0x01,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {

  rclcpp::Node::declare_parameter<std::string>("interface", "can0");
  rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);

  std::string vel_gain_param_name = "vel_gain";
  rclcpp::Node::declare_parameter<double>(vel_gain_param_name, VEL_GAIN_DEFAULT);
  vel_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  auto vel_gain_cb = [this](const rclcpp::Parameter & p) {
    double vel_gain = p.as_double();
    this->vel_gain_data_ = vel_gain;
    this->setVelGains()
  };
  vel_gain_cb_handle_ =
    vel_gain_subscriber_->add_parameter_callback(
        vel_gain_param_name,
        vel_gain_cb
        );

  std::string vel_integrator_gain_param_name = "vel_integrator_gain";
  rclcpp::Node::declare_parameter<double>(vel_integrator_gain_param_name, VEL_INTEGRATOR_GAIN_DEFAULT);
  vel_integrator_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  auto vel_integrator_gain_cb = [this](const rclcpp::Parameter & p) {
    double vel_integrator_gain = p.as_double();
    this->vel_integrator_gain_data_ = vel_integrator_gain;
    this->setVelGains()
  };
  vel_integrator_gain_cb_handle_ =
    vel_integrator_gain_subscriber_->add_parameter_callback(
        vel_integrator_gain_param_name,
        vel_integrator_gain_cb
        );
}

void ODriveCanNode::deinit() {
  can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {

  node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
  std::string interface = rclcpp::Node::get_parameter("interface").as_string();

  if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
    RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
  RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
  return true;
}

void ODriveCanNode::setVelGains() {
  struct can_frame frame;
  frame.can_id = node_id_ << 5 | CmdId::kSetVelGains;
  float gain_vals[2] = {vel_gain_data_,vel_integrator_gain_data_};
  {
    std::unique_lock<std::mutex> guard(gain_param_mutex_);
    write_le<uint32_t>(gain_vals, frame.data);
  }
  frame.can_dlc = 8;
  can_intf_.send_can_frame(frame);
}

void ODriveCanNode::recv_callback(const can_frame& frame) {

  if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

  switch(frame.can_id & 0x1F) {
    case CmdId::kTxSdo:
      if (!verify_length("kTxSdo", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      float vel_gain            = read_le<float>(frame.data + 0);
      float vel_integrator_gain = read_le<float>(frame.data + 4);
      RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d, vel_gain: %f, vel_integrator_gain: %f", node_id_, vel_gain, vel_integrator_gain);
      break;
    default:
      RCLCPP_ERROR(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
      break;
  }
}

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
  bool valid = expected == length;
  RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
  if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
  return valid;
}
