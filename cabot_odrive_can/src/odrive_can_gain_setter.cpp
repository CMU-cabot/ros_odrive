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

#include "odrive_can_gain_setter.hpp"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

const double VEL_GAIN_DEFAULT            = 0.16;
const double VEL_INTEGRATOR_GAIN_DEFAULT = 0.33;

enum CmdId : uint32_t {
  kHeartbeat   = 0x001,
  kRxSdo       = 0x004,
  kTxSdo       = 0x005,
  kGetEncoderEstimates = 0x009,
  kSetVelGains = 0x01b,
};

enum NodeId : uint32_t {
  kMotorLeft  = 0x00,
  kMotorRight = 0x01,
};

enum OpcodeId : uint8_t {
  kRead  = 0x00,
  kWrite = 0x01,
};

enum EndpointId : uint16_t {
  kVelGain = 391,
  kVelIntegratorGain = 392,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {

  rclcpp::Node::declare_parameter<std::string>("interface", "can0");
  rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);

  // initiate parameter callback of "vel_gain"
  std::string vel_gain_param_name = "vel_gain";
  rclcpp::Node::declare_parameter<double>(vel_gain_param_name, VEL_GAIN_DEFAULT);
  vel_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  std::function<void(const rclcpp::Parameter&)> vel_gain_cb = 
    [this](const rclcpp::Parameter & p) {
      double vel_gain = p.as_double();
      this->vel_gain_ = vel_gain;
      while(true) {
        this->set_vel_gains();
        if(this->is_gain_correct()) break;
      }
    };
  vel_gain_cb_handle_ =
    vel_gain_subscriber_->add_parameter_callback(
        vel_gain_param_name,
        vel_gain_cb
        );

  // initiate parameter callback of "vel_integrator_gain"
  std::string vel_integrator_gain_param_name = "vel_integrator_gain";
  rclcpp::Node::declare_parameter<double>(vel_integrator_gain_param_name, VEL_INTEGRATOR_GAIN_DEFAULT);
  vel_integrator_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  std::function<void(const rclcpp::Parameter&)> vel_integrator_gain_cb =
    [this](const rclcpp::Parameter & p) {
      double vel_integrator_gain = p.as_double();
      this->vel_integrator_gain_ = vel_integrator_gain;
      while(true) {
        this->set_vel_gains();
        if(this->is_gain_correct()) break;
      }
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

void ODriveCanNode::set_vel_gains() {
  struct can_frame frame;
  frame.can_id = node_id_ << 5 | CmdId::kSetVelGains;
  {
    std::unique_lock<std::mutex> guard(gain_param_mutex_);
    write_le<float>(vel_gain_,            frame.data + 0);
    write_le<float>(vel_integrator_gain_, frame.data + 4);
  }
  frame.can_dlc = 8;
  can_intf_.send_can_frame(frame);
}

void ODriveCanNode::recv_callback(const can_frame& frame) {

  if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

  switch(frame.can_id & 0x1F) {
    case CmdId::kTxSdo:
      {
        if (!verify_length("kTxSdo", 8, frame.can_dlc)) break;
        std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
        int reserved0    = read_le<uint8_t>(frame.data + 0);
        int endpoint_id = read_le<uint16_t>(frame.data + 1);
        int reserved1    = read_le<uint8_t>(frame.data + 3);
        float val          = read_le<float>(frame.data + 4);
        switch(endpoint_id) {
          case EndpointId::kVelGain:
            vel_gain_actual_ = val;
            is_actual_vel_gain_received_ = true;
            RCLCPP_INFO(rclcpp::Node::get_logger(),
                "recv_callback: node_id: %d, reserved0: %d, endpoint_id: %d,"
                " reserved1: %d, vel_gain: %f",
                node_id_, reserved0, endpoint_id, reserved1, vel_gain_actual_);
            break;
          case EndpointId::kVelIntegratorGain:
            vel_integrator_gain_actual_ = val;
            is_actual_vel_integrator_gain_received_ = true;
            RCLCPP_INFO(rclcpp::Node::get_logger(),
                "recv_callback: node_id: %d, reserved0: %d, endpoint_id: %d,"
                " reserved1: %d, vel_integrator_gain: %f",
                node_id_, reserved0, endpoint_id, reserved1, vel_integrator_gain_actual_);
            break;
          default:
            RCLCPP_ERROR(rclcpp::Node::get_logger(),
                "recv_callback: endpoint_id %d does not exist or callback "
                "process of the endpoint_id is not implemented", endpoint_id);
            break;
        }
        break;
      }
    case CmdId::kHeartbeat:
    case CmdId::kGetEncoderEstimates:
      break;
    default:
      RCLCPP_ERROR(rclcpp::Node::get_logger(),
          "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
      break;
  }
}

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
  bool valid = expected == length;
  RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
  if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
  return valid;
}

bool ODriveCanNode::is_gain_correct() {
  float vel_gain_actual            = get_arbitrary_parameter(EndpointId::kVelGain);
  float vel_integrator_gain_actual = get_arbitrary_parameter(EndpointId::kVelIntegratorGain);
  bool is_vel_gain_correct = vel_gain_ == vel_gain_actual;
  bool is_vel_integrator_gain_correct = vel_integrator_gain_ == vel_integrator_gain_actual;
  return is_vel_gain_correct & is_vel_integrator_gain_correct;
}

float ODriveCanNode::get_arbitrary_parameter(uint16_t endpoint_id) {
  struct can_frame frame;
  frame.can_id = node_id_ << 5 | CmdId::kRxSdo;
  
  uint8_t reserved = 0;
  frame.can_dlc = 4;
  {
    std::unique_lock<std::mutex> guard(gain_param_mutex_);
    write_le<uint8_t>(OpcodeId::kRead, frame.data + 0);
    write_le<uint16_t>(endpoint_id,    frame.data + 1);
    write_le<uint8_t>(reserved,        frame.data + 3);
  }
  can_intf_.send_can_frame(frame);

  float val = 0;
  switch(endpoint_id) {
    case EndpointId::kVelGain:
      while(!is_actual_vel_gain_received_);
      val = vel_gain_actual_;
      is_actual_vel_gain_received_ = false;
      break;
    case EndpointId::kVelIntegratorGain:
      while(!is_actual_vel_integrator_gain_received_);
      val = vel_integrator_gain_actual_;
      is_actual_vel_integrator_gain_received_ = false;
      break;
    default:
      RCLCPP_ERROR(rclcpp::Node::get_logger(), "invalid endpoint_id %d", endpoint_id);
      break;
  }
  return val;
}
