#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "arbitrary_parameter.hpp"

//#include <fstream>
//#include <nlohmann/json.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;

using AxisState = odrive_can::srv::AxisState;

class ODriveCanNode : public rclcpp::Node {
public:
    ODriveCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop); 
    void deinit();
private:
    void recv_callback(const can_frame& frame);
    void subscriber_callback(const ControlMessage::SharedPtr msg);
    void service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response);
    void request_state_callback();
    void ctrl_msg_callback();
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);

    void set_vel_gains();
    bool is_gain_correct();

    template <typename T>
      void get_arbitrary_parameter(uint16_t endpoint_id, T &output_val);
    template <typename T>
      void set_arbitrary_parameter(uint16_t endpoint_id, T val);
    //std::string ODriveCanNode::get_parameter_name_from_number(int16_t endpoint_id);
    
    uint16_t node_id_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    
    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;
    
    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;

    //EpollEvent srv_evt_;
    //uint32_t axis_state_;
    //std::mutex axis_state_mutex_;
    //std::condition_variable fresh_heartbeat_;
    //rclcpp::Service<AxisState>::SharedPtr service_;

    float vel_gain_;
    //float vel_gain_actual_;
    //std::mutex vel_gain_actual_mutex_;
    //std::condition_variable fresh_vel_gain_actual_;
    //bool vel_gain_actual_ready_ = false;

    float vel_integrator_gain_;
    //float vel_integrator_gain_actual_;
    //std::mutex vel_integrator_gain_actual_mutex_;
    //std::condition_variable fresh_vel_integrator_gain_actual_;
    //bool vel_integrator_gain_actual_ready_ = false;

    std::shared_ptr<rclcpp::ParameterEventHandler> vel_gain_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> vel_gain_cb_handle_;
    std::shared_ptr<rclcpp::ParameterEventHandler> vel_integrator_gain_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> vel_integrator_gain_cb_handle_;

    ArbitraryParameter params_;

};

#endif // ODRIVE_CAN_NODE_HPP
