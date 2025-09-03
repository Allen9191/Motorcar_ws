#include <memory>
#include <cstdio>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "zlac8015d.h"
#include "Motorctl.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define FLIP -1

class ModeToZlac : public rclcpp::Node {
public:
  ModeToZlac()
  : Node("Mode_to_zlac")
  {
    // ----- 1. 初始化兩個馬達 -----
    RCLCPP_INFO(this->get_logger(), "Initializing ZLAC motors...");
    
    this->declare_parameter<std::string>("port_f", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("port_r", "/dev/ttyUSB1");
    std::string port_f = this->get_parameter("port_f").as_string();
    std::string port_r = this->get_parameter("port_r").as_string();

    RCLCPP_INFO(this->get_logger(), "port_f=%s", port_f.c_str());
    RCLCPP_INFO(this->get_logger(), "port_r=%s", port_r.c_str());
    
    motorF_.begin(port_f.c_str(), motor_ctl::Comms::BAUDRATE, motor_ctl::Comms::MOTOR_ID);
    motorR_.begin(port_r.c_str(), motor_ctl::Comms::BAUDRATE, motor_ctl::Comms::MOTOR_ID);

    motorF_.set_vel_mode();
    motorR_.set_vel_mode();
    motorF_.enable();
    motorR_.enable();
    RCLCPP_INFO(this->get_logger(), "ZLAC motors ready.");

    // ----- 2. 訂閱 /joy -----
    jcmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/joy_input", 10,std::bind(&ModeToZlac::joy_attitude, this, _1));
    NAVcmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/NAV_input", 10,std::bind(&ModeToZlac::NAV_attitude, this, _1));
 
    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/Mode", 10,std::bind(&ModeToZlac::ModeCH, this, _1));
      RCLCPP_INFO(this->get_logger(), "listener_node started.");
    axes_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wsad_values", 10,std::bind(&ModeToZlac::axes, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "listener_node started.");
    Motor_ = this-> create_wall_timer(50ms,std::bind(&ModeToZlac::ZLACoutput,this));
  }

  ~ModeToZlac() {
    RCLCPP_INFO(this->get_logger(), "Shutting down ZLAC motors...");
    motorF_.disable();
    motorR_.disable();
  }

private:
  double j_yaw_rate{0.0}, j_V_long{0.0};
  double N_yaw_rate{0.0}, N_V_long{0.0};
  int Mode{0};
  float ws{0};
  float ad{0};

  void joy_attitude(const geometry_msgs::msg::Twist::SharedPtr msg) {
    j_yaw_rate = msg->angular.z;
    j_V_long = msg->linear.x;

    RCLCPP_INFO(this->get_logger(), "JOY v=%.3f, yaw=%.3f\n", j_V_long, j_yaw_rate);
  }
  void NAV_attitude(const geometry_msgs::msg::Twist::SharedPtr msg) {
    N_yaw_rate = msg->angular.z;
    N_V_long = msg->linear.x;

    RCLCPP_INFO(this->get_logger(), "NAV v=%.3f, yaw=%.3f\n", N_V_long, N_yaw_rate);
  }
  void ModeCH(const std_msgs::msg::Int32::SharedPtr msg) {
    Mode = msg->data;
    RCLCPP_INFO(this->get_logger(), "Mode: %d\n", Mode);
  }
  void axes(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            ws = msg->data[0];
            ad = msg->data[1];
            RCLCPP_INFO(this->get_logger(), "Move ws=%.2f, ad=%.2f\n", ws, ad);
        }
    }

  void ZLACoutput(){
    
    double rpm_left = 0, rpm_right = 0;

    double j_V_set        = 0.0;
    double j_yawrate_set  = 0.0;
    
    double N_V_set        = motor_ctl::Control::Kv1 * N_V_long + motor_ctl::Control::Kv0;
    double N_yawrate_set  = motor_ctl::Control::Ky1 * N_yaw_rate + motor_ctl::Control::Ky0;
    double to_rpm = motor_ctl::Kinematics::to_RPM;
    
    if (std::abs(j_yaw_rate) < 0.1){
    	j_yawrate_set = 0;
    }else{
    	j_yawrate_set  = motor_ctl::Control::Ky1 * j_yaw_rate + motor_ctl::Control::Ky0;
    }	
    if (std::abs(j_V_long) < 0.1){
    	j_V_set = 0;
    }else{
    	j_V_set        = motor_ctl::Control::Kv1 * j_V_long + motor_ctl::Control::Kv0;
    }

    if (Mode == 99) {
        // B：緊急停止
        rpm_left  = 0;
        rpm_right = 0;
    } else if (Mode == 0){
        // A：導航，接收姿態
        rpm_left  = to_rpm * (N_V_set-(motor_ctl::Kinematics::W/2)*N_yawrate_set);
        rpm_right = -to_rpm * (N_V_set+(motor_ctl::Kinematics::W/2)*N_yawrate_set);
    } else if (Mode == 1) {
        // X：遙控
        rpm_left  = to_rpm*(j_V_set-(motor_ctl::Kinematics::W/2)*j_yawrate_set);
        rpm_right = -to_rpm*(j_V_set+(motor_ctl::Kinematics::W/2)*j_yawrate_set);
    } else if (Mode == 2) {
        // Y：定速
        if (ws == 1) {
        // w：前進
          rpm_left  = motor_ctl::Control::RPM_FWD;
          rpm_right = motor_ctl::Control::RPM_REV;
        } else if (ws == -1) {
          // s：後退
          rpm_left  = motor_ctl::Control::RPM_REV;
          rpm_right = motor_ctl::Control::RPM_FWD;
        } else if (ad == 1) {
          // a：左轉
          rpm_left  = motor_ctl::Control::RPM_TREV;
          rpm_right = motor_ctl::Control::RPM_TREV;
        } else if (ad == -1) {
          // d：右轉
          rpm_left  = motor_ctl::Control::RPM_TFWD;
          rpm_right = motor_ctl::Control::RPM_TFWD;
        }
    }
    RCLCPP_INFO(this->get_logger(), "L_rpm=%.3f, R_rpm=%.3f", rpm_left, rpm_right);

    motorF_.set_rpm(rpm_left,  "LEFT");
    motorF_.set_rpm(rpm_right, "RIGHT");
    motorR_.set_rpm(rpm_left,  "LEFT");
    motorR_.set_rpm(rpm_right, "RIGHT");

    motorF_.get_rpm();
    motorR_.get_rpm();

    RCLCPP_DEBUG(this->get_logger(),
        "Mode: ws=%.1f ad=%.1f rpm_L=%.3f rpm_R=%.3f", ws, ad, rpm_left, rpm_right);
  }

  ZLAC motorF_, motorR_;
  //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr jcmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr NAVcmd_sub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr      mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr axes_sub_;
  rclcpp::TimerBase::SharedPtr Motor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModeToZlac>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
