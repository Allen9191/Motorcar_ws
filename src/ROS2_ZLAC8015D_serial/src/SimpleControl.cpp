#include <memory>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "zlac8015d.h"

#define FLIP -1

using std::placeholders::_1;

class JoyToZlac : public rclcpp::Node {
public:
  JoyToZlac()
  : Node("joy_to_zlac")
  {
    // ----- 1. 初始化兩個馬達 -----
    RCLCPP_INFO(this->get_logger(), "Initializing ZLAC motors...");
    motorF_.begin("/dev/ttyUSB0", 115200, 0x01);
    motorR_.begin("/dev/ttyUSB4", 115200, 0x01);

    motorF_.set_vel_mode();
    motorR_.set_vel_mode();
    motorF_.enable();
    motorR_.enable();
    RCLCPP_INFO(this->get_logger(), "ZLAC motors ready.");

    // ----- 2. 訂閱 /joy -----
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyToZlac::joy_callback, this, _1));
  }

  ~JoyToZlac() {
    RCLCPP_INFO(this->get_logger(), "Shutting down ZLAC motors...");
    motorF_.disable();
    motorR_.disable();
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    // 讀 WASD：joy->axes[0] = w/s，joy->axes[1] = a/d
    double ws = joy->axes.size() > 1 ? joy->axes[1] : 0.0;  // +1=w, -1=s
    double ad = joy->axes.size() > 2 ? joy->axes[2] : 0.0;  // +1=a, -1=d

    // 定義各模式的 RPM
    const int RPM_FWD   = 50;   // w：前進
    const int RPM_REV   = -50;  // s：後退
    const int RPM_TFWD  = 30;    // a：原地左轉
    const int RPM_TREV  = -30;   // d：原地右轉

    int rpm_left = 0, rpm_right = 0;

    if (ws > 0.5) {
        // w：前進
        rpm_left  = RPM_FWD;
        rpm_right = RPM_REV;
    } else if (ws < -0.5) {
        // s：後退
        rpm_left  = RPM_REV;
        rpm_right = RPM_FWD;
    } else if (ad > 0.5) {
        // a：左轉（左輪反轉，右輪正轉）
        rpm_left  = RPM_TREV;
        rpm_right = RPM_TREV;
    } else if (ad < -0.5) {
        // d：右轉（左輪正轉，右輪反轉）
        rpm_left  = RPM_TFWD;
        rpm_right = RPM_TFWD;
    }
    // 其他情況：保持停止（rpm_left=0, rpm_right=0）

    // 呼叫 ZLAC API
    motorF_.set_rpm(rpm_left,  "LEFT");
    motorF_.set_rpm(rpm_right, "RIGHT");
    motorR_.set_rpm(rpm_left,  "LEFT");
    motorR_.set_rpm(rpm_right, "RIGHT");
    
    motorF_.get_rpm();
    motorR_.get_rpm();

    RCLCPP_DEBUG(this->get_logger(),
        "Mode: ws=%.1f ad=%.1f → rpm_L=%d rpm_R=%d",
        ws, ad, rpm_left, rpm_right);
  }

  ZLAC motorF_, motorR_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToZlac>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
