#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

//#include "std_msgs/msg/bool.hpp"
//#include "std_msgs/msg/int8.hpp"
//#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class JoyControlNode : public rclcpp::Node
{
public:
  JoyControlNode()
  : Node("joy_control")
  {
    // 參數宣告（若外部未給值，就用預設）
    linear_axis_  = this->declare_parameter<int>("axis_linear", 1);
    angular_axis_ = this->declare_parameter<int>("axis_angular", 0);
    l_scale_      = this->declare_parameter<double>("scale_linear", 1.5);
    a_scale_      = this->declare_parameter<double>("scale_angular", 1.5);
    //l_scale_plus_ = this->declare_parameter<double>("scale_linear_plus", 1.0);
    //a_scale_plus_ = this->declare_parameter<double>("scale_angular_plus", 1.0);

    // Publisher
    vel_pub_            = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/joy_input", 10);
    mode_pub_           = this->create_publisher<std_msgs::msg::Int32>("/Mode", 10);
    axes_pub_           = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wsad_values", 10);

    //elevator_arrived_pub_= this->create_publisher<std_msgs::msg::Bool>("/elevator", 10);
    //floor_pub_          = this->create_publisher<std_msgs::msg::String>("/Floor", 10);
    //arrive_pub_         = this->create_publisher<std_msgs::msg::Bool>("/arrive", 10);
    //container_pub_      = this->create_publisher<std_msgs::msg::Int8>("/open_container", 10);
    //unlock_pub_         = this->create_publisher<std_msgs::msg::String>("/unlock", 10);

    // Subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyControlNode::joy_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "joy_control node started.");
  }

private:
  /* ---------------- 回呼 ---------------- */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    // 讀取 ws (index 1) 和 ad (index 2)，防止越界
    double x = joy->axes.size() > 7 ? joy->axes[7] : 0.0;  // +1=w, -1=s
    double y = joy->axes.size() > 6 ? joy->axes[6] : 0.0;  // +1=a, -1=d

    std_msgs::msg::Float32MultiArray msg;
    msg.data = { static_cast<float>(x), static_cast<float>(y)};
    axes_pub_->publish(msg);

    /* 1. 速度控制 --------------------------------------------------------- */
    geometry_msgs::msg::Twist twist;
    if (joy->buttons[4]) {                       //L1：加速
      twist.angular.z = a_scale_ * joy->axes[angular_axis_];
      twist.linear.x  = l_scale_ * joy->axes[linear_axis_];
    } else {                                    // 一般模式
      twist.angular.z = 0.8 * joy->axes[angular_axis_];
      twist.linear.x  = 0.8 * joy->axes[linear_axis_];
    }
    vel_pub_->publish(twist);

    /* 2. 模式切換 --------------------------------------------------------- */
    std_msgs::msg::Int32 mode;
    if (joy->buttons[1]) {          // B：緊急停止
      mode.data = 99;
      mode_pub_->publish(mode);
    } else if (joy->buttons[0]) {   // A：導航
      mode.data = 0;
      mode_pub_->publish(mode);
    } else if (joy->buttons[2]) {   // X：遙控
      mode.data = 1;
      mode_pub_->publish(mode);
    } else if (joy->buttons[3]) {   // Y：定速
      mode.data = 2;
      mode_pub_->publish(mode);
    }

    
    /* 3. 解鎖 / 開艙門 ---------------------------------------------------- 
    if (joy->buttons[5]) {                       // R1：解鎖
      unlock_state_ = (unlock_state_ == -1) ? 0 : -1;
      std_msgs::msg::String msg;
      msg.data = std::to_string(unlock_state_);
      unlock_pub_->publish(msg);
    }

    if (joy->buttons[6]) {                       // SELECT：艙門
      container_state_ = (container_state_ == -1) ? 0 : -1;
      std_msgs::msg::Int8 msg;
      msg.data = container_state_;
      container_pub_->publish(msg);
    }
    */

    /* 4. 其他功能（floor/elevator 等）可依需求補上 ------------------------ */
  }

  /* ---------------- 內部狀態 ---------------- */
  int  linear_axis_, angular_axis_;
  double l_scale_, a_scale_;
  //l_scale_plus_, a_scale_plus_;
  //int  container_state_{0}, unlock_state_{0};

  /* ---------------- ROS 2 I/O ---------------- */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr       mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       axes_pub_;  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_sub_;
  //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        elevator_arrived_pub_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      floor_pub_;
  //rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        arrive_pub_;
  //rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr        container_pub_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      unlock_pub_;
};

/* ============================== main ====================================== */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyControlNode>());
  rclcpp::shutdown();
  return 0;
}

