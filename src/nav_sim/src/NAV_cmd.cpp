#include <algorithm>   // for std::max
#include <chrono>      // for durations
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
using namespace std::chrono;

class NAVcmdNode : public rclcpp::Node
{
public:
  NAVcmdNode()
  : Node("MAV_cmd")
  {
    // 參數宣告
    publish_hz_     = declare_parameter<double>("publish_hz",20.0);   // 預設 20Hz
    mode_run_value_ = declare_parameter<int>("mode_run_value",1);      // 達到此 Mode 才發目標速
    
    // Publisher
    NAVcmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/NAV_input", 10);

    // Subscriber
    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/Mode", 10,std::bind(&NAVcmdNode::ModeCH, this, _1));

    const double hz = std::max(1.0, publish_hz_);
    auto period = duration_cast<milliseconds>(duration<double>(1.0 / hz));
    timer_ = create_wall_timer(period, std::bind(&NAVcmdNode::onTimer, this));

    route_ = {
        {0.5, 0.0, 3.0},  
        {0.5, 0.5, 3.0},  
        {0.5, 0.0, 3.0}
    };
stage_start_ = this->now();
    RCLCPP_INFO(this->get_logger(), "joy_control node started.");
  }

private:
  /* ---------------- 回呼 ---------------- */
  void ModeCH(const std_msgs::msg::Int32::SharedPtr msg)
  {
    Mode = msg->data;
    RCLCPP_INFO(this->get_logger(), "Mode: %d\n", Mode);    
  }

  struct Segment { double lin; double ang; double dur; };
  std::vector<Segment> route_;     
  size_t                idx_ = 0;  
  rclcpp::Time          stage_start_; 
  void onTimer()
    {
        geometry_msgs::msg::Twist twist;
        if (Mode != mode_run_value_) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            NAVcmd_pub_->publish(twist);
        return;
        }
        if (idx_ >= route_.size()) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            NAVcmd_pub_->publish(twist);
        return;
        }
        // 取當前段
            const auto &seg = route_[idx_];
            twist.linear.x  = seg.lin;
            twist.angular.z = seg.ang;

        // 段落時間到就切下一段
        double elapsed = (this->now() - stage_start_).seconds();
        if (elapsed >= seg.dur) {
            idx_++;
            stage_start_ = this->now();
        }
        NAVcmd_pub_->publish(twist);
    }
  
  /* ---------------- 內部狀態 ---------------- */
  int Mode = 0;
  int mode_run_value_;
  double publish_hz_;

  /* ---------------- ROS 2 I/O ---------------- */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   NAVcmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr    mode_sub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

/* ============================== main ====================================== */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NAVcmdNode>());
  rclcpp::shutdown();
  return 0;
}

