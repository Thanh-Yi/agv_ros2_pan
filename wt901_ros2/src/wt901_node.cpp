#include "rclcpp/rclcpp.hpp"

#include <wt901/wt901_driver.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class WT901Node : public rclcpp::Node
{
public:
  WT901Node() : Node("wt901_node")
  {
    /* ===== init IMU driver ===== */
    if (wt901_init("/dev/ch34x_imu", 115200) != 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to init WT901 IMU");
      rclcpp::shutdown();
      return;
    }
    
    /* ===== publisher ===== */
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", 100);

    /* ===== timer ===== */
    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&WT901Node::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "WT901 IMU node started");
  }

private:
  void timer_callback()
  {
    /* step driver (1 vòng main.c) */
    wt901_step();

    float roll_deg, pitch_deg, yaw_deg;
    if (!wt901_get_angle(&roll_deg, &pitch_deg, &yaw_deg))
      return;

    /* ===== degree -> rad ===== */
    double roll  = roll_deg  * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw   = yaw_deg   * M_PI / 180.0;

    /* ===== quaternion ===== */
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    sensor_msgs::msg::Imu imu_msg;

    /* ===== header ===== */
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu";

    /* ===== orientation (EKF DÙNG) ===== */
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    /* ===== angular velocity (chưa dùng) ===== */
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    /* ===== linear acceleration (chưa dùng) ===== */
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    /* ===== covariance (RẤT QUAN TRỌNG CHO EKF) ===== */

    // orientation covariance
    imu_msg.orientation_covariance = {
      0.02, 0.0,  0.0,
      0.0,  0.02, 0.0,
      0.0,  0.0,  0.05
    };

    // gyro & acc chưa dùng
    imu_msg.angular_velocity_covariance[0] = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;

    /* ===== publish ===== */
    imu_pub_->publish(imu_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WT901Node>());
  rclcpp::shutdown();
  return 0;
}
