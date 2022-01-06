// Include the C++ standard library headers
#include <memory> // Dynamic memory management
 #include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <string> // String functions
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
#include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
#include "geometry_msgs/msg/twist.hpp" // Handles twist messages in ROS 2

using std::placeholders::_1;
using namespace std::chrono_literals;

class ModbusPublisher : public rclcpp::Node
{
  public:
    // Constructor
    // The name of the node is minimal_subscriber
    ModbusPublisher() 
    : Node("pub_ros2modbus"), count_(0)
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

      RCLCPP_INFO(this->get_logger(), "Node Running");
      
      timer_ = this->create_wall_timer(500ms, std::bind(&ModbusPublisher::timer_callback, this));
    }
 
  private:
    void timer_callback()
    {
      // Create a new message of type String
      auto cmd_vel_msg = geometry_msgs::msg::Twist();

      cmd_vel_msg.linear.x = count_++;
      cmd_vel_msg.linear.y = count_++;
      cmd_vel_msg.linear.z = count_++;

      cmd_vel_msg.angular.x = count_--;
      cmd_vel_msg.angular.y = count_--;
      cmd_vel_msg.angular.z = count_--;
      
      // Print every message to the terminal window      
      RCLCPP_INFO(this->get_logger(),"Publishing: linear: (%f,%f,%f), angular: (%f,%f,%f)", cmd_vel_msg.linear.x,cmd_vel_msg.linear.y,cmd_vel_msg.linear.z, cmd_vel_msg.angular.x,cmd_vel_msg.angular.y,cmd_vel_msg.angular.z);
       
      // Publish the message to the topic named "addison"
      publisher_->publish(cmd_vel_msg);
    }
     
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
  
    // Declaration of the publisher_ attribute
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
   
    // Declaration of the count_ attribute
    size_t count_;
    
};
 
int main(int argc, char * argv[])
{
  // Launch ROS 2
  rclcpp::init(argc, argv);
   
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<ModbusPublisher>());
   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}
