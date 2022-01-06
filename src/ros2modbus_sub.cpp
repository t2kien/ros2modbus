// Include the C++ standard library headers
#include <memory> // Dynamic memory management
 
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
#include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
#include "geometry_msgs/msg/twist.hpp" // Handles twist messages in ROS 2

//modbuscpp
//https://github.com/fanzhe98/modbuspp/blob/master/modbus.h
#include "ros2modbus/modbus.h"

using std::placeholders::_1;

class ModbusSubscriber : public rclcpp::Node
{
  public:
    // Constructor
    // The name of the node is minimal_subscriber
    ModbusSubscriber() 
    : Node("sub_ros2modbus")
    {
      //init modbus
      ID=16;
      SpeedAddress = 5122; 
      Enable_Adr = 5125; //D1029
      hs_tang = 10;
      hs_lai =5;
      Limit_Speed = 500; 
     
      
      mb.modbus_set_slave_id(ID);
      // connect with the server
      mb.modbus_connect();
      //enable AGV
      uint16_t status;
      status = Write_Enable_AGV(1);
      if (status == BAD_CON)
          RCLCPP_INFO(this->get_logger(), "AGV-Enable: Could not connect to device");
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ModbusSubscriber::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Node Running");

    }
    ~ModbusSubscriber() 
    {
       //disable AGV
       uint16_t status;
       status = Write_Enable_AGV(0);
       if (status == BAD_CON)
          RCLCPP_INFO(this->get_logger(), "AGV-Disable: Could not connect to device");
    }
  private:
    // Receives the String message that is published over the topic
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) //const    
    {
    
      uint16_t status;
      uint16_t Start;
      uint16_t LeftSpeed;
      uint16_t RightSpeed;
      
      // Write the message that was received on the console window
      RCLCPP_INFO(this->get_logger(), "I heard velocity command: linear: (%f,%f,%f), angular: (%f,%f,%f)", msg->linear.x,msg->linear.y,msg->linear.z, msg->angular.x,msg->angular.y,msg->angular.z);
      
      //http://wiki.ros.org/simple_drive#simple_drive
      //Twist to Tank Calculation
      //Left wheel velocity (m/s) = linear_speed + angular_speed
      //Right wheel velocity (m/s) = linear_speed - angular_speed
      LeftSpeed = msg->linear.x + msg->angular.z;
      RightSpeed = msg->linear.x - msg->angular.z;
      Start =1;
      if (LeftSpeed > Limit_Speed)
	   LeftSpeed = Limit_Speed;
      if (LeftSpeed < -Limit_Speed)
           LeftSpeed = -1*Limit_Speed;
      if (RightSpeed > Limit_Speed)
           RightSpeed = Limit_Speed;
      if (RightSpeed < -Limit_Speed)
           RightSpeed = -1 * Limit_Speed;
                
      status = Write_Speed_AGV(LeftSpeed, RightSpeed, Start);
      
      if (status == BAD_CON)
          RCLCPP_INFO(this->get_logger(), "Could not connect to device");
          
    }
    
    uint16_t Write_Speed_AGV(uint16_t leftSp, uint16_t rightSp, uint16_t Start) 
    {
        uint16_t tbl_data[3] = {leftSp, rightSp, Start};//start=1 AGV Run; =0 AGV Stop
        uint16_t status;
        status = mb.modbus_write_registers(SpeedAddress, 3, tbl_data);
        return status;
    }
    //Enable AGV
    uint16_t Write_Enable_AGV(uint16_t AGV_Enable) 
    {
        uint16_t status;
        status =mb.modbus_write_register(Enable_Adr, AGV_Enable);
        return status;
    }
        
    // Declare the subscription attribute
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    modbus mb = modbus("192.168.1.100",502);
    uint16_t SpeedAddress;
    uint16_t Enable_Adr; //D1029
    uint16_t hs_tang;
    uint16_t hs_lai;
    uint16_t Limit_Speed;
    
    uint16_t ID;
};
 
int main(int argc, char * argv[])
{
  // Launch ROS 2
  rclcpp::init(argc, argv);
   
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<ModbusSubscriber>());
   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}
