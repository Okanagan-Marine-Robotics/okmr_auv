#ifndef TURNCW_H
#define TURNCW_H

#include "MovementBaseClass.h"  
#include <okmr_msgs/msg/movement_command.hpp>  
#include <geometry_msgs/msg/pose_stamped.hpp>  

class TurnCW : public MovementBaseClass {
public:
    TurnCW(const std::string& name, const BT::NodeConfiguration& config);

//static BT::PortsList providedPorts() {
   //     return {};
   // }



void setCommandData(okmr_msgs::msg::MovementCommand &msg) override;

    BT::NodeStatus tick() override;

  virtual ~TurnCW() = default;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<okmr_msgs::msg::MovementCommand>::SharedPtr publisher_;
    rclcpp::Client<okmr_msgs::srv::Status>::SharedPtr client_;

    bool message_sent_;
    int32_t command_; 


}; 

#endif //TURNCW_H
