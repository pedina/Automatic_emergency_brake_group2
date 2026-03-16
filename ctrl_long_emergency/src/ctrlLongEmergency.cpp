#include "ctrl_long_emergency/ctrlLongEmergency.hpp"


cle::CtrlLongEmergency::CtrlLongEmergency() : Node("control_publisher")
{
    // declare parameters
    this->declare_parameter<bool>("debug_enabled", false);

    // create the subscribers
    m_subTrajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        "/plan/longEmergency/trajectory", 1, std::bind(&CtrlLongEmergency::trajectoryCallback, this, std::placeholders::_1)
    );
    m_subEgo_ = this->create_subscription<crp_msgs::msg::Ego>(
        "/ego", 1, std::bind(&CtrlLongEmergency::egoCallback, this, std::placeholders::_1)
    );

    // create the publisher
    m_pubControl_ = this->create_publisher<autoware_control_msgs::msg::Control>("/control/command/control_cmd", 1);

    // init timer for publishing a command
    m_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CtrlLongEmergency::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "control_publisher node has been started");
}

void cle::CtrlLongEmergency::trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Received trajectory message! Time: %d", msg->header.stamp.sec);
}

void cle::CtrlLongEmergency::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    // read debug param
    double egoVelocity = msg->twist.twist.linear.x;
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Received ego message! Time: %d, Velocity: %f", msg->header.stamp.sec, egoVelocity);
}

void cle::CtrlLongEmergency::timerCallback()
{
    // create message and init with current time
    autoware_control_msgs::msg::Control controlMsg;
    controlMsg.stamp = this->get_clock()->now();
    controlMsg.longitudinal.velocity = 0.0f; // set velocity to 0 for emergency braking

    // publish the message
    m_pubControl_->publish(controlMsg);

    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Control message has been published! Time: %d, Target Velocity: %f", controlMsg.stamp.sec, controlMsg.longitudinal.velocity );
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cle::CtrlLongEmergency>());
    rclcpp::shutdown();
    return 0;
}