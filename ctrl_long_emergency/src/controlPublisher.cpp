#include "ctrl_long_emergency/controlPublisher.hpp"


ctrl_long_emergency::ControlPublisher::ControlPublisher() : Node("control_publisher")
{
    // declare parameters
    this->declare_parameter<std::string>("topic_name", "ctrl_cmd");
    this->declare_parameter<uint8_t>("publish_rate", 5);
    this->declare_parameter<bool>("debug_enabled", false);

    std::string topicName;
    uint8_t     publishRate;

    // retrieve parameters
    this->get_parameter<std::string>("topic_name", topicName);
    this->get_parameter<uint8_t>("publish_rate", publishRate);

    // create the publisher
    m_pubControl_ = this->create_publisher<autoware_control_msgs::msg::Control>(
        topicName,
        1
    );

    // init timer for publishing a command
    m_timer_ = this->create_wall_timer(
        std::chrono::duration<float>(1.0f / static_cast<float>(publishRate)),
        std::bind(&ControlPublisher::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "control_publisher node has been started");
}

void ctrl_long_emergency::ControlPublisher::timerCallback()
{
    // create message and init with current time
    autoware_control_msgs::msg::Control controlMsg;
    controlMsg.stamp = this->get_clock()->now();

    // publish the message
    m_pubControl_->publish(controlMsg);

    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Control message has been published! Time: %d", controlMsg.stamp.sec);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ctrl_long_emergency::ControlPublisher>());
    rclcpp::shutdown();
    return 0;
}