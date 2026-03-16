#include "rclcpp/rclcpp.hpp"
#include "autoware_control_msgs/msg/control.hpp"


namespace ctrl_long_emergency
{

class ControlPublisher : public rclcpp::Node
{
public:
    ControlPublisher();

private:
    void timerCallback();

    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr m_pubControl_;

    rclcpp::TimerBase::SharedPtr m_timer_;
};

} // namespace ctrl_long_emergency