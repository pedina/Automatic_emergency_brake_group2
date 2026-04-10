#ifndef CTRL_LONG_EMERGENCY_HPP
#define CTRL_LONG_EMERGENCY_HPP

#include "rclcpp/rclcpp.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "crp_msgs/msg/ego.hpp"
#include "autoware_control_msgs/msg/control.hpp"


namespace cle
{

class CtrlLongEmergency : public rclcpp::Node
{
public:
    CtrlLongEmergency();

private:
    void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
    void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr m_subTrajectory_;
    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr m_subEgo_;
    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr m_pubControl_;

    rclcpp::TimerBase::SharedPtr m_timer_;

    // values coming from trajectory msg
    double m_trajectoryVelocity;
    long m_trajectoryTime;

    // values coming from ego msg
    double m_egoVelocity;
};

} // namespace cle
#endif // CTRL_LONG_EMERGENCY_HPP