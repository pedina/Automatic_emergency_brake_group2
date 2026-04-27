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

    RCLCPP_INFO(this->get_logger(), "ctrl_long_emergency node has been started");
}

void cle::CtrlLongEmergency::trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
    m_trajectoryVelocity = msg->points[0].longitudinal_velocity_mps;
    m_trajectoryTimeSec = msg->points[0].time_from_start.sec;
    m_trajectoryTimeNanosec = msg->points[0].time_from_start.nanosec;
    m_trajectoryTime = m_trajectoryTimeSec + m_trajectoryTimeNanosec * 1e-9;

    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Received trajectory message! Time: %d, Velocity: %f, Time from start: %f", 
            msg->header.stamp.sec, m_trajectoryVelocity, m_trajectoryTime);
}

void cle::CtrlLongEmergency::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
{
    m_egoVelocity = msg->twist.twist.linear.x;

    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Received ego message! Time: %d, Velocity: %f", msg->header.stamp.sec, m_egoVelocity);
}

void cle::CtrlLongEmergency::timerCallback()
{
    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (!m_egoVelocity) {
        if (isDebugEnabled)
            RCLCPP_INFO(this->get_logger(), "No ego");
        return;
    }

    if (!m_trajectoryVelocity && m_trajectoryVelocity != 0) {
        if (isDebugEnabled)
            RCLCPP_INFO(this->get_logger(), "No velo");
        return;
    }

    if (!(m_trajectoryTime > 0.0) || m_trajectoryTime > 4.0)
        return;
    
    // create message
    autoware_control_msgs::msg::Control controlMsg;
    controlMsg.stamp = this->get_clock()->now();

    double cycleTimeSec = 0.02; // 20 ms
    double cycles = m_trajectoryTime / cycleTimeSec;

    double speedPerCycle = (m_egoVelocity - m_trajectoryVelocity) / cycles;

    double targetSpeed = m_egoVelocity - speedPerCycle;

    if (targetSpeed < 0.0)
        targetSpeed = 0.0;

    controlMsg.longitudinal.velocity = targetSpeed;

    // publish the message
    if (std::isnan(controlMsg.longitudinal.velocity))
        return;

    m_pubControl_->publish(controlMsg);

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