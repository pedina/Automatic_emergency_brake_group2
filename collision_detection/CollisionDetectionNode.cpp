#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iomanip>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "crp_msgs/msg/ego.hpp"
#include "crp_msgs/msg/target_space.hpp"

// Ha a PredictedObject üzenetet is használod, azt is be kell include-olni:
// #include "autoware_perception_msgs/msg/predicted_object.hpp"

class CollisionDetectionNode : public rclcpp::Node
{
public:
    CollisionDetectionNode() : Node("collision_detection_node")
    {
        // Előfizetünk az Ego topikra
        ego_sub_ = this->create_subscription<crp_msgs::msg::Ego>(
            "/ego_topic", 10,
            std::bind(&CollisionDetectionNode::ego_callback, this, std::placeholders::_1));

        // Előfizetünk a TargetSpace topikra
        target_sub_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
            "/target_space_topic", 10,
            std::bind(&CollisionDetectionNode::target_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ütközésérzékelő (AEB) csomópont elindult!");
    }

private:
    // Utolsó ismert Ego állapotok (induláskor 0)
    double current_v_ego_ = 0.0;
    double current_a_ego_ = 0.0;

    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr ego_sub_;
    rclcpp::Subscription<crp_msgs::msg::TargetSpace>::SharedPtr target_sub_;

    // A te eredeti, univerzális TTC függvényed osztálymetódusként
    double calculate_universal_ttc(double distance, double v_ego, double a_ego, double v_target, double a_target, double accel_threshold = 1e-5)
    {
        if (distance <= 0) {
            return 0.0; // Már megtörtént az ütközés
        }

        double rel_velocity = v_ego - v_target;
        double rel_acceleration = a_ego - a_target;

        // 1. ESET: Nincs (vagy elhanyagolható) relatív gyorsulás
        if (std::abs(rel_acceleration) < accel_threshold) {
            if (rel_velocity <= 0) {
                return std::numeric_limits<double>::infinity(); // Távolodunk vagy tartjuk a távolságot
            }
            return distance / rel_velocity;
        }

        // 2. ESET: Van relatív gyorsulás (másodfokú egyenlet)
        double discriminant = std::pow(rel_velocity, 2) + 2 * rel_acceleration * distance;

        if (discriminant < 0) {
            return std::numeric_limits<double>::infinity(); // Nincs ütközés
        }

        double sqrt_disc = std::sqrt(discriminant);
        double t1 = (-rel_velocity + sqrt_disc) / rel_acceleration;
        double t2 = (-rel_velocity - sqrt_disc) / rel_acceleration;

        double min_valid_time = std::numeric_limits<double>::infinity();
        
        if (t1 > 0) min_valid_time = std::min(min_valid_time, t1);
        if (t2 > 0) min_valid_time = std::min(min_valid_time, t2);

        return min_valid_time;
    }

    void ego_callback(const crp_msgs::msg::Ego::SharedPtr msg)
    {
        // Ego sebesség és gyorsulás kinyerése a beágyazott ROS típusokból
        current_v_ego_ = msg->twist.twist.linear.x;
        current_a_ego_ = msg->accel.accel.linear.x;
    }

    void target_callback(const crp_msgs::msg::TargetSpace::SharedPtr msg)
    {
        // 1. Távolság kinyerése
        // ROS rendszerekben a target_pose többnyire az Egohoz viszonyított relatív pozíciót jelenti.
        double distance = msg->target_pose.pose.position.x;

        // 2. Célpont adatainak kinyerése
        double v_target = 0.0;
        double a_target = 0.0;

        // --- BIZONYTALANSÁGI PONT ---
        // Mivel a TargetSpace.msg önmagában nem tartalmaz sebesség mezőt (csak pose-t és tömböket), 
        // valószínűleg a predicted_objects-ből kell kinyerni az adatokat.
        // Példa a bekötésre (a pontos szintaxist az autoware üzenet alapján kell majd igazítanod):
        /*
        if (!msg->relevant_objects.empty()) {
            // Vegyük a legelső (legközelebbi) objektumot
            v_target = msg->relevant_objects[0].kinematics.initial_twist_with_covariance.twist.linear.x;
            a_target = msg->relevant_objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.x;
        }
        */

        // 3. A te TTC számításod futtatása
        double ttc = calculate_universal_ttc(distance, current_v_ego_, current_a_ego_, v_target, a_target);

        // Kimenet ellenőrzése a konzolon
        RCLCPP_INFO(this->get_logger(), "Ego V: %.2f | Távolság: %.2f | Számított TTC: %.2f s", 
                    current_v_ego_, distance, ttc);

        // --- ITT JÖN MAJD A KÖVETKEZŐ LÉPÉS ---
        // Ha a ttc egy kritikus érték alá esik (pl. < 2.0 mp), itt kell majd összeállítanod
        // és kiadnod a kimeneti Trajectory üzenetet a fékparancshoz.
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
