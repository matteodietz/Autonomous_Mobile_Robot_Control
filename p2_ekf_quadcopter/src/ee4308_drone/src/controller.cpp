#include "ee4308_drone/controller.hpp"

namespace ee4308::drone
{
    Controller::Controller(const std::string &name = "controller_ee4308") : Node(name)
    {
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "use_ground_truth", use_ground_truth_, false);
        initParam(this, "enable", enable_, true);
        initParam(this, "lookahead_distance", lookahead_distance_, 1.0);
        initParam(this, "max_xy_vel", max_xy_vel_, 1.0);
        initParam(this, "max_z_vel", max_z_vel_, 0.5);
        initParam(this, "yaw_vel", yaw_vel_, -0.3);
        initParam(this, "kp_xy", kp_xy_, 1.0);
        initParam(this, "kp_z", kp_z_, 1.0);

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            (use_ground_truth_ ? "odom" : "est_odom"), rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbOdom, this, std::placeholders::_1));
        sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbPlan, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1s / frequency_, std::bind(&Controller::cbTimer, this));
    }

    void Controller::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }

    void Controller::cbPlan(const nav_msgs::msg::Path msg)
    {
        plan_ = msg;
    }

    void Controller::cbTimer()
    {
        if (!enable_)
            return;

        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel(0, 0, 0, 0);
            return;
        }

        // ==== make use of ====
        // plan_.poses
        // odom_
        // ee4308::getYawFromQuaternion()
        // std::hypot()
        // std::clamp()
        // std::cos(), std::sin() 
        // lookahead_distance_
        // kp_xy_
        // kp_z_
        // max_xy_vel_
        // max_z_vel_
        // yaw_vel_
        // publishCmdVel()
        // ==== ====

        auto px = odom_.pose.pose.position.x;
        auto py = odom_.pose.pose.position.y;
        auto pz = odom_.pose.pose.position.z;
        auto yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);

        // Find the closest point in the plan
        double min_dist = std::numeric_limits<double>::max();
        size_t min_idx = 0;
        for (size_t i = 0; i < plan_.poses.size(); i++)
        {
            auto &pose = plan_.poses[i].pose;
            double dist = std::hypot(pose.position.x - px, pose.position.y - py);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_idx = i;
            }
        }
        
        // Find the lookahead point, if goal in lookahead distance, lookahead is goal
        size_t lookahead_idx = min_idx;
        while (lookahead_idx < (plan_.poses.size()-1))
        {
            auto &pose = plan_.poses[lookahead_idx + 1].pose;
            double dist = std::hypot(pose.position.x - px, pose.position.y - py);
            if (dist > lookahead_distance_)
            {
                break;
            }
            lookahead_idx++;
        }

        // Compute the position error in world frame
        auto dx = plan_.poses[lookahead_idx].pose.position.x - px;
        auto dy = plan_.poses[lookahead_idx].pose.position.y - py;
        auto dz = plan_.poses[lookahead_idx].pose.position.z - pz;

        // Rotate the position error to body frame
        auto dx_body = dx * std::cos(yaw) + dy * std::sin(yaw);
        auto dy_body = -dx * std::sin(yaw) + dy * std::cos(yaw);

        // Compute the velocity command in body frame
        auto x_vel = kp_xy_ * dx_body;
        auto y_vel = kp_xy_ * dy_body;
        auto z_vel = kp_z_ * dz;

        // Limit the velocity command
        auto vel_norm = std::hypot(x_vel, y_vel);
        if (vel_norm > max_xy_vel_)
        {
            x_vel = max_xy_vel_ * x_vel / vel_norm;
            y_vel = max_xy_vel_ * y_vel / vel_norm;
        }
        z_vel = std::clamp(z_vel, -max_z_vel_, max_z_vel_);


        // publish
        publishCmdVel(x_vel, y_vel, z_vel, yaw_vel_);
    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        pub_cmd_vel_->publish(cmd_vel);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ee4308::drone::Controller>());
    rclcpp::shutdown();
    return 0;
}