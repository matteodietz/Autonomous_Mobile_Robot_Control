#include "ee4308_turtle/controller.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace ee4308::turtle
{
    void Controller::cleanup() { RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

    // ====================================== LAB 1, PROJ 1 ====================================================

    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        tf_ = tf;
        plugin_name_ = name;

        // initialize parameters
        initParam(node_, plugin_name_ + ".desired_linear_vel", desired_linear_vel_, 0.2);
        initParam(node_, plugin_name_ + ".desired_lookahead_dist", desired_lookahead_dist_, 0.4);
        initParam(node_, plugin_name_ + ".max_angular_vel", max_angular_vel_, 1.0);
        initParam(node_, plugin_name_ + ".max_linear_vel", max_linear_vel_, 0.22);
        initParam(node_, plugin_name_ + ".xy_goal_thres", xy_goal_thres_, 0.05);
        initParam(node_, plugin_name_ + ".yaw_goal_thres", yaw_goal_thres_, 0.25);

        // regulated pure pursuit parameters
        initParam(node_, plugin_name_ + ".curvature_threshold", curvature_threshold_, 0.3);
        initParam(node_, plugin_name_ + ".proximity_threshold", proximity_threshold_, 0.2);
        initParam(node_, plugin_name_ + ".lookahead_gain", lookahead_gain_, 1.0);
        initParam(node_, plugin_name_ + ".min_angular_vel", min_angular_vel_, 0.05);
        initParam(node_, plugin_name_ + ".d_0", d_0, 1.0e5);
        initParam(node_, plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_, 0.1);

        // initialize topics
        sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::scanCallback, this, std::placeholders::_1));
    }
//GUVEN FUNCTION
    // geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
    //     const geometry_msgs::msg::PoseStamped &pose,
    //     const geometry_msgs::msg::Twist &velocity,
    //     nav2_core::GoalChecker *goal_checker)
    // {
    //     (void)velocity;     // not used
    //     (void)goal_checker; // not used

    //     // check if path exists
    //     if (global_plan_.poses.empty())
    //     {
    //         RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
    //         return writeCmdVel(0, 0);
    //     }

    //     // get goal pose (contains the "clicked" goal rotation and position)
    //     geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

    //     // get lookahead?
    //     geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;

    //     double linear_vel = 0 * (lookahead_pose.pose.position.x - pose.pose.position.x);
    //     double angular_vel = 0 * getYawFromQuaternion(goal_pose.pose.orientation);

    //     return writeCmdVel(linear_vel, angular_vel);
    // }
//GIVEN FUNCTION
    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist & ,
    nav2_core::GoalChecker * )
    {
        // 2)  If no global path Then
        if (global_plan_.poses.empty()){// 3) Return (0,0)
            return writeCmdVel(0.0, 0.0);}

        // get goal pose 
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        double dx_goal = goal_pose.pose.position.x - pose.pose.position.x;
        double dy_goal = goal_pose.pose.position.y - pose.pose.position.y;
        double dist_to_goal = std::hypot(dx_goal, dy_goal);

        // 5)If the robot is close to the goal Then ▶ Goal is the last point on the path.
        double robot_yaw = tf2::getYaw(pose.pose.orientation);
        if (dist_to_goal < xy_goal_thres_){//6) Return (0,0)

            // heck if robot is facing the goal, if not, rotate (add threshold for this)
            double goal_yaw = tf2::getYaw(goal_pose.pose.orientation); 
            if (std::abs(goal_yaw - robot_yaw) > yaw_goal_thres_){
                double angular_vel = goal_yaw - robot_yaw;
                if (angular_vel > max_angular_vel_){
                    // std::cout << "Angular velocity clipped from " << angular_vel << " to " << max_angular_vel_ << std::endl;
                    angular_vel = max_angular_vel_;
                }
                else if (angular_vel < -max_angular_vel_){
                    // std::cout << "Angular velocity clipped from " << angular_vel << " to " << -max_angular_vel_ << std::endl;
                    angular_vel = -max_angular_vel_;
                }
                else if (std::abs(angular_vel) < min_angular_vel_){
                    if (angular_vel > 0){
                        // std::cout << "Angular velocity clipped from " << angular_vel << " to " << min_angular_vel_ << std::endl;
                        angular_vel = min_angular_vel_;
                    }
                    else{
                        // std::cout << "Angular velocity clipped from " << angular_vel << " to " << -min_angular_vel_ << std::endl;
                        angular_vel = -min_angular_vel_;
                    }
                }
                // std::cout << "Rotating to face goal. Angular velocity = " << angular_vel << std::endl;
                return writeCmdVel(0.0, angular_vel);
            }
            return writeCmdVel(0.0, 0.0);
        }

        // 8)Find the point along the path that is closest to the robot.
        int closest_point_idx = 0;
        double min_dist_sq = 1e9;
        double rx = pose.pose.position.x;
        double ry = pose.pose.position.y;

        for (int i = 0; i < static_cast<int>(global_plan_.poses.size()); i++)
        {
            double dx = global_plan_.poses[i].pose.position.x - rx;
            double dy = global_plan_.poses[i].pose.position.y - ry;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq){
                min_dist_sq = dist_sq;
                closest_point_idx = i;
            }
        }

        // 9)From the closest point, find the lookahead point.
        int lookahead_point_idx = closest_point_idx;
        bool found_lookahead = false;
        for (int j = closest_point_idx; j < static_cast<int>(global_plan_.poses.size()); j++)
        {
            double dx = global_plan_.poses[j].pose.position.x - rx;
            double dy = global_plan_.poses[j].pose.position.y - ry;
            double dist = std::hypot(dx, dy);
            if (dist >= desired_lookahead_dist_){
                lookahead_point_idx = j;
                found_lookahead = true;
                // std::cout << "Lookahead point found at " << lookahead_point_idx << std::endl;
                break;
            }
        }
        if (!found_lookahead){
            lookahead_point_idx = static_cast<int>(global_plan_.poses.size() - 1);
            // std::cout << "Lookahead point not found. Using last point." << std::endl;
        }

        // 10) Transform the lookahead point into the robot frame
        // double robot_yaw = tf2::getYaw(pose.pose.orientation); //todo delete this line
        double lx = global_plan_.poses[lookahead_point_idx].pose.position.x - rx; 
        double ly = global_plan_.poses[lookahead_point_idx].pose.position.y - ry;  
        double x_r =  std::cos(-robot_yaw) * lx - std::sin(-robot_yaw) * ly;
        double y_r =  std::sin(-robot_yaw) * lx + std::cos(-robot_yaw) * ly;

        // // idea for question 5
        // if (x_r < 0){
        //     x_r = -1e-5;
        // }

        double d2 = x_r * x_r + y_r * y_r;
        double c = 0.0;
        // std::cout << "d2: " << d2 << std::endl;
        // std::cout << "y_r: " << y_r << "x_r: " << x_r << std::endl;

        //11)Calculate the curvature c.
        c = 2.0 * y_r / d2;

        // 12) Calculate omega from v and c
        double linear_vel  = desired_linear_vel_;
        double angular_vel = linear_vel * c;

        // // Calculate the curvature heuristic.  ▶ Reg. Pure Pursuit.
        // if (std::abs(c) > curvature_threshold_){
        //     linear_vel *= curvature_threshold_/std::abs(c);
        //     // std::cout << "Curvature heuristic applied. Reduction factor = " << curvature_threshold_/std::abs(c) << std::endl;
        // }
        
        // idea for question 4, works better than the curvature heuristic
        float theta = atan2(y_r, x_r);
        // clip theta to [-pi, pi] should not be necessary because of atan2
        theta = ee4308::limitAngle(theta);

        linear_vel = (1-std::abs(theta)/3.14)*linear_vel;

        // Calculate the obstacle heuristic.  ▶ Reg. Pure Pursuit.
        if (d_0 < proximity_threshold_){
            linear_vel *= d_0/proximity_threshold_;
            std::cout << "Obstacle heuristic applied. Reduction factor = " << d_0/proximity_threshold_ << std::endl;
        }

        // Vary the lookahead.  ▶ Reg. Pure Pursuit. 
        desired_lookahead_dist_ = linear_vel * lookahead_gain_;
        if (desired_lookahead_dist_ < min_lookahead_dist_){
            desired_lookahead_dist_ = min_lookahead_dist_;
        }
        std::cout << "Lookahead distance updated to " << desired_lookahead_dist_ << std::endl;


        // 13) Constrain omega to within largest allowable angular speed.
        if (angular_vel > max_angular_vel_){
            // std::cout << "Angular velocity clipped from " << angular_vel << " to " << max_angular_vel_ << std::endl;
            angular_vel = max_angular_vel_;
        }
        else if (angular_vel < -max_angular_vel_){
            // // std::cout << "Angular velocity clipped from " << angular_vel << " to " << -max_angular_vel_ << std::endl;
            angular_vel = -max_angular_vel_;
        }
        
         


        // 14) Constrain v to within the largest allowable linear speed.
        if (linear_vel > max_linear_vel_) {
            // std::cout << "Linear velocity clipped from " << linear_vel << " to " << max_linear_vel_ << std::endl;
            linear_vel = max_linear_vel_;
        }


        // 15) Return v and omega
        return writeCmdVel(linear_vel, angular_vel);
    }

    void Controller::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    double min_range = std::numeric_limits<double>::max();

        for (const auto &range : msg->ranges){
            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max){
                min_range = std::min(min_range, static_cast<double>(range));
            }
        }

        std::cout << "Closest obstacle distance: " << min_range << std::endl;
        d_0 = min_range;
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)