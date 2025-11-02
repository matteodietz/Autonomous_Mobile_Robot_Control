#include "ee4308_turtle/controller.hpp"

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
        // new parameters
        initParam(node_, plugin_name_ + ".c_h", c_h, 0.5);  // Default value: 0.5
        initParam(node_, plugin_name_ + ".d_prox", d_prox, 0.3);  // Default value: 0.3
        initParam(node_, plugin_name_ + ".g_l", g_l, 1.5);  // Default value: 1.5

        // initialize topics
        // sub_scan_ = node_->create_subscription<some msg type>(
        //     "some topic", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::some_callback, this, std::placeholders::_1));

        // // Subscribe to the laser scan topic
        // sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "/scan", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::scanCallback, this, std::placeholders::_1));
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used
        
        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        // variables
        double goal_x = goal_pose.pose.position.x;
        double goal_y = goal_pose.pose.position.y;
        double goal_yaw = getYawFromQuaternion(goal_pose.pose.orientation);
        double robot_x = pose.pose.position.x;
        double robot_y = pose.pose.position.y;
        double robot_yaw = getYawFromQuaternion(pose.pose.orientation);


        // look if pose is close enough to the goal position
        double goal_dx = goal_x - robot_x;
        double goal_dy = goal_y - robot_y;
        double distance_to_goal = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);

        if (distance_to_goal < xy_goal_thres_)
        {
            // look if orientation is close enough to the goal orientation
            double yaw_error = ee4308::limitAngle(goal_yaw - robot_yaw);
            if (std::abs(yaw_error) > yaw_goal_thres_)
            {
                double omega_0 = 0.5;
                return writeCmdVel(0, omega_0);
            }

            return writeCmdVel(0, 0);
        }

        // find the point that is closest to the robot
        double min_dist = std::numeric_limits<double>::max();  
        int closest_index = 0;

        for (size_t i = 0; i < global_plan_.poses.size(); ++i){
            double path_x_i = global_plan_.poses[i].pose.position.x;
            double path_y_i = global_plan_.poses[i].pose.position.y;
            double dx_i = path_x_i - robot_x;
            double dy_i = path_y_i - robot_y;
            double dist = std::sqrt(dx_i * dx_i + dy_i * dy_i); //distance from the i-th point to the robot

            // update min_dist if dist is smaller than the previous dist & store the index of the closest point
            if(dist < min_dist){
                min_dist = dist;
                closest_index = i;
            }
        }

        //======This part calculates the lookahead point relative from the robots point======//
        // define a lookahead_inndex to iterate through the path
        size_t lookahead_index = closest_index;

        while(lookahead_index < global_plan_.poses.size()){

            // for every following point of the path, get the x,y-position
            double lookahead_x = global_plan_.poses[lookahead_index].pose.position.x;
            double lookahead_y = global_plan_.poses[lookahead_index].pose.position.y;

            // calculate the distance from the robot to the potential lookahead point
            double dl_x = robot_x - lookahead_x;
            double dl_y = robot_y - lookahead_y;
            double lookahead_dist = std::sqrt(dl_x * dl_x + dl_y * dl_y);

            // and test if the distance is bigger or equal to the required lookahead distance
            if(lookahead_dist >= desired_lookahead_dist_)
                break;
            
            lookahead_index++;
        }

       geometry_msgs::msg::PoseStamped lookahead_pose;
       // look for edge cases
       if (lookahead_index >= global_plan_.poses.size()) {
            // Handle the edge case, for example:
            // Set lookahead_pose to the last point on the path
            lookahead_pose = global_plan_.poses.back();
        } else {
            // Save the first valid lookahead point
            lookahead_pose = global_plan_.poses[lookahead_index];
        }

        // save the point with the first lookahead index that fullfils the requirement
        // geometry_msgs::msg::PoseStamped lookahead_pose = global_plan_.poses[lookahead_index];

        // transform lookahead point into robots frame
        double dx = lookahead_pose.pose.position.x - robot_x;
        double dy = lookahead_pose.pose.position.y - robot_y;
        double robot_theta = getYawFromQuaternion(pose.pose.orientation);

        double x_s = std::cos(robot_theta) * dx + std::sin(robot_theta) * dy;
        double y_s = std::cos(robot_theta) * dy - std::sin(robot_theta) * dx;

        // compute the curvature c
        double curvature = 2 * y_s / (x_s * x_s + y_s * y_s);

        // calculate angular velocity
        double angular_vel = desired_linear_vel_ * curvature;

        // curvature heuristic
        double v_c = desired_linear_vel_;
        if (curvature < c_h){

            v_c = desired_linear_vel_ * curvature / c_h;
        }

        // double d_o = 1;  // implemented a LaserScan but don't know if it will work!!
        // proximity heuristic
        double v = v_c;
        if (d_0 < d_prox){

            v = v_c * (d_0 / d_prox);
        }

        // vary lookahead
        double L_h = v * g_l;
        desired_lookahead_dist_ = L_h;

        // clamp angular vel
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        // clamp linear vel
        double linear_vel = std::min(desired_linear_vel_, max_linear_vel_);

        //double linear_vel = 0 * (lookahead_pose.pose.position.x - pose.pose.position.x);
        //double angular_vel = 0 * getYawFromQuaternion(goal_pose.pose.orientation);

        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    // void Controller::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // double min_range = std::numeric_limits<double>::max();

    //     for (const auto &range : msg->ranges){
    //         if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max){
    //             min_range = std::min(min_range, static_cast<double>(range));
    //         }
    //     }

    //     d_0 = min_range;
    // }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)