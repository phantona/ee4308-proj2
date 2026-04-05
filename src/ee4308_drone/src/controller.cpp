#include "ee4308_drone/controller.hpp"

namespace ee4308::drone
{
    Controller::Controller(
        const rclcpp::NodeOptions &options,
        const std::string &name = "controller") 
        : Node(name, options)
    {
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 20.0).as_double();
        this->enable_ = ee4308::getParameter<bool>(this, "enable", true).as_bool();
        this->lookahead_distance_ = ee4308::getParameter<double>(this, "lookahead_distance", 1.0).as_double();
        this->max_xy_vel_ = ee4308::getParameter<double>(this, "max_xy_vel", 1.0).as_double();
        this->max_z_vel_ = ee4308::getParameter<double>(this, "max_z_vel", 0.5).as_double();
        this->yaw_vel_ = ee4308::getParameter<double>(this, "yaw_vel", -0.524).as_double();
        this->kp_xy_ = ee4308::getParameter<double>(this, "kp_xy", 1.0).as_double();
        this->kp_z_ = ee4308::getParameter<double>(this, "kp_z", 1.0).as_double();

        this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubPlan_, this, std::placeholders::_1));

        this->received_odom_ = false;

        this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Controller::callbackTimer_, this));
    }

    void Controller::callbackSubOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->odom_ = msg;
        this->received_odom_ = true;
    }

    void Controller::callbackSubPlan_(const nav_msgs::msg::Path msg)
    {
        this->plan_ = msg;
    }

    void Controller::callbackTimer_()
    {
        if (!enable_)
            return;

        if (!this->received_odom_)
        {
            publishCmdVel_(0, 0, 0, 0);
            return;
        }

        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel_(0, 0, 0, 0);
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
        // publishCmdVel__()
        // =========

        const double drone_x = odom_.pose.pose.position.x;
        const double drone_y = odom_.pose.pose.position.y;
        const double drone_z = odom_.pose.pose.position.z;

        //Finding closest point on path
        size_t closest_idx = 0;
        double closest_dist = std::numeric_limits<double>::infinity();

        for (size_t i=0; i < plan_.poses.size(); ++i) {
            const double px = plan_.poses[i].pose.position.x;
            const double py = plan_.poses[i].pose.position.y;
            const double pz = plan_.poses[i].pose.position.z;

            const double dx = px - drone_x;
            const double dy = py - drone_y;
            const double dz = pz - drone_z;
            const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (dist < closest_dist) {
                closest_dist = dist;
                closest_idx = i;
            }
        }

        //Find lookahead point by searching forward from closest point
        size_t lookahead_idx = plan_.poses.size()-1;

        for (size_t i = closest_idx; i < plan_.poses.size(); ++i) {
            const double px = plan_.poses[i].pose.position.x;
            const double py = plan_.poses[i].pose.position.y;
            const double pz = plan_.poses[i].pose.position.z;

            const double dx = px - drone_x;
            const double dy = py - drone_y;
            const double dz = pz - drone_z;
            const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (dist >= lookahead_distance_) {
                lookahead_idx = i;
                break;
            }
        }

        const auto &target_pose = plan_.poses[lookahead_idx].pose.position;
        const double target_x = target_pose.x;
        const double target_y = target_pose.y;
        const double target_z = target_pose.z;

        //Position error in map frame
        const double err_x_map = target_x - drone_x;
        const double err_y_map = target_y - drone_y;
        const double err_z = target_z - drone_z;

        //Coonvert X-Y error into drone/body frame
        const double yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);

        const double err_x_body = cos_yaw*err_x_map + sin_yaw*err_y_map;
        const double err_y_body = -sin_yaw*err_x_map + cos_yaw*err_y_map;

        //Proportional velocity command
        double cmd_x = kp_xy_*err_x_body;
        double cmd_y = kp_xy_*err_y_body;
        double cmd_z = kp_z_*err_z;

        //Clamp horizontal velocity magnitude
        const double xy_speed = std::hypot(cmd_x, cmd_y);
        if (xy_speed > max_xy_vel_ && xy_speed > 1e-9) {
            const double scale = max_xy_vel_/xy_speed;
            cmd_x *= scale;
            cmd_y *= scale;
        }

        //Clamp vertical velocity
        cmd_z = std::clamp(cmd_z, -max_z_vel_, max_z_vel_);

        //Publish command vel
        publishCmdVel_(cmd_x, cmd_y, cmd_z, yaw_vel_);
    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel_(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        // RCLCPP_INFO(this->get_logger(), "%f,%f,%f,%f", x_vel, y_vel, z_vel, yaw_vel);
        if (!std::isfinite(x_vel) || !std::isfinite(y_vel) || !std::isfinite(z_vel) || !std::isfinite(yaw_vel))
        {
            RCLCPP_WARN(this->get_logger(), 
                "Cmd velocities are inf or nan. Controller or estimator problem. CmdVels(x,y,z,yaw): %6.3f, %6.3f, %6.3f, %6.3f", 
                x_vel, y_vel, z_vel, yaw_vel);
        }
        pub_cmd_vel_->publish(cmd_vel);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Controller);