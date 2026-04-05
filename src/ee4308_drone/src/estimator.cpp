#include "ee4308_drone/estimator.hpp"

namespace ee4308::drone
{
    Estimator::Estimator(
        const rclcpp::NodeOptions &options,
        const std::string &name = "estimator")
        : Node(name, options)
    {
        // parameters
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 10.0).as_double();
        this->var_imu_x_ = ee4308::getParameter<double>(this, "var_imu_x", 0.2).as_double();
        this->var_imu_y_ = ee4308::getParameter<double>(this, "var_imu_y", 0.2).as_double();
        this->var_imu_z_ = ee4308::getParameter<double>(this, "var_imu_z", 0.2).as_double();
        this->var_imu_a_ = ee4308::getParameter<double>(this, "var_imu_a", 0.2).as_double();
        this->var_gps_x_ = ee4308::getParameter<double>(this, "var_gps_x", 0.2).as_double();
        this->var_gps_y_ = ee4308::getParameter<double>(this, "var_gps_y", 0.2).as_double();
        this->var_gps_z_ = ee4308::getParameter<double>(this, "var_gps_z", 0.2).as_double();
        this->var_baro_ = ee4308::getParameter<double>(this, "var_baro", 0.2).as_double();
        this->var_sonar_ = ee4308::getParameter<double>(this, "var_sonar", 0.2).as_double();
        this->var_magnet_ = ee4308::getParameter<double>(this, "var_magnet", 0.2).as_double();
        this->verbose_ = ee4308::getParameter<bool>(this, "verbose", true).as_bool();
        this->frame_id_map_ = ee4308::getParameter<std::string>(this, "map_frame_id", "map").as_string();
        this->frame_id_drone_ = ee4308::getParameter<std::string>(this, "drone_frame_id", "drone/base_link").as_string();
        this->use_ground_truth_ = ee4308::getParameter<bool>(this, "use_ground_truth", false).as_bool();
        double initial_x = ee4308::getParameter<double>(this, "initial_x", -2.0).as_double();
        double initial_y = ee4308::getParameter<double>(this, "initial_y", -2.0).as_double();
        double initial_z = ee4308::getParameter<double>(this, "initial_z", 0.05).as_double();

        // topics
        this->pub_est_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::ServicesQoS());
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1); // use only the most recent
        this->sub_true_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "true_odom", qos, std::bind(&Estimator::callbackSubTrueOdom_, this, std::placeholders::_1)); // ground truth in sim.
        this->sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", qos, std::bind(&Estimator::callbackSubGPS_, this, std::placeholders::_1));
        this->sub_sonar_ = this->create_subscription<sensor_msgs::msg::LaserScan>( // gz has no sonar implementation. laserscan for quick hack.
            "sonar", qos, std::bind(&Estimator::callbackSubSonar_, this, std::placeholders::_1));
        this->sub_magnetic_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "magnetic", qos, std::bind(&Estimator::callbackSubMagnetic_, this, std::placeholders::_1));
        this->sub_baro_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "air_pressure", qos, std::bind(&Estimator::callbackSubBaro_, this, std::placeholders::_1));
        this->sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&Estimator::callbackSubIMU_, this, std::placeholders::_1));

        // states
        this->initial_position_ << initial_x, initial_y, initial_z;
        this->Xx_ << initial_x, 0;
        this->Xy_ << initial_y, 0;
        this->Xz_ << initial_z, 0, 0; // Altitude, Velocity, Bias (initially 0)
        this->Xa_ << 0, 0;
        this->Px_ = Eigen::Matrix2d::Constant(1e3),
        this->Py_ = Eigen::Matrix2d::Constant(1e3),
        this->Pz_ = Eigen::Matrix3d::Identity() * 1e3; // this is 3x3 now
        this->Pa_ = Eigen::Matrix2d::Constant(1e3);
        this->initial_ECEF_ << NAN, NAN, NAN;
        this->Ygps_ << NAN, NAN, NAN;
        this->Ymagnet_ = NAN;
        this->Ybaro_ = NAN;
        this->Ysonar_ = NAN;

        this->last_predict_time_ = this->now().seconds();
        this->initialized_ecef_ = false;
        this->initialized_magnetic_ = false;

        this->timer_ = this->create_timer(
            1s / this->frequency_,
            std::bind(&Estimator::callbackTimer, this));
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF_(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        Eigen::Vector3d ECEF;

        // 1. Calculate the square of the first numerical eccentricity
        double a = RAD_EQUATOR;
        double b = RAD_POLAR;
        double e_sq = 1.0 - (b * b) / (a * a);

        // 2. Calculate the prime vertical radius of curvature
        double N = a / std::sqrt(1.0 - e_sq * sin_lat * sin_lat);

        // 3. Calculate ECEF coordinates
        ECEF(0) = (N + alt) * cos_lat * cos_lon; // x_e
        ECEF(1) = (N + alt) * cos_lat * sin_lon; // y_e
        ECEF(2) = ((b * b) / (a * a) * N + alt) * sin_lat; // z_e

        return ECEF;
    }

    void Estimator::callbackSubGPS_(const sensor_msgs::msg::NavSatFix msg)
    { // avoiding const & due to possibly long calcs.

        constexpr double DEG2RAD = M_PI / 180;
        double lat = msg.latitude * DEG2RAD;  
        double lon = msg.longitude * DEG2RAD; 
        double alt = msg.altitude;

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        if (initialized_ecef_ == false)
        {
            initial_ECEF_ = getECEF_(sin_lat, cos_lat, sin_lon, cos_lon, alt);
            // Save these for the rotation matrix
            init_sin_lat_ = sin_lat;
            init_cos_lat_ = cos_lat;
            init_sin_lon_ = sin_lon;
            init_cos_lon_ = cos_lon;

            initialized_ecef_ = true;
            return;
        }

        // 1. Get current ECEF
        Eigen::Vector3d ECEF = getECEF_(sin_lat, cos_lat, sin_lon, cos_lon, alt);

        // 2. Build Rotation Matrix Re/n (ECEF to NED)
        Eigen::Matrix3d Re_n;
        Re_n << -init_sin_lat_ * init_cos_lon_, -init_sin_lat_ * init_sin_lon_,  init_cos_lat_,
        -init_sin_lon_,                  init_cos_lon_,                  0,
        -init_cos_lat_ * init_cos_lon_, -init_cos_lat_ * init_sin_lon_, -init_sin_lat_;

        // 3. Convert to NED frame
        // The formula is: NED = Re/n.transpose() * (Current_ECEF - Initial_ECEF)
        Eigen::Vector3d NED = Re_n * (ECEF - initial_ECEF_);

        // 4. Convert NED to World Frame (ENU)
        Eigen::Matrix3d Rm_n;
        Rm_n << 0, 1,  0,  // Map X = 
                1, 0 ,  0,  // Map Y = 
                0, 0, -1;  // Map Z = -NED Down (Up)

        // ADD THIS LINE temporary for debugging
        RCLCPP_INFO(this->get_logger(), "DEBUG NED: N: %6.3f, E: %6.3f, D: %6.3f", NED(0), NED(1), NED(2));

        // Final GPS measurement in the World Frame
        Ygps_ = Rm_n * NED + initial_position_;


        // -------------------------------------------------------------------------
        // 2. THE GATE (Place it here!)
        // -------------------------------------------------------------------------
        // Compare the GPS measurement to where the filter currently thinks we are
        double dist_to_gps = std::hypot(Ygps_(0) - Xx_(0), Ygps_(1) - Xy_(0));

        // If the GPS measurement jumps more than 2.0 meters in a single step,
        // it's likely a simulator glitch or an outlier.
        if (dist_to_gps > 2.0)
        {
            RCLCPP_WARN(this->get_logger(), "GPS Glitch! Jump of %6.3f m rejected.", dist_to_gps);
            return; // Exit the function early and skip the update
        }
        // -------------------------------------------------------------------------

        // --- KALMAN CORRECTION ---
        // 1. Observation Matrix for X and Y (2D states)
        Eigen::RowVector2d H;
        H << 1, 0;

        // Update X (2D state)
        double inno_x = Ygps_(0) - (H * Xx_)(0);
        double S_x = (H * Px_ * H.transpose())(0) + var_gps_x_;
        Eigen::Vector2d K_x = Px_ * H.transpose() / S_x;
        Xx_ = Xx_ + K_x * inno_x;
        Px_ = (Eigen::Matrix2d::Identity() - K_x * H) * Px_;

        // Update Y (2D state)
        double inno_y = Ygps_(1) - (H * Xy_)(0);
        double S_y = (H * Py_ * H.transpose())(0) + var_gps_y_;
        Eigen::Vector2d K_y = Py_ * H.transpose() / S_y;
        Xy_ = Xy_ + K_y * inno_y;
        Py_ = (Eigen::Matrix2d::Identity() - K_y * H) * Py_;

        // 2. Observation Matrix for Z (3D state for bias)
        Eigen::RowVector3d Hz; 
        Hz << 1, 0, 0; // GPS only sees altitude, not velocity or bias

        // Update Z (3D state)
        double inno_z = Ygps_(2) - (Hz * Xz_)(0);
        // CRITICAL: Must use Hz (3D) here, not H (2D)!
        double S_z = (Hz * Pz_ * Hz.transpose())(0) + var_gps_z_; 
        Eigen::Vector3d K_z = Pz_ * Hz.transpose() / S_z; 

        Xz_ = Xz_ + K_z * inno_z;
        Pz_ = (Eigen::Matrix3d::Identity() - K_z * Hz) * Pz_; 
    }

    // ================================ Sonar sub callback / EKF Correction ========================================
    void Estimator::callbackSubSonar_(const sensor_msgs::msg::LaserScan msg)
    {
        // Store the measured sonar range in Ysonar_.
        //      Required for terminal printing during demonstration.
        // ==== make use of ====
        // msg.ranges[0]
        // Ysonar_
        // var_sonar_
        // Xz_
        // Pz_
        // .transpose()
        // =========
        
        Ysonar_ = msg.ranges[0];

        // ==== [FOR LAB 2 ONLY] ==== 
        // The following is necessary so that the covariance bubble in RViz does not fill up the screen.
        // For proj 2, comment out the following:
        //Px_ << 0.1, 0, 0, 0.1;
        //Py_ << 0.1, 0, 0, 0.1;
        // =========
        
        if (!std::isfinite(Ysonar_))
        { 
            // if out of range, write to Ysonar_, 
            //     but do not do the KF correction.
            return;
        }

        // if in range, write to Ysonar_, and do the KF correction.
        Eigen::RowVector3d H; // Note: RowVector3d
        H << 1, 0, 0; // It only "sees" position, not velocity or baro_bias.  rest of the sonar math remains the same, Eigen will handle the 3×3 matrix math automatically now that Xz_ and Pz_ are resized.
        
        double R = var_sonar_;

        // 2. Innovation and Innovation Covariance
        // Note: (0) extracts the scalar from the 1x1 matrix result
        double inno = Ysonar_ - (H * Xz_)(0);
        double inno_covariance = (H * Pz_ * H.transpose())(0) + R; 

        // 3. Kalman Gain (Now a 3x1 vector)
        Eigen::Vector3d K = (Pz_ * H.transpose()) / inno_covariance;

        // 4. Update State (Xz_) and Covariance (Pz_)
        Xz_ = Xz_ + K * inno;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity(); // 3x3 Identity matrix
        Pz_ = (I - K * H) * Pz_;
    }

    // ================================ Magnetic sub callback / EKF Correction ========================================
    void Estimator::callbackSubMagnetic_(const sensor_msgs::msg::MagneticField msg)
    {
        // 1. Calculate the measured yaw from magnetic field vectors
        // Note: We use atan2(y, x) to get the angle in radians
        Ymagnet_ = std::atan2(-msg.magnetic_field.y, msg.magnetic_field.x);

        // 2. Observation Model: Magnetometer observes Yaw (index 0)
        Eigen::RowVector2d Ha;
        Ha << 1, 0;
        double R = var_magnet_;

        // 3. Innovation (Measurement - Prediction)
        // CRITICAL: We must limit the angle difference to [-PI, PI] or the filter will flip!
        double innovation = ee4308::limitAngle(Ymagnet_ - Xa_(0));
        double S = (Ha * Pa_ * Ha.transpose())(0) + R;

        // 4. Kalman Gain
        Eigen::Vector2d K = Pa_ * Ha.transpose() / S;

        // 5. Update State and Covariance
        Xa_ = Xa_ + K * innovation;
        Pa_ = (Eigen::Matrix2d::Identity() - K * Ha) * Pa_;
    }

    // ================================ Baro sub callback / EKF Correction ========================================
    void Estimator::callbackSubBaro_(const sensor_msgs::msg::FluidPressure msg)
    {
        // 1. Convert Pressure (Pa) to Altitude (m)
        // Using the WGS84 standard atmospheric model constant: 0.1903
        Ybaro_ = 44330.0 * (1.0 - std::pow(msg.fluid_pressure / SEA_LEVEL_PA, 0.1903));

        if (!std::isfinite(Ybaro_)) return;

        // 2. Augmented Observation Model
        Eigen::RowVector3d Hb;
        Hb << 1.0, 0.0, 1.0; // Baro measurement = Altitude (1.0) + Bias (1.0)
    
        double R = var_baro_;

        // 3. Innovation and Innovation Covariance
        double inno = Ybaro_ - (Hb * Xz_)(0);
        double S = (Hb * Pz_ * Hb.transpose())(0) + R;

        // 4. Kalman Gain (3x1 vector)
        Eigen::Vector3d K = (Pz_ * Hb.transpose()) / S;

        // 5. Update State and Covariance
        // This allows the filter to partition error into actual motion vs. sensor bias
        Xz_ = Xz_ + K * inno;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Pz_ = (I - K * Hb) * Pz_;
    }

    // ================================ IMU sub callback / EKF Prediction ========================================
    void Estimator::callbackSubIMU_(const sensor_msgs::msg::Imu msg)
    {
        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();

        if (dt < ee4308::THRES)
            return;

        // NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!
        // Store the states in Xx_, Xy_, Xz_, and Xa_ for terminal printing.
        // Store the covariances in Px_, Py_, Pz_, and Pa_ for terminal printing.
        // ==== make use of ====
        // msg.linear_acceleration
        // msg.angular_velocity
        // GRAVITY
        // var_imu_x_, var_imu_y_, var_imu_z_, var_imu_a_
        // Xx_, Xy_, Xz_, Xa_
        // Px_, Py_, Pz_, Pa_
        // dt
        // std::cos(), std::sin()
        // =========

        // 1. Get current yaw and angular velocity
        double psi = Xa_(0);
        double omega_z = msg.angular_velocity.z;

        // 2. Rotate Body-Frame Accel to World-Frame Accel
        double ux = msg.linear_acceleration.x;
        double uy = msg.linear_acceleration.y;
        double uz = msg.linear_acceleration.z;
        double ax = ux * std::cos(psi) - uy * std::sin(psi);
        double ay = ux * std::sin(psi) + uy * std::cos(psi);
        double az = uz - GRAVITY;// Coordinate acceleration (subtracting g)

        // 3. Define the Transition Matrices for X, Y, and Yaw (2-state)
        Eigen::Matrix2d F;
        F << 1, dt,
             0, 1;

        Eigen::Vector2d W;
        W << 0.5 * dt * dt,
             dt;

        // 3b. Define the Transition Matrices for Z (3-state)
        Eigen::Matrix3d Fz;
        Fz << 1, dt, 0,
              0, 1,  0,
              0, 0,  1; // The bias is assumed constant in prediction

        Eigen::Vector3d Wz;
        Wz << 0.5 * dt * dt,
              dt,
              0; // Acceleration does not directly affect barometer bias

        // --- PREDICTION UPDATES ---
        
        // standard 2D updates for X, Y, and Yaw
        Xx_ = F * Xx_ + W * ax;
        Xy_ = F * Xy_ + W * ay;
        Xa_ = F * Xa_ + W * omega_z;

        // NEW 3D update for Z
        Xz_ = Fz * Xz_ + Wz * az;

        // --- COVARIANCE UPDATES (P = FPF' + WQW') ---
        
        // standard 2D covariance updates
        Px_ = F * Px_ * F.transpose() + W * var_imu_x_ * W.transpose();
        Py_ = F * Py_ * F.transpose() + W * var_imu_y_ * W.transpose();
        Pa_ = F * Pa_ * F.transpose() + W * var_imu_a_ * W.transpose();

        // NEW 3D covariance update for Z
        Pz_ = Fz * Pz_ * Fz.transpose() + Wz * var_imu_z_ * Wz.transpose();
    }

    void Estimator::callbackSubTrueOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->true_odom_ = msg;
    }

    void Estimator::callbackTimer()
    {
        // ======= Publish and Verbose Ground Truth =======
        if (this->use_ground_truth_)
        {
            this->pub_est_odom_->publish(this->true_odom_);

            if (this->verbose_)
            {
                double t = this->now().seconds();
                std::stringstream ss;
                ss << "BaroBias: " << Xz_(2) << std::endl;  //want to see the Bias value in terminal for report
                ss << std::fixed;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "TruPose" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.x << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.y << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.z << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::getYawFromQuaternion(true_odom_.pose.pose.orientation)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "TruTwis" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.x << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.y << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.z << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.angular.z
                   << std::endl;
                std::cout << ss.str() << std::endl;
            }
        }
        // ======= Publish and Verbose KF =======
        else
        {
            // you can extend this to include velocities if you want, but the topic name may have to change from pose to something else.
            // odom is already taken.
            nav_msgs::msg::Odometry odom;

            odom.header.stamp = this->now();
            odom.child_frame_id = "base_link";     //; std::string(this->get_namespace()) + "/base_footprint";
            odom.header.frame_id = "map"; //; std::string(this->get_namespace()) + "/odom";

            odom.pose.pose.position.x = Xx_[0];
            odom.pose.pose.position.y = Xy_[0];
            odom.pose.pose.position.z = Xz_[0];
            getQuaternionFromYaw(Xa_[0], odom.pose.pose.orientation);
            odom.pose.covariance[0] = Px_(0, 0);
            odom.pose.covariance[7] = Py_(0, 0);
            odom.pose.covariance[14] = Pz_(0, 0);
            odom.pose.covariance[35] = Pa_(0, 0);

            double psi = Xa_[0];
            double v_map_x = Xx_[1];
            double v_map_y = Xy_[1];

            // Rotate Map Velocity -> Body Velocity
            double v_body_x = v_map_x * cos(psi) + v_map_y * sin(psi);
            double v_body_y = -v_map_x * sin(psi) + v_map_y * cos(psi);

            // Assign these to the odom message
            odom.twist.twist.linear.x = v_body_x;
            odom.twist.twist.linear.y = v_body_y;
            odom.twist.twist.linear.z = Xz_[1];
            odom.twist.twist.angular.z = Xa_[1];

            odom.twist.covariance[0] = Px_(1, 1);
            odom.twist.covariance[7] = Py_(1, 1);
            odom.twist.covariance[14] = Pz_(1, 1);
            odom.twist.covariance[35] = Pa_(1, 1);

            pub_est_odom_->publish(odom);

            if (verbose_)
            {
                double t = this->now().seconds();
                std::stringstream ss;
                ss << std::fixed;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Pose" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xx_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xy_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(Xa_(0))
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Twist" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xx_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xy_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xa_(1)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "ErrPose" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.x - Xx_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.y - Xy_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.z - Xz_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(ee4308::getYawFromQuaternion(true_odom_.pose.pose.orientation) - Xa_(0))
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "ErrTwis" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.x - Xx_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.y - Xy_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.z - Xz_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.angular.z - Xa_(1)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "GPS" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(2) << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Baro"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ybaro_ << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                // ss << "\t"
                //    << std::setw(7) << std::setprecision(3) << t << "\t"
                //    << std::setw(7) << "BBias"<< "\t"
                //    << std::setw(7) << "--" << "\t"
                //    << std::setw(7) << "--" << "\t"
                //    << std::setw(7) << std::setprecision(3) << Xz_(2) << "\t"
                //    << std::setw(7) << "--"
                //    << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Sonar"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ysonar_ << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Magnt"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ymagnet_
                   << std::endl;
                std::cout << ss.str() << std::endl;
            }
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Estimator);
