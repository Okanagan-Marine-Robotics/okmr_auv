#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "okmr_msgs/msg/dvl.hpp"
#include "okmr_msgs/msg/sensor_reading.hpp"
#include "okmr_msgs/srv/clear_pose.hpp"
#include "okmr_msgs/srv/get_pose_twist_accel.hpp"
#include "okmr_msgs/srv/set_dead_reckoning_enabled.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

typedef unsigned int uint32;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class DeadReckoningNode : public rclcpp::Node {
   public:
    // Pose estimates using Vector3
    geometry_msgs::msg::Vector3 translation_estimate;
    geometry_msgs::msg::Vector3 rotation_estimate;

    // Current state messages
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::TwistStamped current_twist;
    geometry_msgs::msg::AccelStamped current_accel;
    geometry_msgs::msg::AccelStamped gravity_body;

    // Cached sensor messages
    sensor_msgs::msg::Imu current_imu_msg;
    okmr_msgs::msg::Dvl current_dvl_msg;

    // DVL-derived acceleration
    geometry_msgs::msg::Vector3 dvl_accel;
    float dvl_beams[4] = {0, 0, 0, 0};

    // Filtered/smoothed values using Vector3
    geometry_msgs::msg::Vector3 smoothed_angular_vel;
    geometry_msgs::msg::Vector3 prev_angular_vel;

    // State tracking
    bool gotFirstTime = false;
    bool gotFirstDVLTime = false;
    bool is_dead_reckoning_enabled = false;
    rclcpp::Time last_time;
    rclcpp::Time last_dvl_time;

    // Parameters for filtering
    double update_frequency_ = 200.0;
    double complementary_filter_alpha_ = 0.995;
    double dvl_velocity_alpha_ = 1.0;
    double dvl_accel_alpha_ = 1.0;
    double dvl_accel_smoothing_alpha_ = 0.7;
    double angular_vel_filter_alpha_ = 0.7;
    double angular_accel_filter_alpha_ = 0.7;
    double dvl_altitude_filter_alpha_ = 0.02;

    // TF2 for coordinate transforms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_id_ = "base_link";

    DeadReckoningNode () : Node ("dead_reckoning_node") {
        // Declare parameters with descriptors
        rcl_interfaces::msg::ParameterDescriptor desc;

        desc.description = "Update frequency for state estimation timer in Hz";
        desc.floating_point_range.resize (1);
        desc.floating_point_range[0].from_value = 1.0;
        desc.floating_point_range[0].to_value = 1000.0;
        this->declare_parameter ("update_frequency", update_frequency_, desc);

        desc.description =
            "Complementary filter coefficient for IMU attitude estimation (0.0=accel only, "
            "1.0=gyro only)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("complementary_filter_alpha", complementary_filter_alpha_, desc);

        desc.description =
            "Complementary filter coefficient for DVL velocity fusion (0.0=IMU only, 1.0=DVL only)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("dvl_velocity_alpha", dvl_velocity_alpha_, desc);

        desc.description =
            "Complementary filter coefficient for DVL acceleration fusion (0.0=IMU only, 1.0=DVL "
            "only)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("dvl_accel_alpha", dvl_accel_alpha_, desc);

        desc.description =
            "Leaky integrator coefficient for linear acceleration smoothing (higher=more "
            "smoothing)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("dvl_accel_smoothing_alpha", dvl_accel_smoothing_alpha_, desc);

        desc.description =
            "Leaky integrator coefficient for angular velocity smoothing (higher=more smoothing)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("angular_vel_filter_alpha", angular_vel_filter_alpha_, desc);

        desc.description =
            "Leaky integrator coefficient for angular acceleration smoothing (higher=more "
            "smoothing)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("angular_accel_filter_alpha", angular_accel_filter_alpha_, desc);

        desc.description =
            "Complementary filter coefficient for DVL altitude estimation (0.0=dead reckoning "
            "only, 1.0=DVL altitude only)";
        desc.floating_point_range[0].from_value = 0.0;
        desc.floating_point_range[0].to_value = 1.0;
        this->declare_parameter ("dvl_altitude_filter_alpha", dvl_altitude_filter_alpha_, desc);

        // Get parameters
        update_frequency_ = this->get_parameter ("update_frequency").as_double ();
        complementary_filter_alpha_ =
            this->get_parameter ("complementary_filter_alpha").as_double ();
        dvl_velocity_alpha_ = this->get_parameter ("dvl_velocity_alpha").as_double ();
        dvl_accel_alpha_ = this->get_parameter ("dvl_accel_alpha").as_double ();
        dvl_accel_smoothing_alpha_ = this->get_parameter ("dvl_accel_smoothing_alpha").as_double ();
        angular_vel_filter_alpha_ = this->get_parameter ("angular_vel_filter_alpha").as_double ();
        angular_accel_filter_alpha_ =
            this->get_parameter ("angular_accel_filter_alpha").as_double ();
        dvl_altitude_filter_alpha_ = this->get_parameter ("dvl_altitude_filter_alpha").as_double ();

        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

        // Parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback (
            std::bind (&DeadReckoningNode::on_parameter_change, this, std::placeholders::_1));

        rclcpp::QoS qos_profile (10);  // Create QoS profile with history depth 10
        qos_profile.reliability (rclcpp::ReliabilityPolicy::BestEffort);

        imu_subscription = this->create_subscription<sensor_msgs::msg::Imu> (
            "/imu", qos_profile, std::bind (&DeadReckoningNode::imu_callback, this, _1));

        dvl_subscription = this->create_subscription<okmr_msgs::msg::Dvl> (
            "/dvl", 10, std::bind (&DeadReckoningNode::dvl_callback, this, _1));

        // Publishers
        pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/pose", 10);
        twist_publisher =
            this->create_publisher<geometry_msgs::msg::TwistStamped> ("/velocity", 10);
        accel_publisher =
            this->create_publisher<geometry_msgs::msg::AccelStamped> ("/acceleration", 10);
        gravity_publisher =
            this->create_publisher<geometry_msgs::msg::AccelStamped> ("/gravity", 10);

        // Services
        get_pose_twist_accel_service = this->create_service<okmr_msgs::srv::GetPoseTwistAccel> (
            "/get_pose_twist_accel", std::bind (&DeadReckoningNode::get_pose_twist_accel_callback,
                                               this, std::placeholders::_1, std::placeholders::_2));

        set_dead_reckoning_service = this->create_service<okmr_msgs::srv::SetDeadReckoningEnabled> (
            "/set_dead_reckoning_enabled",
            std::bind (&DeadReckoningNode::set_dead_reckoning_callback, this, std::placeholders::_1,
                       std::placeholders::_2));

        clear_pose_service = this->create_service<okmr_msgs::srv::ClearPose> (
            "/clear_pose", std::bind (&DeadReckoningNode::clear_pose_callback, this,
                                      std::placeholders::_1, std::placeholders::_2));

        // Timer for regular updates
        auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds> (
            std::chrono::duration<double> (1.0 / update_frequency_));

        timer_ =
            this->create_wall_timer (timer_period, std::bind (&DeadReckoningNode::update, this));

        RCLCPP_INFO (this->get_logger (), "DeadReckoningNode initialized with %f Hz update rate",
                     update_frequency_);
    }

   private:
    void imu_callback (const sensor_msgs::msg::Imu& msg) {
        // Cache IMU data - actual processing will happen in update() method
        current_imu_msg = msg;
        if (!gotFirstTime) {
            last_time = this->now ();
            gotFirstTime = true;
        }
    }

    void dvl_callback (const okmr_msgs::msg::Dvl::ConstSharedPtr msg) {
        auto current_dvl_time = this->now ();

        // Transform DVL velocity to base_link frame
        auto transformed_velocity = transform_dvl_data (*msg);
        geometry_msgs::msg::Vector3 recent_dvl_accel;

        // Calculate DVL acceleration using transformed velocities
        if (gotFirstDVLTime && (current_dvl_time - last_dvl_time).seconds () > 0.0) {
            double dvl_dt = (current_dvl_time - last_dvl_time).seconds ();
            auto prev_transformed_velocity = transform_dvl_data (current_dvl_msg);
            recent_dvl_accel.x = (transformed_velocity.x - prev_transformed_velocity.x) / dvl_dt;
            recent_dvl_accel.y = (transformed_velocity.y - prev_transformed_velocity.y) / dvl_dt;
            recent_dvl_accel.z = (transformed_velocity.z - prev_transformed_velocity.z) / dvl_dt;
            // RCLCPP_INFO(this->get_logger(), "%f", dvl_dt);
        } else {
            recent_dvl_accel.x = recent_dvl_accel.y = recent_dvl_accel.z = 0.0;
            dvl_accel.x = dvl_accel.y = dvl_accel.z = 0.0;
            gotFirstDVLTime = true;
        }

        // Cache DVL data and update timing
        current_dvl_msg = *msg;
        last_dvl_time = current_dvl_time;

        // apply dvl smoothing using leaky integrator
        dvl_accel.x = dvl_accel_smoothing_alpha_ * dvl_accel.x +
                      (1.0 - dvl_accel_smoothing_alpha_) * recent_dvl_accel.x;

        dvl_accel.y = dvl_accel_smoothing_alpha_ * dvl_accel.y +
                      (1.0 - dvl_accel_smoothing_alpha_) * recent_dvl_accel.y;

        dvl_accel.z = dvl_accel_smoothing_alpha_ * dvl_accel.z +
                      (1.0 - dvl_accel_smoothing_alpha_) * recent_dvl_accel.z;

        // Apply complementary filter to linear velocity estimate
        current_twist.twist.linear.x = dvl_velocity_alpha_ * transformed_velocity.x +
                                       (1.0 - dvl_velocity_alpha_) * current_twist.twist.linear.x;
        current_twist.twist.linear.y = dvl_velocity_alpha_ * transformed_velocity.y +
                                       (1.0 - dvl_velocity_alpha_) * current_twist.twist.linear.y;
        current_twist.twist.linear.z = dvl_velocity_alpha_ * transformed_velocity.z +
                                       (1.0 - dvl_velocity_alpha_) * current_twist.twist.linear.z;

        for (int i = 0; i < 4; i++) {
            dvl_beams[i] = msg->beam_distances[i];
        }
    }

    void get_pose_twist_accel_callback (
        const std::shared_ptr<okmr_msgs::srv::GetPoseTwistAccel::Request> request,
        std::shared_ptr<okmr_msgs::srv::GetPoseTwistAccel::Response> response) {
        (void)request;  // Unused parameter

        response->pose = current_pose.pose;
        response->twist = current_twist.twist;
        response->accel = current_accel.accel;
        response->success =
            gotFirstTime & gotFirstDVLTime;  // Only return success if we've received at least one
                                             // IMU and DVL measurement
    }

    void set_dead_reckoning_callback (
        const std::shared_ptr<okmr_msgs::srv::SetDeadReckoningEnabled::Request> request,
        std::shared_ptr<okmr_msgs::srv::SetDeadReckoningEnabled::Response> response) {
        is_dead_reckoning_enabled = request->enable;
        response->success = true;
        response->message = request->enable ? "Dead reckoning enabled" : "Dead reckoning disabled";
        RCLCPP_INFO (this->get_logger (), "%s", response->message.c_str ());
    }

    void clear_pose_callback (const std::shared_ptr<okmr_msgs::srv::ClearPose::Request> request,
                              std::shared_ptr<okmr_msgs::srv::ClearPose::Response> response) {
        (void)request;  // Unused parameter

        // Reset pose estimates to zero (clear the integration)
        translation_estimate.x = 0.0;
        translation_estimate.y = 0.0;
        translation_estimate.z = 0.0;

        response->success = true;
        response->message = "Pose cleared - integration reset to origin";
        RCLCPP_INFO (this->get_logger (), "Pose cleared - integration reset to origin");
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change (
        const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name () == "update_frequency") {
                update_frequency_ = param.as_double ();
                if (update_frequency_ <= 0.0) {
                    result.successful = false;
                    result.reason = "Update frequency must be positive";
                    return result;
                }

                // Update timer period
                auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds> (
                    std::chrono::duration<double> (1.0 / update_frequency_));

                timer_->cancel ();
                timer_ = this->create_wall_timer (timer_period,
                                                  std::bind (&DeadReckoningNode::update, this));
            } else if (param.get_name () == "complementary_filter_alpha") {
                complementary_filter_alpha_ = param.as_double ();
            } else if (param.get_name () == "dvl_velocity_alpha") {
                dvl_velocity_alpha_ = param.as_double ();
            } else if (param.get_name () == "dvl_accel_alpha") {
                dvl_accel_alpha_ = param.as_double ();
            } else if (param.get_name () == "angular_vel_filter_alpha") {
                angular_vel_filter_alpha_ = param.as_double ();
            } else if (param.get_name () == "angular_accel_filter_alpha") {
                angular_accel_filter_alpha_ = param.as_double ();
            } else if (param.get_name () == "dvl_altitude_filter_alpha") {
                dvl_altitude_filter_alpha_ = param.as_double ();
            }
        }

        return result;
    }

    void update () {
        if (!gotFirstTime) {
            return;
        }

        auto current_time = this->now ();
        double dt = (current_time - last_time).seconds ();

        if (dt <= 0.0) {
            return;  // Skip if no time has passed
        }

        last_time = current_time;

        // Process IMU data and update angular velocities
        process_imu_data ();

        update_attitude_estimation (dt); 

        // Modified to always set the orientation. 
        tf2::Quaternion q;
        q.setRPY (rotation_estimate.x, rotation_estimate.y, rotation_estimate.z);
        current_pose.pose.orientation = tf2::toMsg (q);
    
        if (is_dead_reckoning_enabled) {
            // Process pose integration if enabled
            integrate_pose (dt);
        }

        // Calculate accelerations with gravity compensation
        calculate_accelerations (dt);

        // Update message headers and publish all data
        publish_state_estimates (current_time);
    }

    geometry_msgs::msg::Vector3 transform_imu_data (const sensor_msgs::msg::Imu& imu_msg,
                                                    bool is_angular) {
        try {
            // Get transform from IMU frame to base_link
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform (
                base_frame_id_, imu_msg.header.frame_id, tf2::TimePointZero);

            // Create a vector3 stamped message for transformation
            geometry_msgs::msg::Vector3Stamped input_vector, output_vector;
            input_vector.header = imu_msg.header;

            if (is_angular) {
                input_vector.vector = imu_msg.angular_velocity;
            } else {
                input_vector.vector = imu_msg.linear_acceleration;
            }

            // Transform the vector
            tf2::doTransform (input_vector, output_vector, transform);
            return output_vector.vector;

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN (this->get_logger (), "Could not transform IMU data: %s", ex.what ());
            // Fallback to no transformation
            if (is_angular) {
                return current_imu_msg.angular_velocity;
            } else {
                return current_imu_msg.linear_acceleration;
            }
        }
    }

    geometry_msgs::msg::Vector3 transform_dvl_data (const okmr_msgs::msg::Dvl& dvl_msg) {
        try {
            // Get transform from DVL frame to base_link
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform (
                base_frame_id_, dvl_msg.header.frame_id, tf2::TimePointZero);

            // Create a vector3 stamped message for transformation
            geometry_msgs::msg::Vector3Stamped input_vector, output_vector;
            input_vector.header = dvl_msg.header;
            input_vector.vector = dvl_msg.velocity;

            // Transform the vector
            tf2::doTransform (input_vector, output_vector, transform);
            return output_vector.vector;

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN (this->get_logger (), "Could not transform DVL data: %s", ex.what ());
            // Fallback to no transformation
            return dvl_msg.velocity;
        }
    }

    void process_imu_data () {
        // Transform angular velocities from IMU frame to base_link
        auto angular_velocity = transform_imu_data (current_imu_msg, true);

        // TODO make these a parameter
        double angular_velocity_roll = (abs (angular_velocity.x) > 0.01) ? angular_velocity.x : 0.0;
        double angular_velocity_pitch =
            (abs (angular_velocity.y) > 0.01) ? angular_velocity.y : 0.0;
        double angular_velocity_yaw = (abs (angular_velocity.z) > 0.01) ? angular_velocity.z : 0.0;

        // Update current twist angular velocity (always published)
        current_twist.twist.angular.x = angular_velocity_roll;
        current_twist.twist.angular.y = angular_velocity_pitch;
        current_twist.twist.angular.z = angular_velocity_yaw;
    }

    void update_attitude_estimation (double dt) {
        // Transform linear acceleration from IMU frame to base_link
        auto linear_accel = transform_imu_data (current_imu_msg, false);
        double ax = linear_accel.x;
        double ay = linear_accel.y;
        double az = linear_accel.z;
        double magnitude = sqrt (ax * ax + ay * ay + az * az);
        ax /= magnitude;
        ay /= magnitude;
        az /= magnitude;

        double alpha = complementary_filter_alpha_;

        double accel_pitch = atan2 (-ax, sqrt (ay * ay + az * az));
        double accel_roll = atan2 (ay, az);

        // RCLCPP_INFO(this->get_logger(), "pitch: %f \t roll: %f", accel_pitch * 180.0 / M_PI,
        // accel_roll * 180.0 / M_PI);

        if (std::abs (accel_pitch) * 180.0 / M_PI > 80.0 ||
            std::abs (accel_roll) * 180.0 / M_PI > 80.0) {
            // alpha = 1.0;
        }

        // Complementary filter for attitude
        rotation_estimate.y = alpha * (rotation_estimate.y + current_twist.twist.angular.y * dt) +
                              (1 - alpha) * accel_pitch;
        rotation_estimate.x = alpha * (rotation_estimate.x + current_twist.twist.angular.x * dt) +
                              (1 - alpha) * accel_roll;
        rotation_estimate.z += current_twist.twist.angular.z * dt;
    }

    void integrate_pose (double dt) {
        // Integrate velocity for pose estimation
        tf2::Quaternion q;
        q.setRPY (rotation_estimate.x, rotation_estimate.y, rotation_estimate.z);
        tf2::Matrix3x3 tf_R (q);
        tf2::Vector3 translation_update = tf_R * tf2::Vector3 (current_twist.twist.linear.x * dt,
                                                               current_twist.twist.linear.y * dt,
                                                               current_twist.twist.linear.z * dt);
        translation_estimate.x += translation_update.x ();
        translation_estimate.y += translation_update.y ();
        translation_estimate.z += translation_update.z ();

        // Calculate average altitude from DVL beam distances
        double dvl_altitude = 0.0;
        int valid_beams = 0;
        for (int i = 0; i < 4; i++) {
            if (dvl_beams[i] > 0.0) {  // Only use valid beam readings
                dvl_altitude += dvl_beams[i];
                valid_beams++;
            }
        }

        if (valid_beams > 0) {
            dvl_altitude /= valid_beams;  // Average the valid beam distances

            // Apply complementary filter for altitude estimation
            translation_estimate.z = dvl_altitude_filter_alpha_ * dvl_altitude +
                                     (1.0 - dvl_altitude_filter_alpha_) * translation_estimate.z;
        }

        // Update pose message
        // current_pose.pose.orientation = tf2::toMsg (q); Handled in update loop now
        current_pose.pose.position.x = translation_estimate.x;
        current_pose.pose.position.y = translation_estimate.y;
        current_pose.pose.position.z = translation_estimate.z;
    }

    void calculate_accelerations (double dt) {
        // Transform linear acceleration from IMU frame to base_link
        /*
        auto linear_accel = transform_imu_data(current_imu_msg, false);
        double ax = linear_accel.x;
        double ay = linear_accel.y;
        double az = linear_accel.z;
        */

        // Calculate linear acceleration with gravity compensation
        tf2::Quaternion q;
        q.setRPY (rotation_estimate.x, rotation_estimate.y, rotation_estimate.z);
        tf2::Matrix3x3 tf_R (q);
        tf2::Vector3 gravity_world (0, 0, -9.807);
        tf2::Vector3 gravity_body_vector = tf_R.transpose () * gravity_world;
        // rotation matrix is orthogonal, so transpose = inverse
        // this is transforming the gravity vector from world to body frame

        gravity_body.accel.linear.x = gravity_body_vector.x ();
        gravity_body.accel.linear.y = gravity_body_vector.y ();
        gravity_body.accel.linear.z = gravity_body_vector.z ();

        // Calculate IMU true linear acceleration (subtract gravity)
        /*
        double imu_accel_x = ax - gravity_body_vector.x();
        double imu_accel_y = ay - gravity_body_vector.y();
        double imu_accel_z = az - gravity_body_vector.z();

        // Combine DVL acceleration with IMU acceleration using complementary filtering

        current_accel.accel.linear.x = dvl_accel_alpha_ * dvl_accel.x + (1.0 - dvl_accel_alpha_) *
        imu_accel_x; current_accel.accel.linear.y = dvl_accel_alpha_ * dvl_accel.y + (1.0 -
        dvl_accel_alpha_) * imu_accel_y; current_accel.accel.linear.z = dvl_accel_alpha_ *
        dvl_accel.z + (1.0 - dvl_accel_alpha_) * imu_accel_z;

        //unused at the moment because weird readings from sim
        //gravity calculation may not be accurate enough
        */

        // infering the linear velocity from the current acceleration
        current_twist.twist.linear.x += current_accel.accel.linear.x * dt;
        current_twist.twist.linear.y += current_accel.accel.linear.z * dt;
        current_twist.twist.linear.z += current_accel.accel.linear.z * dt;

        // Calculate angular acceleration (derivative of smoothed angular velocity)
        smoothed_angular_vel.x = angular_vel_filter_alpha_ * smoothed_angular_vel.x +
                                 (1.0 - angular_vel_filter_alpha_) * current_twist.twist.angular.x;
        smoothed_angular_vel.y = angular_vel_filter_alpha_ * smoothed_angular_vel.y +
                                 (1.0 - angular_vel_filter_alpha_) * current_twist.twist.angular.y;
        smoothed_angular_vel.z = angular_vel_filter_alpha_ * smoothed_angular_vel.z +
                                 (1.0 - angular_vel_filter_alpha_) * current_twist.twist.angular.z;

        current_accel.accel.angular.x =
            angular_accel_filter_alpha_ * current_accel.accel.angular.x +
            (1.0 - angular_accel_filter_alpha_) * (smoothed_angular_vel.x - prev_angular_vel.x) /
                dt;
        current_accel.accel.angular.y =
            angular_accel_filter_alpha_ * current_accel.accel.angular.y +
            (1.0 - angular_accel_filter_alpha_) * (smoothed_angular_vel.y - prev_angular_vel.y) /
                dt;
        current_accel.accel.angular.z =
            angular_accel_filter_alpha_ * current_accel.accel.angular.z +
            (1.0 - angular_accel_filter_alpha_) * (smoothed_angular_vel.z - prev_angular_vel.z) /
                dt;

        prev_angular_vel = smoothed_angular_vel;
    }

    void publish_state_estimates (rclcpp::Time current_time) {
        // Update message headers
        current_pose.header.stamp = current_time;
        current_pose.header.frame_id = "map";
        current_twist.header.stamp = current_time;
        current_twist.header.frame_id = "base_link";
        current_accel.header.stamp = current_time;
        current_accel.header.frame_id = "base_link";
        gravity_body.header.stamp = current_time;
        gravity_body.header.frame_id = "base_link";

        // Publish all state estimates
        pose_publisher->publish (current_pose);

        // convert all angular values to degrees from radians
        geometry_msgs::msg::TwistStamped twist_degrees = current_twist;
        twist_degrees.twist.angular.x = current_twist.twist.angular.x * 180.0 / M_PI;
        twist_degrees.twist.angular.y = current_twist.twist.angular.y * 180.0 / M_PI;
        twist_degrees.twist.angular.z = current_twist.twist.angular.z * 180.0 / M_PI;
        twist_publisher->publish (twist_degrees);

        // convert all angular values to degrees from radians
        geometry_msgs::msg::AccelStamped accel_degrees = current_accel;
        accel_degrees.accel.angular.x = current_accel.accel.angular.x * 180.0 / M_PI;
        accel_degrees.accel.angular.y = current_accel.accel.angular.y * 180.0 / M_PI;
        accel_degrees.accel.angular.z = current_accel.accel.angular.z * 180.0 / M_PI;
        accel_publisher->publish (accel_degrees);

        gravity_publisher->publish (gravity_body);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    rclcpp::Subscription<okmr_msgs::msg::Dvl>::SharedPtr dvl_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr gravity_publisher;
    rclcpp::Service<okmr_msgs::srv::GetPoseTwistAccel>::SharedPtr get_pose_twist_accel_service;
    rclcpp::Service<okmr_msgs::srv::SetDeadReckoningEnabled>::SharedPtr set_dead_reckoning_service;
    rclcpp::Service<okmr_msgs::srv::ClearPose>::SharedPtr clear_pose_service;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main (int argc, char* argv[]) {
    rclcpp::init (argc, argv);
    rclcpp::spin (std::make_shared<DeadReckoningNode> ());
    rclcpp::shutdown ();

    return 0;
}
