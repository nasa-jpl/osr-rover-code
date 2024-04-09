#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;

class Controller : public rclcpp::Node {

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Motor_FL_CON_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Motor_FR_CON_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Motor_ML_CON_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Motor_MR_CON_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Motor_RL_CON_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Motor_RR_CON_pub;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr Servo_FL_CON_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr Servo_FR_CON_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr Servo_RL_CON_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr Servo_RR_CON_pub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;

    std_msgs::msg::Float64MultiArray FL_Wheel_Velocity;
    std_msgs::msg::Float64MultiArray FR_Wheel_Velocity;
    std_msgs::msg::Float64MultiArray RL_Wheel_Velocity;
    std_msgs::msg::Float64MultiArray RR_Wheel_Velocity;
    std_msgs::msg::Float64MultiArray ML_Wheel_Velocity;
    std_msgs::msg::Float64MultiArray MR_Wheel_Velocity;

    trajectory_msgs::msg::JointTrajectoryPoint servo_fr_point;
    trajectory_msgs::msg::JointTrajectoryPoint servo_fl_point;
    trajectory_msgs::msg::JointTrajectoryPoint servo_rr_point;
    trajectory_msgs::msg::JointTrajectoryPoint servo_rl_point;


    double angle_data;

    #define ROVER_WHEEL_RADIUS 0.075

    #define d1 0.177 // Distance from the center to the front-left wheel along the x-axis
    #define d2 0.310 // Distance from the center to the front-left wheel along the y-axis
    #define d3 0.274 // Distance from the center to the front-rear wheels along the y-axis
    #define d4 0.253 // Distance from the center to the center wheel along the x-axis

    double servo_theta = 0;
    double theta = 0;
    double l = 0; // linear, angular turning radius

    double theta_front_closest;
    double theta_front_farthest;

    double angular_velocity_center, vel_middle_closest, vel_corner_closest, vel_corner_farthest, vel_middle_farthest;
    double ang_vel_middle_closest, ang_vel_corner_closest, ang_vel_corner_farthest, ang_vel_middle_farthest;
    double start_time, time, pre_time;

    geometry_msgs::msg::Twist pri_velocity;

    bool delay_ = true;

public:
    Controller() : Node("controller") {
        Motor_FL_CON_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rover_motor_fl_controller/commands", 1);
        Motor_FR_CON_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rover_motor_fr_controller/commands", 1);
        Motor_ML_CON_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rover_motor_ml_controller/commands", 1);
        Motor_MR_CON_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rover_motor_mr_controller/commands", 1);
        Motor_RL_CON_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rover_motor_rl_controller/commands", 1);
        Motor_RR_CON_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rover_motor_rr_controller/commands", 1);

        Servo_FR_CON_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rover_servo_fr_controller/joint_trajectory", 1);
        Servo_FL_CON_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rover_servo_fl_controller/joint_trajectory", 1);
        Servo_RL_CON_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rover_servo_rl_controller/joint_trajectory", 1);
        Servo_RR_CON_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/rover_servo_rr_controller/joint_trajectory", 1);

        servo_fr_point = trajectory_msgs::msg::JointTrajectoryPoint();
        servo_fl_point = trajectory_msgs::msg::JointTrajectoryPoint();
        servo_rr_point = trajectory_msgs::msg::JointTrajectoryPoint();
        servo_rl_point = trajectory_msgs::msg::JointTrajectoryPoint();

        servo_fl_point.velocities.push_back(0.0);
        servo_fl_point.time_from_start = rclcpp::Duration::from_seconds(0.2);

        servo_fr_point.velocities.push_back(0.0);
        servo_fr_point.time_from_start = rclcpp::Duration::from_seconds(0.2);

        servo_rl_point.velocities.push_back(0.0);
        servo_rl_point.time_from_start = rclcpp::Duration::from_seconds(0.2);

        servo_rr_point.velocities.push_back(0.0);
        servo_rr_point.time_from_start = rclcpp::Duration::from_seconds(0.2);
        sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&Controller::msgCallback, this, std::placeholders::_1));

    }


    void msgCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        
        if((pri_velocity.linear.x == msg->linear.x) && (pri_velocity.angular.z == msg->angular.z)) return;

        if(delay_)
        {
            delay_ = false;

            if (msg->angular.z == 0 && msg->linear.x != 0) { // linear velocity
                go_straight(msg);
                publishVelocity();
                publishAngles();
            }

            else if((msg->angular.z != 0)&&(msg->linear.x == 0))  // rotate in place
            {
                servo_fl_point.positions.clear();
                servo_fr_point.positions.clear();
                servo_rl_point.positions.clear();
                servo_rr_point.positions.clear();

                servo_fl_point.positions.push_back(-atan(d3/d1)); // FL

                servo_fr_point.positions.push_back(atan(d3/d1));  // FR

                servo_rl_point.positions.push_back(atan(d2/d1));  // RL

                servo_rr_point.positions.push_back(-atan(d2/d1)); // RR

                rotate_in_place(msg);
                publishAngles();                // servo angle pub
                publishVelocity();
            }

            else if((msg->angular.z != 0)&&(msg->linear.x != 0)) // rotation
            {
                // 1. Calculate turning radius
                twist_to_turning_radius(msg);

                // 2. Calculate servo motor angle
                calculate_servo_angle(l);

                // 3. Calculate wheel angular velocity
                calculate_drive_velocity(msg->linear.x, l);

                // 4. publish
                publishAngles();
                publishVelocity();
            }

            else if((msg->angular.z == 0)&&(msg->linear.x == 0))
            {
                stop();
                publishAngles();
                publishVelocity();
            }
            
            delay_ = true;
        }

        pri_velocity.linear.x = msg->linear.x;
        pri_velocity.angular.z = msg->angular.z;

    }

    void calculate_servo_angle(double l)
    {
        theta_front_closest = atan2(d3, abs(l) - d1);
        theta_front_farthest = atan2(d3, abs(l) + d1);

        servo_fl_point.positions.clear();
        servo_fr_point.positions.clear();
        servo_rl_point.positions.clear();
        servo_rr_point.positions.clear();


        if(l > 0)
        {
            servo_fl_point.positions.push_back(theta_front_closest);
            servo_fr_point.positions.push_back(theta_front_closest);
            servo_rl_point.positions.push_back(-theta_front_closest);
            servo_rr_point.positions.push_back(-theta_front_closest);
        }
        else
        {
            servo_fl_point.positions.push_back(-theta_front_closest);
            servo_fr_point.positions.push_back(-theta_front_closest);
            servo_rl_point.positions.push_back(theta_front_closest);
            servo_rr_point.positions.push_back(theta_front_closest);
        }
        
    }

    void calculate_drive_velocity(float velocity, double l)
    {
        angular_velocity_center = velocity / abs(l);

        vel_middle_closest = (abs(l) - d4) * angular_velocity_center;
        vel_corner_closest = hypot(abs(l) - d1, d3) * angular_velocity_center;
        vel_corner_farthest = hypot(abs(l) + d1, d3) * angular_velocity_center;
        vel_middle_farthest = (abs(l) + d4) * angular_velocity_center;

        ang_vel_middle_closest = vel_middle_closest / ROVER_WHEEL_RADIUS;
        ang_vel_corner_closest = vel_corner_closest / ROVER_WHEEL_RADIUS;
        ang_vel_corner_farthest = vel_corner_farthest / ROVER_WHEEL_RADIUS;
        ang_vel_middle_farthest = vel_middle_farthest / ROVER_WHEEL_RADIUS;

        FL_Wheel_Velocity.data.clear();
        RL_Wheel_Velocity.data.clear();
        ML_Wheel_Velocity.data.clear();
        RR_Wheel_Velocity.data.clear();
        FR_Wheel_Velocity.data.clear();
        MR_Wheel_Velocity.data.clear();


        if (l > 0)  // turning left
        {
            FL_Wheel_Velocity.data.push_back(float(ang_vel_corner_closest));
            RL_Wheel_Velocity.data.push_back(float(ang_vel_corner_closest));
            ML_Wheel_Velocity.data.push_back(float(ang_vel_middle_closest));
            RR_Wheel_Velocity.data.push_back(float(ang_vel_corner_farthest));
            FR_Wheel_Velocity.data.push_back(float(ang_vel_corner_farthest));
            MR_Wheel_Velocity.data.push_back(float(ang_vel_middle_farthest));
        }
        else        // turning right
        {
            FL_Wheel_Velocity.data.push_back(float(ang_vel_corner_farthest));
            RL_Wheel_Velocity.data.push_back(float(ang_vel_corner_farthest));
            ML_Wheel_Velocity.data.push_back(float(ang_vel_middle_farthest));
            RR_Wheel_Velocity.data.push_back(float(ang_vel_corner_closest));
            FR_Wheel_Velocity.data.push_back(float(ang_vel_corner_closest));
            MR_Wheel_Velocity.data.push_back(float(ang_vel_middle_closest));
        }

    }

    void stop()
    {
        FL_Wheel_Velocity.data.clear();
        RL_Wheel_Velocity.data.clear();
        ML_Wheel_Velocity.data.clear();
        RR_Wheel_Velocity.data.clear();
        FR_Wheel_Velocity.data.clear();
        MR_Wheel_Velocity.data.clear();

        FL_Wheel_Velocity.data.push_back(0);
        RL_Wheel_Velocity.data.push_back(0);
        ML_Wheel_Velocity.data.push_back(0);
        RR_Wheel_Velocity.data.push_back(0);
        FR_Wheel_Velocity.data.push_back(0);
        MR_Wheel_Velocity.data.push_back(0);

        servo_fl_point.positions.clear();
        servo_fr_point.positions.clear();
        servo_rl_point.positions.clear();
        servo_rr_point.positions.clear();

        servo_fl_point.positions.push_back(0); // FL

        servo_fr_point.positions.push_back(0);  // FR

        servo_rl_point.positions.push_back(0);  // RL

        servo_rr_point.positions.push_back(0); // RR
    }

    void go_straight(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double velocity_data = msg->linear.x / ROVER_WHEEL_RADIUS;

        FL_Wheel_Velocity.data.clear();
        FL_Wheel_Velocity.data.push_back(velocity_data);

        RL_Wheel_Velocity.data.clear();
        RL_Wheel_Velocity.data.push_back(velocity_data);

        ML_Wheel_Velocity.data.clear();
        ML_Wheel_Velocity.data.push_back(velocity_data);

        FR_Wheel_Velocity.data.clear();
        FR_Wheel_Velocity.data.push_back(velocity_data);

        RR_Wheel_Velocity.data.clear();
        RR_Wheel_Velocity.data.push_back(velocity_data);

        MR_Wheel_Velocity.data.clear();
        MR_Wheel_Velocity.data.push_back(velocity_data);
        
        servo_fl_point.positions.clear();
        servo_fr_point.positions.clear();
        servo_rl_point.positions.clear();
        servo_rr_point.positions.clear();

        servo_fl_point.positions.push_back(0); // FL

        servo_fr_point.positions.push_back(0);  // FR

        servo_rl_point.positions.push_back(0);  // RL

        servo_rr_point.positions.push_back(0); // RR

    }

    void twist_to_turning_radius(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        l = msg->linear.x / msg->angular.z;
    }

    void rotate_in_place(const geometry_msgs::msg::Twist::SharedPtr& msg)
    {
        FL_Wheel_Velocity.data.clear();
        FL_Wheel_Velocity.data.push_back(-float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS));

        RL_Wheel_Velocity.data.clear();
        RL_Wheel_Velocity.data.push_back(-float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS));

        ML_Wheel_Velocity.data.clear();
        ML_Wheel_Velocity.data.push_back(-float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS));

        FR_Wheel_Velocity.data.clear();
        FR_Wheel_Velocity.data.push_back(float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS));

        RR_Wheel_Velocity.data.clear();
        RR_Wheel_Velocity.data.push_back(float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS));

        MR_Wheel_Velocity.data.clear();
        MR_Wheel_Velocity.data.push_back(float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS));
    }

    void publishVelocity() {
        Motor_FL_CON_pub->publish(FL_Wheel_Velocity);
        Motor_FR_CON_pub->publish(FR_Wheel_Velocity);
        Motor_RL_CON_pub->publish(RL_Wheel_Velocity);
        Motor_RR_CON_pub->publish(RR_Wheel_Velocity);
        Motor_ML_CON_pub->publish(ML_Wheel_Velocity);
        Motor_MR_CON_pub->publish(MR_Wheel_Velocity);
    }

    void publishAngles()
    {
        trajectory_msgs::msg::JointTrajectory servo_fr = trajectory_msgs::msg::JointTrajectory();
        trajectory_msgs::msg::JointTrajectory servo_fl = trajectory_msgs::msg::JointTrajectory();
        trajectory_msgs::msg::JointTrajectory servo_rr = trajectory_msgs::msg::JointTrajectory();
        trajectory_msgs::msg::JointTrajectory servo_rl = trajectory_msgs::msg::JointTrajectory();

        servo_fr.joint_names.push_back("front_wheel_joint_R");
        servo_fl.joint_names.push_back("front_wheel_joint_L");
        servo_rr.joint_names.push_back("rear_wheel_joint_R");
        servo_rl.joint_names.push_back("rear_wheel_joint_L");

        servo_fl.points.push_back(servo_fl_point);
        servo_fr.points.push_back(servo_fr_point);
        servo_rl.points.push_back(servo_rl_point);
        servo_rr.points.push_back(servo_rr_point);

        Servo_FL_CON_pub->publish(servo_fl);
        Servo_FR_CON_pub->publish(servo_fr);
        Servo_RL_CON_pub->publish(servo_rl);
        Servo_RR_CON_pub->publish(servo_rr);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    // Using std::chrono to create a rate object
    auto rate = std::chrono::milliseconds(100); // 200 Hz = 5ms per loop

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        // Manually sleep to maintain the loop rate
        std::this_thread::sleep_for(rate);
    }
    rclcpp::shutdown();
    return 0;
}
