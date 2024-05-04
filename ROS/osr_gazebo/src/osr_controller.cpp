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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_wheel_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_pub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;

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

    double FL_data, FR_data, ML_data, MR_data, RL_data, RR_data;

    double FR_servo_data, FL_servo_data, RR_servo_data, RL_servo_data;

    bool delay_ = true;

public:
    Controller() : Node("controller") {
        motor_wheel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 1);
        servo_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/servo_controller/joint_trajectory", 1);

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
                FL_servo_data = -atan(d3/d1);
                FR_servo_data = atan(d3/d1);
                RL_servo_data = atan(d2/d1);
                RR_servo_data = -atan(d2/d1);

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

        if(l > 0)
        {
            FL_servo_data = theta_front_closest;
            FR_servo_data = theta_front_closest;
            RL_servo_data = -theta_front_closest;
            RR_servo_data = -theta_front_closest;
        }
        else
        {
            FL_servo_data = -theta_front_closest;
            FR_servo_data = -theta_front_closest;
            RL_servo_data = theta_front_closest;
            RR_servo_data = theta_front_closest;
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

        if (l > 0)  // turning left
        {
            FL_data = float(ang_vel_corner_closest);
            RL_data = float(ang_vel_corner_closest);

            ML_data = float(ang_vel_middle_closest);
            FR_data = float(ang_vel_corner_farthest);

            RR_data = float(ang_vel_corner_farthest);
            MR_data = float(ang_vel_middle_farthest);
        }
        else        // turning right
        {
            FL_data = float(ang_vel_corner_farthest);
            RL_data = float(ang_vel_corner_farthest);

            ML_data = float(ang_vel_middle_farthest);
            FR_data = float(ang_vel_corner_closest);

            RR_data = float(ang_vel_corner_closest);
            MR_data = float(ang_vel_middle_closest);

        }

    }

    void stop()
    {
        FL_data = 0;
        RL_data = 0;

        ML_data = 0;
        FR_data = 0;

        RR_data = 0;
        MR_data = 0;


        FL_servo_data = 0;
        FR_servo_data = 0;
        RL_servo_data = 0;
        RR_servo_data = 0;
    }

    void go_straight(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double velocity_data = msg->linear.x / ROVER_WHEEL_RADIUS;

        FL_data = velocity_data;
        FR_data = velocity_data;
        ML_data = velocity_data;
        MR_data = velocity_data;
        RL_data = velocity_data;
        RR_data = velocity_data;

        FL_servo_data = 0;
        FR_servo_data = 0;
        RL_servo_data = 0;
        RR_servo_data = 0;

    }

    void twist_to_turning_radius(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        l = msg->linear.x / msg->angular.z;
    }

    void rotate_in_place(const geometry_msgs::msg::Twist::SharedPtr& msg)
    {
        FL_data = -float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS);
        RL_data = -float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS);

        ML_data = -float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS);
        FR_data = float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS);

        RR_data = float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS);
        MR_data = float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS);
    }

    void publishVelocity() {

        std_msgs::msg::Float64MultiArray wheel;

        wheel.data = {ML_data, MR_data,
                        FL_data, FR_data,
                        RL_data, RR_data};

        motor_wheel_pub->publish(wheel);
    }

    void publishAngles()
    {

        auto servo = trajectory_msgs::msg::JointTrajectory();
        servo.joint_names = {"front_wheel_joint_R", "front_wheel_joint_L", "rear_wheel_joint_R", "rear_wheel_joint_L"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {FR_servo_data, FL_servo_data,
                           RR_servo_data, RL_servo_data};
        point.velocities = {0.0, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(0.5);

        servo.points.push_back(point);
        servo_pub->publish(servo);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    // Using std::chrono to create a rate object
    auto rate = std::chrono::milliseconds(600); // 200 Hz = 5ms per loop

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Manually sleep to maintain the loop rate
        std::this_thread::sleep_for(rate);
    }
    rclcpp::shutdown();
    return 0;
}
