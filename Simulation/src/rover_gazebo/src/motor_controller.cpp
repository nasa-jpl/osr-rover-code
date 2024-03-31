#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>


class Controller
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_imu;
    ros::NodeHandle nh_joint;

    ros::Publisher FL_CON_pub;
    ros::Publisher FR_CON_pub;
    ros::Publisher RL_CON_pub;
    ros::Publisher RR_CON_pub;

    ros::Publisher Motor_FL_CON_pub;
    ros::Publisher Motor_FR_CON_pub;
    ros::Publisher Motor_RL_CON_pub;
    ros::Publisher Motor_RR_CON_pub;
    ros::Publisher Motor_ML_CON_pub;
    ros::Publisher Motor_MR_CON_pub;

    ros::Publisher odom_pub;

    ros::Subscriber sub;
    ros::Subscriber imu_sub;
    ros::Subscriber joint_sub;

    std_msgs::Float64 FL_angle;
    std_msgs::Float64 FR_angle;
    std_msgs::Float64 RL_angle;
    std_msgs::Float64 RR_angle;

    std_msgs::Float64 FL_Wheel_Velocity;
    std_msgs::Float64 FR_Wheel_Velocity;
    std_msgs::Float64 RL_Wheel_Velocity;
    std_msgs::Float64 RR_Wheel_Velocity;
    std_msgs::Float64 ML_Wheel_Velocity;
    std_msgs::Float64 MR_Wheel_Velocity;

    nav_msgs::Odometry odom_;

    double fl_vel, fr_vel, ml_vel, mr_vel, rl_vel, rr_vel;
    double ang_cur[6];
    double ang_pre[6];

    geometry_msgs::Twist Linear;

    double x = 0;
    double y = 0;

    #define d1 0.177 // Distance from the center to the front-left wheel along the x-axis
    #define d2 0.310 // Distance from the center to the front-left wheel along the y-axis
    #define d3 0.274 // Distance from the center to the front-rear wheels along the y-axis
    #define d4 0.253 // Distance from the center to the center wheel along the x-axis

    #define ROVER_WHEEL_RADIUS 0.075
    #define ROVER_WHEELBASE 0.506 
    // drive_no_load_rpm = 223.0

    double servo_theta = 0;
    double theta = 0;
    double l = 0; // linear, angular turning radius

    double current_dl, dl, pre_dl;

    double x_postion, y_postion;

    double theta_front_closest;
    double theta_front_farthest;

    double angular_velocity_center, vel_middle_closest, vel_corner_closest, vel_corner_farthest, vel_middle_farthest;
    double ang_vel_middle_closest, ang_vel_corner_closest, ang_vel_corner_farthest, ang_vel_middle_farthest;
    double start_time, time, pre_time;
public:

    Controller()
    {
        FL_CON_pub = nh.advertise<std_msgs::Float64>("FL_CON/command", 2);
        FR_CON_pub = nh.advertise<std_msgs::Float64>("FR_CON/command", 2);
        RL_CON_pub = nh.advertise<std_msgs::Float64>("RL_CON/command", 2);
        RR_CON_pub = nh.advertise<std_msgs::Float64>("RR_CON/command", 2);

        Motor_FL_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_fl_controller/command", 2);
        Motor_FR_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_fr_controller/command", 2);
        Motor_RL_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_rl_controller/command", 2);
        Motor_RR_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_rr_controller/command", 2);
        Motor_ML_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_ml_controller/command", 2);
        Motor_MR_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_mr_controller/command", 2);

        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);

        sub = nh.subscribe("cmd_vel", 5, &Controller::msgCallback, this);
        joint_sub = nh_joint.subscribe("joint_states", 5, &Controller::jointCallback, this);
        imu_sub = nh_imu.subscribe("imu", 5, &Controller::imuCallback, this);
    }

    // sevo angle
    void publishAngles()
    {
        FL_CON_pub.publish(FL_angle);
        FR_CON_pub.publish(FR_angle);
        RL_CON_pub.publish(RL_angle);
        RR_CON_pub.publish(RR_angle);
    }

    // wheel angular velocity
    void publishVelocity() 
    {
        Motor_FL_CON_pub.publish(FL_Wheel_Velocity);
        Motor_FR_CON_pub.publish(FR_Wheel_Velocity);
        Motor_RL_CON_pub.publish(RL_Wheel_Velocity);
        Motor_RR_CON_pub.publish(RR_Wheel_Velocity);
        Motor_ML_CON_pub.publish(ML_Wheel_Velocity);
        Motor_MR_CON_pub.publish(MR_Wheel_Velocity);
    }

    void twist_to_turning_radius(const geometry_msgs::Twist::ConstPtr& msg)
    {
        l = msg->linear.x / msg->angular.z;
    }

    void calculate_servo_angle(double l)
    {
        theta_front_closest = atan2(d3, abs(l) - d1);
        theta_front_farthest = atan2(d3, abs(l) + d1);

        if(l > 0)
        {
            FL_angle.data = theta_front_closest;
            FR_angle.data = theta_front_farthest;
            RL_angle.data = -theta_front_closest;
            RR_angle.data = -theta_front_farthest;
        }
        else
        {
            FL_angle.data = -theta_front_farthest;
            FR_angle.data = -theta_front_closest;
            RL_angle.data = theta_front_farthest;
            RR_angle.data = theta_front_closest;
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
            FL_Wheel_Velocity.data = float(ang_vel_corner_closest);
            RL_Wheel_Velocity.data = float(ang_vel_corner_closest);
            ML_Wheel_Velocity.data = float(ang_vel_middle_closest);
            RR_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            FR_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            MR_Wheel_Velocity.data = float(ang_vel_middle_farthest);
        }
        else        // turning right
        {
            FL_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            RL_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            ML_Wheel_Velocity.data = float(ang_vel_middle_farthest);
            RR_Wheel_Velocity.data = float(ang_vel_corner_closest);
            FR_Wheel_Velocity.data = float(ang_vel_corner_closest);
            MR_Wheel_Velocity.data = float(ang_vel_middle_closest);
        }

    }

    void go_straight(const geometry_msgs::Twist::ConstPtr& msg)
    {
        FL_Wheel_Velocity.data = float(msg->linear.x / ROVER_WHEEL_RADIUS);
        RL_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        ML_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        RR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        FR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        MR_Wheel_Velocity.data = FL_Wheel_Velocity.data;

        FL_angle.data = 0;
        FR_angle.data = 0;
        RL_angle.data = 0;
        RR_angle.data = 0;
    }

    void stop()
    {
        FL_Wheel_Velocity.data = 0;
        RL_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        ML_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        RR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        FR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        MR_Wheel_Velocity.data = FL_Wheel_Velocity.data;

        FL_angle.data = 0;
        FR_angle.data = 0;
        RL_angle.data = 0;
        RR_angle.data = 0;
    }

    void rotate_in_place(const geometry_msgs::Twist::ConstPtr& msg)
    {
        FL_Wheel_Velocity.data = -float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS);
        FR_Wheel_Velocity.data = float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS);
        ML_Wheel_Velocity.data = -float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS);
        MR_Wheel_Velocity.data = float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS);
        RL_Wheel_Velocity.data = -float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS);
        RR_Wheel_Velocity.data = float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS);
    }

    void msgCallback(const geometry_msgs::Twist::ConstPtr& msg) 
    {
        //ROS_INFO("angular : %lf , linear : %lf", msg->angular.z, msg->linear.x); 
        
        if((msg->angular.z != 0)&&(msg->linear.x == 0))  // rotate in place
        {
            FL_angle.data = -atan(d3/d1);    // FL
            FR_angle.data = atan(d3/d1);     // FR
            RL_angle.data = atan(d2/d1);     // RL
            RR_angle.data = -atan(d2/d1);    // RR

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
        else if((msg->angular.z == 0)&&(msg->linear.x != 0)) // linear velocity
        {
            go_straight(msg);
            publishAngles();
            publishVelocity();
        }
        else if((msg->angular.z == 0)&&(msg->linear.x == 0))
        {
            stop();
            publishAngles();
            publishVelocity();
        }
    }

    // imu call_callback
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // start_time = ros::Time::now().toSec();
        // time = start_time - pre_time;
        // pre_time = start_time;

        tf::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        theta = tf::getYaw(quaternion);

    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        // start_time = ros::Time::now().toSec();
        // time = start_time - pre_time;
        // pre_time = start_time;

        fl_vel = double(msg->position[2]);
        fr_vel = double(msg->position[3]);
        ml_vel = double(msg->position[4]);
        mr_vel = double(msg->position[5]);
        rl_vel = double(msg->position[8]);
        rr_vel = double(msg->position[9]);

        Odometry(theta);

    }

//odometry calculate
    void Odometry(double angle)
    {
        current_dl = (fl_vel + fr_vel + ml_vel + mr_vel + rl_vel + rr_vel) * ROVER_WHEEL_RADIUS / 6;
        dl = current_dl - pre_dl;
        
        pre_dl = current_dl;
        
        x_postion += dl*cos(angle);
        y_postion += dl*sin(angle);


        // Update Odometry message
        odom_.header.stamp = ros::Time::now();
        odom_.header.frame_id = "odom";

        odom_.pose.pose.position.x = x_postion;
        odom_.pose.pose.position.y = y_postion;

        // Assuming angle is in radians
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, angle);

        odom_.pose.pose.orientation.x = quaternion.x();
        odom_.pose.pose.orientation.y = quaternion.y();
        odom_.pose.pose.orientation.z = quaternion.z();
        odom_.pose.pose.orientation.w = quaternion.w();


        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom"; 
        transformStamped.child_frame_id = "base_footprint"; 

        transformStamped.transform.translation.x = odom_.pose.pose.position.x;
        transformStamped.transform.translation.y = odom_.pose.pose.position.y;
        transformStamped.transform.translation.z = odom_.pose.pose.position.z;
        
        transformStamped.transform.rotation = odom_.pose.pose.orientation;
        br.sendTransform(transformStamped);
        
        odom_pub.publish(odom_);

       // printf("x_pos : %f\ty_pos : %f\n", x_postion,y_postion);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");

    Controller controller;

    ros::Rate loop_rate(200); //100ms
    //ROS_INFO("HELLO");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

    }
    ros::spinOnce();

    return 0;
}
