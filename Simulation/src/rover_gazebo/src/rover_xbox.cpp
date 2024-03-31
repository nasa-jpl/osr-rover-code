#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Xbox
{
    private:
        ros::NodeHandle nh;
        
        ros::Subscriber joy_sub;
        ros::Publisher cmd_vel_pub;

        geometry_msgs::Twist cmd_vel_msg;

        #define max_linear_value 3.5
        #define max_angular_value 5.0
        #define dead_lock 0.5 
        double linear_return;
        double angular_return;

        double linear_vel_x;
        double angular_vel_z;

    public:
        Xbox()
        {
            cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            joy_sub = nh.subscribe("joy", 1, &Xbox::msgCallback, this);
        }

        void msgCallback(const sensor_msgs::Joy::ConstPtr& msg)
        {
            linear_return = msg->axes[1];
            angular_return = msg->axes[0];

            if(abs(msg->axes[1]) < dead_lock )
            {
                linear_return = 0;
            }

            else if(abs(msg->axes[0]) < dead_lock )
            {
                angular_return = 0;
            }

            linear_vel_x = max_linear_value * linear_return/2;
            angular_vel_z = max_angular_value * angular_return/2;

            cmd_vel_msg.linear.x = linear_vel_x;
            cmd_vel_msg.angular.z = angular_vel_z;
            cmd_vel_pub.publish(cmd_vel_msg);
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "XBOX");
    Xbox Xbox;

    ros::Rate loop_rate(10); // 100ms
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
