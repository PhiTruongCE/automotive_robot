//#include "robot_motion_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "automotive_robot/Point.h"
#include "automotive_robot/Path.h"

#include <math.h>
#include <vector>
#include <string>
#include <string.h>
#include <unistd.h>

#define MV_SPEED 1                          // m/s
#define ANGULAR_SPEED ((20/180)*M_PI)       // radian/s

using namespace std;

ros::Publisher velocity_publisher;
ros::Publisher moveFinished_publisher;
geometry_msgs::Twist vel_msg;
automotive_robot::Point cpos;
double angleXaxis;

void move(double distance);
void rotate(double angular_speed, double angle, bool cloclwise);
void moveCallback(const automotive_robot::Path::ConstPtr &msg);

int main(int argc, char **argv)
{
    // Initiate new ROS node named "talker"
    ros::init(argc, argv, "robot_motion");
    ros::NodeHandle n;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    moveFinished_publisher = n.advertise<std_msgs::String>("update_vision", 1000);

    //	/turtle1/cmd_vel is the Topic name
    //	/geometry_msgs::Twist is the msg type
    ROS_INFO("\n\n\n ******** START *********\n");

    ros::Subscriber sub = n.subscribe("cmd_moving", 1000, moveCallback);
    ros::spin();

    return 0;
}

// direction from p1 to p2
double Direction(const automotive_robot::Point& p1, const automotive_robot::Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    return  atan2(dy, dx);
}

void move(double distance){

    //set a random linear velocity in the x-axis
    vel_msg.linear.x = MV_SPEED;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    velocity_publisher.publish(vel_msg);
    usleep(distance / MV_SPEED * 1000000);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

// rotate clockwise 
void rotate(double relative_angle){
    //set a random linear velocity in the x-axis
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = ANGULAR_SPEED;

    velocity_publisher.publish(vel_msg);
    usleep(relative_angle / ANGULAR_SPEED * 1000000);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void moveCallback(const automotive_robot::Path::ConstPtr& msg)  //robot_motion::Path::ConstPtr& msg
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    unsigned loops = msg->points.size();

    for (unsigned i = 1; i < loops; i++) {
        // 
        cpos = msg->points[i - 1];
        automotive_robot::Point nextPt = msg->points[i];

        //
        double direction = Direction(cpos, nextPt);
        double angleDiff = angleXaxis - direction;
        if (angleXaxis > 0 && direction < (-M_PI + angleXaxis)) angleDiff -= 2 * M_PI;
        else if (angleXaxis < 0 && direction > (M_PI + angleXaxis)) angleDiff += 2 * M_PI;

        //
        double distance = sqrt(pow(cpos.x - nextPt.x, 2) + pow(cpos.y - nextPt.y, 2) * 1.0);

        //
        rotate(angleDiff);
        move(distance);
    }
    cpos = msg->points[loops - 1];

    std_msgs::String done_msg;
    done_msg.data = "done";
    moveFinished_publisher.publish(done_msg);
}
