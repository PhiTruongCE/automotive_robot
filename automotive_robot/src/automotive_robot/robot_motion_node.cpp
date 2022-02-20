//#include "robot_motion_node.hpp"
#include "automotive_robot/Path.h"
#include "automotive_robot/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <cstdint>
#include <nav_msgs/Odometry.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <math.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

//#define CLOSE_MV_SPEED 0.05                              // square_unit/s
#define ALIGN_WHILE_MOVING_ANGULAR_SPEED ((double(7) / 180) * M_PI)      // radian/s

using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

ros::Publisher velocity_publisher;
ros::Publisher moveFinished_publisher;
geometry_msgs::Twist vel_msg;
automotive_robot::Point cpos;
double odomAngleXaxis = 0;

void rotate(bool rotate_right, double direction);
void moveCallback(const automotive_robot::Path::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
double z = 0;
int main(int argc, char **argv) {
    // Initiate new ROS node named "talker"
//     ros::init(argc, argv, "robot_motion");
//     ros::NodeHandle n;

//     velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
//     moveFinished_publisher = n.advertise<std_msgs::String>("update_vision", 1000);

//     ROS_INFO("\n\n\n ******** START *********\n");

//     ros::Subscriber sub = n.subscribe("cmd_moving", 1000, moveCallback);
//     ros::Subscriber Odometry_sub = n.subscribe("odom", 1000, odomCallback);
//     std_msgs::String triggered_msg;
//     triggered_msg.data = "done";

//     ROS_INFO("\n\n\n Before sleep\n");
//     sleep(20);
//     ROS_INFO("\n\n\n Now Starting!\n");
//     moveFinished_publisher.publish(triggered_msg);

//     ros::MultiThreadedSpinner Spinner(2);

//     Spinner.spin();
//     ros::waitForShutdown();

//     return 0;
        ros::init(argc, argv, "robot_motion");
        ros::NodeHandle n;

        velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        ROS_INFO("\n\n\n ******** START *********\n");

        ros::Subscriber Odometry_sub = n.subscribe("odom", 1000, odomCallback);

        ros::MultiThreadedSpinner Spinner(2);
        

        cout<<"Rotate"<<endl;
        rotate(true, (double(45) / 180) * M_PI);

        // vel_msg.linear.x = 0;
        // vel_msg.linear.y = 0;
        // vel_msg.linear.z = 0;
        // // set a random angular velocity in the y-axis
        // vel_msg.angular.x = 0;
        // vel_msg.angular.y = 0;
        // vel_msg.angular.z = 1;
        // velocity_publisher.publish(vel_msg);

        // //while(1){
        // //ros::spinOnce();
        // //if (z < 0.1) {cout << "z: " << z << endl;
        // //velocity_publisher.publish(vel_msg);}
        // //else {cout << "yasdfsa\n";break;}}
        // // ros::spinOnce();
        // // Spinner.spin();
        ros::spin();
        ros::waitForShutdown();

        return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  odomAngleXaxis = yaw;
  cpos.x = msg->pose.pose.position.x;
  cpos.y = msg->pose.pose.position.y;
  z = msg->twist.twist.angular.z;
}

// angle of p1p2 against Ox   
double Direction(const volatile automotive_robot::Point &p1, const automotive_robot::Point &p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  return atan2(dy, dx);
}

const float MAX_MV_SPEED = 0.18;      // square_unit/s
const float deceleration_time = 3000; // miliseconds


const int smooth_level = 20;
const int small_deceleration_duration = deceleration_time / smooth_level;
const float vel_change = MAX_MV_SPEED / smooth_level;
const float close_rotate_tolerance_while_moving = ((double(0.5) / 180) * M_PI);
const int small_rotate_time = (close_rotate_tolerance_while_moving / ALIGN_WHILE_MOVING_ANGULAR_SPEED) * 1000;
const float stable_moving_tolerance =(MAX_MV_SPEED * deceleration_time / 2000) * (MAX_MV_SPEED * deceleration_time / 2000);

void print_anglular_diff_periodically(int64_t &mark_last_print, double current_angle_diff, const string typeOfMoving){
    auto now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    unsigned short print_period = 200;
    if (now - mark_last_print > print_period){
        mark_last_print = now;
        cout << typeOfMoving <<": " << (current_angle_diff / M_PI * 180) << " Velocity: " << vel_msg.linear.x << endl;
    }
}

void stop_moving_stop_rotating(){
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void stop_rotating(){
    vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);
}

void adjust_moving_direction(bool &is_rotating, double current_angle_diff, int64_t &mark_start_rotate, int64_t now){
    if (abs(current_angle_diff) > close_rotate_tolerance_while_moving) {
        mark_start_rotate = now; // mark the time the robot starts rotating
        vel_msg.angular.z = (current_angle_diff < 0) ? ALIGN_WHILE_MOVING_ANGULAR_SPEED : -ALIGN_WHILE_MOVING_ANGULAR_SPEED;
        velocity_publisher.publish(vel_msg);
        is_rotating = true;
    } 
    else if (is_rotating && now - mark_start_rotate >= small_rotate_time){
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
        is_rotating = false;
    }
}

void partial_accelerate(const int64_t now, int64_t &mark_last_accelerate, int &small_acceleration_duration_modification, const int small_acceleration_duration){
    if (now - mark_last_accelerate >= small_acceleration_duration_modification) {
        mark_last_accelerate = now; // mark temp as soon as possible
        vel_msg.linear.x += vel_change;
        velocity_publisher.publish(vel_msg);
        cout << "Time: " << now - mark_last_accelerate << endl;
        small_acceleration_duration_modification = small_acceleration_duration - (now - mark_last_accelerate - small_acceleration_duration_modification);
        cout << "Modification: " << small_acceleration_duration_modification << endl << endl;
    }
}

void moving_accelerate(const automotive_robot::Point &next_point){   // chi lam dung chuc nang tang toc
    const float acceleration_time = 3000; // miliseconds
    const int small_acceleration_duration = acceleration_time / smooth_level;

    vel_msg.linear.x = vel_change;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    velocity_publisher.publish(vel_msg);

    auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto mark_last_accelerate = start;
    auto mark_start_rotate = start;
    auto mark_last_print = start;
    
    auto now = start;
    bool is_rotating = false;
    int small_acceleration_duration_modification = small_acceleration_duration;
    start -= small_acceleration_duration; // bc we move the robot immadiately
    do {
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        double direction = Direction(cpos, next_point);
        double current_angle_diff = odomAngleXaxis - direction;
        partial_accelerate(now, mark_last_accelerate, small_acceleration_duration_modification, small_acceleration_duration);
        adjust_moving_direction(is_rotating, current_angle_diff, mark_start_rotate, now);
        print_anglular_diff_periodically(mark_last_print, current_angle_diff, "Accelerate");
    } while (now - start <= acceleration_time && (cpos.x - next_point.x) * (cpos.x - next_point.x) + (cpos.y - next_point.y) * (cpos.y - next_point.y) > stable_moving_tolerance);
}

void stable_moving(const automotive_robot::Point &next_point, const double keep_moving_distance){
    vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum, bc robot speed might not be maximum
    vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);

    bool is_rotating = false;
    auto mark_last_print = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); // used for cout
    auto mark_start_rotate = mark_last_print;

    while ((cpos.x - next_point.x) * (cpos.x - next_point.x) + (cpos.y - next_point.y) * (cpos.y - next_point.y) > keep_moving_distance) {
        double direction = Direction(cpos, next_point);
        double current_angle_diff = odomAngleXaxis - direction;
        auto now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        adjust_moving_direction(is_rotating, current_angle_diff, mark_start_rotate, now);
        print_anglular_diff_periodically(mark_last_print, current_angle_diff, "Stable Moving");
    }
}

void partial_decelerate(const int64_t now, int64_t &mark_last_decelerate, int &small_deceleration_duration_modification, const int small_deceleration_duration){
    if (now - mark_last_decelerate >= small_deceleration_duration_modification) {
        mark_last_decelerate = now; // mark temp as soon as possible
        vel_msg.linear.x -= vel_change;
        velocity_publisher.publish(vel_msg);
        cout << "Time: " << now - mark_last_decelerate << endl;
        small_deceleration_duration_modification = small_deceleration_duration - (now - mark_last_decelerate - small_deceleration_duration_modification);
        cout << "Modification: " << small_deceleration_duration_modification << endl << endl;
    }
}

void moving_decelerate(const automotive_robot::Point &next_point){
    vel_msg.linear.x -= vel_change; // force velocity to decrease immediately
    vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);

    auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto now = start;
    auto mark_start_rotate = start;
    auto mark_last_print = start;
    auto mark_last_decelerate = start;
    start -= small_deceleration_duration;
    bool is_rotating = false;
    int small_deceleration_duration_modification = small_deceleration_duration;

    do {
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        double direction = Direction(cpos, next_point);
        double current_angle_diff = odomAngleXaxis - direction;
        partial_decelerate(now, mark_last_decelerate, small_deceleration_duration_modification, small_deceleration_duration);
        adjust_moving_direction(is_rotating, current_angle_diff, mark_start_rotate, now);
        print_anglular_diff_periodically(mark_last_print, current_angle_diff, "Decelerate");
    } while (now - start <= deceleration_time * 0.9);
}

void very_slow_moving(const automotive_robot::Point &next_point){
    vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
    velocity_publisher.publish(vel_msg);

    bool is_rotating = false;
    auto mark_last_print = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    int64_t mark_start_rotate;

    while ((cpos.x - next_point.x) * (cpos.x - next_point.x) + (cpos.y - next_point.y) * (cpos.y - next_point.y) > stable_moving_tolerance / 400) {
        double direction = Direction(cpos, next_point);
        double current_angle_diff = odomAngleXaxis - direction;
        if (abs(current_angle_diff) > 1.57) {
            stop_moving_stop_rotating();
            break;
        }
        auto now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        adjust_moving_direction(is_rotating, current_angle_diff, mark_start_rotate, now);
        print_anglular_diff_periodically(mark_last_print, current_angle_diff, "Very Slow");
    }
}

void move_to_next_point(const automotive_robot::Point &next_point) {
    moving_accelerate(next_point);
    stable_moving(next_point, stable_moving_tolerance);
    moving_decelerate(next_point);
    very_slow_moving(next_point);
    stop_moving_stop_rotating();
}

const double STABLE_ANGULAR_SPEED = ((double(30) / 180) * M_PI); // radian/s
const double rotate_deceleration_time = 1000; //miliseconds


void rotate_accelerate(bool rotate_right, double direction){
    const int rotate_accelerate_time = 1500;    // miliseconds
    const int rotate_smooth_level = 15;
    const int small_rotate_acceleration_duration = rotate_accelerate_time / (rotate_smooth_level - 1);
    const double rotate_change = STABLE_ANGULAR_SPEED/rotate_smooth_level*((rotate_right)?-1:1);
    const double accelerate_rotate_tolerance = ((STABLE_ANGULAR_SPEED)*(STABLE_ANGULAR_SPEED) - (0.2*STABLE_ANGULAR_SPEED)*(0.2*STABLE_ANGULAR_SPEED))
                                                /(2*0.8*STABLE_ANGULAR_SPEED/(rotate_deceleration_time/1000));

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = rotate_change;

    velocity_publisher.publish(vel_msg);

    auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto temp = start;
    auto now = start;
    int small_rotate_acceleration_duration_modification = small_rotate_acceleration_duration;
    double current;
    do {
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        current = odomAngleXaxis - direction;
        if (now - temp >= small_rotate_acceleration_duration_modification) {
            cout << "rotate_speed:" << vel_msg.angular.z << "Direction: " << direction << endl; 
            vel_msg.angular.z += rotate_change;
            velocity_publisher.publish(vel_msg);
            small_rotate_acceleration_duration_modification = small_rotate_acceleration_duration - (now - temp - small_rotate_acceleration_duration_modification);
            temp = now; // mark temp as soon as possible
        }
    } while (now - start <= rotate_accelerate_time && abs(current) > accelerate_rotate_tolerance*1.15);
}

void stable_rotate(double direction){
    const double stable_rotate_tolerance = ((STABLE_ANGULAR_SPEED)*(STABLE_ANGULAR_SPEED) - (0.2*STABLE_ANGULAR_SPEED)*(0.2*STABLE_ANGULAR_SPEED))
                                                /(2*0.8*STABLE_ANGULAR_SPEED/(rotate_deceleration_time/1000));
    while (abs(direction - odomAngleXaxis) > stable_rotate_tolerance*1.15) {
    }
}

void rotate_decelerate(bool rotate_right, double direction){
    const int rotate_smooth_level = 15;
    const int small_rotate_deceleration_duration = rotate_deceleration_time / (rotate_smooth_level - 1);
    const double rotate_change = STABLE_ANGULAR_SPEED*0.8/rotate_smooth_level*((rotate_right)?-1:1);

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z -= rotate_change; // -= not = 

    velocity_publisher.publish(vel_msg);

    auto start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto temp = start;
    auto now = start;
    int small_rotate_deceleration_duration_modification = small_rotate_deceleration_duration;
    double current;
    do {
        now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        current = odomAngleXaxis - direction;
        if (now - temp >= small_rotate_deceleration_duration_modification) {
            vel_msg.angular.z -= rotate_change;
            velocity_publisher.publish(vel_msg);
            small_rotate_deceleration_duration_modification = small_rotate_deceleration_duration - (now - temp - small_rotate_deceleration_duration);
            temp = now; // mark temp as soon as possible
        }
    } while (now - start <= rotate_deceleration_time);
}

void very_slow_rotate(double direction){
    const double close_rotate_tolerance = ((double(0.5) / 180) * M_PI);
    double before = abs(odomAngleXaxis - direction);
    while (before > close_rotate_tolerance) {
        double after = abs(direction - odomAngleXaxis);
        if (after > before) {
            vel_msg.angular.z = -vel_msg.angular.z;
            velocity_publisher.publish(vel_msg);
        }
        before = after;
    }
}

// rotate clockwise
void rotate(bool rotate_right, double direction) { 
    cout<<"Start rotate accelerate"<<endl; 
    rotate_accelerate(rotate_right, direction);
    cout<<"Start rotate stable"<<endl;
    stable_rotate(direction);
    cout<<"Start rotate decelerate"<<endl;
    rotate_decelerate(rotate_right, direction);
    cout<<"Start rotate slow"<<endl;
    very_slow_rotate(direction);

    stop_rotating();
    usleep(500000);
}

bool determine_rotate_direction(const automotive_robot::Point &next_point, double &direction){ //return true if rotate right, else return false if rotate left
    direction = Direction(cpos, next_point);
    double angleDiff = direction - odomAngleXaxis;
    if (odomAngleXaxis > 0 && M_PI < (odomAngleXaxis - direction)) angleDiff = 2 * M_PI - odomAngleXaxis + direction;
    else if (odomAngleXaxis < 0 && M_PI < (-odomAngleXaxis + direction)) angleDiff = -2 * M_PI + direction + odomAngleXaxis;
    return ((angleDiff < 0) ? true : false); 
}

float distance(const automotive_robot::Point &pointA, const automotive_robot::Point &pointB){
    return sqrt((pointA.x - pointB.x)*(pointA.x - pointB.x) + (pointA.y - pointB.y)*(pointA.y - pointB.y));
}

void translation_of_point(const float *translationary_vector, const automotive_robot::Point &toBe_translated_point, automotive_robot::Point &translated_point){
    translated_point.x = toBe_translated_point.x + translationary_vector[0];
    translated_point.y = toBe_translated_point.y + translationary_vector[1];
}

void calculate_start_bending_vector_and_stop_bending_vector(const automotive_robot::Path::ConstPtr &msg, 
    float *vector_nextPoint_to_start_bending_point, float *vector_nextPoint_to_stop_bending_point, const float start_bending_distance,int i){
    vector_nextPoint_to_start_bending_point[0] = msg->points[i - 1].x - msg->points[i].x; 
    vector_nextPoint_to_start_bending_point[1] = msg->points[i - 1].y - msg->points[i].y;
    vector_nextPoint_to_stop_bending_point[0] = msg->points[i + 1].x - msg->points[i].x; 
    vector_nextPoint_to_stop_bending_point[1] = msg->points[i + 1].y - msg->points[i].y;
    
    float distance_from_nextPoint_to_start_point = sqrt(vector_nextPoint_to_start_bending_point[0]*vector_nextPoint_to_start_bending_point[0] 
        + vector_nextPoint_to_start_bending_point[1]*vector_nextPoint_to_start_bending_point[1]);
    float distance_from_nextPoint_to_end_point = sqrt(vector_nextPoint_to_stop_bending_point[0]*vector_nextPoint_to_stop_bending_point[0] 
        + vector_nextPoint_to_stop_bending_point[1]*vector_nextPoint_to_stop_bending_point[1]);
    
    vector_nextPoint_to_start_bending_point[0] = vector_nextPoint_to_start_bending_point[0]/distance_from_nextPoint_to_start_point*start_bending_distance;
    vector_nextPoint_to_start_bending_point[1] = vector_nextPoint_to_start_bending_point[1]/distance_from_nextPoint_to_start_point*start_bending_distance;

    vector_nextPoint_to_stop_bending_point[0] = vector_nextPoint_to_stop_bending_point[0]/distance_from_nextPoint_to_end_point*start_bending_distance;
    vector_nextPoint_to_stop_bending_point[1] = vector_nextPoint_to_stop_bending_point[1]/distance_from_nextPoint_to_end_point*start_bending_distance;
}

void moveCallback(const automotive_robot::Path::ConstPtr &msg) {
    unsigned loops = msg->points.size();
    ROS_INFO("I heard: [%f,%f]", msg->points[loops - 1].x, msg->points[loops - 1].y);

    double direction;
    bool rotate_right =  determine_rotate_direction(msg->points[1], direction);
    rotate(rotate_right, direction);

    if(loops == 2){
        automotive_robot::Point nextPt = msg->points[1];
        ROS_INFO("CPOS: [%f : %f]", cpos.x, cpos.y);
        ROS_INFO("Next Point: [%f : %f]", nextPt.x, nextPt.y);
        move_to_next_point(nextPt);
    }
    
    else {
        cout<<"Co nhieu duong gap khuc"<<endl;
        
        moving_accelerate(msg->points[1]);

        vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
                                        // might not be maximum
        vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
        velocity_publisher.publish(vel_msg);

        // stable moving 
        loops--;    // pay attention please
        for(unsigned i = 1 ; i < loops; i++){
            automotive_robot::Point next_point = msg->points[i];
            float start_bending_distance = 0.15*distance(msg->points[i - 1], next_point); 
            automotive_robot::Point start_bending_point;
            automotive_robot::Point stop_bending_point;
            float vector_nextPoint_to_start_bending_point[2];
            float vector_nextPoint_to_stop_bending_point[2];

            calculate_start_bending_vector_and_stop_bending_vector(msg, vector_nextPoint_to_start_bending_point, vector_nextPoint_to_stop_bending_point, start_bending_distance,i);
            translation_of_point(vector_nextPoint_to_start_bending_point, msg->points[i], start_bending_point);
            translation_of_point(vector_nextPoint_to_stop_bending_point, msg->points[i], stop_bending_point);

            vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
            // might not be maximum
            stop_rotating();
            
            stable_moving(next_point, start_bending_distance);

            stop_rotating();
            // be huong voi ban kinh cong R
            float alpha = acos((vector_nextPoint_to_start_bending_point[0]*vector_nextPoint_to_stop_bending_point[0] 
            + vector_nextPoint_to_start_bending_point[1]*vector_nextPoint_to_stop_bending_point[1]) / (start_bending_distance*start_bending_distance));

            float R = tan(alpha/2)*start_bending_distance;

            automotive_robot::Point direction_forcing_point = msg->points[i+1];
            double direction;
            bool rotate_right = determine_rotate_direction(direction_forcing_point, direction);

            vel_msg.angular.z = ((rotate_right) ? -1 : 1)*MAX_MV_SPEED/R;     // v = R*W;
            velocity_publisher.publish(vel_msg);    // start bending
            direction = Direction(next_point, direction_forcing_point);
            while(abs(odomAngleXaxis - direction) > close_rotate_tolerance_while_moving){}

            stop_rotating();    // stop bending
        }
        
        automotive_robot::Point last_point = msg->points[loops];
        stable_moving(last_point, stable_moving_tolerance);
        stop_rotating();

        moving_decelerate(last_point);
        stop_rotating();
        
        very_slow_moving(last_point);

        stop_moving_stop_rotating();
        ROS_INFO("Moving done!");
    }
    std_msgs::String done_msg;
    done_msg.data = "done";
    moveFinished_publisher.publish(done_msg);
}