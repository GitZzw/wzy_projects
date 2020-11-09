//
// Created by up on 2020/9/22.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include<cv_bridge/cv_bridge.h>

#include <cstdlib>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <chrono>
#include<sstream>

#include<librealsense2/rs.hpp>

using namespace std;

/*
 * global variable
 */
geometry_msgs::PoseStamped plane_atti_msg;
cv::Point2i target_left_up;
cv::Point2i target_right_down;
cv::Point2i pickup_left_up;
cv::Point2i pickup_right_down;
int target_width;
int target_height;
bool targetDetectFlag = false;

Eigen::Vector4d target_position_of_img;
Eigen::Vector4d target_position_of_drone;
Eigen::Vector4d target_position_of_world;
geometry_msgs::Vector3 drone_euler;
geometry_msgs::Vector3 drone_euler_init;
Eigen::Quaterniond drone_quaternion;
Eigen::Vector3d drone_pos_vision;
geometry_msgs::PoseStamped msg_drone_pos_vision;
geometry_msgs::PoseStamped msg_target_pose_from_img;
bool got_attitude_init = false;


//param
double cx_color = 323.89544677734375;
double cy_color = 246.41885375976562;
double fx_color = 606.1327514648438;
double fy_color = 605.5372924804688;

Eigen::Isometry3d tf_image_to_enu;
Eigen::Isometry3d tf_camera_to_drone;
Eigen::Isometry3d tf_drone_to_world;
Eigen::Vector3d tf_camera_drone;

//yolo
float target_width_world = 0.45;
float scale_size = 2;
cv::Point2f yolo_center;

std::queue<geometry_msgs::PoseStamped> pose_queue; //飞机姿态的quene
double image_ros_time = 0.0;


// function declarations
void plane_attitude_sub(const geometry_msgs::PoseStamped::ConstPtr& msg);
void target_corner_sub(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void pickup_corner_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void tf_param_set();
void get_init_yaw();
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);


//main function
int main(int argc, char **argv) {

    ros::init(argc, argv, "pnp_target_node");
    ros::NodeHandle nh;
    ros::Rate rate(50);

    // 订阅飞机姿态,并存放到quene里
    ros::Subscriber plane_attitude_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",1,plane_attitude_sub);
    // 订阅识别目标,如果有将targetDetectFlag置为1，并且根据内参换算出物体相对相机的实际位置
    ros::Subscriber target_corner_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("yolo_target_corner",1,target_corner_sub);
    // 物体相对相机的位置
    ros::Publisher msg_target_pose_from_img = nh.advertise<geometry_msgs::PoseStamped>("topic_target_pose_from_img",1);
    // 物体相对飞机的最终位置
    ros::Publisher drone_pos_vision_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_pos_vision",1);

    //tf 参数设置
    tf_param_set();

    //获取初始飞机姿态
    while (! got_attitude_init){
        ros::spinOnce();
        ROS_INFO("getting yaw init ... ");
        get_init_yaw(); //git initial yaw
        rate.sleep();
    }


    while(ros::ok()){
         ros::spinOnce();//执行一次回调
         if(!targetDetectFlag){
             ROS_ERROR("Vision failed , local_position is needed !!!");
             //修改标志位
             msg_target_pose_from_img.header.stamp = ros::Time::now();
             msg_target_pose_from_img.pose.position.z = 0;
             msg_target_pose_from_img.pose.position.x = 0;
             msg_target_pose_from_img.pose.position.y = 0;
             msg_target_pose_from_img.pose.orientation.x = -1;
             //修改标志位
             msg_drone_pos_vision.header.stamp = ros::Time::now();
             msg_drone_pos_vision.pose.position.z = 0;
             msg_drone_pos_vision.pose.position.x = 0;
             msg_drone_pos_vision.pose.position.y = 0;
             msg_drone_pos_vision.pose.orientation.x = -1;
         }
         else{
               //发布相对相机的物体实际位置
               target_position_of_img.x() = msg_target_pose_from_img.pose.position.x;
               target_position_of_img.y() = msg_target_pose_from_img.pose.position.y;
               target_position_of_img.z() = msg_target_pose_from_img.pose.position.z;
               //发布相机的物体实际位置^^^

               //坐标转换，乘TF矩阵得到物体相对飞机的实际坐标点target_position_of_drone
               target_position_of_drone = tf_camera_to_drone * (tf_image_to_enu * target_position_of_img);
               //坐标转换，乘TF矩阵得到物体相对飞机的实际坐标点target_position_of_drone ^^^

               //计算该物体该时刻的飞机姿态
               geometry_msgs::PoseStamped synchronized_att;
               while(1){
                   if(pose_queue.empty()){ //如果为空
                       cout<<"pose_quene is empty"<<endl;
                       break;
                   }
                   else{ //不为空
                       synchronized_att = pose_queue.front();
                       if(image_ros_time >= synchronized_att.header.stamp.toSec()){
                           break;
                       }else{
                           pose_queue.pop();
                       }
                   }
               }
               //计算该物体该时刻的飞机姿态^^^

               //飞机姿态转换为euler角
               drone_euler = quaternion2euler(synchronized_att.pose.orientation.x,synchronized_att.pose.orientation.y,synchronized_att.pose.orientation.z,synchronized_att.pose.orientation.w);
               //飞机姿态转换为euler角^^^

               //减去初始euler角
               drone_euler.z  = drone_euler.z - drone_euler_init.z;
               //减去初始euler角^^^

               //减去初始后转换为四元数
               drone_quaternion = euler2quaternion_eigen(drone_euler.x,drone_euler.y,drone_euler.z);
               //减去初始后转换为四元数^^^

               //去除飞机姿态对距离预测影响
               tf_drone_to_world = Eigen::Isometry3d::Identity();
               tf_drone_to_world.prerotate(drone_quaternion.toRotationMatrix());
               tf_drone_to_world.pretranslate(Eigen::Vector3d(0,0,0));
               //去除飞机姿态对距离预测影响^^^

               //计算不受影响的距离
               target_position_of_world = tf_drone_to_world * target_position_of_drone;
               drone_pos_vision.x() = target_position_of_world.x();
               drone_pos_vision.y() = target_position_of_world.y();
               drone_pos_vision.z() = target_position_of_world.z();
               //计算不受影响的距离^^^


               msg_drone_pos_vision.header.stamp = ros::Time::now();
               msg_drone_pos_vision.pose.position.x = drone_pos_vision.x();
               msg_drone_pos_vision.pose.position.y = drone_pos_vision.y();
               msg_drone_pos_vision.pose.position.z = drone_pos_vision.z();
               msg_drone_pos_vision.pose.orientation.x = 1;//标志位
         }

        msg_target_pose_from_img.publish(msg_target_pose_from_img);
        drone_pos_vision_pub.publish(msg_drone_pos_vision);
        rate.sleep();
    }
    return 0;
}

void plane_attitude_sub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(! *msg){
        ROS_ERROR("NO MAVROS LOCAL POSITION DATA")
    }
    else{
        plane_atti_msg = *msg;
        //cout << "msg->pose.orientation.z: " << msg->pose.orientation.z << endl;
        pose_queue.push(*msg);
    }
}

void target_corner_sub(const geometry_msgs::QuaternionStamped::ConstPtr& msg){

    targetDetectFlag = false;
    if(msg->quaternion.x < 0) //没有识别到目标
    {
         cout<<'没有目标'<<endl;

         targetDetectFlag = false;
    }
    else{
        //记录图像获取时间
        cout<<'目标出现'<<endl;
        image_ros_time = msg->header.stamp.toSec();
        target_width = int(msg->quaternion.z - msg->quaternion.x);
        target_height = int(msg->quaternion.w - msg->quaternion.y);
        target_left_up.x = int(max(0,int(msg->quaternion.x)));
        target_left_up.y = int(max(0,int(msg->quaternion.y)));
        target_right_down.x = int(min(640,int(msg->quaternion.z)));
        target_right_down.y = int(min(640,int(msg->quaternion.w)));


        yolo_center.x = (target_left_up.x + target_right_down.x)/2.0;
        yolo_center.y = (target_left_up.y + target_right_down.y)/2.0;
        //根据小孔成像原理估计深度
        msg_target_pose_from_img.header.stamp = ros::Time::now();
        msg_target_pose_from_img.pose.orientation.x = 1;
        msg_target_pose_from_img.pose.position.z = scale_size * target_width_world * fx_color / sqrt(target_width*target_height) ;
        msg_target_pose_from_img.pose.position.x = msg_target_pose_from_img.pose.position.z * (yolo_center.x-cx) / fx_color;
        msg_target_pose_from_img.pose.position.y = msg_target_pose_from_img.pose.position.z * (yolo_center.y-cy) / fy_color;

        targetDetectFlag = true;
    }


}

void tf_param_set() {
    //image coordinate to ENU coordinate

    tf_image_to_enu = Eigen::Isometry3d::Identity();
    tf_image_to_enu.matrix() << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;


    //the camera position of drone, now only translate , not rotation yet
    // tf is based on ENU axis
    tf_camera_drone[0] = 0.06;
    tf_camera_drone[1] = -0.05;
    tf_camera_drone[2] = 0.1;
    Eigen::Vector3d pose_camera_of_drone;
    pose_camera_of_drone.x() = tf_camera_drone[0];
    pose_camera_of_drone.y() = tf_camera_drone[1];
    pose_camera_of_drone.z() = tf_camera_drone[2];


    tf_camera_to_drone = Eigen::Isometry3d::Identity();
    tf_camera_to_drone.matrix() << 1, 0, 0, pose_camera_of_drone.x(),
            0, 1, 0, pose_camera_of_drone.y(),
            0, 0, 1, pose_camera_of_drone.z(),
            0 ,0, 0, 1;


}

void get_init_yaw() {
    //获取初始飞机姿态
    drone_euler_init = quaternion2euler(plane_atti_msg.pose.orientation.x,plane_atti_msg.pose.orientation.y,plane_atti_msg.pose.orientation.z,plane_atti_msg.pose.orientation.w);
    if(abs(drone_euler_init.z) > 0.000001)
        got_attitude_init = true;
}


/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw){
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw){
    Eigen::Quaterniond temp;
    temp.w() = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x() = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y() = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z() = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w){
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y = - asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}
