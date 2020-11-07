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
cv::Point2i target_left_up_origin;
cv::Point2i target_right_down;
cv::Point2i target_right_down_origin;
cv::Point2i pickup_left_up;
cv::Point2i pickup_right_down;
cv::Point2i roi_target_left_up;
cv::Point2i roi_target_right_down;
int target_width;
int target_height;
bool targetDetectFlag = false;
float roi_enlarge_ratio = 0.3;


//img roi pnp
cv::Mat ir_img;
cv::Mat ir_img_crop;
cv::Mat ir_img_color_show;
cv::Rect roi;
int ir_width;
int ir_height;
double area;
cv::Rect rect;
cv::Mat ir_binary;
//!dilate
int structElementSize = 1;
cv::Mat dilatestructElement = getStructuringElement(cv::MORPH_RECT,cv::Size(3 * structElementSize + 1, 3 * structElementSize + 1));
cv::Mat ir_erode;
cv::Mat ir_dilate;
vector<vector<cv::Point>> ir_contours;
vector<vector<cv::Point>> ir_contours_final;
vector<cv::Point2f> marker_pixels;
cv::Point2d marker_sum;
cv::Point2d marker_center;
cv::Point2f left_up,left_down,right_up,right_down;
vector<cv::Point2f> marker_sequence_pixel;
cv::Vec3d outputRvecRaw,outputTvecRaw;
Eigen::Vector4d target_position_of_img;
Eigen::Vector4d target_position_of_drone;
Eigen::Vector4d target_position_of_world;
geometry_msgs::Vector3 drone_euler;
geometry_msgs::Vector3 drone_euler_init;
geometry_msgs::PoseStamped drone_euler_msg;
Eigen::Quaterniond drone_quaternion;
Eigen::Vector3d drone_pos_vision;
Eigen::Vector3d drone_pos_vision_prev;
Eigen::Vector3d drone_pos_vision_in_enu;
geometry_msgs::PoseStamped msg_drone_pos_vision;
geometry_msgs::PoseStamped msg_target_pose_from_img;
float smooth_threshold = 2;
bool got_attitude_init = false;
bool vision_prepare_ok = false;
bool opticalGoodFlag = false;
bool roiGoodFlag = false;
bool pnpGoodFlag = false;
bool yoloGoodFlag = false;

//optical follow
cv::Mat prevImg;
cv::Mat nextImg;
vector<cv::Point2f> prevImgPts;
vector<cv::Point2f> nextImgPts;
vector<uchar> status;
vector<float> err;
//cv::TermCriteria termcrit(cv::TermCriteria::EPS, 50, 0.001);
cv::TermCriteria termcrit(cv::TermCriteria::EPS, 100, 0.001);
cv::Size winSize(25, 25);

//param
double fx = 389.0441589355469;
double fy = 389.0441589355469;
double cx = 315.0856018066406;
double cy = 242.38375854492188;

double fx_color = 323.89544677734375;
double fy_color = 246.41885375976562;
double cx_color = 606.1327514648438;
double cy_color = 605.5372924804688;

cv::Mat cameraMatrix;
vector<double> distCoeffs;
vector<cv::Point3f> marker_struct;
Eigen::Isometry3d tf_image_to_enu;
Eigen::Isometry3d tf_camera_to_drone;
Eigen::Isometry3d tf_drone_to_world;
Eigen::Vector3d tf_camera_drone;
float lowPassParam = 0.1;

//yolo
float target_width_world = 0.45;
float scale_size = 2;
cv::Point2f yolo_center;
float yolo_area;
cv::Point3f target_yolo_distance;
cv::Point3f yolo_ratio(1.0,1.0,1.0);

std::queue<geometry_msgs::PoseStamped> pose_queue;
double image_ros_time = 0.0;

// function declarations
void plane_attitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void target_corner_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void pickup_corner_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void camera_param_set();
void tf_param_set();
void get_init_yaw();
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);
bool optical_follow(cv::Mat &nextImg, vector<cv::Point2f> &outputPointsVector);
bool roi_process(cv::Mat &_ir_img, vector<cv::Point2f> &outputPointsVector);
bool pnp_process(vector<cv::Point2f> &imgPoints);
bool yolo_process(cv::Mat &frame);
string Convert(float Num);

int main(int argc, char **argv) {

    ros::init(argc, argv, "pnp_target_node");
    ros::NodeHandle nh;
    ros::Rate rate(50);


    // 订阅 飞机姿态
    ros::Subscriber plane_attitude = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",1,plane_attitude_cb);
    ros::Subscriber target_corner_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("yolo_target_corner",1,target_corner_cb);
    ros::Subscriber pickup_corner_sub = nh.subscribe<geometry_msgs::QuaternionStamped>("yolo_pickup_corner",1,pickup_corner_cb);

    ros::Publisher plane_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_plane_pnp_pose_from_car",1);
    ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_target_pose_from_img",1);
    ros::Publisher drone_euler_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_plane_euler",1);
    ros::Publisher drone_pos_vision_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_pos_vision",1);

    //! realsense init
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_INFRARED,640,480,RS2_FORMAT_Y8,60);
    rs2::pipeline_profile profile=pipe.start(pipe_config);
    rs2::frameset data;
    rs2::frame ir;

    //! param init
    camera_param_set(); //for pnp--zzw comment
    tf_param_set();//coordinate transform--zzw comment
    while (! got_attitude_init){
        ros::spinOnce();
        ROS_INFO("getting yaw init ... ");
        get_init_yaw(); //git initial yaw
        rate.sleep();
    }


    bool get_ir_img = false;
    int count = 0;
    while (!get_ir_img ){
        ROS_WARN("wait for ir image ... %d %%",count*10);
        data = pipe.wait_for_frames();
        ir = data.get_infrared_frame();
        ROS_WARN("get ir data");
        //获取宽高
        ir_width = ir.as<rs2::video_frame>().get_width();
        ir_height = ir.as<rs2::video_frame>().get_height();

        if(ir_width == 640){
            get_ir_img = true;
            count++;
        }
    }

    // create window
//    cv::namedWindow("ir_img_color_show");

    while(ros::ok()){
        //ROS_INFO_THROTTLE(1,"in main loop");
        ros::spinOnce();
        data = pipe.wait_for_frames();
        ir = data.get_infrared_frame();
        //图片获取异常处理
        if(ir.as<rs2::video_frame>().get_width() != 640){
            ROS_ERROR("IR img can not be acquired ! continue ...");
            continue;
        }
        ir_img = cv::Mat(cv::Size(ir_width,ir_height),CV_8UC1,(void*)ir.get_data(),cv::Mat::AUTO_STEP);
        if(ir_img.empty()){
            ROS_ERROR("main loop, NO ir_img !!!");
            continue;
        }

        ///chg
        image_ros_time = ros::Time::now().toSec() - 0.06;

        cv::cvtColor(ir_img,ir_img_color_show,cv::COLOR_GRAY2BGR);
//        ROS_INFO_THROTTLE(1,"left_up_x = %d, left_up_y = %d, right_down_x = %d, right_down_y = %d",target_left_up.x, target_left_up.y, target_right_down.x, target_right_down.y);
        cv::rectangle(ir_img_color_show,cv::Point(target_left_up.x,target_left_up.y),cv::Point(target_right_down.x,target_right_down.y),(50,200,200), 3);
//        cv::rectangle(ir_img_color_show,cv::Point(100,100),cv::Point(300,300),(50,200,200), 3);

//        cv::waitKey(1);

        /**
         * if (op_flag):
         *      if (op_follow): op_flag = true
         *      else: op_flag = false
         * else: roi_process
         *
         * if (pnp_process)
         * elif (yolo_process)
         * else: failed!, gps!
         */
        lowPassParam = 0.1;

        //change_zzw -- test
        opticalGoodFlag = false;
         if(opticalGoodFlag)
         {
             if(optical_follow(ir_img,marker_sequence_pixel)){
                 opticalGoodFlag = true;
             }
             else{
                 opticalGoodFlag = false;
             }
         }
         //change_zzw -- test
         opticalGoodFlag = true;
         if(!opticalGoodFlag){
             roiGoodFlag = roi_process(ir_img,marker_sequence_pixel);
         }

//        /*
//        * test
//        */
//        opticalGoodFlag = false;
//        roiGoodFlag = false;
//        pnpGoodFlag = false;
//        /*
//         * test
//         */

        //change_zzw -- test
        opticalGoodFlag = false;
        roiGoodFlag = false;
        if(opticalGoodFlag || roiGoodFlag){
             pnpGoodFlag = pnp_process(marker_sequence_pixel);
             if(pnpGoodFlag){
                 opticalGoodFlag = true;
             }
             else{
                 opticalGoodFlag = false;
                 marker_sequence_pixel.clear();
             }
         }


         //change_zzw -- test
         pnpGoodFlag = false;
         if(!pnpGoodFlag){
             lowPassParam = 0.1;
             yoloGoodFlag = yolo_process(ir_img);
         }

          //change_zzw -- test
         yoloGoodFlag = true;
         if(!yoloGoodFlag){
             ROS_ERROR("Vision failed , local_position is needed !!!");
             if(pnpGoodFlag)
                 yoloGoodFlag = true;
         }

        //publish
        target_pose_pub.publish(msg_target_pose_from_img);



        target_position_of_img.x() = msg_target_pose_from_img.pose.position.x;
        target_position_of_img.y() = msg_target_pose_from_img.pose.position.y;
        target_position_of_img.z() = msg_target_pose_from_img.pose.position.z;
        //target_position_of_drone 相对飞机的实际坐标点 --zzw commit
        target_position_of_drone = tf_camera_to_drone * (tf_image_to_enu * target_position_of_img);
        //ROS_INFO("target_x_of drone: %f, target_y_of drone: %f, target_z_of drone: %f", target_position_of_drone[0],target_position_of_drone[1], target_position_of_drone[2]);

        geometry_msgs::PoseStamped synchronized_att;
        while(1){
            if(!pose_queue.empty()){
                synchronized_att = plane_atti_msg;
                //ROS_ERROR("synchronized_att Errror!!!");
                break;
            }
            synchronized_att = pose_queue.front();
            if(image_ros_time >= synchronized_att.header.stamp.toSec()){
                break;
            }else{
                pose_queue.pop();
            }
        }

        drone_euler = quaternion2euler(synchronized_att.pose.orientation.x,synchronized_att.pose.orientation.y,synchronized_att.pose.orientation.z,synchronized_att.pose.orientation.w);

        drone_euler.z  = drone_euler.z - drone_euler_init.z;
        //ROS_INFO("drone_euler.z: %f, drone_euler_init.z: %f",drone_euler.z,drone_euler_init.z);
        drone_quaternion = euler2quaternion_eigen(drone_euler.x,drone_euler.y,drone_euler.z);
        //ROS_INFO("drone_roll: %f, drone_pitch: %f ,drone_yaw: %f", drone_euler.x,drone_euler.y,drone_euler.z);
        drone_euler_msg.header.stamp = ros::Time::now();
        drone_euler_msg.pose.position.x = drone_euler.x;
        drone_euler_msg.pose.position.y = drone_euler.y;
        drone_euler_msg.pose.position.z = drone_euler.z;
        drone_euler_pub.publish(drone_euler_msg);

        tf_drone_to_world = Eigen::Isometry3d::Identity();
        //! the inverse of rotationmatrix == the transpose of rotationmatrix
        tf_drone_to_world.prerotate(drone_quaternion.toRotationMatrix());
        tf_drone_to_world.pretranslate(Eigen::Vector3d(0,0,0));
        target_position_of_world = tf_drone_to_world * target_position_of_drone;
        drone_pos_vision.x() = target_position_of_world.x();
        drone_pos_vision.y() = target_position_of_world.y();
        drone_pos_vision.z() = target_position_of_world.z();


        //low pass filter
        if(!targetDetectFlag && !pnpGoodFlag) //no yolo and no pnp, keep current pose
        {
            cv::putText(ir_img_color_show,
                        "  keep current pose ",
                        cv::Point(200, 460), cv::FONT_HERSHEY_SIMPLEX ,0.8,cv::Scalar(0,100,250),2,8,false);

            drone_pos_vision.x() = drone_pos_vision_prev.x();
            drone_pos_vision.y() = drone_pos_vision_prev.y();
            drone_pos_vision.z() = drone_pos_vision_prev.z();
        }
        else{
            if (abs(drone_pos_vision_prev.x()) > 0.1){
                drone_pos_vision.x() = drone_pos_vision_prev.x() + lowPassParam*(drone_pos_vision.x() - drone_pos_vision_prev.x());
                drone_pos_vision.y() = drone_pos_vision_prev.y() + lowPassParam*(drone_pos_vision.y() - drone_pos_vision_prev.y());
                drone_pos_vision.z() = drone_pos_vision_prev.z() + lowPassParam*(drone_pos_vision.z() - drone_pos_vision_prev.z());
                drone_pos_vision_prev.x() = drone_pos_vision.x();
                drone_pos_vision_prev.y() = drone_pos_vision.y();
                drone_pos_vision_prev.z() = drone_pos_vision.z();
            }
            else{
                drone_pos_vision_prev.x() = drone_pos_vision.x();
                drone_pos_vision_prev.y() = drone_pos_vision.y();
                drone_pos_vision_prev.z() = drone_pos_vision.z();
            }

        }



       // cout << "drone_pos_vision_prev.x()" << drone_pos_vision_prev.x() << endl;
       // cout << "drone_pos_vision x y z: " << drone_pos_vision.x() << " " << drone_pos_vision.y() << " " << drone_pos_vision.z() << endl;
        cv::putText(ir_img_color_show,
                    "drone_pos_vision x : " + Convert(drone_pos_vision.x()) + "    y: " + Convert(drone_pos_vision.y())  + "    z:" + Convert(drone_pos_vision.z()),
                    cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX ,0.6,cv::Scalar(0,0,255),1,8,false);

        if(opticalGoodFlag){
            cv::putText(ir_img_color_show,
                        " opticalGoodFlag ",
                        cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX ,0.8,cv::Scalar(0,255,0),2,8,false);
        }
        else{
            if(roiGoodFlag) {
                cv::putText(ir_img_color_show,
                            " roiGoodFlag ",
                            cv::Point(0, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2, 8, false);
            }
            }

        if(pnpGoodFlag){
            cv::putText(ir_img_color_show,
                        " pnpGoodFlag ",
                        cv::Point(0, 140), cv::FONT_HERSHEY_SIMPLEX ,0.8,cv::Scalar(255,255,0),2,8,false);
        }

        if(yoloGoodFlag){
            cv::putText(ir_img_color_show,
                        " yoloGoodFlag ",
                        cv::Point(0, 180), cv::FONT_HERSHEY_SIMPLEX ,0.8,cv::Scalar(0,255,255),2,8,false);
        }
        else{
            cv::putText(ir_img_color_show,
                        " vision failed ",
                        cv::Point(0, 460), cv::FONT_HERSHEY_SIMPLEX ,0.8,cv::Scalar(0,100,250),2,8,false);
        }

        msg_drone_pos_vision.header.stamp = ros::Time::now();
        msg_drone_pos_vision.pose.position.x = drone_pos_vision.x();
        msg_drone_pos_vision.pose.position.y = drone_pos_vision.y();
        msg_drone_pos_vision.pose.position.z = drone_pos_vision.z();
        if (pnpGoodFlag || yoloGoodFlag){
            msg_drone_pos_vision.pose.orientation.x = 1;
        }
        else{
            msg_drone_pos_vision.pose.orientation.x = -1;
        }



//        //valid if the value is right
//        if (fabs(drone_pos_vision.x()+3) < 2 && fabs(drone_pos_vision.y()) < 2)
//        {
//            cout << "the x y value is within 2m and 2m" << endl;
//            pnp_pose_is_good = true;
//            cs_road_to_enu(drone_pos_vision.x(), drone_pos_vision.y(), drone_pos_vision_in_enu.x(),drone_pos_vision_in_enu.y());
//            drone_pos_vision_msg.pose.position.x = drone_pos_vision_in_enu.x();
//            drone_pos_vision_msg.pose.position.y = drone_pos_vision_in_enu.y();
//        }
//        else{
//            cout << "the x error is : " << drone_pos_vision.x() << "the y error is : " << drone_pos_vision.y() << endl;
//        }
        drone_pos_vision_pub.publish(msg_drone_pos_vision);

        cv::imshow("ir_img_color_show", ir_img_color_show);
        cv::waitKey(1);
        rate.sleep();

    }

    return 0;
}

void plane_attitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    plane_atti_msg = *msg;
    //cout << "msg->pose.orientation.z: " << msg->pose.orientation.z << endl;
    pose_queue.push(*msg);
}

void target_corner_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    /*
     *  configure img corner for crop it
     */
    targetDetectFlag = false;
    int wait_count = 10;
    if(msg->quaternion.x < 0)
    {   wait_count --;
        if(wait_count > 0){
            target_left_up.x = roi_target_left_up.x;
            target_left_up.y = roi_target_left_up.y;
            target_right_down.x = roi_target_right_down.x;
            target_right_down.y = roi_target_right_down.y;
            ROS_WARN("target corner detect failed ! use last ROI");
        }
        else{
            ROS_ERROR("target corner detect failed ! over 10 times");
            target_left_up.x = 0;
            target_left_up.y = 0;
            target_right_down.x = 2;
            target_right_down.y = 2;
        }
    }
    else{
        target_width = int(msg->quaternion.z - msg->quaternion.x);
        target_height = int(msg->quaternion.w - msg->quaternion.y);
        target_left_up_origin.x = int(max(0,int(msg->quaternion.x)));
        target_left_up_origin.y = int(max(0,int(msg->quaternion.y)));
        target_right_down_origin.x = int(min(640,int(msg->quaternion.z)));
        target_right_down_origin.y = int(min(640,int(msg->quaternion.w)));
        //remove roi_enlarge_ratio     --zzw commit
        // target_left_up.x = int(max(0,int(msg->quaternion.x - target_width * roi_enlarge_ratio)));
        // target_left_up.y = int(max(0,int(msg->quaternion.y - target_height * roi_enlarge_ratio)));
        // target_right_down.x = int(min(640,int(msg->quaternion.z + target_width * roi_enlarge_ratio)));
        // target_right_down.y = int(min(480,int(msg->quaternion.w + target_height * roi_enlarge_ratio)));
        target_left_up.x = target_left_up_origin.x;
        target_left_up.y = target_left_up_origin.y;
        target_right_down.x = target_right_down_origin.x;
        target_right_down.y = target_right_down_origin.y;


        //save last ROI region when no yolo bbox input
        roi_target_left_up.x = target_left_up.x;
        roi_target_left_up.y = target_left_up.y;
        roi_target_right_down.x = target_right_down.x;
        roi_target_right_down.y = target_right_down.y;
        targetDetectFlag = true;
    }


}

void pickup_corner_cb(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    /*
     *  configure img corner for crop it
     */
    if(msg->quaternion.x < 0)
    {
        //ROS_WARN_THROTTLE(1, "pickup corner detect failed !");
        pickup_left_up.x = 0;
        pickup_left_up.y = 0;
        pickup_right_down.x = 5;
        pickup_right_down.y = 5;
    }
    pickup_left_up.x = msg->quaternion.x;
    pickup_left_up.y = msg->quaternion.y;
    pickup_right_down.x = msg->quaternion.z;
    pickup_right_down.y = msg->quaternion.w;
}

void camera_param_set() {
    //camera param

    // for pnp zzw add comment
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    distCoeffs = std::vector<double>{0.0, -0.0, -0.0, 0.0, 0};
    cameraMatrix.at<double>(0, 0) = fx; // wzy test
    cameraMatrix.at<double>(0, 2) = cx;
    cameraMatrix.at<double>(1, 1) = fy; // wzy test
    cameraMatrix.at<double>(1, 2) = cy;
    cameraMatrix.at<double>(2, 2) = 1;


    //target param

    marker_struct.clear();
    //the vector sequence is left_up, left_down, right_up, right_down
    //world points use m unit, image coordinate system, x is right, y is down, z is forward
    marker_struct.emplace_back(cv::Point3f(-0.250,-0.205,0.0));
    marker_struct.emplace_back(cv::Point3f(-0.250,0.205,0.0));
    marker_struct.emplace_back(cv::Point3f(0.250,-0.205,0.0));
    marker_struct.emplace_back(cv::Point3f(0.250,0.205,0.0));

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
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}
Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw)
{
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
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y = - asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

bool optical_follow(cv::Mat &frame, vector<cv::Point2f> &outputPointsVector)
{
    ROS_WARN("optical_follow function");
    if (frame.empty())
    {
        ROS_ERROR("optical_follow ,NO frame !!!");
        return false;
    }
    nextImg = frame;
    prevImgPts = outputPointsVector;
    nextImgPts.resize(prevImgPts.size());
    cout << "outputPointsVector[0][1][2][3]: " << endl
    << outputPointsVector[0] << endl
    << outputPointsVector[1] << endl
    << outputPointsVector[2] << endl
    << outputPointsVector[3] << endl;
    if (prevImgPts.size() != 4)
    {
        swap(nextImg, prevImg);
        ROS_WARN("prevImgPts.size() != 4 but == %d", int(prevImgPts.size()));
        return false;
    }

    for (int i = 0; i <= 1; ++i) {
        for (int j = 2; j <=3 ; ++j) {
            if(abs(prevImgPts[i].x - prevImgPts[j].x) < 1){
                ROS_ERROR("optical x axis mixed !!");
                return false;
            }

        }
    }

    if(abs(prevImgPts[0].y - prevImgPts[1].y) < 1 || abs(prevImgPts[0].y - prevImgPts[3].y) < 1 || abs(prevImgPts[2].y - prevImgPts[1].y) < 1 || abs(prevImgPts[2].y - prevImgPts[3].y) < 1)
    {
        ROS_ERROR("optical y axis mixed !!");
        return false;
    }

    if (prevImg.empty())
    {
        nextImg.copyTo(prevImg);
        ROS_INFO("prevImg is empty ,so nextImg copy to prevImg");
    }

//    calcOpticalFlowPyrLK(prevImg, nextImg, prevImgPts, nextImgPts, status, err, winSize,
//                         0, termcrit, 0, 0.001);
    calcOpticalFlowPyrLK(prevImg, nextImg, prevImgPts, nextImgPts, status, err, winSize,
                         3, termcrit, 0, 0.001);
    ROS_INFO("complete opticalFlow");

    cout << prevImgPts[0] << "follow to " << nextImgPts[0] << endl;
    size_t i, k;
    for (i = k = 0; i < nextImgPts.size(); i++) {
        if (!status[i])
            continue;
        nextImgPts[k++] = nextImgPts[i];// according to status, only assign detected element into nextImgPts
        circle(ir_img_color_show, nextImgPts[i], 4, cv::Scalar(0,255,0), 3, 8);
    }


    if (prevImgPts.size() == nextImgPts.size()) {
        outputPointsVector = nextImgPts;
        swap(nextImg, prevImg);
        swap(prevImgPts, nextImgPts);
        //! 判断4个点是否组成矩形，否则放弃这4个点
        if(abs(abs(outputPointsVector[0].y - outputPointsVector[1].y)-abs(outputPointsVector[2].y - outputPointsVector[3].y)) < 50){
            if(abs(abs(outputPointsVector[0].x -outputPointsVector[2].x) - abs(outputPointsVector[1].x - outputPointsVector[3].x)) < 50){
                ROS_INFO("optical follow , the topology of points is rectangle");
                return true;
            }
            else
                {return false;}
        }
        else
            {return false;}

    }
    else {
        swap(nextImg, prevImg);
        ROS_ERROR("pointsFollower: failed.");
        return false;
    }
}


bool roi_process(cv::Mat &_ir_img, vector<cv::Point2f> &outputPointsVector) {
    ROS_WARN("roi_process function");
    if(_ir_img.empty())
    {
        ROS_ERROR("roi_process, NO _ir_img !!!");
        return false;
    }
    roi.x = target_left_up.x;
    roi.y = target_left_up.y;
    roi.width = min(int(target_width*(1+2*roi_enlarge_ratio)),640 - roi.x);
    roi.height = min(int(target_height*(1+2*roi_enlarge_ratio)),480 - roi.y);
    ir_img_crop = _ir_img(roi);

    if(ir_img_crop.empty()){
        ROS_ERROR("roi_process crop, NO ir_img_crop !!!");
        return false;
    }

//    threshold(ir_img_crop, ir_binary, 50, 255, cv::THRESH_BINARY);
//    threshold(ir_img_crop, ir_binary, 180, 255, cv::THRESH_BINARY);
    threshold(ir_img_crop, ir_binary, 180, 255, cv::THRESH_BINARY);


//    imshow("ir_binary", ir_binary);
    dilate(ir_binary, ir_dilate, dilatestructElement);
    findContours(ir_dilate, ir_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    //判断区域大小和横纵比是圆形灯珠区域
    ir_contours_final.clear();
    for (int i = 0; i < ir_contours.size(); ++i) {
        area = cv::contourArea(ir_contours[i]);
        if(area < 10) {
            continue;
        }
        rect = boundingRect(ir_contours[i]);
        float ratio = float(rect.width) / float(rect.height);
        if(ratio < 1.1 && ratio >0.9){
            ir_contours_final.push_back(ir_contours[i]);
        }
    }

    //对齐到原图的像素点坐标
    for (int i = 0; i < ir_contours_final.size(); ++i) {
        for (int j = 0; j < ir_contours_final[i].size(); ++j) {
            ir_contours_final[i][j].x += target_left_up.x;
            ir_contours_final[i][j].y += target_left_up.y;
        }
    }
    ROS_INFO_THROTTLE(0.5,"findcontours complete");

    drawContours(ir_img_color_show, ir_contours_final, -1, (250, 100, 0), -1);
//    imshow("contours", ir_img_color_show);
//    cv::waitKey(1);
    if (ir_contours_final.size() == 4) {
        ROS_INFO_THROTTLE(0.5," get 4 contours ");
        //find each point and center
        marker_sum.x = 0;
        marker_sum.y = 0;
        marker_pixels.clear();
        for (int i = 0; i < ir_contours_final.size(); i++) {
            cv::Rect bbox;
            bbox = boundingRect(ir_contours_final[i]);
            marker_pixels.emplace_back(cv::Point2f((bbox.tl() + bbox.br()).x / 2.0, (bbox.tl() + bbox.br()).y / 2.0));
            marker_sum += cv::Point2d((bbox.tl() + bbox.br()).x / 2.0, (bbox.tl() + bbox.br()).y / 2.0);
        }
        marker_center = marker_sum / 4.0;

        //依据每个点和中心点的上下左右关系确定对应的 left_up, left_down, right_up, right_down
        for (int j = 0; j < marker_pixels.size(); ++j) {
            if (marker_pixels[j].x < marker_center.x) {
                if (marker_pixels[j].y < marker_center.y)
                    left_up = marker_pixels[j];
                else
                    left_down = marker_pixels[j];
            } else {
                if (marker_pixels[j].y < marker_center.y)
                    right_up = marker_pixels[j];
                else
                    right_down = marker_pixels[j];
            }
        }

        //按序放入marker序列中
        outputPointsVector.clear();
        outputPointsVector.emplace_back(left_up);
        outputPointsVector.emplace_back(left_down);
        outputPointsVector.emplace_back(right_up);
        outputPointsVector.emplace_back(right_down);
        return true;
    }
    else{
        ROS_WARN("the number of contours is not 4 but  %d" ,int(ir_contours_final.size()));
        return false;
    }
}


bool pnp_process(vector<cv::Point2f> &imgPoints)
{
    if(!opticalGoodFlag && !roiGoodFlag){
        return false;
    }
    ROS_WARN("pnp_process function");
    //! 判断4个点是否组成矩形，否则放弃这4个点
//    if(abs(abs(imgPoints[0].y - imgPoints[1].y)-abs(imgPoints[2].y - imgPoints[3].y)) < 50){
//        if(abs(abs(imgPoints[0].x -imgPoints[2].x) - abs(imgPoints[1].x - imgPoints[3].x)) < 50){

            for (int i = 0; i < imgPoints.size(); i++) {
                circle(ir_img_color_show, imgPoints[i], 2, cv::Scalar(0,0,255), 2, 8);
            }

            ROS_INFO("the topology of points is rectangle");
            //solvePnP
            solvePnP(marker_struct, imgPoints, cameraMatrix, distCoeffs, outputRvecRaw, outputTvecRaw, false,
                     cv::SOLVEPNP_EPNP);
            target_position_of_img[0] = outputTvecRaw.val[0];
            target_position_of_img[1] = outputTvecRaw.val[1];
            target_position_of_img[2] = outputTvecRaw.val[2];
            target_position_of_img[3] = 1.0;
            ROS_WARN("target_x: %f, target_y: %f, target_z: %f", target_position_of_img[0],target_position_of_img[1], target_position_of_img[2]);
            msg_target_pose_from_img.header.stamp = ros::Time::now();
            msg_target_pose_from_img.pose.position.x = target_position_of_img[0];
            msg_target_pose_from_img.pose.position.y = target_position_of_img[1];
            msg_target_pose_from_img.pose.position.z = target_position_of_img[2];

            if(abs(msg_target_pose_from_img.pose.position.x) > 2 || abs(msg_target_pose_from_img.pose.position.y) > 2 || abs(msg_target_pose_from_img.pose.position.z) > 4 || abs(msg_target_pose_from_img.pose.position.z) < 2){

                cv::putText(ir_img_color_show,
                        "!! pnp result over limit !!",
                        cv::Point(300, 140), cv::FONT_HERSHEY_SIMPLEX ,0.8,cv::Scalar(255,255,0),2,8,false);

                return false;
            }

            return true;
        }
//        else{
//            ROS_ERROR("not rectangle, up and down error of x axis is: %f", abs(left_up.x -right_up.x) - abs(left_down.x - right_down.x));
//            return false;
//        }
//    }
//    else{
//        ROS_WARN("not rectangle, left and right error of y axis is: %f ", abs(left_down.y - left_up.y)-abs(right_up.y - right_down.y));
//        return false;
//    }
//}

bool yolo_process(cv::Mat &frame){
    //ROS_WARN("yolo_process function");
    if(frame.empty()){
        ROS_ERROR("yolo_process, NO frame");
        return false;
    }
    if(targetDetectFlag){
      //  ROS_INFO("targetDetectFlag");
        yolo_center.x = (target_left_up_origin.x + target_right_down_origin.x)/2.0;
        yolo_center.y = (target_left_up_origin.y + target_right_down_origin.y)/2.0;
        cv::circle(ir_img_color_show,cv::Point(yolo_center.x,yolo_center.y),3,(255,0,0),10);
//        yolo_center.x = (target_left_up.x + target_right_down.x)/2.0;
//        yolo_center.y = (target_left_up.x + target_right_down.y)/2.0;
        //ROS_INFO("target_left_up.x: %d", target_left_up.x);
        //ROS_INFO("target_left_up.y: %d", target_left_up.y);
        //ROS_INFO("target_right_down.x: %d", target_right_down.x);
        //ROS_INFO("target_right_down.y: %d", target_right_down.y);
        //ROS_INFO("target_width: %d", target_width);
        //ROS_INFO("target_height: %d", target_height);

//        ROS_INFO("yolo_center.x: %f", yolo_center.x);
 //       ROS_INFO("yolo_center.y: %f", yolo_center.y);
//        yolo_area = abs((target_left_up.x - target_right_down.x)*(target_left_up.y - target_right_down.y));
//        ROS_INFO("yolo_area: %f", yolo_area);
        //根据小孔成像原理估计深度
        target_yolo_distance.z = scale_size * target_width_world * fx_color / sqrt(target_width*target_height) ;
        target_yolo_distance.x = target_yolo_distance.z * (yolo_center.x-cx) / fx_color;
        target_yolo_distance.y = target_yolo_distance.z * (yolo_center.y-cy) / fy_color;

        msg_target_pose_from_img.header.stamp = ros::Time::now();
        msg_target_pose_from_img.pose.position.x = target_yolo_distance.x;
        msg_target_pose_from_img.pose.position.y = target_yolo_distance.y;
        msg_target_pose_from_img.pose.position.z = target_yolo_distance.z;
        return true;
    }
    else{
        return false;
    }
}

string Convert(float Num)
{
ostringstream oss;
oss<<Num;
string str(oss.str());
return str;
}
