
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <string>
#include <sstream>

#include "element.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Header.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "geometry_msgs/PointStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include "tf/transform_datatypes.h"
// #include <tf/transform_listener.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/TransformStamped.h>

// #define _USE_MESSAGE_FILTERS_SYNC_

#define __APP_NAME__ "objects_compare"


float objects_color_rgba[12][4] = {
    {250.0, 40.0,   170.0,  0.0},
    {250.0, 40.0,   170.0,  0.0},
    {250.0, 40.0,   170.0,  0.0},
    {250.0, 40.0,   170.0,  0.0},
    {250.0, 0.0,    0.0,    0.0},
    {250.0, 20.0,   0.0,    0.0},
    {0.0,   250.0,  0.0,    0.0},
    {0.0,   250.0,  30.0,   0.0},
    {250.0, 120.0,  0.0,    0.0},
    {250.0, 30.0,   0.0,    0.0},
    {250.0, 40.0,   70.0,   0.0},
    {250.0, 40.0,   70.0,   0.0}
};


class ObjectsCompareApp {
public:
    ObjectsCompareApp();
    ~ObjectsCompareApp();

    void sync_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& detected_obj_ptr, 
        const autoware_msgs::DetectedObjectArray::ConstPtr& real_obj_ptr);

    void process(const autoware_msgs::DetectedObjectArray& detected_obj,
        const autoware_msgs::DetectedObjectArray& real_obj);

private:
    ros::NodeHandle nh_;

    typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray, 
        autoware_msgs::DetectedObjectArray> ObjectsSyncPolicy;
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray>* detected_obj_sub_ ;       
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray>* real_obj_sub_; 
    message_filters::Synchronizer<ObjectsSyncPolicy>* sync_;
    
    std::string detected_objects_topic_;
    std::string real_objects_topic_;

    std::vector<std_msgs::ColorRGBA> color_rgba_;
    ele::TrajectoryList<ele::Item, 20> detected_trajectory_list_;
    ele::TrajectoryList<ele::Item, 20> real_trajectory_list_;
};


// 
// 
// 
ObjectsCompareApp::ObjectsCompareApp() {
    ros::NodeHandle param_("~");
    param_.param<std::string>("detected_objects_topic", 
        detected_objects_topic_, 
        "/detection/lidar_detector/objects");
    param_.param<std::string>("real_objects_topic", 
        real_objects_topic_, 
        "/detection/lidar_real/objects");

    ROS_INFO("[%s] detected_objects_topic: %s", 
        __APP_NAME__, 
        detected_objects_topic_.c_str());
    ROS_INFO("[%s] real_objects_topic_: %s", 
        __APP_NAME__, 
        real_objects_topic_.c_str());

    detected_obj_sub_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh_, 
        detected_objects_topic_, 1);
    real_obj_sub_  = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(nh_, 
        real_objects_topic_, 1);
    sync_ = new message_filters::Synchronizer<ObjectsSyncPolicy>(ObjectsSyncPolicy(10),
        *detected_obj_sub_, *real_objects_topic_);
    sync_->registerCallback(boost::bind(&ObjectsCompareApp::sync_callback, 
        this, _1, _2));

    for (int i=0; i<12; i++) {
        std_msgs::ColorRGBA tmp_color;
        tmp_color.r = objects_color_rgba[i][0];
        tmp_color.g = objects_color_rgba[i][1];
        tmp_color.b = objects_color_rgba[i][2];
        tmp_color.a = objects_color_rgba[i][3];
        color_rgba_.push_back(tmp_color);       
    }
}

//
//
//
ObjectsCompareApp::~ObjectsCompareApp() {
    delete detected_obj_sub_;
    delete real_obj_sub_;
    delete sync_;
}

//
//
//
void ObjectsCompareApp::sync_callback(const autoware_msgs::DetectedObjectArray::ConstPtr& detected_obj_ptr, 
        const autoware_msgs::DetectedObjectArray::ConstPtr& real_obj_ptr); {
    //TODO: 
    if (detected_obj_ptr == nullptr || real_obj_ptr == nullptr) {
        std::cout << "lost detected objects or real objects." << std::endl;
        return ; 
    }
    std::cout << "got both objects list." << std::endl;
    process(*detected_obj_ptr, *real_obj_ptr);
}


//
//
//
void ObjectsCompareApp::process(const autoware_msgs::DetectedObjectArray& detected_obj,
        const autoware_msgs::DetectedObjectArray& real_obj) {
    
    for (auto & obj : detected_obj.objects) {
        ele::Item item = {
            obj.header.stamp.toSec(),
            0,
            {obj.pose.position.x, obj.pose.position.y},
            {obj.dimensions.x, obj.dimensions.y},
            {obj.velocity.linear.x, obj.velocity.linear.y},
            {obj.acceleration.linear.x, obj.acceleration.linear.y}
        };

        detected_trajectory_list_.push(obj.id, item);
        detected_trajectory_list_.pop();
    }
    for (auto & obj : real_obj.objects) {
        ele::Item item = {
            obj.header.stamp.toSec(),
            0,
            {obj.pose.position.x, obj.pose.position.y},
            {obj.dimensions.x, obj.dimensions.y},
            {obj.velocity.linear.x, obj.velocity.linear.y},
            {obj.acceleration.linear.x, obj.acceleration.linear.y}
        };

        real_trajectory_list_.push(obj.id, item);
        real_trajectory_list_.pop();
    }
    std::cout << "detected size: " << detected_trajectory_list_.size_ << std::endl;
    std::cout << "real size: " << real_trajectory_list_.size_ << std::endl; 
    std::cout << "real pose: " << real_trajectory_list_.trajectories_.back().items_.back().pose.x 
        << " " << real_trajectory_list_.trajectories_.back().items_.back().pose.x << std::endl;
}

int main (int argc, char** argv) {

    ros::init(argc, argv, __APP_NAME__);
    ObjectsCompareApp app;
    ros::spin();
    return 0;
}