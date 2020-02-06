
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <string>
#include <sstream>

#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "derived_object_msgs/ObjectArray.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define _USE_MESSAGE_FILTERS_SYNC_

#define __APP_NAME__ "carla_objects"



// uint8 OBJECT_DETECTED=0
// uint8 OBJECT_TRACKED=1
// uint8 CLASSIFICATION_UNKNOWN=0
// uint8 CLASSIFICATION_UNKNOWN_SMALL=1
// uint8 CLASSIFICATION_UNKNOWN_MEDIUM=2
// uint8 CLASSIFICATION_UNKNOWN_BIG=3
// uint8 CLASSIFICATION_PEDESTRIAN=4
// uint8 CLASSIFICATION_BIKE=5
// uint8 CLASSIFICATION_CAR=6
// uint8 CLASSIFICATION_TRUCK=7
// uint8 CLASSIFICATION_MOTORCYCLE=8
// uint8 CLASSIFICATION_OTHER_VEHICLE=9
// uint8 CLASSIFICATION_BARRIER=10
// uint8 CLASSIFICATION_SIGN=11

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

std::string derived_object_classification[12] = {
    "unknown",
    "unknown_small",
    "unknown_medium",
    "unknown_big",
    "pedestrain",
    "bike",
    "car",
    "truck",
    "motorcycle",
    "other_vehicle",
    "barrier",
    "sign"
};


class CarlaObjectsApp {
public:
    CarlaObjectsApp() {
        ros::NodeHandle param_("~");
        param_.param<std::string>("self_odometry_topic", 
            self_odometry_topic_, 
            "/carla/hero/odometry");
        param_.param<std::string>("sub_objects_topic", 
            sub_objects_topic_, 
            "/carla/objects");
        param_.param<std::string>("pub_objects_topic", 
            pub_objects_topic_, 
            "/carla/detected/objects");
        param_.param<double>("roi_range_distance", 
            roi_range_distance_, 
            60.0);
        param_.param<double>("roi_range_x", 
            roi_range_x_, 
            60.0);
        param_.param<double>("roi_range_y", 
            roi_range_y_, 
            60.0);
        param_.param<double>("roi_range_delta", 
            roi_range_delta_, 
            0.5);

        ROS_INFO("[%s] self_odometry_topic: %s", 
            __APP_NAME__, 
            self_odometry_topic_.c_str());
        ROS_INFO("[%s] sub_objects_topic: %s", 
            __APP_NAME__, 
            sub_objects_topic_.c_str());
        ROS_INFO("[%s] pub_objects_topic: %s", 
            __APP_NAME__, 
            pub_objects_topic_.c_str());
        ROS_INFO("[%s] roi_range_distance: %s", 
            __APP_NAME__, 
            std::to_string(roi_range_distance_).c_str());
        ROS_INFO("[%s] roi_range_x: %s", 
            __APP_NAME__, 
            std::to_string(roi_range_x_).c_str());
        ROS_INFO("[%s] roi_range_y: %s", 
            __APP_NAME__, 
            std::to_string(roi_range_y_).c_str());
        ROS_INFO("[%s] roi_range_delta: %s", 
            __APP_NAME__, 
            std::to_string(roi_range_delta_).c_str());

#ifdef _USE_MESSAGE_FILTERS_SYNC_
        odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, 
            self_odometry_topic_, 1);
        obj_sub_  = new message_filters::Subscriber<derived_object_msgs::ObjectArray>(nh_, 
            sub_objects_topic_, 1);
        sync_ = new message_filters::Synchronizer<ObjectsSyncPolicy>(ObjectsSyncPolicy(10),
            *odom_sub_, *obj_sub_);
        sync_->registerCallback(boost::bind(&CarlaObjectsApp::sync_callback, 
            this, _1, _2));
#else // _USE_MESSAGE_FILTERS_SYNC_
        sub_self_odometry_ = nh_.subscribe(self_odometry_topic_, 
            1, &CarlaObjectsApp::odometry_callback, this);
        sub_carla_objects_ = nh_.subscribe(sub_objects_topic_, 
            1, &CarlaObjectsApp::objects_callback, this);
#endif // _USE_MESSAGE_FILTERS_SYNC_

        pub_autoware_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
            pub_objects_topic_, 1);

        for (int i=0; i<12; i++) {
            std_msgs::ColorRGBA tmp_color;
            tmp_color.r = objects_color_rgba[i][0];
            tmp_color.g = objects_color_rgba[i][1];
            tmp_color.b = objects_color_rgba[i][2];
            tmp_color.a = objects_color_rgba[i][3];
            color_rgba_.push_back(tmp_color);       
        }
    }
    ~CarlaObjectsApp() {
#ifdef _USE_MESSAGE_FILTERS_SYNC_
        delete odom_sub_;
        delete obj_sub_;
        delete sync_;
#endif // _USE_MESSAGE_FILTERS_SYNC_
    }

#ifdef _USE_MESSAGE_FILTERS_SYNC_
    void sync_callback(const nav_msgs::Odometry::ConstPtr& pOdom, 
        const derived_object_msgs::ObjectArray::ConstPtr& pObj);
#else // _USE_MESSAGE_FILTERS_SYNC_
    void objects_callback(const derived_object_msgs::ObjectArray::ConstPtr& pOdom);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& pOdom);
#endif // _USE_MESSAGE_FILTERS_SYNC_

    void process(const derived_object_msgs::ObjectArray& in_objects,
        const nav_msgs::Odometry& self_odometry);

private:
    ros::NodeHandle nh_;

#ifdef _USE_MESSAGE_FILTERS_SYNC_
    // typedef message_filters::Subscriber<derived_object_msgs::ObjectArray> FllterSubObj;
    // typedef message_filters::Subscriber<nav_msgs::Odometry> FilterSubOdom;
    // typedef message_filters::Synchronizer<SyncPolicyT> FilterSync;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
        derived_object_msgs::ObjectArray> ObjectsSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_ ;       
    message_filters::Subscriber<derived_object_msgs::ObjectArray>* obj_sub_; 
    message_filters::Synchronizer<ObjectsSyncPolicy>* sync_;
#else // _USE_MESSAGE_FILTERS_SYNC_
    ros::Subscriber sub_carla_objects_;
    ros::Subscriber sub_self_odometry_;
#endif // _USE_MESSAGE_FILTERS_SYNC_

    ros::Publisher pub_autoware_objects_;

    std::string sub_objects_topic_;
    std::string self_odometry_topic_;
    std::string pub_objects_topic_;
    double roi_range_distance_;
    double roi_range_x_;
    double roi_range_y_;
    double roi_range_delta_;
    std::vector<std_msgs::ColorRGBA> color_rgba_;

};
#ifdef _USE_MESSAGE_FILTERS_SYNC_
void CarlaObjectsApp::sync_callback(const nav_msgs::Odometry::ConstPtr& pOdom, 
    const derived_object_msgs::ObjectArray::ConstPtr& pObj) {
    //TODO: 
    if (pObj == nullptr || pOdom == nullptr) {
        std::cout << "lost objects or self odometry." << std::endl;
        return ; 
    }
    std::cout << "got objects or self odometry." << std::endl;
    process(*pObj, *pOdom);
}
#else // _USE_MESSAGE_FILTERS_SYNC_
void CarlaObjectsApp::objects_callback(
    const derived_object_msgs::ObjectArray::ConstPtr& objects) {
    std::cout << "objects_callback" << std::endl;
}
void CarlaObjectsApp::odometry_callback( 
    const nav_msgs::Odometry::ConstPtr& self_odometry) {
    std::cout << "odometry_callback" << std::endl;
}
#endif // _USE_MESSAGE_FILTERS_SYNC_

void CarlaObjectsApp::process(const derived_object_msgs::ObjectArray& in_objects,
    const nav_msgs::Odometry& self_odometry) {

    autoware_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_objects.header;
    // 
    std::stringstream ss;
    ss << "[";

    #define X_BOTTOM 0
    #define X_TOP    1
    #define Y_BOTTOM 2
    #define Y_TOP    3
    double boundings[4];
    boundings[X_BOTTOM] = self_odometry.pose.pose.position.x - roi_range_x_;
    boundings[X_TOP] = self_odometry.pose.pose.position.x + roi_range_x_;
    boundings[Y_BOTTOM] = self_odometry.pose.pose.position.y - roi_range_y_;
    boundings[Y_TOP] = self_odometry.pose.pose.position.y + roi_range_y_;

    for (auto &obj : in_objects.objects) {
        if (obj.pose.position.x < boundings[X_BOTTOM] ||
            obj.pose.position.x > boundings[X_TOP] ||
            obj.pose.position.y < boundings[Y_BOTTOM] ||
            obj.pose.position.y > boundings[X_TOP] ) {
            continue;
        }
        if (obj.pose.position.x < self_odometry.pose.pose.position.x + roi_range_delta_ &&
            obj.pose.position.x > self_odometry.pose.pose.position.x - roi_range_delta_ &&
            obj.pose.position.y < self_odometry.pose.pose.position.y + roi_range_delta_ &&
            obj.pose.position.y > self_odometry.pose.pose.position.y - roi_range_delta_ ) {
            continue;
        }
        autoware_msgs::DetectedObject detected_object;
        detected_object.header = obj.header;
        // uint32                          id
        detected_object.id = obj.id;
        if (obj.object_classified && obj.classification < 12) {
            detected_object.label = derived_object_classification[obj.classification];
            detected_object.color = color_rgba_[obj.classification];
        } else {
            detected_object.label = "unknown"; 
            detected_object.color = color_rgba_[0];
        }
        detected_object.score = 1.0;
        detected_object.space_frame = obj.header.frame_id;
        // geometry_msgs/Pose              pose
        detected_object.pose = obj.pose;
        // geometry_msgs/Vector3           dimensions
        if (obj.shape.type == obj.shape.BOX) {
            detected_object.dimensions.x = obj.shape.dimensions[obj.shape.BOX_X];
            detected_object.dimensions.y = obj.shape.dimensions[obj.shape.BOX_Y];
            detected_object.dimensions.z = obj.shape.dimensions[obj.shape.BOX_Z];
        }
        // geometry_msgs/Vector3           variance
        detected_object.variance.x = 0.0;
        detected_object.variance.y = 0.0;
        detected_object.variance.z = 0.0;
        // geometry_msgs/Twist             velocity
        detected_object.velocity = obj.twist;
        // geometry_msgs/Twist             acceleration
        detected_object.acceleration.linear = obj.accel.linear;
        detected_object.acceleration.angular = obj.accel.angular;
        // sensor_msgs/PointCloud2         pointcloud
        // geometry_msgs/PolygonStamped    convex_hull
        detected_object.convex_hull.header = obj.header;
        detected_object.convex_hull.polygon = obj.polygon;
        // autoware_msgs/LaneArray         candidate_trajectories
        // bool                            pose_reliable
        detected_object.pose_reliable = true;
        // bool                            velocity_reliable
        detected_object.velocity_reliable = true;
        // bool                            acceleration_reliable
        detected_object.acceleration_reliable = true;
        detected_object.valid = true;

        detected_objects.objects.push_back(detected_object);
        ss << obj.id << " ";
    }
    ss << "]";
    std::cout << ss.str() << std::endl;
    pub_autoware_objects_.publish(detected_objects);
}

int main (int argc, char** argv) {
    ros::init(argc, argv, __APP_NAME__);
    CarlaObjectsApp app;
    ros::spin();
    return 0;
}