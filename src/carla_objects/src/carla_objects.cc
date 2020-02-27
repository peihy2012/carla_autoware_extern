
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <string>
#include <sstream>

# include<ctime>
# define  NUMMOD  10000
# define  NUMDEV  10000.0

#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "derived_object_msgs/ObjectArray.h"
#include "geometry_msgs/PointStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf/transform_datatypes.h"

// #include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#define _USE_MESSAGE_FILTERS_SYNC_

#define __APP_NAME__ "carla_objects"

// #include <iostream>
#include <random>
#include <ctime>

// std::mt19937 rnd(time(0));

class RandomError {
public:
    RandomError() : random_gen_(time(0)){}
    ~RandomError() {}
    RandomError(time_t tm): random_gen_(time(0)) {}
    uint_fast32_t gen() {
        return random_gen_();
    }
    double error_rate () {
        uint_fast32_t rndint = random_gen_();
        int* rnd = reinterpret_cast<int*> (&rndint);
        int unit = 0x7FFFFFFF;
        return (double)(*rnd)/ (double)unit;
    }
    std::mt19937 random_gen_;
};

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
        param_.param<std::string>("pub_real_objects_topic", 
            pub_real_objects_topic_, 
            "/carla/real/objects");
        param_.param<double>("roi_range_distance", 
            roi_range_distance_, 
            200.0);
        param_.param<double>("roi_range_x", 
            roi_range_x_, 
            100.0);
        param_.param<double>("roi_range_y", 
            roi_range_y_, 
            60.0);
        param_.param<double>("roi_range_delta", 
            roi_range_delta_, 
            0.5);
        param_.param<double>("pose_deviation_delta", 
            pose_deviation_delta_, 
            0.05);
        param_.param<double>("dimension_deviation_delta", 
            dimension_deviation_delta_, 
            0.05);
        param_.param<double>("angle_deviation_delta", 
            angle_deviation_delta_, 
            0.05);

        ROS_INFO("[%s] self_odometry_topic: %s", 
            __APP_NAME__, 
            self_odometry_topic_.c_str());
        ROS_INFO("[%s] sub_objects_topic: %s", 
            __APP_NAME__, 
            sub_objects_topic_.c_str());
        ROS_INFO("[%s] pub_objects_topic: %s", 
            __APP_NAME__, 
            pub_objects_topic_.c_str());
        ROS_INFO("[%s] pub_real_objects_topic: %s", 
            __APP_NAME__, 
            pub_real_objects_topic_.c_str());
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
        ROS_INFO("[%s] pose_deviation_delta: %s", 
            __APP_NAME__, 
            std::to_string(pose_deviation_delta_).c_str());
        ROS_INFO("[%s] dimension_deviation_delta: %s", 
            __APP_NAME__, 
            std::to_string(dimension_deviation_delta_).c_str());
        ROS_INFO("[%s] angle_deviation_delta: %s", 
            __APP_NAME__, 
            std::to_string(angle_deviation_delta_).c_str());


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

        tf_listener_ptr_ = new tf2_ros::TransformListener(tf_buffer_);

        pub_autoware_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
            pub_objects_topic_, 1);
        pub_real_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
            pub_real_objects_topic_, 1);


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
        delete tf_listener_ptr_;
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
    ros::Publisher pub_real_objects_;
    

    std::string sub_objects_topic_;
    std::string self_odometry_topic_;
    std::string pub_objects_topic_;
    std::string pub_real_objects_topic_;

    double roi_range_distance_;
    double roi_range_x_;
    double roi_range_y_;
    double roi_range_delta_;
    double pose_deviation_delta_;
    double dimension_deviation_delta_;
    double angle_deviation_delta_;
    std::vector<std_msgs::ColorRGBA> color_rgba_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_ptr_;

    RandomError rnd_;
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
    //TODO:
}
void CarlaObjectsApp::odometry_callback( 
    const nav_msgs::Odometry::ConstPtr& self_odometry) {
    std::cout << "odometry_callback" << std::endl;
    //TODO:
}
#endif // _USE_MESSAGE_FILTERS_SYNC_
void CarlaObjectsApp::process(const derived_object_msgs::ObjectArray& in_objects,
    const nav_msgs::Odometry& self_odometry) {

    geometry_msgs::TransformStamped transformStamped;
    // geometry_msgs::PointStamped point_map, point_velo;
    try{
        transformStamped = tf_buffer_.lookupTransform( "hero/lidar/lidar1", "map",
            ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }   
    tf2::Quaternion q_rot;
    tf2::convert(transformStamped.transform.rotation , q_rot);
    q_rot.normalize();

    autoware_msgs::DetectedObjectArray detected_objects;
    autoware_msgs::DetectedObjectArray real_objects;
    detected_objects.header = in_objects.header;
    detected_objects.header.frame_id = "velodyne";
    real_objects.header = in_objects.header;
    real_objects.header.frame_id = "velodyne";
    // 
    #define X_BOTTOM 0
    #define X_TOP    1
    #define Y_BOTTOM 2
    #define Y_TOP    3
    double boundings[4];
    boundings[X_BOTTOM] = self_odometry.pose.pose.position.x - roi_range_distance_;
    boundings[X_TOP] = self_odometry.pose.pose.position.x + roi_range_distance_;
    boundings[Y_BOTTOM] = self_odometry.pose.pose.position.y - roi_range_distance_;
    boundings[Y_TOP] = self_odometry.pose.pose.position.y + roi_range_distance_;

    for (auto &obj : in_objects.objects) {
        if (obj.pose.position.x < boundings[X_BOTTOM] ||
            obj.pose.position.x > boundings[X_TOP] ||
            obj.pose.position.y < boundings[Y_BOTTOM] ||
            obj.pose.position.y > boundings[Y_TOP] ) {
            continue;
        }

        if (obj.pose.position.x < self_odometry.pose.pose.position.x + roi_range_delta_ &&
            obj.pose.position.x > self_odometry.pose.pose.position.x - roi_range_delta_ &&
            obj.pose.position.y < self_odometry.pose.pose.position.y + roi_range_delta_ &&
            obj.pose.position.y > self_odometry.pose.pose.position.y - roi_range_delta_ ) {
            continue;
        }

        autoware_msgs::DetectedObject detected_object;
        tf2::doTransform (obj.pose, detected_object.pose, transformStamped);
        if (detected_object.pose.position.x > roi_range_x_ || 
            detected_object.pose.position.x < -roi_range_x_ || 
            detected_object.pose.position.y > roi_range_y_ || 
            detected_object.pose.position.y < -roi_range_y_ ) {
            continue;
        }

        
        detected_object.header = obj.header;
        detected_object.header.frame_id = "velodyne";
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
        // detected_object.space_frame = obj.header.frame_id;
        detected_object.space_frame = "velodyne";

        // geometry_msgs/Pose              pose
        // detected_object.pose = obj.pose;
        // detected_object.pose.position.x = point_velo.point.x;
        // detected_object.pose.position.y = point_velo.point.y;
        // detected_object.pose.position.z = point_velo.point.z;
        // Stuff the new rotation back into the pose. This requires conversion into a msg type
        // tf2::convert(q_new, detected_object.pose.orientation);
        // tf2::doTransform (obj.pose, detected_object.pose, transformStamped);
        // if (obj.pose.position.x > roi_range_x_ || 
        //     obj.pose.position.x < -roi_range_x_ || 
        //     obj.pose.position.y > roi_range_y_ || 
        //     obj.pose.position.x < -roi_range_y_ ) {
        //     continue;
        // }
        // geometry_msgs/Vector3           dimensions
        if (obj.shape.type == obj.shape.BOX) {
            if( obj.shape.dimensions[obj.shape.BOX_X] > obj.shape.dimensions[obj.shape.BOX_Y] ) {
                detected_object.dimensions.x = obj.shape.dimensions[obj.shape.BOX_X];
                detected_object.dimensions.y = obj.shape.dimensions[obj.shape.BOX_Y];
            } else {
                detected_object.dimensions.x = obj.shape.dimensions[obj.shape.BOX_Y];
                detected_object.dimensions.y = obj.shape.dimensions[obj.shape.BOX_X]; 
            }
            detected_object.dimensions.z = obj.shape.dimensions[obj.shape.BOX_Z];
        }
        // geometry_msgs/Vector3           variance
        detected_object.variance.x = 0.0;
        detected_object.variance.y = 0.0;
        detected_object.variance.z = 0.0;
        // geometry_msgs/Twist             velocity
        //TODO: 
        // detected_object.velocity = obj.twist;
        tf2::doTransform (obj.twist.linear, 
            detected_object.velocity.linear, transformStamped);

        // tf2::Quaternion q_angular_map;
        // q_angular_map.setEulerZYX(obj.twist.angular.z, 
        //     obj.twist.angular.y, obj.twist.angular.x); // Y P R
        // tf2::Quaternion q_angular_lidar_tf2 = q_rot * q_angular_map;
        // q_angular_lidar_tf2.normalize();
        // geometry_msgs::Quaternion  q_angular_lidar = tf2::toMsg(q_angular_lidar_tf2);
        // tf::Quaternion q_angular_lidar_tf;
        // tf::quaternionMsgToTF(q_angular_lidar, q_angular_lidar_tf);
        // tf::Matrix3x3(q_angular_lidar_tf).getRPY(detected_object.velocity.angular.x, 
        //     detected_object.velocity.angular.y, detected_object.velocity.angular.z);


        // // geometry_msgs/Twist             acceleration
        // //TODO: 
        tf2::doTransform (obj.accel.linear, 
            detected_object.acceleration.linear, transformStamped);

        // tf2::Quaternion q_angulard_map;
        // q_angular_map.setEulerZYX(obj.accel.angular.z, 
        //     obj.accel.angular.y, obj.accel.angular.x); // Y P R
        // tf2::Quaternion q_angulard_lidar_tf2 = q_rot * q_angulard_map;
        // q_angulard_lidar_tf2.normalize();
        // geometry_msgs::Quaternion  q_angulard_lidar = tf2::toMsg(q_angulard_lidar_tf2);
        // tf::Quaternion q_angulard_lidar_tf;
        // tf::quaternionMsgToTF(q_angulard_lidar, q_angulard_lidar_tf);
        // tf::Matrix3x3(q_angulard_lidar_tf).getRPY(detected_object.acceleration.angular.x, 
        //     detected_object.acceleration.angular.y, detected_object.acceleration.angular.z);

        // sensor_msgs/PointCloud2         pointcloud
        // geometry_msgs/PolygonStamped    convex_hull
        detected_object.convex_hull.header = obj.header;
        detected_object.convex_hull.polygon = obj.polygon;
        // autoware_msgs/LaneArray         candidate_trajectories
        // bool                            pose_reliable
        detected_object.pose_reliable = true;
        // bool                            velocity_reliable
        detected_object.velocity_reliable = false;
        // bool                            acceleration_reliable
        detected_object.acceleration_reliable = false;
        detected_object.valid = true;

        real_objects.objects.push_back(detected_object);

        detected_object.pose.position.x *= 1 + pose_deviation_delta_ * rnd_.error_rate();
        detected_object.pose.position.y *= 1 + pose_deviation_delta_ * rnd_.error_rate();
        detected_object.pose.position.z *= 1 + pose_deviation_delta_ * rnd_.error_rate();
        detected_object.dimensions.x *= 1 + dimension_deviation_delta_ * rnd_.error_rate();
        detected_object.dimensions.y *= 1 + dimension_deviation_delta_ * rnd_.error_rate();
        detected_object.dimensions.z *= 1 + dimension_deviation_delta_ * rnd_.error_rate();
        tf::Quaternion q_ori_lidar_tf;
        tf::quaternionMsgToTF(detected_object.pose.orientation, q_ori_lidar_tf);
        double roll, pitch, yaw;
        tf::Matrix3x3(q_ori_lidar_tf).getRPY(roll, pitch, yaw);
        yaw *= 1 + angle_deviation_delta_ * rnd_.error_rate();
        tf::Quaternion out_quaternion;
        out_quaternion.setRPY(roll, pitch, yaw);
        detected_object.pose.orientation.x = out_quaternion.getX();
        detected_object.pose.orientation.y = out_quaternion.getY();
        detected_object.pose.orientation.z = out_quaternion.getZ();
        detected_object.pose.orientation.w = out_quaternion.getW();
        detected_objects.objects.push_back(detected_object);

        // std::stringstream ss;
        // ss << "[";
        // ss << obj.id ;
        // ss << "] " << detected_object.label << " " << detected_object.space_frame.c_str();
        // ss << "(" << detected_object.pose.position.x
        //     << ", " << detected_object.pose.position.y
        //     << ", " << detected_object.pose.position.z 
        //     << ") (" << detected_object.pose.orientation.x
        //     << ", " << detected_object.pose.orientation.y
        //     << ", " << detected_object.pose.orientation.z
        //     << ", " << detected_object.pose.orientation.w
        //     << ") (" << detected_object.velocity.linear.x
        //     << ", " << detected_object.velocity.linear.y
        //     << ", " << detected_object.velocity.linear.z
        //     << ")" << std::endl;
        // std::cout << ss.str() ;
    }
    pub_real_objects_.publish(real_objects);
    pub_autoware_objects_.publish(detected_objects);
}

int main (int argc, char** argv) {
    srand(time(0));
    ros::init(argc, argv, __APP_NAME__);
    CarlaObjectsApp app;
    ros::spin();
    return 0;
}