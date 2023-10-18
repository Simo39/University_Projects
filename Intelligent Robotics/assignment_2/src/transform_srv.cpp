// Standard Library
#include <string>

// ROS Library
#include <ros/ros.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// User-defined service
#include "assignment_2/Transform.h"

class TransformPoses{
    public:

    TransformPoses() : tfListener(this->buffer){
        service_server = nh.advertiseService("/transform_poses", &TransformPoses::doService, this);
    }

    private:

    bool doService(assignment_2::Transform::Request &req, assignment_2::Transform::Response &res){
        std::string target_frame = req.target_frame;
        std::vector<geometry_msgs::PoseStamped> input = req.input;
        std::vector<geometry_msgs::PoseStamped> output;
        
        // Transform input poses from their frame_id to target_frame
        for(geometry_msgs::PoseStamped input_pose : input){
            geometry_msgs::TransformStamped transform;
            transform = buffer.lookupTransform(target_frame, input_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped output_pose;
            tf2::doTransform(input_pose, output_pose, transform);
            output.push_back(output_pose);
        }

        res.output = output;
        
        ROS_INFO("Transformed %d poses into frame: %s", output.size(), target_frame.c_str());

        return true;       
    }

    ros::NodeHandle nh;
    ros::ServiceServer service_server;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "transform_poses");
    TransformPoses service;
    ROS_INFO("Transform Service READY");
    ros::spin();
    return 0;
}