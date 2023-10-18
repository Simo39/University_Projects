/*
This node implements a service that detects the apriltags and sends the response that
cosists of the poses of the tag with respect to the target frame (sent as goal) and the
dimensions of the shapes corresponding to the tags
*/


// Standard Library
#include <string>

// ROS Library
#include "ros/ros.h"

#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

#include "geometry_msgs/PoseStamped.h"
#include "shape_msgs/SolidPrimitive.h"

// User-defined service
#include "assignment_2/DetectAndTransform.h"
#include "assignment_2/Transform.h"

class DetectAndTransform{
    public:
    DetectAndTransform(){
        service_server = nh.advertiseService("/detect_and_transform", &DetectAndTransform::doService, this);
        transform_service = nh.serviceClient<assignment_2::Transform>("/transform_poses");
    }

    private:

    bool doService(assignment_2::DetectAndTransform::Request &req, assignment_2::DetectAndTransform::Response &res){

        std::string target_frame = req.target_frame;

        apriltag_ros::AprilTagDetectionArray detections_msg;
        boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedPtr;

        // Waiting for a message to be published in topic /tag_detections
        sharedPtr  = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh);

        // Saving the message as a non-pointer
        detections_msg = *sharedPtr;

        std::vector<int> ids;
        std::vector<geometry_msgs::PoseStamped> input_apriltag_poses;
        std::vector<shape_msgs::SolidPrimitive> shapes;
        
        // Retrive apriltag detections and their object shapes and centers
        for(apriltag_ros::AprilTagDetection detection : detections_msg.detections){
            geometry_msgs::PoseStamped apriltag_pose;
            shape_msgs::SolidPrimitive shape;

            // Get the id and save it
            int id = detection.id[0];
            ids.push_back(id);

            // Get the apriltag pose and save it          
            apriltag_pose.header = detections_msg.header;
            apriltag_pose.pose = detection.pose.pose.pose;
            input_apriltag_poses.push_back(apriltag_pose);

            // Get the shape and save it
            shape = getShape(id);
            shapes.push_back(shape);
        }

        // Set first response parameter
        res.ids = ids;
        res.shapes = shapes;

        // If no target_frame is passed, then set response poses without transforming them and send reponse
        if(req.target_frame.size() == 0){
            res.apriltag_poses = input_apriltag_poses;
            ROS_INFO("Detected %lu apriltag poses", input_apriltag_poses.size());
            return true;
        }

        // Call Transform service
        assignment_2::Transform tf_srv;

        // Set transform service request for apriltags
        tf_srv.request.target_frame = req.target_frame;
        tf_srv.request.input = input_apriltag_poses;

        // Send request
        if(transform_service.call(tf_srv)){
            // Set response poses with the ones with the new frame_id set to "target_frame"
            res.apriltag_poses = tf_srv.response.output;
        }
        else{
            std::cout << "[ERROR] Failed into transforming the poses" << std::endl;
            return false;
        }

        // Set transform service request for object centers
        tf_srv.request.target_frame = req.target_frame;
        
        ROS_INFO("Detected %lu apriltag poses and transformed them into frame: %s", res.apriltag_poses.size(), target_frame.c_str());
        
        return true;
    }

    shape_msgs::SolidPrimitive getShape(int id){
        shape_msgs::SolidPrimitive shape;
        // Blue object
        if(id == 1){
            shape = getBlueShape();
        }
        // Red object
        else if(id == 3){
            shape = getRedShape();
        }
        // Green object
        else if(id == 2){
            shape = getGreenShape();
        }
        // Other objects
        else{
            shape = getOtherShape();
        }
        return shape;
    }

    shape_msgs::SolidPrimitive getBlueShape(){
        shape_msgs::SolidPrimitive shape;

        // Manually measured sizes
        double h = 0.13;
        double r = 0.03;
        // Increase the size by delta 
        double delta = 0.01;

        shape.type = shape.CYLINDER;
        shape.dimensions.resize(2);
        shape.dimensions[0] = (h);
        shape.dimensions[1] = r;
        return shape;
    }
    
    shape_msgs::SolidPrimitive getRedShape(){
        shape_msgs::SolidPrimitive shape;

        // Manually measured sizes:
        double dx = 0.05;
        double dy = 0.05;
        double dz = 0.05;
        // Increase the size by delta 
        double delta = 0.01;     

        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = (dx);
        shape.dimensions[1] = (dy);
        shape.dimensions[2] = (dz);
        return shape;
    }
    
    shape_msgs::SolidPrimitive getGreenShape(){
        shape_msgs::SolidPrimitive shape;

        // Manually measured size
        double dx = 0.05;
        double dy = 0.07;
        double dz = 0.035;
        // Increase the size by delta 
        double delta = 0.01;

        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = (dx);
        shape.dimensions[1] = (dy);
        shape.dimensions[2] = (dz);
        return shape;
    }
    
    shape_msgs::SolidPrimitive getOtherShape(){
        shape_msgs::SolidPrimitive shape;

        // Manually measured sizes
        double h = 0.25;
        double r = 0.08;
        // Increase the size by delta 
        double delta = 0.01;

        shape.type = shape.CYLINDER;
        shape.dimensions.resize(2);
        shape.dimensions[0] = (h) + delta;
        shape.dimensions[1] = r + delta;
        return shape;
    }

    ros::NodeHandle nh;
    ros::ServiceServer service_server;
    ros::ServiceClient transform_service;
};

int main(int argc, char** argv){

    //create service node
    ros::init(argc, argv, "apriltag_detection_and_transform");

    DetectAndTransform service;
    
    ROS_INFO("Detect and Transform Service READY");

    ros::spin();

    return 0;
}