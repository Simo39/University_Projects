/*
This node implments a service that return the requested pose for the head or arm or navigation
based on the request. In this node are saved all the hard-coded poses
*/

// C++ Library
#include <fstream>
#include <vector>

// ROS Library
#include <ros/ros.h>
#include <apriltag_ros/continuous_detector.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectory.h>

// User-defined Library
#include <assignment_2/Poses.h>

class GetPoses{
    public:
    GetPoses(std::string file_name): arm_cartesian(2), navigation_pose(6){
        service = n.advertiseService("get_poses_service", &GetPoses::serviceCallback, this);
        setPoses(file_name);
        ROS_INFO("Ready to send poses!");
    }

    private:

    void setPoses(std::string file_name){
        // Set frame_id for arm cartesian
        arm_cartesian[0].header.frame_id = "base_footprint";
        arm_cartesian[1].header.frame_id = "base_footprint";
        // Initialize head_joint
        head_joint.joint_names.resize(2);
        head_joint.joint_names[0] = "head_1_joint";
        head_joint.joint_names[1] = "head_2_joint";
        head_joint.points.resize(1);
        head_joint.points[0].positions.resize(2);
        head_joint.points[0].time_from_start = ros::Duration(2.0);

        // Acces the file
        std::ifstream file(file_name);
        if(!file.is_open()){
            throw "ERROR on reading input file";
        }

        // Retrieve the coordinates from the file
        for(int point_count = 0; point_count < 9; point_count++){
            // Arm poses
            if(point_count < 2){
                arm_cartesian[point_count].pose.position.x = getNumberFromFile(file);
                arm_cartesian[point_count].pose.position.y = getNumberFromFile(file);
                arm_cartesian[point_count].pose.position.z = getNumberFromFile(file);
                double roll = getNumberFromFile(file);
                double pitch = getNumberFromFile(file);
                double yaw = getNumberFromFile(file);
                arm_cartesian[point_count].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            }
            // Joint position
            else if(point_count == 2){
                head_joint.points[0].positions[0] = getNumberFromFile(file);
                head_joint.points[0].positions[1] = getNumberFromFile(file);
            }
            // Robot poses
            else{
                navigation_pose[point_count-3].position.x = getNumberFromFile(file);
                navigation_pose[point_count-3].position.y = getNumberFromFile(file);
                navigation_pose[point_count-3].position.z = getNumberFromFile(file);
                double roll = getNumberFromFile(file);
                double pitch = getNumberFromFile(file);
                double yaw = getNumberFromFile(file);
                navigation_pose[point_count-3].orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            }
        }
    }

    double getNumberFromFile(std::ifstream &file){
        std::string line;
        while(line[0] == '#' || line.empty()){
            std::getline(file, line);
        }
        return std::stod(line);
    }

    bool serviceCallback(assignment_2::Poses::Request &req, assignment_2::Poses::Response &res){    // Request variables
        std::string group_space = req.request[0];       //type of request
        std::string pose = req.request[1];              //pose request

        //poses for the arm in cartesian space
        if(group_space == "arm_cartesian")
        {
            //default arm pose for navigation
            if(pose == "default")
            {
                //send response
                res.pose_cartesian = arm_cartesian[0];

                ROS_INFO("Cartesian space default arm position sent");
            }

            //safe moving pose with object picked
            if(pose == "moving_object")
            {
                //send response
                res.pose_cartesian = arm_cartesian[1];

                ROS_INFO("Cartesian space safe moving pose with object picked sent");
            }
        }
        
        //poses for the head in joint space
        if(group_space == "head_joint")
        {
            //default head pose for navigation
            if(pose == "default")
            {
                //send response
                res.pose_joint = head_joint;

                ROS_INFO("Joint space default head position sent");
            }
        }

        //poses for the robot navigation
        if(group_space == "navigation")
        {
            //waypoint after the corridor
            if(pose == "waypoint_end_corridor")
            {
                //send response
                res.navigation_pose = navigation_pose[0];

                ROS_INFO("End-corridor waypoint sent");
            }

            //waypoint for going from table to cilinders
            if(pose == "waypoint_table_to_cilinders")
            {
                //send response
                res.navigation_pose = navigation_pose[1];

                ROS_INFO("Table to cilinders waypoint sent");
            }

            //waypoint for going from cilinders to table
            if(pose == "waypoint_cilinders_to_table")
            {
                //send response
                res.navigation_pose = navigation_pose[2];

                ROS_INFO("Cilinders to table waypoint sent");
            }

            //waypoint for picking blue object
            if(pose == "waypoint_blue")
            {
                //send response
                res.navigation_pose = navigation_pose[3];

                ROS_INFO("Blue object waypoint sent");
            }

            //waypoint for picking red object
            if(pose == "waypoint_red")
            {
                //send response
                res.navigation_pose = navigation_pose[4];

                ROS_INFO("Red object waypoint sent");
            }

            //waypoint for picking green object
            if(pose == "waypoint_green")
            {
                //send response
                res.navigation_pose = navigation_pose[5];

                ROS_INFO("Green object waypoint sent");
            }
        }
        return true;
    }

    ros::NodeHandle n;
    ros::ServiceServer service;
    std::vector<geometry_msgs::PoseStamped> arm_cartesian;
    trajectory_msgs::JointTrajectory head_joint;
    std::vector<geometry_msgs::Pose> navigation_pose;
};

int main(int argc, char **argv)
{
    //create service node
    ros::init(argc, argv, "get_poses_service_node");

    if(argc != 2){
        std::cout << "ERROR get_poses node wants as input txt file with the poses." << std::endl;
    }

    GetPoses get_poses(argv[1]);

    ros::spin();

    return 0;
}