/*
This node implements an action server that handles the pick and place routine. Basically it
manages the navigation through the waypoints at the end of the corridor, at the coffe table
and the one for the detection of the cylindric tables. It sends the request to the pick and
place action servers and takes the result from the navigation server (assignment1) to 
detect the corresponding cylindrical table to place the object
*/


// Standard Library
#include <cmath>
#include <map>
#include <string>
#include <vector>

// ROS Library
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>

// User-defined Library
// Assignment_1
#include <assignment_1/Circle.h>   
#include <assignment_1/TiagoNavigationAction.h>
// Assignment_2
#include <assignment_2/PickAction.h>
#include <assignment_2/PickAndPlaceAction.h>
#include <assignment_2/PlaceAction.h>
#include <assignment_2/Poses.h>

class PickAndPlaceAction{
    public:

    PickAndPlaceAction(std::string name):
    action_server(nh, name, boost::bind(&PickAndPlaceAction::executePickAndPlace, this, _1), false), server_name(name),
    navigation("navigation_server", true), pick("picking_routine_server", true), place("place_routine_server", true)
    {
        // Initialize client service
        get_poses_client = nh.serviceClient<assignment_2::Poses>("get_poses_service");

        ROS_INFO("Waiting for the robot navigation server");
        navigation.waitForServer();
        ROS_INFO("Waiting for the picking routine server");
        pick.waitForServer();
        ROS_INFO("Waiting for the placing routine server");
        place.waitForServer();
        ROS_INFO("Servers READY");
        
        action_server.start();

        setColorMap();

        ROS_INFO("Pick and Place Action Server READY");
    }

    protected:

    //======================================//
    // ACTION SERVER METHODS IMPLEMENTATION //
    //======================================//

    void executePickAndPlace(const assignment_2::PickAndPlaceGoalConstPtr &goal){
        // Check if it is in starting pose
        if(!starting_pose){
            setStartingPose();
            starting_pose = true;
        }
        
        // Retrieve the object id to pick
        goal_id = goal->id;
        
        // Move to the relative waypoint for the picking routine
        std::string pick_waypoint = "waypoint_" + colors[goal_id];
        waypoint.pose = getWaypointPose(pick_waypoint);
        moveTo(pick_waypoint, waypoint);
        
        // Start picking routine
        setFeedback("Starting object picking procedure");
        // Create the goal
        pick_goal.id = goal_id;
        setFeedback("Send goal to pick routine Server");
        // Send the goal
        pick.sendGoal(pick_goal,
            boost::bind(&PickAndPlaceAction::pickDoneCb, this, _1, _2),
            boost::bind(&PickAndPlaceAction::pickActiveCb, this),
            boost::bind(&PickAndPlaceAction::pickFeedbackCb, this, _1));
        setFeedback("Waiting picking action");
        // Wait for the pick routine to complete
        pick.waitForResult();
        setFeedback("Object picked");
        
        // Move to the waypoint in front of the cylindrical tables
        waypoint.pose = getWaypointPose("waypoint_table_to_cilinders");
        moveTo("waypoint_table_to_cilinders", waypoint);
        
        // Detect and store cylindrical table poses
        if(!cylindrical_tables_detected){
            detectPlacingTables(navigation.getResult(), waypoint.pose);
            cylindrical_tables_detected = true;
        }

        // Move to the relative cylindrical table for placing routine
        std::string table_name = "table_" + colors[goal_id];
        moveTo(table_name, cylinder_tables_approach_poses[goal_id]);
        
        // Start placing routine routine
        setFeedback("Starting object placing procedure");
        setFeedback("Send goal to pick routine Server");
        // Create the goal
        place_goal.id = goal_id;
        place_goal.table_pose = cylinder_tables_poses[goal_id];
        // Send the goal
        place.sendGoal(place_goal,
            boost::bind(&PickAndPlaceAction::placeDoneCb, this, _1, _2),
            boost::bind(&PickAndPlaceAction::placeActiveCb, this),
            boost::bind(&PickAndPlaceAction::placeFeedbackCb, this, _1));
        setFeedback("Waiting placing action");
        // Wait for the place routine to complete
        place.waitForResult();
        setFeedback("Object placed");
        
        // Move to resting waypoint
        waypoint.pose = getWaypointPose("waypoint_cilinders_to_table");
        moveTo("waypoint_cilinders_to_table", waypoint);

        // Send result to client
        ROS_INFO("Sending result");
        setFeedback("Robot is sending results");

        // Set the result to saccessful and send it
        action_result.completed = true;
        action_server.setSucceeded(action_result);

        ROS_INFO("Result sent");
    }

    //=======================================//
    // ACTION CLIENTS METHODS IMPLEMENTATION //
    //=======================================//

    //********************//
    // Pick Action Client //
    //********************//

    void pickDoneCb(const actionlib::SimpleClientGoalState& state,
                const assignment_2::PickResultConstPtr& result){
        ROS_INFO("Goal %s", state.toString().c_str());

        ROS_INFO("Object picked");
    }


    // Called when the goal becomes active
    void pickActiveCb(){
        ROS_INFO("Robot is now ready to start to pick the object");
    }

    //called when feedback is sent
    void pickFeedbackCb(const assignment_2::PickFeedbackConstPtr& feedback){
        ROS_INFO("%s", feedback->status.c_str());
    }

    //*********************//
    // Place Action Client //
    //*********************//

    void placeDoneCb(const actionlib::SimpleClientGoalState& state,
                const assignment_2::PlaceResultConstPtr& result){
        ROS_INFO("Goal %s", state.toString().c_str());

        ROS_INFO("Object placed");
    }


    // Called when the goal becomes active
    void placeActiveCb(){
        ROS_INFO("Robot is now ready to start to place the object");
    }

    //called when feedback is sent
    void placeFeedbackCb(const assignment_2::PlaceFeedbackConstPtr& feedback){
        ROS_INFO("%s", feedback->status.c_str());
    }

    private:

    //===============//
    // CLASS METHODS //
    //===============//

    //this function sets the head to default position
    void setHeadPosition(){
        // Create an action client for the head controller
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_client("head_controller/follow_joint_trajectory", true);
        // Head goal pose in joint space
        control_msgs::FollowJointTrajectoryGoal head_goal;  //head goal pose in joint space
        
        // Get the head pose through the poses service
        poses_service.request.request = {"head_joint", "default"};
        get_poses_client.call(poses_service);

        // Set the goal for the head pose
        head_goal.trajectory = poses_service.response.pose_joint;

        // Wait for the action server to come up
        while (!head_client.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the head_controller/follow_joint_trajectory action server to come up");
        }

        // Send the goal
        head_client.sendGoal(head_goal);

        // Wait for the action to complete
        while (!head_client.waitForResult(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for head movement to complete");
        }

        ROS_INFO("Head moved succesfully");
    }

    //this function sets the arm to default position for navigation
    void setArmToDefaultPosition(){

        // Move group and planner
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");    //group of joints
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;                   //plan
        geometry_msgs::PoseStamped arm_pose;                                            //arm goal pose in cartesian space

        // Define and send the request to the poses server
        poses_service.request.request = {"arm_cartesian", "default"};
        get_poses_client.call(poses_service);
        arm_pose = poses_service.response.pose_cartesian;

        // Some params for planning 
        group_arm_torso.setPlanningTime(45.0);                      //maximum time allowed for planning, then return failure
        group_arm_torso.setPlannerId("SBLkConfigDefault");          //planner
        group_arm_torso.setPoseReferenceFrame("base_footprint");    //reference frame 
        group_arm_torso.setPoseTarget(arm_pose);                    //set goal pose
        group_arm_torso.setStartStateToCurrentState();              //current state of the robot set as start state
        group_arm_torso.setMaxVelocityScalingFactor(1.0);           //max joint velocity set (1.0)

        // Plan the arm trajectory
        bool success = static_cast<bool>(group_arm_torso.plan(my_plan));

        // If plan not found
        if ( !success )
            throw std::runtime_error("No plan found");

        // Move the arm
        group_arm_torso.move();
        ROS_INFO("Arm goal reached");
    }

    // This function sets the robot to default position, that is get to the waypoint at
    // the end of the corridor and move both head and arm to start pick and place routine
    void setStartingPose(){

        setFeedback("Going to Starting Pose");

        //move to waypoint after corridor
        waypoint.pose = getWaypointPose("waypoint_end_corridor");
        moveTo("waypoint_end_corridor", waypoint);

        //move the head
        setHeadPosition();

        //move the arm
        setArmToDefaultPosition();
    }
    
    // This function sets the correspondence between ids and objects to pick
    void setColorMap(){
        colors.insert({1, "blue"});
        colors.insert({2, "green"});
        colors.insert({3, "red"});
    }

    // This function uses the server developed in assignment1 to move tiago to a goal pose
    void moveTo(std::string name, assignment_1::TiagoNavigationGoal pose){
        // Send goal to navigation server
        navigation.sendGoal(pose);

        setFeedback("Moving to " + name);

        //wait the action to complete
        while(!navigation.waitForResult(ros::Duration(5.0)))
        {
            ROS_INFO("Moving to %s", name.c_str());
        }

        ROS_INFO("Navigation completed succesfully");

        setFeedback("Arrived to " + name);
    }

    // This function is used to get the poses of the waypoints for navigation
    geometry_msgs::Pose getWaypointPose(std::string waypoint_name){
        // Get the desired pose of that waypoint from the dedicated service
        poses_service.request.request = {"navigation", waypoint_name};
        get_poses_client.call(poses_service);
        return poses_service.response.navigation_pose;
    }

    // This funciton compute the poses of the tables based on the result from the action server
    // of assignment1 for navigation
    void detectPlacingTables(const assignment_1::TiagoNavigationResultConstPtr& result, geometry_msgs::Pose current_pose){
        std::vector<assignment_1::Circle> circles;              //detection result
        std::vector<std::vector<double>> target;                //cylindrical table poses 

        ROS_INFO("Object detection result received");
        circles = result->circles;
        ROS_INFO("N. of detected cylinders: %d", int(circles.size()));
     
        // There are how the table are positioned from right to left
        // wrt the robot pose when it detects them the first time, 
        // that is: blue (1), green (2), red (3)
        std::vector<int> color_ids{1, 2, 3};

        // x and y coordinates fo the robot current pose wrt map frame
        double robot_x = current_pose.position.x;
        double robot_y = current_pose.position.y;

        for(int i = 0; i < colors.size(); i++){
            assignment_1::Circle circle = circles[i];
            assignment_1::TiagoNavigationGoal robot_pose;
            std::vector<double> output;

            // Get infos about detected tables
            double x = circle.center_x;
            double y = circle.center_y;
            double radius = circle.radius;
            int color_id = color_ids[i];

            // Distance from the table center
            double distance = 3 * radius;

            // Orientation of the robot
            double orientation = computeOrientationToPlacingTables(x, y, robot_x, robot_y);

            // Compute robot pose
            robot_pose.pose.position.x = x - distance * cos(orientation);
            robot_pose.pose.position.y = y - distance * sin(orientation);
            robot_pose.pose.position.z = 0;
            robot_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, orientation);

            cylinder_tables_approach_poses.insert({color_id, robot_pose});

            // Add the center
            geometry_msgs::PoseStamped table_pose;
            table_pose.header.frame_id = "map";
            table_pose.pose.position.x = x;
            table_pose.pose.position.y = y;
            table_pose.pose.position.z = 0; 
            table_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0); 
            cylinder_tables_poses.insert({color_id, table_pose});
        }
    }

    // This function computes the orientation of the cylindrical table
    // The orientation is then used to find the goal pose in front of the table
    double computeOrientationToPlacingTables(double table_x, double table_y, double robot_x, double robot_y){
        // ASSUMPTION:
        // all the tables are in front of the robot, at most on the same x axis;
        // but never behind it (by means with an y < 0)
        double x = table_x - robot_x;
        double y = table_y - robot_y;

        // Compute the orientation as the same orientation of a vector
        // going from the robot to the table (theta of polar coordinates of this vector)
        double orientation = atan(y/x);

        // If table in front of the robot: already in the correct orientation
        if(x == 0){
            orientation = M_PI_2;
        }
        // If table on the robot right: it has to rotate by -90 degrees
        else if(y == 0 && x > 0){
            orientation = 0;
        }
        // If table on the robot left: it has to rotate by +90 degrees
        else if(y == 0 && x < 0){
            orientation = M_PI;
        }
        // Table is on the robot left, but not along is x axis
        else if(x < 0){
            orientation = M_PI + orientation;
        }
        // Default table is on the robot right, but not along is x axis

        return orientation;
    }

    void setFeedback(std::string feedback){
        action_feedback.robot_status = feedback;
        action_server.publishFeedback(action_feedback);
    }

    //=================//
    // CLASS VARIABLES //
    //=================//

    // Node initialization
    ros::NodeHandle nh;
    
    // Action Server
    actionlib::SimpleActionServer<assignment_2::PickAndPlaceAction> action_server;
    assignment_2::PickAndPlaceFeedback action_feedback; //message to publish feedback
    assignment_2::PickAndPlaceResult action_result;     //message to publish result

    // Action
    actionlib::SimpleActionClient<assignment_1::TiagoNavigationAction> navigation;
    assignment_1::TiagoNavigationGoal waypoint;  //nagivation goal
    actionlib::SimpleActionClient<assignment_2::PickAction> pick;
    assignment_2::PickGoal pick_goal;
    actionlib::SimpleActionClient<assignment_2::PlaceAction> place;
    assignment_2::PlaceGoal place_goal;

    // Services
    ros::ServiceClient get_poses_client;        //get_poses client
    assignment_2::Poses poses_service;          //poses service msg

    // Variables
    std::map<int, std::string> colors;
    int goal_id;
    std::string server_name;
    bool starting_pose = false;
    bool cylindrical_tables_detected = false;
    std::map<int, assignment_1::TiagoNavigationGoal> cylinder_tables_approach_poses;
    std::map<int, geometry_msgs::PoseStamped> cylinder_tables_poses;   //ordered poses for cylindrical tables
};

int main(int argc, char **argv){

    // Create action node
    ros::init(argc, argv, "pick_and_place_server");

    PickAndPlaceAction pick_n_place("pick_and_place_server");

    ros::spin();

    return 0;
}