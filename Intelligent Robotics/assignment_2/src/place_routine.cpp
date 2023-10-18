/*
This node implements an action server that handles the place routine. The server receives as
goal the id of the object to pick and the pose of the cylindrical table where to place it.
The node performs all the steps up to having the arm on a safe position for navigation
*/

// Standard Library
#include <string>

// ROS Library
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf/transform_broadcaster.h>

// ROS Messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/SolidPrimitive.h>

// User-defined Library
// Assignment_2
#include <assignment_2/DetectAndTransform.h>
#include <assignment_2/PlaceAction.h>
#include <assignment_2/PlanScene_AddObj.h>
#include <assignment_2/PlanScene_ClearScene.h>
#include <assignment_2/Poses.h>
#include <assignment_2/Transform.h>

class PlaceRoutineAction{
    public:
    
    PlaceRoutineAction(std::string name):
    action_server(nh, name, boost::bind(&PlaceRoutineAction::executePick, this, _1), false), server_name(name){
        // Initialize client services
        detection_client = nh.serviceClient<assignment_2::DetectAndTransform>("/detect_and_transform");
        add_planning_scene_client = nh.serviceClient<assignment_2::PlanScene_AddObj>("/add_collision_obj");
        clear_planning_scene_client = nh.serviceClient<assignment_2::PlanScene_ClearScene>("/clear_scene");
        transform_client = nh.serviceClient<assignment_2::Transform>("/transform_poses");
        get_poses_client = nh.serviceClient<assignment_2::Poses>("get_poses_service");


        action_server.start();
        ROS_INFO("Placing Routine Action Server READY");
    }

    protected:

    void executePick(const assignment_2::PlaceGoalConstPtr &goal){

        //get the pose of the cylindrical table with respect to base footprint frame
        std::vector<geometry_msgs::PoseStamped> table_poses;
        table_poses.push_back(goal->table_pose);
        table_poses = transform("base_footprint", table_poses);

        // Here follow a set of steps to complete the place routine
        ROS_INFO("Add Cylindrical table");
        addCylindricTable(table_poses[0]);                          //add the cylindric table to the planning scene
        ROS_INFO("Placing OBJECT ACTION EXECTUTION of object");
        setFeedback("Start placing object");
        approPlace(table_poses[0]);                                 //approach from the top the place pose
        getCloser(goal->id);                                        //get closet
        setFeedback("Object placed");
        openGripper();                                              //open the gripper
        detach(goal->id);                                           //detach the object
        moveTorso(0.35);                                            //move the torso to departs
        setFeedback("Moving arm to safe-moving pose");
        setArmToDefaultPosition();                                  //set the arm to a default safe pose for navigation
        ROS_INFO("Cleaning planning scene");
        clearPlanningScene();                                       //clear the planning scene

        // Set the result as completed and send it to the client
        action_result.completed = true;
        action_server.setSucceeded(action_result);
    }

    private:

    // This function is used to send feedback to the client
    void setFeedback(std::string feedback){
        action_feedback.status = feedback;
        action_server.publishFeedback(action_feedback);
    }

    // This function adds the cylindrical table to the planning scene
    // Its pose is given by the detection from the routine of assignment_1
    void addCylindricTable(geometry_msgs::PoseStamped table_pose){
        
        // Set the table pose at ground level
        table_pose.pose.position.z = 0;
        
        cylinder_table_shape.type = cylinder_table_shape.CYLINDER;
        cylinder_table_shape.dimensions.resize(3);
        cylinder_table_shape.dimensions[0] = 1.5;
        cylinder_table_shape.dimensions[1] = 0.25;

        // Add cylindric table to planning scene
        assignment_2::PlanScene_AddObj srv;
        srv.request.id = 0;
        srv.request.pose = table_pose;
        srv.request.shape = cylinder_table_shape;
        add_planning_scene_client.call(srv);
        
        // Add a raised floor to avoid the object colliding with the floor
        geometry_msgs::PoseStamped pose;
        shape_msgs::SolidPrimitive shape;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base_footprint";
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        // Define table dimensions
        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = 2;
        shape.dimensions[1] = 2;
        shape.dimensions[2] = 0.3;

        // Add the raised floor
        addCollisionObject(-1, pose, shape);

        // Add a wall on the left side to avoid the object colliding with other cylindrical talbes
        pose.pose.position.x = 0.5;
        pose.pose.position.y = 0.9;
        pose.pose.position.z = 0.5;

        // Define table dimensions
        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = 1;
        shape.dimensions[1] = 0.2;
        shape.dimensions[2] = 1;

        // Add the wall
        addCollisionObject(-2, pose, shape);

        // Add a wall on the right side to avoid the object colliding with other cylindrical talbes
        pose.pose.position.x = 0.5;
        pose.pose.position.y = -0.9;
        pose.pose.position.z = 0.5;

        // Define table dimensions
        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = 1;
        shape.dimensions[1] = 0.2;
        shape.dimensions[2] = 1;

        // Add the wall
        addCollisionObject(-3, pose, shape);
    }

    // This function adds the detected object to the planning scene
    void addCollisionObject(int id, geometry_msgs::PoseStamped pose, shape_msgs::SolidPrimitive shape){
        // Prepare request to add collision object service
        assignment_2::PlanScene_AddObj srv;
        
        // Get the id of the object to add
        srv.request.id = id;

        // Create the request for the planning scene service
        srv.request.pose = pose;
        srv.request.shape = shape;

        // Send request
        add_planning_scene_client.call(srv);
    }

    // This function clears the planning scene
    void clearPlanningScene(){
        // Prepare request to clear the collision objects, table but
        assignment_2::PlanScene_ClearScene srv;
        // Send the request, that is empty
        clear_planning_scene_client.call(srv);
    }

    // This function approaches the place pose from the top
    // The height of approach is given by the dimensions of the object and the gripper
    void approPlace(geometry_msgs::PoseStamped goal_pose)
    {
        //reach the place position from the top at a distance equal to 
        //three times fingers length from the table
        double z_offset = 2*gripper_length + cylinder_table_shape.dimensions[0]/2;

        //Update the vertical dimension for the approach
        goal_pose.pose.position.z = z_offset;

        //set the gripper orientation
        goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 1.57, 0);
    
        //move the arm
        moveArm(goal_pose);
    }

    // Function to get closer to the table to place the object
    void getCloser(int id)
    {
        //Pick pose for the arm
        geometry_msgs::PoseStamped pick_pose; 

        //get closer to the placing pose by a distance equal to 2 times the gripper length
        //from the table
        double z_offset = 1*gripper_length + cylinder_table_shape.dimensions[0]/2;
        
        //Get the current end-effector pose
        pick_pose = getCurrPose("gripper_grasping_frame");

        //Update the vertical dimension
        pick_pose.pose.position.z = z_offset;

        //Move the arm
        moveArm(pick_pose);
        ROS_INFO_STREAM("Ready to close the gripper");
    }

    // Function used to move the arm given a goal pose in cartesian space
    void moveArm(geometry_msgs::PoseStamped pose)
    {
        //select group of joints and other settings
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        group_arm_torso.setPlanningTime(45.0);      //maximum time allowed for planning, then return failure
        group_arm_torso.setEndEffectorLink("gripper_grasping_frame");
        group_arm_torso.setPoseReferenceFrame("base_footprint");    //reference frame 
        group_arm_torso.setStartStateToCurrentState();              //current state of the robot set as start state
        group_arm_torso.setMaxVelocityScalingFactor(1.0);           //max joint velocity set (1.0)

        //set goal target
        group_arm_torso.setPoseTarget(pose);
    
        //plan the arm trajectory
        bool success = static_cast<bool>(group_arm_torso.plan(my_plan));

        //if plan not found
        if ( !success )
            throw std::runtime_error("No plan found");
        
        group_arm_torso.move();

        ROS_INFO("Arm goal reached");
    }

    // Function used to move the torso given a goal in joint space
    void moveTorso(double joint_value)
    {
        // Create an action client for the torso controller
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client("torso_controller/follow_joint_trajectory", true);

        // Wait for the action server to come up
        while (!torso_client.waitForServer(ros::Duration(5.0)))
        {
        ROS_INFO("Waiting for the head_controller/follow_joint_trajectory action server to come up");
        }

        //Create a goal message
        control_msgs::FollowJointTrajectoryGoal goal;

        //Set the joint names
        goal.trajectory.joint_names.resize(1);
        goal.trajectory.joint_names[0] = "torso_lift_joint";

        //Set the joint space goal
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions.resize(2);
        goal.trajectory.points[0].positions[0] = joint_value;
        
        goal.trajectory.points[0].time_from_start = ros::Duration(3.0); // Take 3 second to execute the motion

        //Send the goal
        torso_client.sendGoal(goal);

        // Wait for the action to complete
        while (!torso_client.waitForResult(ros::Duration(1.0)))
        {
            ROS_INFO("Waiting for head movement to complete");
        }

        // Check if the action was successful
        if (torso_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The head was moved successfully");
        }
        else
        {
            ROS_ERROR("Failed to move the head");
        }
    }

    // This function opens the gripper
    void openGripper()
    {
        // Create an action client for the head controller
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("/parallel_gripper_controller/follow_joint_trajectory", true);

        // Wait for the action server to come up
        while (!client.waitForServer(ros::Duration(5.0)))
        {
        ROS_INFO("Waiting for server");
        }
        ROS_INFO("Closing gripper");

        // Create a goal message
        control_msgs::FollowJointTrajectoryGoal goal;

        // Set the joint names
        goal.trajectory.joint_names.resize(2);
        goal.trajectory.joint_names[0] = "gripper_left_finger_joint";
        goal.trajectory.joint_names[1] = "gripper_right_finger_joint";


        //Set the joint values
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions.resize(1);
        goal.trajectory.points[0].positions[0] = 0.08;
        goal.trajectory.points[0].positions[1] = 0.08; 
        goal.trajectory.points[0].time_from_start = ros::Duration(1.0); // Take 1 second to execute the motion
        
        // Send the goal
        client.sendGoal(goal);

        // Wait for the action to complete
        while (!client.waitForResult(ros::Duration(1.0)))
        {
        ROS_INFO("Waiting for gripper movement to complete");
        }
        ROS_INFO("Gripper closed");
    }

    // This function returns the current pose of a given reference frame with respet to base_footprint frame
    geometry_msgs::PoseStamped getCurrPose(std::string target_frame)
    {
        // Select group of joints and other settings
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        group_arm_torso.setPlanningTime(45.0);                          // Maximum time allowed for planning, then return failure
        group_arm_torso.setPlannerId("SBLkConfigDefault");
        group_arm_torso.setEndEffectorLink(target_frame);         
        group_arm_torso.setPoseReferenceFrame("base_footprint");        // Reference frame 
        group_arm_torso.setStartStateToCurrentState();                  // Current state of the robot set as start state
        group_arm_torso.setMaxVelocityScalingFactor(1.0);               // Max joint velocity set (1.0)

        //get and return current pose
        geometry_msgs::PoseStamped current_pose = group_arm_torso.getCurrentPose();
        return current_pose;
    }

    // This function detaches the object from the robot body
    void detach(int id)
    {

        // Create a client for the link attacher service
        detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

        // Create a request message for the detach service
        gazebo_ros_link_attacher::AttachRequest req;

        //if blue exagon
        if(id == 1)
        {
            req.model_name_1 = "Hexagon";
            req.link_name_1 = "Hexagon_link";
        }

        //if green triangle
        if(id == 2)
        {
            req.model_name_1 = "Triangle";
            req.link_name_1 = "Triangle_link";
        }

        //if red cube
        if(id == 3)
        {
            req.model_name_1 = "cube";
            req.link_name_1 = "cube_link";
        }
        

        //tiago link
        req.model_name_2 = "tiago";
        req.link_name_2 = "arm_7_link";

        // Send the request to the link attacher service
        gazebo_ros_link_attacher::AttachResponse res;
        if (detach_client.call(req, res)) {
            ROS_INFO("Objects successfully detacched");
        }
        else {
            ROS_ERROR("Failed to detach objects");
        }
    }
    
    // This function sets the arm in a safe pose for navigation
    void setArmToDefaultPosition()
    {
        
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");    //group of joints
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;                   //plan
        geometry_msgs::PoseStamped arm_pose;                                            //arm goal pose in cartesian space

        // Define and send the request
        poses_service.request.request = {"arm_cartesian", "default"};
        get_poses_client.call(poses_service);
        arm_pose = poses_service.response.pose_cartesian;

        group_arm_torso.setPlanningTime(45.0);                      //maximum time allowed for planning, then return failure
        group_arm_torso.setPlannerId("SBLkConfigDefault");          //planner
        group_arm_torso.setPoseReferenceFrame("base_footprint");    //reference frame 
        group_arm_torso.setPoseTarget(arm_pose);                   //set goal pose
        group_arm_torso.setStartStateToCurrentState();              //current state of the robot set as start state
        group_arm_torso.setMaxVelocityScalingFactor(1.0);           //max joint velocity set (1.0)

        
        //plan the arm trajectory
        bool success = static_cast<bool>(group_arm_torso.plan(my_plan));

        //if plan not found
        if ( !success )
            throw std::runtime_error("No plan found");

        //execture the plan
        group_arm_torso.move();

        ROS_INFO("Arm goal reached");
    }

    std::vector<geometry_msgs::PoseStamped> transform(std::string frame_id, std::vector<geometry_msgs::PoseStamped> poses){
        assignment_2::Transform srv;
        srv.request.target_frame = frame_id;
        srv.request.input = poses;
        transform_client.call(srv);
        return srv.response.output;
    }

    ros::NodeHandle nh;

    // Action Server Variables
    actionlib::SimpleActionServer<assignment_2::PlaceAction> action_server;
    assignment_2::PlaceFeedback action_feedback; //message to publish feedback
    assignment_2::PlaceResult action_result;     //message to publish result

    // Services
    ros::ServiceClient detection_client;
    ros::ServiceClient add_planning_scene_client;
    ros::ServiceClient clear_planning_scene_client;
    ros::ServiceClient transform_client;
    ros::ServiceClient detach_client;
    ros::ServiceClient get_poses_client;        //get_poses client
    assignment_2::Poses poses_service;          //poses service msg

    // Class variables
    std::string server_name;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    double gripper_length = 0.11;                                   //gripper fingers dimension
    shape_msgs::SolidPrimitive cylinder_table_shape;                //cylinder shape dimensions
};

int main(int argc, char **argv){
    // Create server node
    ros::init(argc, argv, "place_routine_server");
    
    PlaceRoutineAction place("place_routine_server");

    ros::spin();
}