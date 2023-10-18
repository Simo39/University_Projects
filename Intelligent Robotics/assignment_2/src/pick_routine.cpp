/*
This node implements an action server that handles the pick routine. The server receives as
goal the id of the object to pick and performs all the steps up to having the arm on a
safe position for navigation
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
#include <tf2/LinearMath/Quaternion.h>

// ROS Messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// User-defined Library
// Assignment_2
#include <assignment_2/DetectAndTransform.h>
#include <assignment_2/PickAction.h>
#include <assignment_2/PlanScene_AddObj.h>
#include <assignment_2/PlanScene_ClearScene.h>
#include <assignment_2/PlanScene_RemoveObj.h>
#include <assignment_2/Poses.h>
#include <assignment_2/Transform.h>


class PickRoutineAction{
    public:
    
    PickRoutineAction(std::string name):
    action_server(nh, name, boost::bind(&PickRoutineAction::executePick, this, _1), false), server_name(name){
        // Initialize client services
        detection_client = nh.serviceClient<assignment_2::DetectAndTransform>("/detect_and_transform");
        add_planning_scene_client = nh.serviceClient<assignment_2::PlanScene_AddObj>("/add_collision_obj");
        clear_planning_scene_client = nh.serviceClient<assignment_2::PlanScene_ClearScene>("/clear_scene");
        remove_planning_scene_client = nh.serviceClient<assignment_2::PlanScene_RemoveObj>("/remove_collision_obj");
        transform_client = nh.serviceClient<assignment_2::Transform>("/transform_poses");
        get_poses_client = nh.serviceClient<assignment_2::Poses>("get_poses_service");

        action_server.start();
        ROS_INFO("Picking Routine Action Server READY");
    }

    protected:

    void executePick(const assignment_2::PickGoalConstPtr &goal){

        //add the coffe table to the planning scene
        ROS_INFO("Creating table in planning scene");
        addCoffeTableToPlanningScene();

        //detect the objects on the table
        ROS_INFO("Detect objects on the table");
        detectObjects();

        geometry_msgs::PoseStamped goal_pose;
        shape_msgs::SolidPrimitive goal_shape;
        
        //create the planning scene
        ROS_INFO("Creation of Planning Scene");

        //for each detected object on the table
        for(int i = 0; i < ids.size(); i++){
            int id = ids[i];
            //get its position and shape
            geometry_msgs::PoseStamped pose = apriltag_poses[i];
            shape_msgs::SolidPrimitive shape = shapes[i];

            //and add it to the planning scene
            addCollisionObject(id, pose, shape);
            ROS_INFO("Added object id[%d] to planning scene", id);

            //if the object is the one to pick
            if(id == goal->id)
            {
                //set goal pose and shape
                goal_pose = pose;
                goal_shape = shape;
            }
        }

        //now follow a set of steps to complete the pick routine
        setFeedback("Planning scene created");
        setFeedback("Approaching object");
        approachObject(goal_pose, goal->id);            //approach the object
        getCloser(goal_pose, goal_shape, goal->id);     //get closer to the object
        setFeedback("Object approached");
        removeCollisionObject(goal->id);                //remove it from the planning scene
        closeGriper();                                  //close the gripper
        attach(goal->id);                               //attach the object to the robot
        setFeedback("Object picked");
        moveTorso(0.35);                                //move torso up to depart
        setSafeMovingPose();                            //set the arm to a safe pose for navigation
        setFeedback("Arm in a safe pose for navigation");
        clearPlanningScene();                           //reset the planning scene
        setFeedback("Planning scene cleared");

        //set the pick as succeded and send the result
        action_result.completed = true;
        action_server.setSucceeded(action_result);
    }

    private:

    // This function adds the detected object to the planning scene
    void addCollisionObject(int id, geometry_msgs::PoseStamped pose, shape_msgs::SolidPrimitive shape){
        // Prepare request to add collision object service
        assignment_2::PlanScene_AddObj srv;
        
        // Get the id of the object to add
        srv.request.id = id;

        // Set the height of the center of the object in between the table and the apriltag
        // If the object is approximated with a cube, height is the third dimension
        if(id == 3 || id == 2)
        {
            pose.pose.position.z += -shape.dimensions[2]/2;
        }

        // If the object is approximated with a cylinder, heights is the first dimension
        if(id == 1 || id == 4 || id == 5 || id == 6 || id == 7)
        {
           pose.pose.position.z += -shape.dimensions[0]/2; 
        }

        // Create the request for the planning scene service
        srv.request.pose = pose;
        srv.request.shape = shape;

        // Send request
        add_planning_scene_client.call(srv);
    }

    // This function adds the coffe table to the planning scene. It corresponds to a 
    // parallelepiped with the same height
    void addCoffeTableToPlanningScene(){
        // Define table pose
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base_footprint";
        pose.pose.position.x = 0.95;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        // Define table dimensions
        shape_msgs::SolidPrimitive shape;
        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = 1;
        shape.dimensions[1] = 2;
        shape.dimensions[2] = 1.55;

        // Add the table to the planning scene
        addCollisionObject(0, pose, shape); 

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        // Define table dimensions
        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = 2;
        shape.dimensions[1] = 2;
        shape.dimensions[2] = 0.3;

        // Add a platform on the base to avoid the picket object to touch the floor
        addCollisionObject(-1, pose, shape);
        
        pose.pose.position.x = 0;
        pose.pose.position.y = 0.3;
        pose.pose.position.z = 1;

        // Define table dimensions
        shape.type = shape.BOX;
        shape.dimensions.resize(3);
        shape.dimensions[0] = 0.2;
        shape.dimensions[1] = 0.1;
        shape.dimensions[2] = 2;

        // Add a barrier on the left side to avoid the object to collide with the robot
        addCollisionObject(-2, pose, shape);  
        
    }

    // This function send the request to the detection service to get the detected objects
    void detectObjects(){

        // Prepare request to detection service
        assignment_2::DetectAndTransform srv;
        srv.request.target_frame = "base_footprint";

        // Send Request
        detection_client.call(srv);

        // Collect reponse and save it
        ids = srv.response.ids;
        apriltag_poses = srv.response.apriltag_poses;
        shapes = srv.response.shapes;
        setFeedback("Objects detected");
    }

    // This function removes a collision object from the planning scene (given its ID)
    void removeCollisionObject(int id){
        // Prepare request to remove collision object service
        assignment_2::PlanScene_RemoveObj srv;
        srv.request.id = id;
        // Send request
        remove_planning_scene_client.call(srv);
    }

    // This function sends the request to the planning scene service to fully clear it
    void clearPlanningScene(){

        // Prepare request to clear the collision objects, table but
        assignment_2::PlanScene_ClearScene srv;

        // Send the request, that is empty
        clear_planning_scene_client.call(srv);
    }
    
    // This function completes the approach from the top to the object to pick
    // The height for the approach is a function of the size of the gripper
    void approachObject(geometry_msgs::PoseStamped appro_pose, int id)
    {
        //reach the object from the top at a distance equal to two times fingers length
        appro_pose.pose.position.z += 2*gripper_length;
        
        //Get the current roll-pitch-yaw angles of the end-effector
        tf::Quaternion quat;
        tf::quaternionMsgToTF(appro_pose.pose.orientation, quat);
        double current_roll, current_pitch, current_yaw;
        tf::Matrix3x3(quat).getRPY(current_roll, current_pitch, current_yaw);

        //Update the orientation of the gripper
        if(id == 2)
        {
            appro_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(current_roll-0.8, current_pitch+1.57, current_yaw);
        }
        else
        {
            appro_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(current_roll, current_pitch+1.5, current_yaw);
        }

        //Move the arm
        moveArm(appro_pose);
        ROS_INFO_STREAM("Approach completed");
    }

    // This function moves the arm to be sufficiently close to the gripper to pick it
    void getCloser(geometry_msgs::PoseStamped appro_pose, shape_msgs::SolidPrimitive shape, int id)
    {
        //Pick pose for the arm
        geometry_msgs::PoseStamped pick_pose; 
           
        //z coordinate offset, how much to go down with the gripper
        double z_offset;

        //Get the current end-effector pose
        pick_pose = getCurrPose("gripper_grasping_frame");

        //compute the difference between the z position of current gripper pose VS apriltag
        double z_diff = pick_pose.pose.position.z - appro_pose.pose.position.z;

        //compute the space in between the object and the fingers
        z_diff = z_diff - gripper_length;

        //compute how much the arm need to get down based on shape dimension
        if(id == 1)
        {
            //half of the height of the (approximated) blu cilinder
            z_offset = z_diff + shape.dimensions[0]/2;
        }
        else
        {
            //half of the height of red and (approximated) green boxes
            z_offset = z_diff + shape.dimensions[2]/2;
        }

        //Update the vertical dimension
        pick_pose.pose.position.z += -z_offset;

        //Move the arm
        moveArm(pick_pose);
        ROS_INFO_STREAM("Ready to close the gripper");
    }
    
    // This function simply closes the gripper
    void closeGriper()
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
        goal.trajectory.points[0].positions[0] = 0;
        goal.trajectory.points[0].positions[1] = 0; 
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

    // Given a goal pose, this function moves the arm
    void moveArm(geometry_msgs::PoseStamped pose)
    {
        // Select group of joints and other settings
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        group_arm_torso.setPlanningTime(45.0);                          // Maximum time allowed for planning, then return failure
        group_arm_torso.setPlannerId("SBLkConfigDefault");
        group_arm_torso.setEndEffectorLink("gripper_grasping_frame");         
        group_arm_torso.setPoseReferenceFrame("base_footprint");        // Reference frame 
        group_arm_torso.setStartStateToCurrentState();                  // Current state of the robot set as start state
        group_arm_torso.setMaxVelocityScalingFactor(1.0);               // Max joint velocity set (1.0)

        //set goal target
        group_arm_torso.setPoseTarget(pose);
    
        //plan the arm trajectory
        bool success = static_cast<bool>(group_arm_torso.plan(my_plan));

        //if plan not found
        if ( !success )
            throw std::runtime_error("No plan found");
        
        //move the arm
        group_arm_torso.move();
    }

    // Given the joint space value, this function moves the torso
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

    // This function attaches the object to the robot
    void attach(int id)
    {

        // Create a client for the link attacher service
        attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

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
        if (attach_client.call(req, res)) {
            ROS_INFO("Objects successfully attacched");
        }
        else {
            ROS_ERROR("Failed to attach objects");
        }
    }

    // This function sets the pose of the arm in a safe position for navigation
    void setSafeMovingPose()
    {
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");    //group of joints
        geometry_msgs::PoseStamped moving_pose;                                         //arm goal pose in cartesian space

        // Define and send the request to get the pose
        poses_service.request.request = {"arm_cartesian", "moving_object"};
        get_poses_client.call(poses_service);
        moving_pose = poses_service.response.pose_cartesian;
        
        //move arm
        moveArm(moving_pose);
        ROS_INFO("Arm is now in safe position for navigation");
    }
    
    // This function is used to send the feedback to the client
    void setFeedback(std::string feedback){
        action_feedback.status = feedback;
        action_server.publishFeedback(action_feedback);
    }
    
    ros::NodeHandle nh;

    // Action Server Variables
    actionlib::SimpleActionServer<assignment_2::PickAction> action_server;
    assignment_2::PickFeedback action_feedback; //message to publish feedback
    assignment_2::PickResult action_result;     //message to publish result

    // Services
    ros::ServiceClient detection_client;
    ros::ServiceClient add_planning_scene_client;
    ros::ServiceClient clear_planning_scene_client;
    ros::ServiceClient remove_planning_scene_client;
    ros::ServiceClient transform_client;
    ros::ServiceClient attach_client;
    ros::ServiceClient get_poses_client;                //get_poses client
    assignment_2::Poses poses_service;                  //poses service msg

    // Class variables
    std::vector<int> ids;
    std::vector<geometry_msgs::PoseStamped> apriltag_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes;    
    std::string server_name;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    double gripper_length = 0.11;                                   //gripper fingers dimension

};

int main(int argc, char **argv){

    // Create server node
    ros::init(argc, argv, "picking_routine_server");
    
    PickRoutineAction pick("picking_routine_server");

    ros::spin();
}