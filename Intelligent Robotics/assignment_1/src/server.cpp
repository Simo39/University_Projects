#include <math.h>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>                         //library to implement transformations
#include <tf2_ros/transform_listener.h>             //library to implement transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>    //library to implement transformations

#include <actionlib/server/simple_action_server.h>  //library to implement actions
#include <actionlib/client/simple_action_client.h>  //library to implement actions
#include <sensor_msgs/LaserScan.h>                  //message sent by topic scan, used in service request
#include <move_base_msgs/MoveBaseAction.h>          //message sends to Tiago as goal

#include <assignment_1/TiagoNavigationAction.h>     //action to navigate to poseB
#include <assignment_1/TiagoCorridorAction.h>       //action to navigate through the corridor
#include <assignment_1/DetectObstacles.h>           //service for obstacle detection
#include <assignment_1/Circle.h>                    //message for circle, used in service response


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<assignment_1::TiagoCorridorAction> CorridorClient;

class TiagoNavigationAction{

protected:  //protected variables of the class action
    ros::NodeHandle nh_;                                                //node handle
    actionlib::SimpleActionServer<assignment_1::TiagoNavigationAction> as_;      //action server
    std::string action_name_;                                           //action name
    assignment_1::TiagoNavigationFeedback feedback; //message to publish feedback
    assignment_1::TiagoNavigationResult result;     //message to publish result

    public:

    TiagoNavigationAction(std::string name) : 
        as_(nh_, name, boost::bind(&TiagoNavigationAction::executeCB, this, _1), false), action_name_(name)
    {   
        as_.start();
        ROS_INFO("Ready to start robot navigation");
    }

    ~TiagoNavigationAction(void)
    {

    }

    //callback function when the goal is received
    void executeCB(const assignment_1::TiagoNavigationGoalConstPtr &goal)
    {
        //-----------------------
        //INITIALIZATION
        //----------------------
        MoveBaseClient ac_move_base("move_base", true);     //move_base action client 
        CorridorClient ac_corridor("move_corridor", true);  //corridor action client 
        move_base_msgs::MoveBaseGoal poseB;                 //move_base action goal
        assignment_1::TiagoCorridorGoal CorridorGoal;       //corridor action goal
        sensor_msgs::LaserScan::ConstPtr scan;              //laser scan
        float narrow_distance = 0.7;                        //parameter to check narrow spaces
        float laser_angle_min = 0;                          //angle min for laser scan
        float laser_angle_max = 0;                          //angle max for laser scan
        float laser_angle_increment = 0;                    //angle increment for laser scan
        int right_index = 30;                               //ranges vector index for right side wall
        int left_index = 636;                               //ranges vector index for left side wall
        float range_angle = 70;                             //left-right angle direction to detect walls


        //====================
        //NAVIGATION TO poseB
        //====================

        //define goal poseB
        poseB.target_pose.header.frame_id = "map";
        poseB.target_pose.header.stamp = ros::Time::now();
        
        //set poseB position and orientation 
        poseB.target_pose.pose.position.x = goal->pose.position.x;
        poseB.target_pose.pose.position.y = goal->pose.position.y;
        poseB.target_pose.pose.position.z = goal->pose.position.z;
        poseB.target_pose.pose.orientation.x = goal->pose.orientation.x;
        poseB.target_pose.pose.orientation.y = goal->pose.orientation.y;
        poseB.target_pose.pose.orientation.z = goal->pose.orientation.z;
        poseB.target_pose.pose.orientation.w = goal->pose.orientation.w;
        
        ROS_INFO("PoseB goal received");

        //update feedback
        feedback.robot_status = "Robot ready to start navigation";
        as_.publishFeedback(feedback);
 
        //wait for the move_base action server to come up
        while(!ac_move_base.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server...");
        }
        
        //send move_base server action goal
        ac_move_base.sendGoal(poseB);
        ROS_INFO("Robot started moving to poseB");

        //update feedback
        feedback.robot_status = "Robot started navigation to poseB";
        as_.publishFeedback(feedback);

        //while robot navigating to poseB check if it enters in a narrows space
        //this control is done with frequency 1Hz
        while(!ac_move_base.waitForResult(ros::Duration(1.0))){

            //get laser scan
            scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");
            
            //get laser scan parameters
            laser_angle_increment = scan->angle_increment;
            laser_angle_min = scan->angle_min;
            laser_angle_max = scan->angle_max;

            //check if the robot is in a narrow space
            if(checkCorridor(scan->ranges))
            {

                //====================
                //CORRIDOR ROUTINE
                //====================

                ROS_INFO("Robot in a narrow space");

                //update feedback
                feedback.robot_status = "Robot in a narrow space";
                as_.publishFeedback(feedback);

                //cancel move_base goal 
                ac_move_base.cancelAllGoals();
                
                //wait for the corridor action server to come up
                while(!ac_corridor.waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the corridor action server...");
                }

                //sending corridor server goal to activate the routine for the
                //navigation in narrow spaces
                ac_corridor.sendGoal(CorridorGoal);

                ROS_INFO("Robot is navigating through the corridor");
                //update feedback
                feedback.robot_status = "Robot is navigating through the corridor";
                as_.publishFeedback(feedback);

                //wait for the robot to reach the end of the corridor
                ac_corridor.waitForResult();
                ROS_INFO("Robot arrived at the end of the corridor");
                //update feedback
                feedback.robot_status = "Robot arrived at the end of the corridor";
                as_.publishFeedback(feedback);

                //send again poseB goal to resume poseB navigation
                ac_move_base.sendGoal(poseB);
                ROS_INFO("Robot resumed navigation to poseB");
                //update feedback
                feedback.robot_status = "Robot resumed navigation to poseB";
                as_.publishFeedback(feedback);
            }

            ROS_INFO("Robot navigating to poseB");
            //update feedback
            feedback.robot_status = "Robot navigating to poseB";
            as_.publishFeedback(feedback);
        }

        
        ROS_INFO("Robot reached poseB");
        //update feedback
        feedback.robot_status = "Robot reached to poseB";
        as_.publishFeedback(feedback);
        


        //===============================
        //SEND GOAL TO OBASTACLES SERVER
        //===============================
        
        ROS_INFO("Waiting for scan");

        //update feedback
        feedback.robot_status = "Robot started cylinder detection";
        as_.publishFeedback(feedback);

        sensor_msgs::LaserScan laser_data;
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr;

        //waiting for a message to be published in topic /scan
        sharedPtr  = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh_);

        //saving the message as a non-pointer
        laser_data = *sharedPtr;

        ROS_INFO("Message received");

        //registering to the obstacle detection service
        ros::ServiceClient service_client = nh_.serviceClient<assignment_1::DetectObstacles>("/detect_obstacles");

        assignment_1::DetectObstacles srv_msg;          //service message
        //setting service request
        srv_msg.request.laser_scan = laser_data;

        //vector that will store detection results
        std::vector<assignment_1::Circle> circles;
        
        ROS_INFO("Sending service request: Detection Obstacles");
        ROS_INFO("Waiting for response...");
        //sending the service request
        if(service_client.call(srv_msg)){
            //if service request succeeded
            ROS_INFO("Service response received: Obstacles detected");

            //store response (detection result) into a std::vector
            circles = srv_msg.response.circles;
        }
        else{
            //if service request not succeeded
            //abort the action
            ROS_ERROR("Failed to send obstacles detection request");
            //sent an empty vector as results
            result.circles = circles;
            as_.setAborted(result, "Unable to reach the obstacles detection service");
            
            feedback.robot_status = "Robot failed obstacles detection";
            as_.publishFeedback(feedback);

            return;
        }
        printf("\n");

        //update feedback
        feedback.robot_status = "Robot completed obstacles detection";
        as_.publishFeedback(feedback);
        
        //==============================================
        //TRANSFORM POINTS base_laser_link -> map
        //==============================================

        feedback.robot_status = "Robot is changing obstacle coordinates from base_laser_link to base_link";
        as_.publishFeedback(feedback);

        //setting the trasformer
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener tfListener(buffer);
        geometry_msgs::TransformStamped transform;

        //source and target frame
        std::string source_frame = laser_data.header.frame_id;

        std::string target_frame = "map";

        ROS_INFO("Waiting for transform...");
        
        //wait for transform
        while(!buffer.canTransform(target_frame, source_frame, ros::Time(0))){
            transform = buffer.lookupTransform(target_frame, source_frame, ros::Time(), ros::Duration(1.0));
        }

        ROS_INFO("Transforming points from %s frame to %s frame", source_frame.c_str(), target_frame.c_str());

        //transform each circle center
        for(int i = 0; i < circles.size(); i++){
            geometry_msgs::Point input, output;
            input.x = circles[i].center_x;
            input.y = circles[i].center_y;
            input.z = 0;
            tf2::doTransform(input, output, transform);
            circles[i].center_x = output.x;
            circles[i].center_y = output.y;
        }

        //=========================
        //SENDIND RESULT TO CLIENT
        //=========================
        
        ROS_INFO("Sending result");

        feedback.robot_status = "Robot is sending results";
        as_.publishFeedback(feedback);

        //set result as succeeded and send it
        result.circles = circles;
        as_.setSucceeded(result);

        ROS_INFO("Result sent");
    }

    //funciton to check if the robot is in a narrow space (corridor)
    bool checkCorridor(std::vector<float> ranges)
    {

        bool corridor = true;           //true -> inside corridor, false -> not inside corridor
        float narrow_distance = 0.7;    //inside corridor if robot has walls on bot sides closer than narrow_distance

        //check right side
        for(int i = 30; i < 60; i++)
        {
            //check if there is not anymore close wall on the right side of the robot
            if(ranges[i] > narrow_distance)
            {
                corridor = false;
                break;
            }
        }

        //if still a wall on the right side, check the left side
        if(corridor)
        {
            for(int i = 606; i < 636; i++)
            {
                //check if there is not anymore close wall on the left side of the robot
                if(ranges[i] > narrow_distance)
                {
                    corridor = false;
                    break;
                }
            }      
        }
        return corridor;
    }
};


int main(int argc, char** argv)
{
    //main function creates the action and spins the node. It will be running waiting for goals
    ros::init(argc, argv, "navigation_server"); 
    TiagoNavigationAction TiagoNavigation("navigation_server");
    ros::spin();
    return 0;
}
