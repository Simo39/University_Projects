#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>  //library to implement the action clients
#include <assignment_1/TiagoNavigationAction.h>     //same for the server, generates the message
#include <assignment_1/Circle.h>                    //message for circle, used in service response                 

using namespace assignment_1;

//this typedef might be useless
typedef actionlib::SimpleActionClient<TiagoNavigationAction> Client;

//called when action done
void doneCb(const actionlib::SimpleClientGoalState& state,
            const TiagoNavigationResultConstPtr& result)
{
    ROS_INFO("Goal %s", state.toString().c_str());

    //printing the final result

    std::vector<assignment_1::Circle> circles;
        
    ROS_INFO("Result received");
    circles = result->circles;
    ROS_INFO("N. of detected cylinders: %d", int(circles.size()));
    for(int i = 0; i < circles.size(); i++){
        assignment_1::Circle circle = circles[i];
        float x = circle.center_x;
        float y = circle.center_y;
        float radius = circle.radius;
        ROS_INFO("Cylinder #%d: radius = %9.5f       center coordinates = ( %10.6f, %10.6f)", i+1, radius, x, y);
    }

    ros::shutdown();
}


// Called when the goal becomes active
void activeCb()
{
    ROS_INFO("Robot is now ready to start");
}

//called when feedback is sent
void feedbackCb(const TiagoNavigationFeedbackConstPtr& feedback)
{
    ROS_INFO("%s", feedback->robot_status.c_str());
}


int main (int argc, char **argv)
{    
    ros::init(argc, argv, "navigation_client");

    if(argc != 8){
        ROS_INFO("USAGE: rosrun assignment_1 client <position.x> <position.y> <position.z> <orientation.x> <orientation.y> <orientation.z>  <orientation.w>");
        return 1;
    }
    
    actionlib::SimpleActionClient<assignment_1::TiagoNavigationAction> ac("navigation_server", true);

    ROS_INFO("Waiting for the robot to start navigation...");
    ac.waitForServer();
    
    assignment_1::TiagoNavigationGoal goal;  //here creates the goal

    //poseB goal
    goal.pose.position.x = std::atof(argv[1]);
    goal.pose.position.y = std::atof(argv[2]);
    goal.pose.position.z = std::atof(argv[3]);
    goal.pose.orientation.x = std::atof(argv[4]);
    goal.pose.orientation.y = std::atof(argv[5]);
    goal.pose.orientation.z = std::atof(argv[6]);
    goal.pose.orientation.w = std::atof(argv[7]);
    
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //and sends the goal

    ros::spin();
    return 0;
} 
