/*
This is the main node of the package. It sends the request to the human node to get
the order to pick and place the objects, then it sends the request to the pick and
place action server for each object
*/


// Standard Library
#include <vector>

// ROS Library
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> 

// assignment_2 Library
#include <assignment_2/PickAndPlaceAction.h>
#include <tiago_iaslab_simulation/Objs.h>

//called when action done
void doneCb(const actionlib::SimpleClientGoalState& state,
            const assignment_2::PickAndPlaceResultConstPtr& result)
{
    ROS_INFO("Goal %s", state.toString().c_str());
    ROS_INFO("Object picked and placed");
}

// Called when the goal becomes active
void activeCb()
{
    ROS_INFO("Robot is now ready to start to pick and place the object");
}

//called when feedback is sent
void feedbackCb(const assignment_2::PickAndPlaceFeedbackConstPtr& feedback)
{
    ROS_INFO("%s", feedback->robot_status.c_str());
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "human_client");

    ros::NodeHandle nh;

    //human client
    ros::ServiceClient human_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");

    //send the request to the human service
    tiago_iaslab_simulation::Objs human_service;
    human_service.request.ready = true;
    human_service.request.all_objs = true;

    //if response not received
    if(!human_client.call(human_service)){
        std::cout << "[ERROR] Not received the list from human_objects_srv" << std::endl;
        return 1;
    }

    //get the order of the object for the pick and place routine
    std::vector<int> ids = human_service.response.ids;
    actionlib::SimpleActionClient<assignment_2::PickAndPlaceAction> action_client("pick_and_place_server", true);

    //wait for the pick and place server
    ROS_INFO("Waiting for the Pick and Place Server");
    action_client.waitForServer();
    ROS_INFO("Pick and Place Server READY");

    //pick and place server goal
    assignment_2::PickAndPlaceGoal goal;

    //for all the object in the ordered list, send the goal to pick and place server
    for(int id : ids){
        ROS_INFO("Sending goal for object: %d", id);
        goal.id = id;
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        action_client.waitForResult();
    }

    //task completed
    ROS_INFO("Pick and Place of given objects DONE");

    return 0;
}