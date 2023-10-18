#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>  //library to implement actions
#include <actionlib/client/simple_action_client.h>  //library to implement actions
#include <assignment_1/TiagoCorridorAction.h>       //this inclueds the action msg generated from the .action file
#include <sensor_msgs/LaserScan.h>                  //message sent by topic scan, used in service request
#include <geometry_msgs/Twist.h>                    //twist for cmd_vel message

class TiagoCorridorAction{

protected:  //protected variables of the class action
    ros::NodeHandle nh_;                                                        //node handle
    actionlib::SimpleActionServer<assignment_1::TiagoCorridorAction> as_;       //action server
    std::string action_name_;                                                   //action name
    assignment_1::TiagoCorridorFeedback feedback; //message to publish feedback
    assignment_1::TiagoCorridorResult result;     //message to publish result
    bool termination;                             //flag if the robot is in a narrow space
    float rotation = 0;                           //angular rotation around z axis
    public:

    TiagoCorridorAction(std::string name) : 
        as_(nh_, name, boost::bind(&TiagoCorridorAction::executeCB, this, _1), false), action_name_(name)
    {
        ROS_INFO("Ready to nagivate the robot through the corridor...");
        as_.start();
        
    }

    ~TiagoCorridorAction(void)
    {

    }

    //when the goal is received the following is performed
    void executeCB(const assignment_1::TiagoCorridorGoalConstPtr &goal)
    {
        termination = true;

        //velocity message
        geometry_msgs::Twist vel_msg;

        //subscrber to laser scan topic
        ros::Subscriber laser_sub = nh_.subscribe("scan", 1000, &TiagoCorridorAction::laserScanCallback, this);
        
        
        //velocity publisher
        ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 1000);
        
        //while the robot is in a narrow space
        while(termination)
        {
            vel_msg.angular.z = rotation; //angular speed
            vel_msg.linear.x = 0.3; //linear speed
            vel_pub.publish(vel_msg); //publish velocity command
        }

        //robot outside the narrow space
        //set corridor navigation as completed
        result.completed = true;
        
        laser_sub.shutdown();

        ROS_INFO("Navigation through the corridor completed!");
        as_.setSucceeded(result);

    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {

        //compute rotation value
        //ranges in index 606 corresponds to around 80 degress on the left side
        //ranges in index 60 corresponds to around -80 degress on the right side
        rotation = scan->ranges[606] - scan->ranges[60]; 

        //check if robot is still in the corridor
        termination = checkCorridor(scan->ranges);

        return;
    }



    //functiton to check if the robot is in a narrow space (corridor)
    //for both sides, consider a set of angles (left side from 70 to 80, right side from -70 to -80)
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
    ros::init(argc, argv, "move_corridor"); 
    TiagoCorridorAction TiagoCorridor("move_corridor");

    ros::spin();
    return 0;
}