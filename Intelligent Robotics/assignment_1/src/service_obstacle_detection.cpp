#include <math.h>
#include <limits>
#include <tuple>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>                          //message sent by topic scan, used in service request

#include <assignment_1/DetectObstacles.h>                   //service message
#include <assignment_1/Circle.h>                            //message for circle, used in service response

#include <assignment_1/utility_obstacle_detection_srv.h>    //utility personal library for obstacle detection

bool detectObstacles(assignment_1::DetectObstacles::Request &req, assignment_1::DetectObstacles::Response &res){

    printf("\n");
    ROS_INFO("REQUEST RECEIVED");

    //saving request as a LaserScan message
    sensor_msgs::LaserScan laser_data;
    laser_data = req.laser_scan;

    //exploiting LaserScan message
    std::vector<float> ranges = laser_data.ranges;
    float angle_min = laser_data.angle_min;
    float d_theta = laser_data.angle_increment;

    //trasforming polar coordinate points (LaserScan data) into cartesian coordinate points
    std::vector<Point> points = getPoints(ranges, angle_min, d_theta);

    //grouping points that are close to each other
    std::vector<std::vector<Point>> clusters = clustering(points);

    //delete clusters with less than 3 points
    //we are interest into circles
    for(int i=0; i < clusters.size(); i++){
        if(clusters[i].size() < 3){
            clusters.erase(clusters.begin() + i);
            i--;
        }
    }
    
    std::vector<Circle> circles;
    std::vector<float> circle_scores;

    //for each cluster compute circle_score and save the result
    for(int i = 0; i < clusters.size(); i++){
        std::vector<Point> cluster = clusters[i];
        Point a = cluster[0];                           //first cluster point
        Point b = cluster[floor(cluster.size()/2)];     //middle cluster point
        Point c = cluster[cluster.size() - 1];          //last cluster point
        Circle circle;
        circle.setCircle(a, b, c);                      //compute circle
        circles.push_back(circle);                      //save circle
        float score = circle_score(cluster, circle);    //compute circle_score
        circle_scores.push_back(score);                 //save score
    }

    std::vector<float> line_scores;

    //for each cluster compute line_score and save the result
    for(int i = 0; i < clusters.size(); i++){
        std::vector<Point> cluster = clusters[i];
        Point start = cluster[0];                       //first cluster point
        Point end = cluster[cluster.size() - 1];        //last cluster point
        float score = line_score(cluster, start, end);  //compute line_score
        line_scores.push_back(score);                   //save score
    }

    //====================
    // OUTPUT ONLY - START
    //====================
    
    printf("\n");
    ROS_INFO("ALL THE CLUSTERS");

    //display for each cluster some infos and its scores
    for(int i = 0; i < clusters.size(); i++){
        int size = clusters[i].size();
        float radius = circles[i].radius;
        float l_score = line_scores[i];
        float c_score = circle_scores[i];
        ROS_INFO("CLUSTER #%2d - size: %3d | radius: %9.5f | line_score: %9.3f | circle_score: %9.3f",
            i, size, radius, l_score, c_score);
    }

    printf("\n");
    ROS_INFO("LINE FILTER: keep only line_score < 20");
    
    //LINE FILTER

    //a cluster is a line if it has a line_score above line_threshold
    float line_threshold = 30;

    //filtering clusters that are considered line: line_score < line_threshold
    for(int i = 0; i < clusters.size(); i++){
        int size = clusters[i].size();
        float radius = circles[i].radius;
        float l_score = line_scores[i];
        float c_score = circle_scores[i];
        if(line_scores[i] < line_threshold){
            ROS_INFO("CLUSTER #%2d - size: %3d | radius: %9.5f | line_score: %9.3f | circle_score: %9.3f",
                i, size, radius, l_score, c_score);
        }
    }

    printf("\n");
    ROS_INFO("NOT CIRCLE FILTER: keep only circle_score > 100");

    //CIRCLE FILTER

    //a cluster is a circle if it has a circle_score above circle_threshold
    float circle_threshold = 100;

    //keeping clusters that are considered circles: circle_score < circle_threshold
    for(int i = 0; i < clusters.size(); i++){
        int size = clusters[i].size();
        float radius = circles[i].radius;
        float l_score = line_scores[i];
        float c_score = circle_scores[i];
        if(circle_scores[i] > circle_threshold && line_scores[i] < line_threshold){
            ROS_INFO("CLUSTER #%2d - size: %3d | radius: %9.5f | line_score: %9.3f | circle_score: %9.3f",
                i, size, radius, l_score, c_score);
        }
    }

    printf("\n");
    
    //==================
    // OUTPUT ONLY - END
    //==================
    
    ROS_INFO("SENDING RESPONSE");

    //vector of messages Circle
    std::vector<assignment_1::Circle> result;

    //for each cluster, apply LINE and CIRCLE filter to obtain the final result
    for(int i = 0; i < clusters.size(); i++){
        if(circle_scores[i] > circle_threshold && line_scores[i] < line_threshold){
            assignment_1::Circle circle;                //message for circle
            circle.center_x = circles[i].center.x;
            circle.center_y = circles[i].center.y;
            circle.radius = circles[i].radius;
            result.push_back(circle);                   //saving final result     
        }
    }

    //set the response with the final result
    res.circles = result;
    printf("\n");
    ROS_INFO("RESPONSE SENT");
    return true;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "service_obstacle_detection");
    ros::NodeHandle n;

    //registering the service
    ros::ServiceServer service = n.advertiseService("/detect_obstacles", detectObstacles);
    
    ROS_INFO("SERVICE READY");

    ros::spin();

    return 0;
}