#include <math.h>
#include <limits>
#include <tuple>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>                          //message sent by topic scan, used in service request

#include <assignment_1/DetectObstacles.h>                   //service message
#include <assignment_1/Circle.h>                            //message for circle, used in service response

#include <assignment_1/utility_obstacle_detection_srv.h>    //utility personal library for obstacle detection

//set the variable of struct Circle 
void Circle::setCircle(Point p1, Point p2, Point p3){

        //REFERENCE:  https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/

        // circle equation in form: x^2 + y^2 + 2*g*x + 2*f*y + c = 0
        // where centre is (h = -g, k = -f) and radius r
        // as r^2 = h^2 + k^2 - c

        float x1 = p1.x, y1 = p1.y;
        float x2 = p2.x, y2 = p2.y;
        float x3 = p3.x, y3 = p3.y;

        float x12 = x1 - x2;
        float x13 = x1 - x3;

        float y12 = y1 - y2;
        float y13 = y1 - y3;

        float y31 = y3 - y1;
        float y21 = y2 - y1;

        float x31 = x3 - x1;
        float x21 = x2 - x1;

        // x1^2 - x3^2
        float sx13 = pow(x1, 2) - pow(x3, 2);

        // y1^2 - y3^2
        float sy13 = pow(y1, 2) - pow(y3, 2);

        float sx21 = pow(x2, 2) - pow(x1, 2);
        float sy21 = pow(y2, 2) - pow(y1, 2);

        float f = ((sx13) * (x12)
                + (sy13) * (x12)
                + (sx21) * (x13)
                + (sy21) * (x13))
                / (2 * ((y31) * (x12) - (y21) * (x13)));
        float g = ((sx13) * (y12)
                + (sy13) * (y12)
                + (sx21) * (y13)
                + (sy21) * (y13))
                / (2 * ((x31) * (y12) - (x21) * (y13)));

        float c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1;

        // equation of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
        // where centre is (h = -g, k = -f) and radius r
        // as r^2 = h^2 + k^2 - c
        float h = -g;
        float k = -f;
        float sqr_of_r = h * h + k * k - c;

        // r is the radius
        float r = sqrt(sqr_of_r);

        //construction
        this->f = f;
        this->g = g;
        this->c = c;
        this->radius = r;
        this->center = Point{-g, -f};
    
    };

//compute the distance among two points
float points_distance(Point a, Point b){

    float x_part = pow(a.x - b.x, 2);
    float y_part = pow(a.y - b.y, 2); 

    return sqrt(x_part + y_part);
}

//trasform laser scan points from polar coord to cartesian coord
std::vector<Point> getPoints(std::vector<float> ranges, float angle_min, float d_theta){

    const double INF = std::numeric_limits<double>::infinity();

    std::vector<Point> points;

    for(int i=0; i<ranges.size(); i++){
        if(ranges[i] < INF){
            float theta = angle_min + d_theta*i;
            Point point;
            point.x = ranges[i]*cos(theta);
            point.y = ranges[i]*sin(theta);
            points.push_back(point);
        }
    }

    return points;

}

//group points that are close to each other
std::vector<std::vector<Point>> clustering(std::vector<Point> points){
    
    std::vector<std::vector<Point>> clusters;

    //compute the average distance among consecutive points
    float avg_dist = 0;
    for(int i = 0; i<points.size()-1; i++){
        float dist = points_distance(points[i], points[i+1]);
        avg_dist += dist/(points.size()-1);
    }

    float error = avg_dist * 2;

    //group consecutive points into the same cluster
    //if the distance among them is less than 
    //the double of the average distance
    //among consecutive points
    for(int i = 0; i < points.size()-1; i++){
        std::vector<Point> cluster;
        while(i < points.size()-1 && points_distance(points[i], points[i+1]) < error){
            cluster.push_back(points[++i]);
        }
        clusters.push_back(cluster);
    }

    return clusters;
}

//compute circle score
float circle_score(std::vector<Point> points, Circle circle){
    float score = 0;
    for(int j = 0; j < points.size(); j++){
        float dist_center = points_distance(points[j], circle.center);
        float dist_circle = pow(dist_center - circle.radius, 2);
        score += dist_circle / points.size();
    }
    score = sqrt(score);
    return 1/score;
}

//compute distance from a point to a line, that is defined by two points
float line_point_distance(Point start_line, Point end_line, Point p){

    //SOURCE: 

    float x1 = start_line.x, y1 = start_line.y;
    float x2 = end_line.x, y2 = end_line.y;

    float x3 = p.x, y3 = p.y;
    
    float x21 = x2 - x1;
    float y21 = y2 - y1;

    float y13 = y1 - y3;
    float x13 = x1 - x3;

    float x21_2 = pow(x21, 2);
    float y21_2 = pow(y21, 2);

    float distance = abs(x21 * y13 - x13 * y21) / sqrt(x21_2 + y21_2);

    return distance;
}

//compute line score
float line_score(std::vector<Point> points, Point p1, Point p2){
        Point start = p1;
        Point end = p2;
        float score = 0;
        for(int j = 0; j < points.size(); j++){
            Point p = points[j];
            float distance = line_point_distance(start, end, p);
            score += pow(distance, 2) / points.size();
        }
        score = sqrt(score);
        return 1/score;    
}

