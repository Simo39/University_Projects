#include <math.h>
#include <limits>
#include <tuple>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>                          //message sent by topic scan, used in service request

#include <assignment_1/DetectObstacles.h>                   //service message
#include <assignment_1/Circle.h>                            //message for circle, used in service response

// struct that define a point
struct Point{
    float x;
    float y;
};

// struct that define a circle circle
// circle equation in form: x^2 + y^2 + 2*g*x + 2*f*y + c = 0
// where centre is (h = -g, k = -f) and radius r
// as r^2 = h^2 + k^2 - c
struct Circle{
    float g;
    float f;
    float c;
    float radius;
    Point center;
    //function that set the above variables
    void setCircle(Point p1, Point p2, Point p3);
};

//compute the distance among two points
float points_distance(Point a, Point b);

//trasform laser scan points from polar coord to cartesian coord
std::vector<Point> getPoints(std::vector<float> ranges, float angle_min, float d_theta);

//group points that are close to each other
std::vector<std::vector<Point>> clustering(std::vector<Point> points);

//compute circle score
float circle_score(std::vector<Point> points, Circle circle);

//compute distance from a point to a line, that is defined by two points
float line_point_distance(Point start_line, Point end_line, Point p);

//compute line score
float line_score(std::vector<Point> points, Point p1, Point p2);

