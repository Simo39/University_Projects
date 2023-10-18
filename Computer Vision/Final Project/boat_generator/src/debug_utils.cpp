
#include "debug_utils.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

void debug() {
	std::cout << "DEBUG POINT" << std::endl;
}
void debug(int point) {
	std::cout << "DEBUG #" << point << std::endl;
}
void debug(std::string name) {
	std::cout << "DEBUG POINT: " << name << std::endl;
}
void debug(std::string name, int value) {
	std::cout << "DEBUG - " << name << ": " << value << std::endl;
}
void debug(std::string name, double value) {
	std::cout << "DEBUG - " << name << ": " << value << std::endl;
}
void debug(std::string name, std::string value) {
	std::cout << "DEBUG - " << name << ": " << value << std::endl;
}
void debug(std::string name, cv::Mat value) {
	std::string window = "DEBUG - " + name;
	cv::namedWindow(window);
	cv::imshow(window, value);
	cv::waitKey();
	cv::destroyWindow(window);
}