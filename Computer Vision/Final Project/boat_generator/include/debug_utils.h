#pragma once
#include <opencv2/core.hpp>
#include <iostream>

void debug();
void debug(int point);
void debug(std::string name);
void debug(std::string name, int value);
void debug(std::string name, double value);
void debug(std::string name, std::string value);
void debug(std::string name, cv::Mat value);