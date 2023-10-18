#include "BoWCreator.h"
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>

using namespace cv;

int main(int argc, char** argv) {
	if (argc != 2) {
		std::cerr << "A path to a directory with images is requested" << std::endl;
	}
	std::string path = argv[1];
	BoWCreator creator(path);
	Mat dictionary = creator.createBoW();
	std::string file = "dictionary.yml";
	FileStorage fs(file, FileStorage::WRITE);
	fs << "vocabulary" << dictionary;
	fs.release();
	std::cout << "\t" << file << " CREATION DONE" << std::endl;

	return 0;
}