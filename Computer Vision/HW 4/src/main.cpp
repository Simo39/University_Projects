#include "panoramic_utils.h"
#include "PanoramicImage.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <regex>

using namespace cv;

void inputError(char* argv[]);
float fromCharToNumber(char* argv[], int pos);

int main(int argc, char* argv[]) {
	try{
		if (argc != 4) {
			inputError(argv);
			return -1;
		}
		int fov = static_cast<int>(fromCharToNumber(argv, 2));
		if (fov <= 0) {
			std::cout << "ERROR:\tAs second argument it's requested a number greater than 0" << std::endl;
			inputError(argv);
			return -1;
		}
		float ratio = fromCharToNumber(argv, 3);
		if (ratio <= 0) {
			std::cout << "ERROR:\tAs third argument it's requested a number greater than 0" << std::endl;
			inputError(argv);
			return -1;
		}
		String dir_path = static_cast<String>(argv[1]);
		//dataset loading
		std::vector<String> file_list;
		utils::fs::glob(dir_path, "*.png", file_list);
		if (file_list.empty())
			utils::fs::glob(dir_path, "*.bmp", file_list);
		if (file_list.empty()) {
			std::cout << "ERROR:\tEMPTY DIRECTORY\n" << dir_path << " doesn't contain any file.\n" << std::endl;
			inputError(argv);
			return -1;
		}

		PanoramicImage panoramic(file_list, fov);

		//SIFT computation
		Mat result = panoramic.composeWithSIFT(ratio);

		double scale = 0.3;
		Size newSize(result.cols * scale, result.rows * scale);

		namedWindow("SIFT", WINDOW_NORMAL);
		resizeWindow("SIFT", newSize);
		moveWindow("SIFT", 0, 0);
		imshow("SIFT", result);
		waitKey(0);
		destroyAllWindows();

		//ORB computation
		result = panoramic.composeWithORB(ratio);

		namedWindow("ORB", WINDOW_NORMAL);
		resizeWindow("ORB", newSize);
		moveWindow("ORB", 0, 0);
		imshow("ORB", result);
		waitKey(0);
		destroyAllWindows();
	}
	catch (const std::invalid_argument exception) {
		std::cerr << exception.what() << std::endl;
		return -1;
	}
	return 0;
}

//function called in case of error
void inputError(char* argv[]) {
	std::string program = static_cast<std::string>(argv[0]);
	int pos = program.rfind("\\");
	if (pos == -1)
		pos = program.rfind("/");
	program = program.substr(pos + 1);
	std::cout << "ERROR on using: " << program << std::endl;
	std::cout << "Correct use:\n\n\t" << program << " <DIRECTORY_PATH> <FIELD_OF_VIEW> <RATIO>\n" << std::endl;
	std::cout << "DIRECTORY_PATH: path to the directory containing the image dataset in format *.png or *.bmp" << std::endl;
	std::cout << "FIELD_OF_VIEW:  field of view of the camera used to take the images stored in the directory" << std::endl;
	std::cout << "RATIO:          minimum distance ratio threshold" << std::endl;
}

//check if the input string is an integer and cast it
float fromCharToNumber(char* argv[], int pos) {
	std::string input = static_cast<std::string>(argv[pos]);
	if (!std::regex_match(input, std::regex("[0-9]+(\.[0-9])?[0-9]*")))
		return -1;
	return std::stof(input);
}