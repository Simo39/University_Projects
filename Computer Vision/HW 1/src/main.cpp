#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <algorithm>

using namespace cv;

void inputError(char* argv[]) {
	std::string program = static_cast<std::string>(argv[0]);
	int pos = program.rfind("\\");
	if (pos == -1)
		pos = program.rfind("/");
	program = program.substr(pos + 1);
	std::cout << "ERROR on using: " << program << std::endl;
	std::cout << "Correct use:\n\n\t" << program << " DIRECTORY_PATH TEST_IMAGE\n" << std::endl;
	std::cout << "DIRECTORY_PATH: path to the directory containing the calibartion checkerboard images of type *.png " << std::endl;
	std::cout << "TEST_IMAGE:     path to the image to test the calibartion" << std::endl;
}

void points3Dinitialization(int rows, int cols, float squareSize, std::vector<Point3f>& points3D) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			points3D.push_back(Point3f{ squareSize * j, squareSize * i, 0 });
		}
	}
}

void showCorners(Mat img, Size innerCorners_qty, std::vector<Point2f> points, bool found, String file) {
	drawChessboardCorners(img, innerCorners_qty, points, found);
	int cut = String("res/checkerboard_images\\").size();
	//the img is resized for a better view on the screen
	double scale{ 2.0 };
	Size new_size(static_cast<int>(img.cols / scale), static_cast<int>(img.rows / scale));
	Mat resized_img;
	resize(img, resized_img, new_size, 0, 0, INTER_AREA);

	imshow(file.substr(cut), resized_img);
	waitKey(0);
}

int main(int argc, char* argv[]) {

	if (argc != 3) {
		inputError(argv);
		return -1;
	}
	String dir_path = static_cast<String>(argv[1]);
	String test_path = static_cast<String>(argv[2]);
	Mat test_img = imread(test_path);
	if (test_img.empty()) {
		inputError(argv);
		return -1;
	}

	//dataset loading
	std::vector<String> file_list;
	utils::fs::glob(dir_path, "*.png", file_list);
	if (file_list.empty()) {
		std::cout <<"EMPTY DIRECTORY\n" << dir_path << " doesn't contain any file." << std::endl;
		return 0;
	}

	//corners initialization
	int columns{ 6 }, rows{ 5 };
	std::vector<std::vector<Point2f>> points2D;
	std::vector<std::vector<Point3f>> points3D;
	//3D points initialization
	float squareSize{ 0.11f };
	std::vector<Point3f> temp;
	points3Dinitialization(rows, columns, squareSize, temp);
	for (int i = 0; i < file_list.size(); i++) {
		points3D.push_back(temp);
	}
	//2D points detection
	Size innerCorners_qty{ columns, rows };
	for (String file : file_list) {
		Mat img = imread(file, IMREAD_GRAYSCALE);
		std::vector<Point2f> points;
		bool found = findChessboardCorners(img, innerCorners_qty, points,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (found) {
			cornerSubPix(img, points, Size(22, 22), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001));
			points2D.push_back(points);
			//showCorners(img, innerCorners_qty, points, found, file);
		}
	}

	//camera calibration
	Mat img = imread(file_list[0], IMREAD_GRAYSCALE);

	Mat cameraMatrix;
	std::vector<Mat> rotMat, transMat; //extrinsic parameters for each img
	std::vector<double> distCoeffs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;

	std::cout << "Camera calibration";
	double overall_RMS = calibrateCamera(points3D, points2D, img.size(), cameraMatrix, distCoeffs, rotMat, transMat, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors);
	std::cout << "\t[DONE]" << std::endl;


	std::cout << "Focal lenght: " << std::endl;
	std::cout << "\tf_x: " << cameraMatrix.at<double>(0, 0) << std::endl;
	std::cout << "\tf_y: " << cameraMatrix.at<double>(1, 1) << std::endl;

	std::cout << "Optical center (in pixel coords): " << std::endl;
	std::cout << "\tc_x: " << cameraMatrix.at<double>(0, 2) << std::endl;
	std::cout << "\tc_y: " << cameraMatrix.at<double>(1, 2) << std::endl;

	std::cout << "Radial distorsion params: " << std::endl;
	std::cout << "\tk_1: " << distCoeffs[0] << std::endl;
	std::cout << "\tk_2: " << distCoeffs[1] << std::endl;
	if (distCoeffs.size() > 4)
		std::cout << "\tk_3: " << distCoeffs[4] << std::endl;

	std::cout << "Tangential distorsion params: " << std::endl;
	std::cout << "\tp_1: " << distCoeffs[2] << std::endl;
	std::cout << "\tp_2: " << distCoeffs[3] << std::endl;

	//Point reprojections and relative mean errors
	std::vector<double> mean_errors;
	double best_error, worst_error;
	int best_index{ 0 }, worst_index{ 0 };
	bool first{ true };
	for (int i = 0; i < points3D.size(); i++) {
		//3D points reprojections with the new computed intrinsic and extrinsic parameters
		std::vector<Point2f> projectedPoints;
		projectPoints(points3D[i], rotMat[i], transMat[i], cameraMatrix, distCoeffs, projectedPoints);

		//mean error computation of a single image
		mean_errors.push_back(0);
		for (int j = 0; j < projectedPoints.size(); j++) {
			//euclidian distance among the old 2D point and the new one
			mean_errors[i] += norm(projectedPoints[j] - points2D[i][j]);
		}
		mean_errors[i] /= projectedPoints.size();

		//initialization of the best and worst error
		if (first) {
			best_error = mean_errors[i];
			worst_error = best_error;
			first = false;
		}

		//best and worst error computation
		if (mean_errors[i] > worst_error) {
			worst_error = mean_errors[i];
			worst_index = i;
		}
		if (mean_errors[i] < best_error) {
			best_error = mean_errors[i];
			best_index = i;
		}
	}

	std::cout << "Mean Errors:" << std::endl;
	double tot{ 0 };
	for (double error : mean_errors)
		tot += error;
	double overall_mean_error = tot / mean_errors.size();
	std::cout << "\tOVERALL mean error: " << overall_mean_error << std::endl;
	std::cout << "\tBEST  at " << best_index << ": " << mean_errors[best_index] << std::endl;
	std::cout << "\tWORST at " << worst_index << ": " << mean_errors[worst_index] << std::endl;
	int cut = String("res/checkerboard_images\\").size();
	std::cout << "\tBest  calibration perforfmed by: " << file_list[best_index].substr(cut) << std::endl;
	std::cout << "\tWorst calibration perforfmed by: " << file_list[worst_index].substr(cut) << std::endl;
	
	//best and worst RMS error computation
	int bestRMS_index{ 0 }, worstRMS_index{ 0 };
	double bestRMS{ perViewErrors[0] }, worstRMS{ bestRMS };
	for (int i = 0; i < perViewErrors.size(); i++) {
		double el = perViewErrors[i];
		if (el > worstRMS) {
			worstRMS = el;
			worstRMS_index = i;
		}
		if (el < bestRMS) {
			bestRMS = el;
			bestRMS_index = i;
		}
	}
	std::cout << "RMS Errors:" << std::endl;
	std::cout << "\tOVERALL RMS error  : " << overall_RMS << std::endl;
	std::cout << "\tBEST  at " << bestRMS_index << ": " << bestRMS << std::endl;
	std::cout << "\tWORST at " << worstRMS_index << ": " <<worstRMS << std::endl;
	std::cout << "\tBest  calibration perforfmed by: " << file_list[bestRMS_index].substr(cut) << std::endl;
	std::cout << "\tWorst calibration perforfmed by: " << file_list[worstRMS_index].substr(cut) << std::endl;
	
	//Undistortion and correction of the test image
	Mat map1, map2, result;

	//image remapping
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, test_img.size(), CV_32FC1, map1, map2);
	remap(test_img,result, map1, map2, INTER_CUBIC);

	//showinng the result
	double scale{ 3.0 };
	Size new_size(static_cast<int>(test_img.cols/scale), static_cast<int>(test_img.rows / scale));
	resize(test_img, test_img, new_size, 0, 0, INTER_AREA);
	resize(result, result, new_size, 0, 0, INTER_AREA);
	hconcat(test_img, result, result);
	imshow("Input and Output", result);
	waitKey(0);

	return 0;
}