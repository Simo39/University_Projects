#include "Dictionary.h"
#include <iostream>
#include <limits>

Dictionary::Dictionary(std::string folder_path) {
	Mat dictionary;
	FileStorage fs(folder_path, FileStorage::READ);
	fs["vocabulary"] >> dictionary;
	fs.release();
	bowDE.setVocabulary(dictionary);
}

std::vector<KeyPoint> Dictionary::extractKeypoints(Mat img, float ratio_threshold) {
	std::vector<KeyPoint> keypoints;
	detector->detect(img, keypoints);
	std::vector<std::vector<int>> kp_index;
	Mat bowDescriptor;
	Mat descriptors;
	bowDE.compute(img, keypoints, bowDescriptor, &kp_index, &descriptors);
	std::vector<std::vector<int>> points_histogram;
	std::vector<KeyPoint> new_kp;
	for (int i = 0; i < kp_index.size(); i++) {
		std::vector<int> points{};
		if (kp_index[i].size() != 0) {
			//keypoints descriptor of i-th cluster
			Mat keypoints_descriptors = getListOfDescriptors(kp_index[i], descriptors);
			Mat cluster_bowDescriptor = bowDE.getVocabulary().row(i);
			points = thresholdPoints(kp_index[i], keypoints_descriptors, cluster_bowDescriptor, ratio_threshold);
			for (int index : points)
				new_kp.push_back(keypoints[index]);
		}
		points_histogram.push_back(points);
	}
	computeAccuracy(points_histogram, keypoints.size());
	return new_kp;
}

Mat Dictionary::getListOfDescriptors(std::vector<int> indices, Mat descriptors) {
	Mat descriptors_list(0, 128, descriptors.type());
	for (int index : indices)
		descriptors_list.push_back(descriptors.row(index));
	return descriptors_list;
}

std::vector<int> Dictionary::thresholdPoints(std::vector<int> indices, Mat descriptors, Mat cluster_descriptor, float ratio_threshold) {
	std::vector<DMatch> matches;
	matcher->match(descriptors, cluster_descriptor, matches);
	float min_distance = std::numeric_limits<float>::infinity();
	for (DMatch match : matches) {
		if (match.distance < min_distance)
			min_distance = match.distance;
	}
	min_distance = ratio_threshold * min_distance;
	std::vector<int> new_indices;
	for (int i = 0; i < matches.size(); i++) {
		if (matches[i].distance <= min_distance) {
			new_indices.push_back(indices[i]);
		}
	}
	return new_indices;
}

void Dictionary::computeAccuracy(std::vector<std::vector<int>> histogram, int total_points) {
	int sum = 0;
	for (std::vector<int> points : histogram) {
		sum += points.size();
	}
	last_accuracy = (static_cast<float>(sum)) / total_points;
}

float Dictionary::getAccuracy() {
	return last_accuracy;
}