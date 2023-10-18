
#include "debug_utils.h"
#include "FilteringTest.h"
#include "Localization.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/saliency.hpp>
#include "opencv2/ximgproc/segmentation.hpp"
#include <exception>
#include <iostream>

using namespace cv;

const std::string INPUT_WINDOW_NAME = "Input";
const std::string RESULT_WINDOW_NAME = "Result";
const std::string WATERSHED_WINDOW_NAME = "Watershed";
const std::string SPECTRAL_RESIDUAL_WINDOW_NAME = "Static Saliency - Spectral Residual";
const std::string FINE_GRAINED_WINDOW_NAME = "Static Saliency - Fine Grained";
const std::string MAP_WINDOW_NAME = "Map";
const std::string SS_FAST_WINDOW_NAME = "Selective Search - Fast";
const std::string SS_QUALITY_WINDOW_NAME = "Selective Search - Quality";
const std::string CANNY_WINDOW_NAME = "Canny";
const std::string GAUSSIAN_WINDOW_NAME = "Gaussian";
const std::string BILATERAL_WINDOW_NAME = "Bilateral Filter";
const std::string MORPHOLOGIC_WINDOW_NAME = "Morphologic Operator";
const std::string HISTOGRAM_EQUALIZATION_WINDOW_NAME = "Histogram Equalization";
const int WINDOW_FLAGS = WINDOW_NORMAL | WINDOW_KEEPRATIO | WINDOW_GUI_EXPANDED;
const double WINDOW_SIZE_THRESHOLD = 1.8;
const Size WINDOW_SIZE(400 * WINDOW_SIZE_THRESHOLD, 300 * WINDOW_SIZE_THRESHOLD);

Mat input_image;

std::vector<Mat> good_results_images;
std::vector<std::vector<std::string>> good_results_descriptors;
std::vector<std::string> result_descriptors;

Mat adaptiveHistogramEqualizer(Mat input, HistogramEqualizer* data) {
	std::vector<int> thresholds{ 0,120,79 };
	if (data != NULL) {
		thresholds[1] = data->getThreshold(1);		//lightness
		thresholds[2] = data->getThreshold(2);		//saturation
	}
	HistogramEqualizer histgrams(input, thresholds);
	namedWindow(HISTOGRAM_EQUALIZATION_WINDOW_NAME, WINDOW_FLAGS);
	createTrackbar("L-channel Threshold", HISTOGRAM_EQUALIZATION_WINDOW_NAME, &thresholds[1], 255, onChangeLightnessThreshold, &histgrams);
	createTrackbar("S-channel Threshold", HISTOGRAM_EQUALIZATION_WINDOW_NAME, &thresholds[2], 255, onChangeSaturationThreshold, &histgrams);
	histgrams.equalizeChannel(1);
	histgrams.equalizeChannel(2);
	imshow(HISTOGRAM_EQUALIZATION_WINDOW_NAME, histgrams.getResult());
	waitKey();
	destroyWindow(HISTOGRAM_EQUALIZATION_WINDOW_NAME);
	result_descriptors.push_back("EQUALIZATION RESULTS:");
	result_descriptors.push_back("\tL-channel mean: " + std::to_string(histgrams.getMean(1)));
	result_descriptors.push_back("\tS-channel mean: " + std::to_string(histgrams.getMean(2)));
	result_descriptors.push_back("\tL-channel Threshold: " + std::to_string(histgrams.getThreshold(1)));
	result_descriptors.push_back("\tS-channel Threshold: " + std::to_string(histgrams.getThreshold(2)));
	return histgrams.getResult();
}
void onChangeLightnessThreshold(int pos, void* userdata) {
	HistogramEqualizer* hist = static_cast<HistogramEqualizer*>(userdata);
	hist->setThreshold(1, pos);
	hist->equalizeChannel(1);
	imshow(HISTOGRAM_EQUALIZATION_WINDOW_NAME, hist->getResult());
}
void onChangeSaturationThreshold(int pos, void* userdata) {
	HistogramEqualizer* hist = static_cast<HistogramEqualizer*>(userdata);
	hist->setThreshold(2, pos);
	hist->equalizeChannel(2);
	imshow(HISTOGRAM_EQUALIZATION_WINDOW_NAME, hist->getResult());
}

Mat useCanny(Mat input, CannyFilter* data) {
	int ratio = 2;
	int threshold = 268;
	if (data != NULL) {
		ratio = data->getRatio();
		threshold = data->getLowerThreshold();
	}
	CannyFilter canny(input, ratio, threshold);
	namedWindow(CANNY_WINDOW_NAME, WINDOW_FLAGS);
	createTrackbar("Lower Threshold", CANNY_WINDOW_NAME, &threshold, 500, onChangeFirstThreshold, &canny);
	createTrackbar("Ratio", CANNY_WINDOW_NAME, &ratio, 3, onChangeRatio, &canny);
	canny.doFilter();
	showRegions(canny.getResult());
	imshow(CANNY_WINDOW_NAME, canny.getResult());
	waitKey();
	destroyWindow(CANNY_WINDOW_NAME);
	result_descriptors.push_back("CANNY RESULTS:");
	result_descriptors.push_back("\t    ratio: " + std::to_string(canny.getRatio()));
	result_descriptors.push_back("\tthreshold: " + std::to_string(canny.getLowerThreshold()));
	return canny.getResult();
}
void onChangeFirstThreshold(int pos, void* userdata) {
	CannyFilter* canny = static_cast<CannyFilter*>(userdata);
	canny->setLowerThreshold(pos);
	canny->doFilter();
	showRegions(canny->getResult());
	imshow(CANNY_WINDOW_NAME, canny->getResult());
}
void onChangeRatio(int pos, void* userdata) {
	if (pos < 2)
		pos = 2;
	CannyFilter* canny = static_cast<CannyFilter*>(userdata);
	canny->setRatio(pos);
	canny->doFilter();
	showRegions(canny->getResult());
	imshow(CANNY_WINDOW_NAME, canny->getResult());
}

Mat useGaussian(Mat input, GaussianFilter* data) {
	int kernel_size = 5;
	int sigma = 9;
	if (data != NULL) {
		kernel_size = data->getSize();
		sigma = data->getSigma();
	}
	GaussianFilter gaussian(input, kernel_size, sigma);
	namedWindow(GAUSSIAN_WINDOW_NAME, WINDOW_FLAGS);
	createTrackbar("Kernel", GAUSSIAN_WINDOW_NAME, &kernel_size, 100, onSetKernelGF, static_cast<void*>(&gaussian));
	createTrackbar("Sigma", GAUSSIAN_WINDOW_NAME, &sigma, 100, onSetSigmaGF, static_cast<void*>(&gaussian));
	gaussian.doFilter();
	imshow(GAUSSIAN_WINDOW_NAME, gaussian.getResult());
	waitKey();
	destroyWindow(GAUSSIAN_WINDOW_NAME);
	result_descriptors.push_back("GAUSSIAN FILTER RESULTS:");
	result_descriptors.push_back("\tkernel size: " + std::to_string(gaussian.getSize()));
	result_descriptors.push_back("\t      sigma: " + std::to_string(gaussian.getSigma()));
	return gaussian.getResult();
}
void onSetKernelGF(int pos, void* userdata) {
	GaussianFilter* img = static_cast<GaussianFilter*>(userdata);
	img->setSize(pos);
	img->doFilter();
	imshow(GAUSSIAN_WINDOW_NAME, img->getResult());
}
void onSetSigmaGF(int pos, void* userdata) {
	GaussianFilter* img = static_cast<GaussianFilter*>(userdata);
	img->setSigma(pos);
	img->doFilter();
	imshow(GAUSSIAN_WINDOW_NAME, img->getResult());
}

Mat useBilateral(Mat input, BilateralFilter* data) {
	int sigma_range = 72;
	int sigma_space = 5;
	if (data != NULL) {
		sigma_range = data->getSigmaRange();
		sigma_space = data->getSigmaSpace();
	}
	BilateralFilter bilateral(input, sigma_range, sigma_space);
	namedWindow(BILATERAL_WINDOW_NAME, WINDOW_FLAGS);
	createTrackbar("S_Range", BILATERAL_WINDOW_NAME, &sigma_range, 200, onSetSigmaRangeBF, static_cast<void*>(&bilateral));
	createTrackbar("S_Space", BILATERAL_WINDOW_NAME, &sigma_space, 10, onSetSigmaSpaceBF, static_cast<void*>(&bilateral));
	bilateral.doFilter();
	imshow(BILATERAL_WINDOW_NAME, bilateral.getResult());
	waitKey();
	destroyWindow(BILATERAL_WINDOW_NAME);
	result_descriptors.push_back("BILATERAL FILTER RESULTS:");
	result_descriptors.push_back("\tsigma range: " + std::to_string(bilateral.getSigmaRange()));
	result_descriptors.push_back("\tsigma space: " + std::to_string(bilateral.getSigmaSpace()));
	return bilateral.getResult();
}
void onSetSigmaRangeBF(int pos, void* userdata) {
	BilateralFilter* img = static_cast<BilateralFilter*>(userdata);
	img->setSigmaRange(pos);
	img->doFilter();
	imshow(BILATERAL_WINDOW_NAME, img->getResult());
}
void onSetSigmaSpaceBF(int pos, void* userdata) {
	BilateralFilter* img = static_cast<BilateralFilter*>(userdata);
	img->setSigmaSpace(pos);
	img->doFilter();
	imshow(BILATERAL_WINDOW_NAME, img->getResult());
}

Mat useMorphOperator(Mat input, MorphologicOperator* data) {
	int morphological_operator = 3;
	int element = 2;
	int kernel_size = 11;
	int iterations = 1;
	if (data != NULL) {
		morphological_operator = data->getMorphOperator();
		element = data->getElement();
		kernel_size = data->getSize();
		iterations = data->getIterations();
	}
	MorphologicOperator morphologic(input, morphological_operator, element, kernel_size, iterations);
	namedWindow(MORPHOLOGIC_WINDOW_NAME, WINDOW_FLAGS);
	createTrackbar("Operator", MORPHOLOGIC_WINDOW_NAME, &morphological_operator, 6, onSetMorphOperator, static_cast<void*>(&morphologic));
	createTrackbar("Element", MORPHOLOGIC_WINDOW_NAME, &element, 2, onSetElement, static_cast<void*>(&morphologic));
	createTrackbar("Size", MORPHOLOGIC_WINDOW_NAME, &kernel_size, 50, onSetSize, static_cast<void*>(&morphologic));
	createTrackbar("Iterations +1", MORPHOLOGIC_WINDOW_NAME, &iterations, 50, onSetIterations, static_cast<void*>(&morphologic));
	morphologic.doFilter();
	showRegions(morphologic.getResult());
	imshow(MORPHOLOGIC_WINDOW_NAME, morphologic.getResult());
	waitKey();
	destroyWindow(MORPHOLOGIC_WINDOW_NAME);
	result_descriptors.push_back("MORPHOLOGIC OPERATOR RESULTS:");
	result_descriptors.push_back("\t operator: " + std::to_string(morphologic.getMorphOperator()));
	result_descriptors.push_back("\t  element: " + std::to_string(morphologic.getElement()));
	result_descriptors.push_back("\t     size: " + std::to_string(morphologic.getSize()));
	result_descriptors.push_back("\titerations: " + std::to_string(morphologic.getIterations()));
	return morphologic.getResult();
}
void onSetMorphOperator(int pos, void* userdata) {
	MorphologicOperator* img = static_cast<MorphologicOperator*>(userdata);
	img->setMorphOperator(pos);
	img->doFilter();
	showRegions(img->getResult());
	imshow(MORPHOLOGIC_WINDOW_NAME, img->getResult());
}
void onSetElement(int pos, void* userdata) {
	MorphologicOperator* img = static_cast<MorphologicOperator*>(userdata);
	img->setElement(pos);
	img->doFilter();
	showRegions(img->getResult());
	imshow(MORPHOLOGIC_WINDOW_NAME, img->getResult());
}
void onSetSize(int pos, void* userdata) {
	MorphologicOperator* img = static_cast<MorphologicOperator*>(userdata);
	img->setSize(pos);
	img->doFilter();
	showRegions(img->getResult());
	imshow(MORPHOLOGIC_WINDOW_NAME, img->getResult());
}
void onSetIterations(int pos, void* userdata) {
	MorphologicOperator* img = static_cast<MorphologicOperator*>(userdata);
	img->setIterations(pos + 1);
	img->doFilter();
	showRegions(img->getResult());
	imshow(MORPHOLOGIC_WINDOW_NAME, img->getResult());
}

Mat Localization::useSelectiveSearch(Mat input, bool fastSearch) {
	Ptr<ximgproc::segmentation::SelectiveSearchSegmentation> ss = ximgproc::segmentation::createSelectiveSearchSegmentation();
	ss->setBaseImage(input);
	std::string win_name;
	if (fastSearch){
		ss->switchToSelectiveSearchFast();
		win_name = SS_FAST_WINDOW_NAME;
	}
	else{
		ss->switchToSelectiveSearchQuality();
		win_name = SS_QUALITY_WINDOW_NAME;
	}

	std::vector<Rect> rects;
	ss->process(rects);
	std::string descriptor_1 = "Total Number of Region Proposals: " + std::to_string(rects.size());
	std::cout << descriptor_1 << std::endl;

	int numShowRects = 100;
	int increment = 50;
	std::string descriptor_2 = "Considered Number of Region: ";
	Mat result;
	namedWindow(win_name, WINDOW_FLAGS);
	while (true) {
		Mat output = input.clone();
		for (int i = 0; i < rects.size(); i++) {
			if (i < numShowRects) {
				rectangle(output, rects[i], Scalar(0, 255, 0));
			}
			else {
				break;
			}
		}
		imshow(win_name, output);
		std::cout << descriptor_2 << numShowRects << std::endl;
		std::cout << "Press:" << std::endl;
		std::cout << "      m for more windows" << std::endl;
		std::cout << "      l for less windows" << std::endl;
		std::cout << "      q to quit" << std::endl;
		char k = waitKey();

		//if 'm' for more is pressed, it shows more regions
		if (k == 'm') {
			numShowRects += increment;
		}
		//if 'l' for less is pressed, it show less regions
		else if (k == 'l' && numShowRects > increment) {
			numShowRects -= increment;
		}
		//save teh result and exit
		else if (k == 'q') {
			result = output.clone();
			descriptor_2 += std::to_string(numShowRects);
			break;
		}
	}
	destroyWindow(win_name);
	result_descriptors.push_back(descriptor_1);
	result_descriptors.push_back(descriptor_2);
	return result;
}

Mat Localization::useStaticSaliencyDetection(Mat input, bool spectralResidual) {
	Ptr<saliency::StaticSaliency> ss = saliency::StaticSaliencySpectralResidual::create();
	std::string win_name = SPECTRAL_RESIDUAL_WINDOW_NAME;
	if (!spectralResidual) {
		Ptr<saliency::StaticSaliency> ss = saliency::StaticSaliencyFineGrained::create();
		win_name = FINE_GRAINED_WINDOW_NAME;
	}
	Mat map, output;
	namedWindow(MAP_WINDOW_NAME, WINDOW_FLAGS);
	namedWindow(win_name, WINDOW_FLAGS);
	ss->computeSaliency(input, map);
	ss->computeBinaryMap(map, output);
	imshow(MAP_WINDOW_NAME,map);
	imshow(win_name, output);
	waitKey();
	destroyWindow(win_name);
	destroyWindow(MAP_WINDOW_NAME);
	return output;
}

Mat Localization::useWatershed(Mat input, Mat seeds)
{
	std::vector<std::vector<Point> > contours;
	findContours(seeds, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	Mat markers = Mat::zeros(seeds.size(), CV_32S);
	for (int i = 0; i < contours.size(); i++)
		drawContours(markers, contours, i, Scalar(i + 1), -1);
	circle(markers, Point(5, 5), 3, Scalar(255), -1);
	watershed(input, markers);
	markers.convertTo(markers, CV_8U, 10);
	std::vector<Vec3b> colors;
	for (int i = 0; i < contours.size(); i++)
	{
		int b = theRNG().uniform(0, 256);
		int g = theRNG().uniform(0, 256);
		int r = theRNG().uniform(0, 256);
		colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}
	Mat output = Mat::zeros(markers.size(), CV_8UC3);
	for (int i = 0; i < markers.rows; i++)
	{
		for (int j = 0; j < markers.cols; j++)
		{
			int index = markers.at<int>(i, j);
			if (index > 0 && index <= static_cast<int>(contours.size()))
			{
				output.at<Vec3b>(i, j) = colors[index - 1];
			}
		}
	}
	namedWindow(WATERSHED_WINDOW_NAME, WINDOW_FLAGS);
	imshow(WATERSHED_WINDOW_NAME, output);
	waitKey();
	destroyWindow(WATERSHED_WINDOW_NAME);
	return output;
}

void showRegions(Mat mask) {
	Mat result = createImageWithRegions(mask);
	imshow(RESULT_WINDOW_NAME, result);
}
Mat createImageWithRegions(Mat mask) {
	std::vector<Rect> regions = createRegions(mask);
	regions = clustering(regions);
	Mat result = input_image.clone();
	for (int i = 0; i < regions.size(); i++) {
		Scalar color(255, 0, 0);
		rectangle(result, regions[i].tl(), regions[i].br(), color, 2);
	}
	return result;
}
std::vector<Rect> createRegions(Mat mask) {
	std::vector<std::vector<Point> > contours;
	findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<Point> > contours_poly(contours.size());
	std::vector<Rect> boundRect(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		Scalar color(255, 0, 0);
		approxPolyDP(contours[i], contours_poly[i], 3, true);
		boundRect[i] = boundingRect(contours_poly[i]);
	}
	return boundRect;
}
std::vector<Rect> getRegions(Mat mask) {
	std::vector<std::vector<Point> > contours;
	findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<Point> > contours_poly(contours.size());
	std::vector<Rect> boundRect(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		Scalar color(255, 0, 0);
		approxPolyDP(contours[i], contours_poly[i], 3, true);
		boundRect[i] = boundingRect(contours_poly[i]);
	}
	return boundRect;
}

void testInitialaizer(std::string title) {
	result_descriptors.push_back(title);
	namedWindow(RESULT_WINDOW_NAME, WINDOW_FLAGS);
	imshow(RESULT_WINDOW_NAME, input_image);
}
void saveResult(Mat image) {
	good_results_images.push_back(image);
	good_results_descriptors.push_back(result_descriptors);
	result_descriptors.clear();
}
void showSavedResult() {
	std::cout << "+---------------------------------------------------------------------------------------+" << std::endl;
	std::cout << "|----------------------------------------RESULTS----------------------------------------|" << std::endl;
	std::cout << "+---------------------------------------------------------------------------------------+" << std::endl;
	for (int i = 0; i < good_results_images.size(); i++) {
		std::string win_name = good_results_descriptors[i][0];
		namedWindow(win_name, WINDOW_FLAGS);
		std::cout << win_name << std::endl;
		for (int j = 1; j < good_results_descriptors[i].size(); j++) {
			std::cout << "\t" << good_results_descriptors[i][j] << std::endl;
		}
		imshow(win_name, good_results_images[i]);
		std::cout << "+---------------------------------------------------------------------------------------+" << std::endl;
	}
	waitKey();
	destroyAllWindows();
}

std::vector<Rect> clustering(std::vector<Rect> regions) {
	std::vector<std::vector<Rect>> clusters;
	clusters = cluster_rects(regions, 0);
	regions.clear();
	for (std::vector<Rect> cluster : clusters) {
		Rect region = union_of_rects(cluster);
		regions.push_back(region);
	}
	return regions;
}
std::vector<std::vector<Rect>> cluster_rects(const std::vector<Rect>& rects, const double th)
{
	std::vector<int> labels;
	int n_labels = partition(rects, labels,
		[th](const Rect& lhs, const Rect& rhs) {
			double i = static_cast<double>((lhs & rhs).area());
			double ratio_intersection_over_lhs_area = i / static_cast<double>(lhs.area());
			double ratio_intersection_over_rhs_area = i / static_cast<double>(rhs.area());
			return (ratio_intersection_over_lhs_area > th) || (ratio_intersection_over_rhs_area > th);
		});

	std::vector<std::vector<Rect>> clusters(n_labels);
	for (size_t i = 0; i < rects.size(); ++i) {
		clusters[labels[i]].push_back(rects[i]);
	}

	return clusters;
}
Rect union_of_rects(const std::vector<Rect>& cluster)
{
	Rect one;
	if (!cluster.empty())
	{
		one = cluster[0];
		for (const auto& r : cluster) { one |= r; }
	}
	return one;
}

void testRegionProposal(Mat image) {
	Mat bilateral, gaussian,
		selectiveSearchFast, selectiveSearchQuality,
		watershed, equalized,
		ssSpectralResidual, ssFineGrained,
		result;

	std::string title;

	good_results_images.clear();
	good_results_descriptors.clear();

	input_image = image.clone();

	namedWindow(INPUT_WINDOW_NAME, WINDOW_FLAGS);
	namedWindow(RESULT_WINDOW_NAME, WINDOW_FLAGS);
	resizeWindow(INPUT_WINDOW_NAME, WINDOW_SIZE);
	resizeWindow(RESULT_WINDOW_NAME, WINDOW_SIZE);
	imshow(INPUT_WINDOW_NAME, image);
	imshow(RESULT_WINDOW_NAME, image);
	waitKey();
	/*
	title = "ADAPTIVE HISTOGRAM EQUALIZATION";
	testInitialaizer(title);
	equalized = adaptiveHistogramEqualizer(image);
	saveResult(equalized);
	*/
	Mat bw_image;
	if (image.channels() == 3) {
		cvtColor(image, bw_image, COLOR_BGR2GRAY);
	}

	title = "GAUSSIAN FILTERING CASCADE";
	testInitialaizer(title);
	GaussianFilter gauss_data(bw_image, 7, 10);
	gaussian = useGaussian(bw_image, &gauss_data);
	CannyFilter gauss_canny(Mat(), 2, 49);
	gaussian = useCanny(gaussian, &gauss_canny);
	MorphologicOperator gauss_afterCanny(gaussian, 6, 2, 7, 3);
	gaussian = useMorphOperator(gaussian, &gauss_afterCanny);
	MorphologicOperator gauss_afterMorph(gaussian, 7, 1, 7, 1);
	gaussian = useMorphOperator(gaussian, &gauss_afterMorph);
	gaussian = createImageWithRegions(gaussian);
	saveResult(gaussian);

	title = "BILATERAL FILTERING CASCADE";
	testInitialaizer(title);
	bilateral = useBilateral(bw_image);
	CannyFilter bil_canny(Mat(), 2, 134);
	bilateral = useCanny(bilateral, &bil_canny);
	MorphologicOperator bil_afterCanny(bilateral, 5, 2, 9, 3);
	bilateral = useMorphOperator(bilateral, &bil_afterCanny);
	MorphologicOperator bil_afterMorph(bilateral, 3, 1, 3, 2);
	bilateral = useMorphOperator(bilateral, &bil_afterMorph);
	bilateral = createImageWithRegions(bilateral);
	saveResult(bilateral);

	showSavedResult();
}