#pragma once

#include "debug_utils.h"
#include "Filter.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <exception>
#include <iostream>

using namespace cv;

Mat adaptiveHistogramEqualizer(Mat input, HistogramEqualizer* data = NULL);
void onChangeLightnessThreshold(int pos, void* userdata);
void onChangeSaturationThreshold(int pos, void* userdata);

Mat useCanny(Mat input, CannyFilter* data = NULL);
void onChangeFirstThreshold(int pos, void* userdata);
void onChangeRatio(int pos, void* userdata);

Mat useGaussian(Mat input, GaussianFilter* data = NULL);
void onSetKernelGF(int pos, void* userdata);
void onSetSigmaGF(int pos, void* userdata);

Mat useBilateral(Mat input, BilateralFilter* data = NULL);
void onSetSigmaRangeBF(int pos, void* userdata);
void onSetSigmaSpaceBF(int pos, void* userdata);

Mat useMorphOperator(Mat input, MorphologicOperator* data = NULL);
void onSetMorphOperator(int pos, void* userdata);
void onSetElement(int pos, void* userdata);
void onSetSize(int pos, void* userdata);
void onSetIterations(int pos, void* userdata);

Mat useSelectiveSearch(Mat input, bool fastSearch = true);

Mat useStaticSaliencyDetection(Mat input, bool spectralResidual = true);

Mat useWatershed(Mat input, Mat seeds);

void showRegions(Mat mask);
Mat createImageWithRegions(Mat mask);
std::vector<Rect> createRegions(Mat mask);
std::vector<Rect> getRegions(Mat mask);
std::vector<Rect> clustering(std::vector<Rect> regions);
std::vector<std::vector<Rect>> cluster_rects(const std::vector<Rect>& rects, const double th);
Rect union_of_rects(const std::vector<Rect>& cluster);

void testInitialaizer(std::string title);
void saveResult(Mat image);
void showSavedResult();

void testRegionProposal(Mat image);