#ifndef LAB5_PANORAMIC_IMAGE_H
#define LAB5_PANORAMIC_IMAGE_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

/*
* 
* PanoramicImage class.
* 
* This class takes an image file list and the field of view with which the images have been taken
* and it composes the images in a unique panoramic image.
* 
* It's possible to choose 2 methods to compose the panoramic image:
*		- SIFT method
*		- ORB  method
* 
*/


class PanoramicImage {
public:
	/*
	* Constructor
	* 
	* It takes the image file list and the field of view to reproject each image into the cylinder.
	*/
	PanoramicImage(std::vector<cv::String> file_list, double field_of_view);
	//return the panoramic image computed with SIFT
	cv::Mat composeWithSIFT(float ratio_thresold = 4);
	//return the panoramic image computed with ORB
	cv::Mat composeWithORB(float ratio_thresold = 4);
	//return the last composed image
	cv::Mat getImage();
	//check if there is a composed image
	bool isEmpty();
private:
	//class variables
	//resulting image
	cv::Mat panoramic_image;
	//supporting variables to compute the panoramic image
	std::vector<cv::Mat> projectedImages;
	std::vector<std::vector<cv::KeyPoint>> keypoints;
	std::vector<std::vector<cv::DMatch>> matches;
	std::vector<float> x_translations;

	//features extraction and matching with consecutive images
	void extractAndMatch_SIFT();
	void extractAndMatch_ORB();
	//extract and compute keypoints and their descriptors
	template <typename T> void detect(cv::Ptr<T>& detector,	std::vector<cv::Mat>& descriptors);
	//match the keypoints among pairs of images
	template <typename T> void match(cv::Ptr<T>& matcher, std::vector<cv::Mat>& descriptors);
	//compute the average translation for each pair of image.
	void computeTranslations(float ratio_thresold);
	//compose the panoramic image from the computed translations
	void compose();
	//reset the computed panoramic image to generate a new one
	void reset();
};

#endif //LAB5_PANORAMIC_IMAGE_H
