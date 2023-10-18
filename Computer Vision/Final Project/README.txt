PROGRAM: boat_generator

This program try to detect one o more images and then it compare its results with a ground truth

It accepts only one parameter and it's a folder containing:
	- folder with one o more images (any kind of extension)
	- ground_truth.txt file
	- positive.yml
	- negative.yml

If any of these files (images directory included) is missing, the program will end with unexpected results.

----------------------------------------------------------------------------------------------------------------

PROGRAM: BoWGenerator

This program generates a Bag of Word (of SIFT features) from a collection of images.

It accepts only one parameter: a folder containing the images dataset.
If the parameter is not a folder containing images, it will end with unexpected results.

----------------------------------------------------------------------------------------------------------------

NOTE THAT: in the test directory some BoW are already generated from the provided dataset.

----------------------------------------------------------------------------------------------------------------
 GROUND TRUTH

The ground truth file has been built by using the OpenCV app "opencv_annotations", then this program use the same specifics
to read the ground truth file:

folder/image.ext n_samples x y width height

Example:

test/01.jpg 2 0 0 20 20
test/02.png 0
test/09.jpg 1 20 20 400 500