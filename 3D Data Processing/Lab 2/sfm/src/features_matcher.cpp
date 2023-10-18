#include "features_matcher.h"

#include <iostream>
#include <map>
#include <opencv2/core/traits.hpp>

namespace {
    template<typename T>
    void FscanfOrDie(FILE *fptr, const char *format, T *value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            std::cerr << "Invalid UW data file.";
            exit(-1);
        }
    }
}

FeatureMatcher::FeatureMatcher(cv::Mat intrinsics_matrix, cv::Mat dist_coeffs, double focal_scale) {
    intrinsics_matrix_ = intrinsics_matrix.clone();
    dist_coeffs_ = dist_coeffs.clone();
    new_intrinsics_matrix_ = intrinsics_matrix.clone();
    new_intrinsics_matrix_.at<double>(0, 0) *= focal_scale;
    new_intrinsics_matrix_.at<double>(1, 1) *= focal_scale;
}

cv::Mat FeatureMatcher::readUndistortedImage(const std::string &filename) {
    cv::Mat img = cv::imread(filename), und_img, dbg_img;
    cv::undistort(img, und_img, intrinsics_matrix_, dist_coeffs_, new_intrinsics_matrix_);

    return und_img;
}

void FeatureMatcher::extractFeatures() {
    features_.resize(images_names_.size());
    descriptors_.resize(images_names_.size());
    feats_colors_.resize(images_names_.size());


    for (int i = 0; i < images_names_.size(); i++) {
        std::cout << "Computing descriptors for image " << i << std::endl;
        cv::Mat img = readUndistortedImage(images_names_[i]);

        //////////////////////////// Code to be completed (1/5) /////////////////////////////////
        // Extract salient points + descriptors from i-th image, and store them into
        // features_[i] and descriptors_[i] vector, respectively
        // Extract also the color (i.e., the cv::Vec3b information) of each feature, and store
        // it into feats_colors_[i] vector
        /////////////////////////////////////////////////////////////////////////////////////////

        // initialize AKAZE detector and descriptor
        cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
        detector->setThreshold(1e-4);
        detector->setNOctaves(10);
        detector->setNOctaveLayers(20);

        // detect keypoints and their descriptors
        detector->detectAndCompute(img, cv::noArray(), features_[i], descriptors_[i], false);

        // for each keypoint retrieve its corresponding color
        std::vector<cv::Vec3b> keypoints_color;
        for (int j = 0; j < features_[i].size(); j++) {
            // retrieve the keypoints coords
            cv::Point2f keypoint_coords = features_[i][j].pt;
            // obtain the color in the img
            cv::Vec3b keypoint_color = img.at<cv::Vec3b>(keypoint_coords);
            keypoints_color.push_back(keypoint_color);
        }
        // save the color values
        feats_colors_[i] = keypoints_color;
    }
}

void FeatureMatcher::exhaustiveMatching() {
    for (int i = 0; i < images_names_.size() - 1; i++) {
        for (int j = i + 1; j < images_names_.size(); j++) {
            std::cout << "Matching image " << i << " with image " << j << std::endl;
            std::vector<cv::DMatch> matches, inlier_matches;

            //////////////////////////// Code to be completed (2/5) /////////////////////////////////
            // Match descriptors between image i and image j, and perform geometric validation,
            // possibly discarding the outliers (remember that features have been extracted
            // from undistorted images that now has  new_intrinsics_matrix_ as K matrix and
            // no distortions)
            // As geometric models, use both the Essential matrix and the Homograph matrix,
            // both by setting new_intrinsics_matrix_ as K matrix
            // Do not set matches between two images if the amount of inliers matches
            // (i.e., geomatrically verified matches) is small (say <= 10 matches)
            /////////////////////////////////////////////////////////////////////////////////////////

            // create the matcher
            cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
            //cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);

            // match the features
            matcher->match(descriptors_[i], descriptors_[j], matches, cv::noArray());
            //DEGUG: std::cout << "  [INFO] Matches " << matches.size() << std::endl;

            // retrieve point coordinates from the matches
            std::vector<cv::Point2f> src_points, dst_points;
            for (int k = 0; k < matches.size(); k++) {
                src_points.push_back(features_[i][matches[k].queryIdx].pt);
                dst_points.push_back(features_[j][matches[k].trainIdx].pt);
            }

            //definition of the masks used to compute the inliers
            cv::Mat inlier_H, inlier_E;

            // compute the homography using RANSAC
            //DEGUG: std::cout << "  [INFO] Calculate homography" << std::endl;
            cv::findHomography(src_points, dst_points, inlier_H, cv::RANSAC, 1.0);

            // compute the essential using RANSAC
            //DEGUG: std::cout << "  [INFO] Calculate essential" << std::endl;
            cv::findEssentialMat(src_points, dst_points, new_intrinsics_matrix_, cv::RANSAC, 0.999,
                                 1.0, inlier_E);

            // number of inliers found from the two models (homography and essential)
            int tot_inlier_E = cv::sum(inlier_E)[0];
            int tot_inlier_H = cv::sum(inlier_H)[0];

            //if we have more than 10 inliers for E
            if (tot_inlier_E > tot_inlier_H && tot_inlier_E > 10){
                //DEGUG: std::cout << "  [INFO] Choose the best: ESSENTIAL with " << std::to_string(tot_inlier_E)
                //DEGUG: << " inliers against " << std::to_string(tot_inlier_H) << std::endl;

                // take as match only the inliers
                for (int id_match = 0; id_match < src_points.size(); id_match++) {
                    if (inlier_E.at<uchar>(id_match)) {
                        inlier_matches.push_back(matches[id_match]);
                    }
                }
            }
            // if we have more than 10 inliers for H
            else if(tot_inlier_H > 10){
                //DEGUG: std::cout << "  [INFO] Choose the best: HOMOGRAPHY with " << std::to_string(tot_inlier_H)
                //DEGUG: << " inliers against " << std::to_string(tot_inlier_E) << std::endl;
                // take as match only the inliers
                for (int id_match = 0; id_match < src_points.size(); id_match++) {
                    if (inlier_H.at<uchar>(id_match)) {
                        inlier_matches.push_back(matches[id_match]);
                    }
                }
            }
            // we don't have more than 10 inliers for E neither for H
            else{
                //DEGUG: std::cout << "  [INFO] NO MODEL HAS BEEN CHOSEN -> Essential: " << tot_inlier_E
                //DEGUG:  << " inliers | Homography:  " << tot_inlier_H << "inliers" << std::endl;
                inlier_matches = {};
            }

            setMatches(i, j, inlier_matches);
        }
    }
}


void FeatureMatcher::writeToFile(const std::string &filename, bool normalize_points) const {
    FILE *fptr = fopen(filename.c_str(), "w");

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    };

    fprintf(fptr, "%d %d %d\n", num_poses_, num_points_, num_observations_);

    double *tmp_observations;
    cv::Mat dst_pts;
    if (normalize_points) {
        cv::Mat src_obs(num_observations_, 1, cv::traits::Type<cv::Vec2d>::value,
                        const_cast<double *>(observations_.data()));
        cv::undistortPoints(src_obs, dst_pts, new_intrinsics_matrix_, cv::Mat());
        tmp_observations = reinterpret_cast<double *>(dst_pts.data);
    } else {
        tmp_observations = const_cast<double *>(observations_.data());
    }

    for (int i = 0; i < num_observations_; ++i) {
        fprintf(fptr, "%d %d", pose_index_[i], point_index_[i]);
        for (int j = 0; j < 2; ++j) {
            fprintf(fptr, " %g", tmp_observations[2 * i + j]);
        }
        fprintf(fptr, "\n");
    }

    if (colors_.size() == 3 * num_points_) {
        for (int i = 0; i < num_points_; ++i)
            fprintf(fptr, "%d %d %d\n", colors_[i * 3], colors_[i * 3 + 1], colors_[i * 3 + 2]);
    }

    fclose(fptr);
}

void FeatureMatcher::testMatches(double scale) {
    // For each pose, prepare a map that reports the pairs [point index, observation index]
    std::vector<std::map<int, int> > cam_observation(num_poses_);
    for (int i_obs = 0; i_obs < num_observations_; i_obs++) {
        int i_cam = pose_index_[i_obs], i_pt = point_index_[i_obs];
        cam_observation[i_cam][i_pt] = i_obs;
    }

    for (int r = 0; r < num_poses_; r++) {
        for (int c = r + 1; c < num_poses_; c++) {
            int num_mathces = 0;
            std::vector<cv::DMatch> matches;
            std::vector<cv::KeyPoint> features0, features1;
            for (auto const &co_iter: cam_observation[r]) {
                if (cam_observation[c].find(co_iter.first) != cam_observation[c].end()) {
                    features0.emplace_back(observations_[2 * co_iter.second], observations_[2 * co_iter.second + 1],
                                           0.0);
                    features1.emplace_back(observations_[2 * cam_observation[c][co_iter.first]],
                                           observations_[2 * cam_observation[c][co_iter.first] + 1], 0.0);
                    matches.emplace_back(num_mathces, num_mathces, 0);
                    num_mathces++;
                }
            }
            cv::Mat img0 = readUndistortedImage(images_names_[r]),
                    img1 = readUndistortedImage(images_names_[c]),
                    dbg_img;

            cv::drawMatches(img0, features0, img1, features1, matches, dbg_img);
            cv::resize(dbg_img, dbg_img, cv::Size(), scale, scale);
            cv::Mat resize_image;
            cv::resize(dbg_img, resize_image, cv::Size(dbg_img.cols * 2 / 3, dbg_img.rows * 2 / 3));
            cv::imshow("", resize_image);
            if (cv::waitKey() == 27)
                return;
        }
    }
}

void FeatureMatcher::setMatches(int pos0_id, int pos1_id, const std::vector<cv::DMatch> &matches) {

    const auto &features0 = features_[pos0_id];
    const auto &features1 = features_[pos1_id];

    auto pos_iter0 = pose_id_map_.find(pos0_id),
            pos_iter1 = pose_id_map_.find(pos1_id);

    // Already included position?
    if (pos_iter0 == pose_id_map_.end()) {
        pose_id_map_[pos0_id] = num_poses_;
        pos0_id = num_poses_++;
    } else
        pos0_id = pose_id_map_[pos0_id];

    // Already included position?
    if (pos_iter1 == pose_id_map_.end()) {
        pose_id_map_[pos1_id] = num_poses_;
        pos1_id = num_poses_++;
    } else
        pos1_id = pose_id_map_[pos1_id];

    for (auto &match: matches) {

        // Already included observations?
        uint64_t obs_id0 = poseFeatPairID(pos0_id, match.queryIdx),
                obs_id1 = poseFeatPairID(pos1_id, match.trainIdx);
        auto pt_iter0 = point_id_map_.find(obs_id0),
                pt_iter1 = point_id_map_.find(obs_id1);
        // New point
        if (pt_iter0 == point_id_map_.end() && pt_iter1 == point_id_map_.end()) {
            int pt_idx = num_points_++;
            point_id_map_[obs_id0] = point_id_map_[obs_id1] = pt_idx;

            point_index_.push_back(pt_idx);
            point_index_.push_back(pt_idx);
            pose_index_.push_back(pos0_id);
            pose_index_.push_back(pos1_id);
            observations_.push_back(features0[match.queryIdx].pt.x);
            observations_.push_back(features0[match.queryIdx].pt.y);
            observations_.push_back(features1[match.trainIdx].pt.x);
            observations_.push_back(features1[match.trainIdx].pt.y);

            // Average color between two corresponding features (suboptimal since we shouls also consider
            // the other observations of the same point in the other images)
            cv::Vec3f color = (cv::Vec3f(feats_colors_[pos0_id][match.queryIdx]) +
                               cv::Vec3f(feats_colors_[pos1_id][match.trainIdx])) / 2;

            colors_.push_back(cvRound(color[2]));
            colors_.push_back(cvRound(color[1]));
            colors_.push_back(cvRound(color[0]));

            num_observations_++;
            num_observations_++;
        }
            // New observation
        else if (pt_iter0 == point_id_map_.end()) {
            int pt_idx = point_id_map_[obs_id1];
            point_id_map_[obs_id0] = pt_idx;

            point_index_.push_back(pt_idx);
            pose_index_.push_back(pos0_id);
            observations_.push_back(features0[match.queryIdx].pt.x);
            observations_.push_back(features0[match.queryIdx].pt.y);
            num_observations_++;
        } else if (pt_iter1 == point_id_map_.end()) {
            int pt_idx = point_id_map_[obs_id0];
            point_id_map_[obs_id1] = pt_idx;

            point_index_.push_back(pt_idx);
            pose_index_.push_back(pos1_id);
            observations_.push_back(features1[match.trainIdx].pt.x);
            observations_.push_back(features1[match.trainIdx].pt.y);
            num_observations_++;
        }
//    else if( pt_iter0->second != pt_iter1->second )
//    {
//      std::cerr<<"Shared observations does not share 3D point!"<<std::endl;
//    }
    }
}

void FeatureMatcher::reset() {
    point_index_.clear();
    pose_index_.clear();
    observations_.clear();
    colors_.clear();

    num_poses_ = num_points_ = num_observations_ = 0;
}