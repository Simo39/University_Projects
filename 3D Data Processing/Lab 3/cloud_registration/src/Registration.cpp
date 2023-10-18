#include <chrono>
#include "Registration.h"


Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename) {
    // TO COMPLETEs
    open3d::io::ReadPointCloud(cloud_source_filename, source_);
    open3d::io::ReadPointCloud(cloud_target_filename, target_);
}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target) {
    // TO COMPLETE
    source_ = cloud_source;
    target_ = cloud_target;
}


void Registration::draw_registration_result() {
    //visualize target and source with two different colors
    // TO COMPLETE
    //copy source and target in temporary variables in order to protect the original points cloud
    open3d::geometry::PointCloud source_temp, target_temp;
    source_temp = source_;
    target_temp = target_;
    //setting color: blue for source and red for target
    source_temp.PaintUniformColor(Eigen::Vector3d(1, 0, 0));
    target_temp.PaintUniformColor(Eigen::Vector3d(0, 0, 1));
    //apply transformation
    source_temp = source_temp.Transform(transformation_);
    //show point clouds
    auto source_ptr = std::make_shared<open3d::geometry::PointCloud>(source_temp);
    auto target_ptr = std::make_shared<open3d::geometry::PointCloud>(target_temp);
    open3d::visualization::DrawGeometries({source_ptr, target_ptr});
}


void Registration::preprocess(open3d::geometry::PointCloud pcd, double voxel_size,
                              std::shared_ptr<open3d::geometry::PointCloud> &pcd_down_ptr,
                              std::shared_ptr<open3d::pipelines::registration::Feature> &pcd_fpfh) {
    //downsample, estimate normals and compute FPFH features

    // TO COMPLETE

    //downsampling
    pcd_down_ptr = pcd.VoxelDownSample(voxel_size);

    //normals estimation
    double radius = voxel_size * 2;
    pcd_down_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius, 30));

    //FPFH features computating
    double radius_feature = radius * 5;
    pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_down_ptr,
                                                                   open3d::geometry::KDTreeSearchParamHybrid(
                                                                           radius_feature,
                                                                           100));

    return;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_global_registration(double voxel_size) {
    // remember to apply the transformation_ matrix to source_cloud
    // create two point cloud to contain the downsampled point cloud and two structure to contain the features
    // call the Registration::preprocess function on target and transformed source
    // execute global transformation and update the transformation matrix
    // TO COMPLETE

    std::shared_ptr<open3d::geometry::PointCloud> source_down, target_down;
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh, target_fpfh;

    //transform the source cloud
    open3d::geometry::PointCloud source_transformed = source_.Transform(transformation_);

    //preprocess source and target and compute their features
    preprocess(source_transformed, voxel_size, source_down, source_fpfh);
    preprocess(target_, voxel_size, target_down, target_fpfh);

    open3d::pipelines::registration::RegistrationResult result;

    double distance_threshold = 0.0;

    //variable to choose among FastGlobalRegistration or RegistrationRansac
    bool fast = true;

    //FAST GLOBAL REGISTRATION
    if (fast) {
        //set the distance_threshold for the registration
        distance_threshold = voxel_size * 0.5;

        //timer start
        auto t_start = std::chrono::high_resolution_clock::now();

        //compute the registration
        result = open3d::pipelines::registration::FastGlobalRegistrationBasedOnFeatureMatching(
                *source_down, *target_down,
                *source_fpfh, *target_fpfh,
                open3d::pipelines::registration::FastGlobalRegistrationOption(1.4, false, true, distance_threshold)
        );

        //timer stop
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << "\n\t\t[INFO] Elapsed time for Registration with Fast Global: "
                  << std::chrono::duration<double, std::milli>(t_end - t_start).count()
                  << " ms" << std::endl;
    }
    //RANSAC REGISTRATION METHOD
    else {
        //set the distance_threshold for the registration
        distance_threshold = voxel_size * 1.5;

        //set some parameter to provide them later on to the RANSAC registration algorithm
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength checkers_1(0.9);
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance checkers_2(distance_threshold);

        std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker >> checkers;
        checkers.push_back(
                std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>(checkers_1));
        checkers.push_back(
                std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>(checkers_2));

        //timer start
        auto t_start = std::chrono::high_resolution_clock::now();

        //compute the registration
        result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
                *source_down, *target_down, *source_fpfh, *target_fpfh,
                true, distance_threshold,
                open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 3,
                static_cast<const std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> &> (checkers)
        );

        //timer stop
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << "\n\t\t[INFO] Elapsed time for Registration with RANSAC: "
                  << std::chrono::duration<double, std::milli>(t_end - t_start).count()
                  << " ms" << std::endl;
    }

    //update the transformation matrix
    transformation_ = result.transformation_;

    return result;
}

open3d::pipelines::registration::RegistrationResult
Registration::execute_icp_registration(double threshold, double relative_fitness, double relative_rmse,
                                       int max_iteration) {
    // To COMPLETE
    open3d::pipelines::registration::RegistrationResult result;
    //compute the registration with ICP
    result = open3d::pipelines::registration::RegistrationICP(source_, target_, threshold, transformation_,
                                                              open3d::pipelines::registration::TransformationEstimationPointToPoint(
                                                                      false),
                                                              open3d::pipelines::registration::ICPConvergenceCriteria(
                                                                      relative_fitness, relative_rmse, max_iteration));

    //update the transformation matrix
    transformation_ = result.transformation_;

    return result;
}


void Registration::set_transformation(Eigen::Matrix4d init_transformation) {
    transformation_ = init_transformation;
}


Eigen::Matrix4d Registration::get_transformation() {
    return transformation_;
}


void Registration::write_tranformation_matrix(std::string filename) {
    std::ofstream outfile(filename);
    if (outfile.is_open()) {
        outfile << transformation_;
        outfile.close();
    }
}

void Registration::save_merged_cloud(std::string filename) {
    //clone input
    open3d::geometry::PointCloud source_clone = source_;
    open3d::geometry::PointCloud target_clone = target_;

    source_clone.Transform(transformation_);
    open3d::geometry::PointCloud merged = target_clone + source_clone;
    open3d::io::WritePointCloud(filename, merged);
}

