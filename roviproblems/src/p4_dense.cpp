#include <iostream>

// rovi includes
#include "../includes/p4_dense.hpp"
#include "../includes/rw_camera.hpp"

#include <rw/rw.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/impl/transforms.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>

//#include <pcl/io/point_cloud_image_extractors.h>

const std::string experiments_path = "../experiment_data/p4_dense/";
const std::string images_path = "images/";
const std::string pcls_path = "pcls/";

const std::string imgL_path = experiments_path + images_path + "tsukuba_l.png";
const std::string imgR_path = experiments_path + images_path + "tsukuba_r.png";

using pclPoint = pcl::PointXYZRGBNormal;
using pclCloud = pcl::PointCloud<pclPoint>;
using FeatureT = pcl::FPFHSignature33;
using FeatureEstimationT = pcl::FPFHEstimation<pclPoint,pclPoint,FeatureT>;
using FeatureCloudT = pcl::PointCloud<FeatureT>;
using ColorHandlerT = pcl::visualization::PointCloudColorHandlerCustom<pclPoint>;

USE_ROBWORK_NAMESPACE
using namespace robwork;


void savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        for (size_t j = 0; j < points.cols; j++) {
            // Check if points are too far away
            // If not save them in point cloud
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < max_z) {
                pclPoint pn;
                pn.x = xyz[0];
                pn.y = xyz[1];
                pn.z = xyz[2];
                pn.r = bgr[2];
                pn.g = bgr[1];
                pn.b = bgr[0];
                dst->at(i, j) = pn;
            }
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}

cv::Mat disparity_map(cv::Mat imgL, cv::Mat imgR, int num_disparities = 16, int block_size = 3)
{
    auto sgbm = cv::StereoSGBM::create(0, num_disparities, block_size);
    cv::Mat disparity_map_;
    sgbm->compute(imgL, imgR, disparity_map_);
    return disparity_map_;
}

cv::Mat norm_mat8b(cv::Mat mat_)
{
    cv::Mat mat_norm;
    cv::normalize(mat_, mat_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    return mat_norm;
}

cv::Mat define_Q(int img_width, int img_height, double Tx, double f)
{
    // Define Q matrix as per document
    
    
    cv::Mat Q = cv::Mat::zeros(4,4,CV_64F);
    Q.at<double>(0,0) = 1;
    Q.at<double>(1,1) = 1;
    Q.at<double>(3,2) = -1./Tx; 
    Q.at<double>(0,3) = -img_width/2.;
    Q.at<double>(1,3) = -img_height/2.;
    Q.at<double>(2,3) = f;
    return Q;
}

cv::Mat reproject3D(cv::Mat disparity_map_, cv::Mat Q)
{
    cv::Mat mat_3Dimg;
    cv::reprojectImageTo3D(disparity_map_, mat_3Dimg, Q, true);
    return mat_3Dimg;
}



void dense_stereo(std::string imgL_path, std::string imgR_path, std::string object_path, double f, double Tx, double z_threshold)
{
    std::cout << "Dense stereo started " << std::endl;
    cv::Mat imgL = cv::imread(imgL_path);
    cv::Mat imgR = cv::imread(imgR_path);

    if (imgL.empty() || imgR.empty()) {
        std::cout << "Error loading the images" << std::endl;
        return;
    }

    cv::Mat colors = imgL;

    cv::cvtColor(imgL, imgL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgR, imgR, cv::COLOR_BGR2GRAY);

    cv::Mat disparity_map_ = disparity_map(imgL, imgR, 128, 7);

    disparity_map_ = norm_mat8b(disparity_map_);

    cv::imshow("test", disparity_map_);
    if ( (char) 27 == (char) cv::waitKey(0) ) {

    }

    cv::Mat Q = define_Q(imgL.cols, imgL.rows, Tx, f);

    cv::Mat points_scene = reproject3D(disparity_map_, Q);

    const std::string scene_name = "scene.pcd";
    savePointCloud(experiments_path + pcls_path + scene_name, points_scene, colors, z_threshold);

    pclCloud::Ptr pcl_scene(new pclCloud);

    // Replace the path below with the path where you saved your file
    pcl::io::loadPCDFile(experiments_path + pcls_path + scene_name, *pcl_scene);

    pclCloud::Ptr pcl_scene_filtered(new pclCloud);    
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pclPoint> sor;
    sor.setInputCloud(pcl_scene);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter (*pcl_scene_filtered);

    const std::string scene_filtered_name = "scene_filtered.pcd";

    pcl::io::savePCDFileASCII(experiments_path + pcls_path + scene_filtered_name, *pcl_scene_filtered);

    pclCloud::Ptr pcl_scene_vox(new pclCloud); 
    const float leaf = 0.001f;
    // Create the filtering object
    pcl::VoxelGrid<pclPoint> vox;
    vox.setInputCloud (pcl_scene_filtered);
    vox.setLeafSize (leaf, leaf, leaf);
    vox.filter (*pcl_scene_vox);

    // Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimation<pclPoint,pclPoint> nest;
    nest.setKSearch(10);
    nest.setInputCloud (pcl_scene_vox);
    nest.compute (*pcl_scene_vox);

    // Estimate features
    pclCloud::Ptr object (new pclCloud);

    // Replace the path below with the path where you saved your file
    pcl::io::loadPCDFile(object_path, *object);

    vox.setInputCloud (object);
    vox.filter (*object);

    pclCloud::Ptr object_aligned (new pclCloud);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.025);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    
    fest.setInputCloud (pcl_scene_vox);
    fest.setInputNormals (pcl_scene_vox);
    fest.compute (*scene_features);

    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<pclPoint,pclPoint,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (pcl_scene_vox);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
    }

    if (align.hasConverged ())
    {
        // Print results
        printf ("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers().size (), object->size ());
        
        // Show alignment
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (pcl_scene_vox, ColorHandlerT (pcl_scene_vox, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
    }

    // Create a k-d tree for scene
    pcl::search::KdTree<pclPoint> tree;
    tree.setInputCloud(pcl_scene_vox);

    pclCloud::Ptr cloud_icp (new pclCloud(*object_aligned, align.getInliers()));
    // Compute inliers and RMSE
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;
    tree.nearestKSearch(*cloud_icp, std::vector<int>(), 1, idx, distsq);
    float rmse = 0;

    for(size_t i = 0; i < distsq.size(); ++i)
            rmse += distsq[i][0];
    rmse = sqrtf(rmse / cloud_icp->size());

    std::cout << "RMSE " << rmse << std::endl;

    size_t iterations = 50000;
    pcl::IterativeClosestPoint<pclPoint, pclPoint> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (pcl_scene_vox);
    {
        pcl::ScopeTime t("ICP alignment");
        icp.align (*cloud_icp);
    }

    if (icp.hasConverged ())
    {
        // Print results
        printf ("\n");
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        
        //Transform cloud
        pcl::transformPointCloud(*object_aligned, *object_aligned, transformation);

        // Show alignment
        pcl::visualization::PCLVisualizer visu("ICP alignment");
        visu.addPointCloud (pcl_scene_vox, ColorHandlerT (pcl_scene_vox, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }

    pclCloud::Ptr cloud_final (new pclCloud(*object_aligned, align.getInliers()));
    // Compute inliers and RMSE
    std::vector<std::vector<int> > idx_final;
    std::vector<std::vector<float> > distsq_final;
    tree.nearestKSearch(*cloud_final, std::vector<int>(), 1, idx_final, distsq_final);
    float rmse_final = 0;

    for(size_t i = 0; i < distsq_final.size(); ++i)
            rmse_final += distsq_final[i][0];
    rmse_final = sqrtf(rmse_final / cloud_final->size());

    std::cout << "RMSE post ICP " << rmse_final << std::endl;

    pclPoint centroid;
    pcl::computeCentroid(*cloud_final, centroid);

    std::cout << centroid << std::endl;


    return;
}

void test()
{

    Log::infoLog() << "Running test on dense stereo..." << std::endl;

    //Loading workcell
    Log::infoLog() << "Loading workcell..." << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("../workcell/Scene.wc.xml");
    if(wc == nullptr)
    {
        Log::errorLog() << "Workcell could not be loaded from " << "../workcell/Scene.wc.xml" << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Workcell could not be loaded from ../workcell/Scene.wc.xml");
    }
    Log::infoLog() << "Workcell has been loaded succesfully" << std::endl;

    State state = wc->getDefaultState();

    pclCloud::Ptr object(new pclCloud);
    
    pcl::io::loadOBJFile(std::string("../workcell/parts/bottle.stl"), *object);
    pcl::io::savePCDFileASCII("../experiment_data/p4_dense/pcls/object.pcd", *object);
    /*
    
    
    generateImages(wc, state);

    double Tx = 0.02;
    double f = 514.682;
    double z_threshold = 5000;

    std::string imgL_path = "../experiment_data/p4_dense/images/Camera_left.PPN";
    std::string imgR_path = "../experiment_data/p4_dense/images/Camera_right.PPN";
    std::string object_path = "../experiment_data/p4_dense/pcls/object.pcd";
    
    dense_stereo(object_path, f, Tx, z_threshold);
    */
}