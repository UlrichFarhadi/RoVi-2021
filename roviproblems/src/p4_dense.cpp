/*
Table x = -39.750 : 39.750
Table y = 29.750 : 59.750

*/

#include <iostream>
#include <fstream>

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
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/common/impl/transforms.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>

// Robworks includes
#include <rw/rw.hpp>
#include <rw/geometry.hpp>

using namespace rw::math;
using namespace rw::geometry;

//#include <pcl/io/point_cloud_image_extractors.h>

const std::string experiments_path = "../experiment_data/p4_dense/";
const std::string images_path = "images/";
const std::string pcls_path = "pcls/";

const std::string imgL_path = experiments_path + images_path + "tsukuba_l.png";
const std::string imgR_path = experiments_path + images_path + "tsukuba_r.png";

using pclPoint = pcl::PointXYZRGBNormal;
using pclCloud = pcl::PointCloud<pclPoint>;
using FeatureT = pcl::FPFHSignature33;
//using FeatureT = pcl::Histogram<153>;
//using FeatureEstimationT = pcl::SpinImageEstimation<pclPoint,pclPoint,FeatureT>;
using FeatureEstimationT = pcl::FPFHEstimation<pclPoint,pclPoint,FeatureT>;
using FeatureCloudT = pcl::PointCloud<FeatureT>;
using ColorHandlerT = pcl::visualization::PointCloudColorHandlerCustom<pclPoint>;

USE_ROBWORK_NAMESPACE
using namespace robwork;

void spatialFilter(pclCloud::Ptr input_cloud, pclCloud::Ptr &output_cloud, std::string filterFieldName, float min, float max)
{
    pcl::PassThrough<pclPoint> temp;
    temp.setInputCloud(input_cloud);
    temp.setFilterFieldName(filterFieldName);
    temp.setFilterLimits(min, max);
    temp.filter(*output_cloud);
}

pclCloud::Ptr filter_PointCloud(pclCloud::Ptr cloud_ptr_in, rw::geometry::Plane plane, const double MAX_DISTANCE)
{
    double largest_distance = 0;
    pclCloud::Ptr new_cloud(new pclCloud());
    for(auto iter = cloud_ptr_in->begin(); iter != cloud_ptr_in->end(); iter++)
    {
        rw::math::Vector3D<> point(iter->x, iter->y, iter->z);
        if(plane.distance(point) > largest_distance)
            largest_distance = plane.distance(point);
        if(plane.distance(point) <= MAX_DISTANCE)
        {
            new_cloud->push_back(*iter);
        }
    }
    std::cout << largest_distance << std::endl;
    return new_cloud;
}

void scale_PointCloud(pclCloud::Ptr cloud, double scale)
{
    for(auto iter = cloud->begin(); iter != cloud->end(); ++iter)
    {
        iter->x = iter->x * scale;
        iter->y = iter->y * scale;
        iter->z = iter->z * scale;
    }
    return;
}

void computePointCloudCentroid(pclCloud::Ptr cloud, pclPoint &centroid)
{
    pclPoint temp;
    temp.x = 0;
    temp.y = 0;
    temp.z = 0;
    for(auto iter = cloud->begin(); iter != cloud->end(); ++iter)
    {
        temp.x += iter->x;
        temp.y += iter->y;
        temp.z += iter->z;
    }
    temp.x = temp.x / cloud->size();
    temp.y = temp.y / cloud->size();
    temp.z = temp.z / cloud->size();
    centroid = temp;
    return;
}

void write_PointCloud_CSV(pclCloud::Ptr cloud, std::string path)
{
    std::ofstream f(path);
    for(auto iter = cloud->begin(); iter != cloud->end(); ++iter)
    {
        f << iter->x << "," << iter->y << "," << iter->z << std::endl;
    }
    f.close();
}
pclCloud::Ptr makePointCloud(cv::Mat points, cv::Mat colors, double max_z)
{
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
    return dst;
}

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
    //auto sgbm = cv::StereoSGBM::create(0, num_disparities, block_size);
    auto sbm = cv::StereoBM::create(num_disparities, block_size);
    cv::Mat disparity_map_;
    sbm->compute(imgL,imgR,disparity_map_);
    disparity_map_ *= 1./16.;
    //sgbm->compute(imgL, imgR, disparity_map_);
    return disparity_map_;
}

cv::Mat norm_mat8b(cv::Mat mat_)
{
    cv::Mat mat_norm;
    cv::normalize(mat_, mat_norm, 0, 255, cv::NORM_MINMAX);
    mat_norm.convertTo(mat_norm, CV_8UC3);
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

Transform3D<> convertEigenToRW(const Eigen::Matrix4f &e)
{
    Transform3D<> T;
    for(int i = 0; i < e.rows()-1; ++i)
    {
        for(int j = 0; j < e.cols(); ++j)
        {
            T(i,j) = e(i,j);
        }
    }
    return T;
}

void dense_stereo(std::string imgL_path, std::string imgR_path, std::string object_path, double f, double Tx, Transform3D<> T_WC)
{
    assert(thresholds.size() == 3);
    std::cout << "Dense stereo started " << std::endl;
    cv::Mat imgL = cv::imread(imgL_path);
    cv::Mat imgR = cv::imread(imgR_path);

    std::cout << "Images loaded" << std::endl;

    if (imgL.empty() || imgR.empty()) {
        std::cout << "Error loading the images" << std::endl;
        return;
    }

    cv::Mat colors = imgL;

    cv::cvtColor(imgL, imgL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgR, imgR, cv::COLOR_BGR2GRAY);

    cv::Mat disparity_map_;
    /*std::cout << "Try different stereo parameters" << std::endl;
    while (true) {
        int nDisparities, BlockSize;
        std::cout << "Choose nDisparities: ";
        std::cin >> nDisparities;
        std::cout << "Choose SADWindowSize: ";
        std::cin >> BlockSize;

        std::cout << "Using nDisparities " << nDisparities << " and Blocksize " << BlockSize << std::endl;

        disparity_map_ = disparity_map(imgL, imgR, nDisparities, BlockSize);

        disparity_map_ = norm_mat8b(disparity_map_);

        cv::imshow("Stereo Disparity", disparity_map_);
        // Press esc to choose settings
        if ( (char) 27 == (char) cv::waitKey(0) ) {
            break;
        }

    }*/

    disparity_map_ = disparity_map(imgL, imgR, 160, 5);
    std::cout << "Disparity map computed" << std::endl;

    cv::Mat visualise_disparity_map = norm_mat8b(disparity_map_);
    
    /*cv::imshow("Disparity map", visualise_disparity_map);
    while (true)
    {
        if ( (char) 27 == (char) cv::waitKey(0) ) {
            break;
        }
    }*/

    cv::Mat Q = define_Q(imgL.cols, imgL.rows, Tx, f);

    cv::Mat points_scene = reproject3D(disparity_map_, Q);
    std::cout << "Repojection computed" << std::endl;

    pclCloud::Ptr pcl_scene = makePointCloud(points_scene, colors, 3);
    std::cout << "Pointcloud computed" << std::endl;

    /*pcl::transformPointCloud(*pcl_scene, *pcl_scene, T_WC.e());
    std::cout << "Pointcloud transformed" << std::endl;*/

    pclCloud::Ptr z_axis(new pclCloud);
    pclCloud::Ptr y_axis(new pclCloud);
    pclCloud::Ptr x_axis(new pclCloud);
    for(double i = 0; i <= 0.2; i+= 0.005f)
    {
        pclPoint z;
        z.x = 0;
        z.y = 0;
        z.z = i;

        pclPoint y;
        y.x = 0;
        y.y = i;
        y.z = 0;

        pclPoint x;
        x.x = i;
        x.y = 0;
        x.z = 0;

        z_axis->push_back(z);
        y_axis->push_back(y);
        x_axis->push_back(x);

    }
    
    std::cout << "World frame Pointcloud created" << std::endl;
    pcl::visualization::PCLVisualizer visu_un("UnFiltered");
    visu_un.addPointCloud (pcl_scene, ColorHandlerT (pcl_scene, 255.0, 255.0, 0.0), "Unfiltered");
    visu_un.addPointCloud (z_axis, ColorHandlerT (z_axis, 0.0, 0.0, 255.0), "z");
    visu_un.addPointCloud (y_axis, ColorHandlerT (y_axis, 0.0, 255.0, 0.0), "y");
    visu_un.addPointCloud (x_axis, ColorHandlerT (x_axis, 255.0, 0.0, 0.0), "x");
    visu_un.spin ();

    pclCloud::Ptr pcl_scene_vox(new pclCloud); 
    const float leaf = 0.005f;
    // Create the filtering object
    pcl::VoxelGrid<pclPoint> vox;
    vox.setInputCloud (pcl_scene);
    vox.setLeafSize (leaf, leaf, leaf);
    vox.filter (*pcl_scene_vox);
    std::cout << "Vox for scene performed" << std::endl;    

    /*pcl::visualization::PCLVisualizer visu_vox("Vox scene");
    visu_vox.addPointCloud (pcl_scene_vox, ColorHandlerT (pcl_scene_vox, 0.0, 255.0, 0.0), "vox");
    visu_vox.spin();*/

    spatialFilter(pcl_scene_vox, pcl_scene_vox, "z", -0.9, 0);
    spatialFilter(pcl_scene_vox, pcl_scene_vox, "x", -0.1, 0.1);
    spatialFilter(pcl_scene_vox, pcl_scene_vox, "y", -1, 0.150);

    std::cout << "Starting filtering" << std::endl;
    pclCloud::Ptr pcl_scene_filtered(new pclCloud);    
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pclPoint> sor;
    sor.setInputCloud(pcl_scene_vox);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter (*pcl_scene_filtered);
    std::cout << "Filtered. Points remaining " << pcl_scene_filtered->size() << std::endl;
    write_PointCloud_CSV(pcl_scene_filtered,"../experiment_data/p4_dense/pcls/scene_points.csv");



    
    /*while(true)
    {
        int zl, zh;
        std::cout << "z_l: " << std::endl;
        std::cin >> zl;
        std::cout << "z_h: " << std::endl;
        std::cin >> zh;

        spatialFilter(pcl_scene_filtered, pcl_scene_filtered, "z", zl, zh);

        pcl::visualization::PCLVisualizer visu("Filtered");
        visu.addPointCloud (pcl_scene_filtered, ColorHandlerT (pcl_scene_filtered, 255.0, 255.0, 0.0), "filtered");
        visu.addPointCloud (z_axis, ColorHandlerT (z_axis, 0.0, 0.0, 255.0), "z");
        visu.addPointCloud (y_axis, ColorHandlerT (y_axis, 0.0, 255.0, 0.0), "y");
        visu.addPointCloud (x_axis, ColorHandlerT (x_axis, 255.0, 0.0, 0.0), "x");
        visu.spin ();
    }*/
    pcl::visualization::PCLVisualizer visu_post("Filtered");
    visu_post.addPointCloud (pcl_scene_filtered, ColorHandlerT (pcl_scene_filtered, 255.0, 255.0, 0.0), "filtered");
    visu_post.addPointCloud (z_axis, ColorHandlerT (z_axis, 0.0, 0.0, 255.0), "z");
    visu_post.addPointCloud (y_axis, ColorHandlerT (y_axis, 0.0, 255.0, 0.0), "y");
    visu_post.addPointCloud (x_axis, ColorHandlerT (x_axis, 255.0, 0.0, 0.0), "x");
    visu_post.spin ();
    const std::string scene_filtered_name = "scene_filtered.pcd";

    pcl::io::savePCDFileASCII(experiments_path + pcls_path + scene_filtered_name, *pcl_scene_filtered);
    std::cout << "Filtered" << std::endl;
    pcl::visualization::PCLVisualizer visu("Filtered");
    visu.addPointCloud (pcl_scene_filtered, ColorHandlerT (pcl_scene_filtered, 0.0, 255.0, 0.0), "filtered");
    visu.spin ();

    // Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimation<pclPoint,pclPoint> nest;
    nest.setKSearch(5);
    nest.setInputCloud (pcl_scene_filtered);
    std::cout << "Computing scene normals....";
    nest.compute (*pcl_scene_filtered);

    // Estimate features
    pclCloud::Ptr object (new pclCloud);

    // Replace the path below with the path where you saved your file
    pcl::io::loadPCDFile(object_path, *object);

    vox.setInputCloud (object);
    vox.filter (*object);

    spatialFilter(object, object, "x", 0, 10);
    
    write_PointCloud_CSV(object,"../experiment_data/p4_dense/pcls/object_points.csv");
    pcl::visualization::PCLVisualizer visu_obj("obj");
    visu_obj.addPointCloud (object, ColorHandlerT (object, 255.0, 255.0, 0.0), "obj");
    visu_obj.addPointCloud (pcl_scene_filtered, ColorHandlerT (pcl_scene_filtered, 255.0, 0.0, 0.0), "scene");
    visu_obj.addPointCloud (z_axis, ColorHandlerT (z_axis, 0.0, 0.0, 255.0), "z_axis");
    visu_obj.addPointCloud (y_axis, ColorHandlerT (y_axis, 0.0, 255.0, 0.0), "y_axis");
    visu_obj.addPointCloud (x_axis, ColorHandlerT (x_axis, 255.0, 0.0, 0.0), "x_axis");
    visu_obj.spin();

    nest.setInputCloud (object);
    nest.compute (*object);

    pclCloud::Ptr object_aligned (new pclCloud);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.02);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    
    fest.setInputCloud (pcl_scene_filtered);
    fest.setInputNormals (pcl_scene_filtered);
    fest.compute (*scene_features);


    // Perform alignment
    Transform3D<> T_Global;
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<pclPoint,pclPoint,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (pcl_scene_filtered);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (500000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.90f); // Required inlier fraction for accepting a pose hypothesis
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
        visu.addPointCloud (pcl_scene_filtered, ColorHandlerT (pcl_scene_filtered, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();

        T_Global = convertEigenToRW(transformation);
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
    }

    // Create a k-d tree for scene
    pcl::search::KdTree<pclPoint> tree;
    tree.setInputCloud(pcl_scene_filtered);

    //pclCloud::Ptr cloud_icp (new pclCloud(*object_aligned, align.getInliers()));
    pclCloud::Ptr cloud_icp (new pclCloud(*object_aligned));
    // Compute inliers and RMSE
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;
    tree.nearestKSearch(*cloud_icp, std::vector<int>(), 1, idx, distsq);
    float rmse = 0;

    for(size_t i = 0; i < distsq.size(); ++i)
            rmse += distsq[i][0];
    rmse = sqrtf(rmse / cloud_icp->size());

    std::cout << "RMSE " << rmse << std::endl;

    Transform3D<> T_Local;
    size_t iterations = 50000;
    pcl::IterativeClosestPoint<pclPoint, pclPoint> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (pcl_scene_filtered);
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
        /*
        Transform3D<> rot(Vector3D<>(0,0,0), RPY<>(M_2_PI,0,0));
        Eigen::Matrix4f ROT{
            {rot.P().x, rot.R()}
            };*/
        pcl::transformPointCloud(*object_aligned, *object_aligned, transformation);
        pcl::transformPointCloud(*z_axis, *z_axis, transformation);
        pcl::transformPointCloud(*y_axis, *y_axis, transformation);
        pcl::transformPointCloud(*x_axis, *x_axis, transformation);

        // Show alignment
        pcl::visualization::PCLVisualizer visu("ICP alignment");
        visu.addPointCloud (pcl_scene_filtered, ColorHandlerT (pcl_scene_filtered, 255.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 255.0, 255.0), "object_aligned");
        visu.addPointCloud (z_axis, ColorHandlerT (z_axis, 0.0, 0.0, 255.0), "z_axis");
        visu.addPointCloud (y_axis, ColorHandlerT (y_axis, 0.0, 255.0, 0.0), "y_axis");
        visu.addPointCloud (x_axis, ColorHandlerT (x_axis, 255.0, 0.0, 0.0), "x_axis");
        visu.spin();

        T_Local = convertEigenToRW(transformation);
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

    Transform3D<> T_CO = T_Global * T_Local;

    Transform3D<> T_WO(Vector3D<>(0.000, 0.473, 0.210), RPY<>(-1.571, 0, 1.571));

    T_WC = T_WO * inverse(T_CO); 
    std::cout << "RANSAC" << std::endl;
    std::cout << "XYZ: " << T_WC.P() << std::endl;
    std::cout << "RPY: " << RPY<>(T_WC.R()) << std::endl;

    Transform3D<> T_Final = T_WC * T_CO;
    std::cout << "RANSAC" << std::endl;
    std::cout << "XYZ: " << T_Final.P() << std::endl;
    std::cout << "RPY: " << RPY<>(T_Final.R()) << std::endl;
    

    std::cout << "RMSE post ICP " << rmse_final << std::endl;





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
    
    generateImages(wc, state);
    std::cout << "Images generated" << std::endl;


    std::string imgL_path = "../experiment_data/p4_dense/images/Camera_left.ppm";
    std::string imgR_path = "../experiment_data/p4_dense/images/Camera_right.ppm";
    std::string object_path = "../experiment_data/p4_dense/pcls/output_vox.pcd";
    
    //Transformation of points into camera frame (Cameras are on the same plane)
    std::string camera_l_name = "Camera_Left";
    std::string camera_r_name = "Camera_Right";
    Transform3D<> Pl = getProjectionMatrix(camera_l_name, wc, state);
    Transform3D<> Pr = getProjectionMatrix(camera_r_name, wc, state);
    double Tx = fabs(Pl.P()[0] - Pr.P()[0]);
    double f = 514.682;
    std::cout << "Tx is: " << Tx << std::endl;
    dense_stereo(imgR_path, imgL_path, object_path, f, Tx, inverse(getProjectionMatrix(camera_l_name, wc, state)));
    
    
}

