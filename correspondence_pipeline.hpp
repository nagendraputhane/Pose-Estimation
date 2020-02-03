#include <iostream>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/random_sample.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
using namespace cv;

class Initializations
{
    public:
    void loadPCDFiles (const PointCloud<PointXYZ>::Ptr &src, const PointCloud<PointXYZ>::Ptr &tgt)
    {
        loadPCDFile ("/home/iq9/nagendra/fpfh_pipeline/office1_keypoints.pcd", *src);
        loadPCDFile ("/home/iq9/nagendra/fpfh_pipeline/office1_keypoints.pcd", *tgt);

    }

    void randomSample (const PointCloud<PointXYZ>::Ptr &src, const PointCloud<PointXYZ>::Ptr &tgt)
    {
        pcl::RandomSample<pcl::PointXYZ> random_sample;
        random_sample.setSeed(getTickCount());
        random_sample.setInputCloud(src);
        random_sample.setSample(500);
        random_sample.filter(*src);

        savePCDFileBinary ("/home/iq9/nagendra/fpfh_pipeline/source.pcd", *src);

        pcl::RandomSample<pcl::PointXYZ> random_samplee;
        random_samplee.setSeed(getTickCount());
        random_samplee.setInputCloud(tgt);
        random_samplee.setSample(500);
        random_samplee.filter(*tgt);
    }

    void downSample (const PointCloud<PointXYZ>::Ptr &src, const PointCloud<PointXYZ>::Ptr &tgt,
                     PointCloud<PointXYZ>::Ptr &srcFiltered, PointCloud<PointXYZ>::Ptr &tgtFiltered)
    {
        pcl::VoxelGrid<PointXYZ> sor;
        sor.setInputCloud (src);
        sor.setLeafSize(0.005f, 0.005f, 0.005f);
        sor.filter(*srcFiltered);

        sor.setInputCloud (tgt);
        sor.filter(*tgtFiltered);
    }

    void transformPC (const PointCloud<PointXYZ>::Ptr &tgt, PointCloud<PointXYZ>::Ptr &tgtTransformed)
    {
        float theta = M_PI/4;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        transform.translation() << 20.5, 4.0, 18.0;
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
        transform.rotate (Eigen::AngleAxisf (2*theta, Eigen::Vector3f::UnitY()));
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
        transform.scale (1);

        pcl::transformPointCloud (*tgt, *tgtTransformed, transform);
        cout << endl << transform.matrix() << endl;
        savePCDFileBinary ("/home/iq9/nagendra/fpfh_pipeline/target_transformed.pcd", *tgtTransformed);
    }

    void getMat(PointCloud<PointXYZ>::Ptr &src, PointCloud<PointXYZ>::Ptr &tgt,
                Mat source, Mat target)
    {

        int i = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = src->begin(); it != src->end(); it++)
        {
            source.at<float>(i,0) = it->x;
            source.at<float>(i,1) = it->y;
            source.at<float>(i,2) = it->z;
            i++;
        }
        i = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = tgt->begin(); it != tgt->end(); it++)
        {
            target.at<float>(i,0) = it->x;
            target.at<float>(i,1) = it->y;
            target.at<float>(i,2) = it->z;
            i++;
        }
    }
    void getCorrPC (int gc, Mat source, Mat target, Mat corrSrs, Mat corrTgt,
                    Correspondences &good_correspondences)
    {
        for(int i = 0; i < gc; i++)
        {
            corrSrs.at<float>(i,0) = source.at<float>(good_correspondences.at (i).index_query, 0);
            corrTgt.at<float>(i,0) = target.at<float>(good_correspondences.at (i).index_match, 0);
            corrSrs.at<float>(i,1) = source.at<float>(good_correspondences.at (i).index_query, 1);
            corrTgt.at<float>(i,1) = target.at<float>(good_correspondences.at (i).index_match, 1);
            corrSrs.at<float>(i,2) = source.at<float>(good_correspondences.at (i).index_query, 2);
            corrTgt.at<float>(i,2) = target.at<float>(good_correspondences.at (i).index_match, 2);
        }
    }
};

class CorrespondencePipeline
{
    public:
        void estimateNormals (const PointCloud<PointXYZ>::Ptr &src, const PointCloud<PointXYZ>::Ptr &tgt, PointCloud<Normal> &normals_src, PointCloud<Normal> &normals_tgt)
        {
          NormalEstimation<PointXYZ, Normal> normal_est;
          normal_est.setInputCloud (src);
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
          normal_est.setSearchMethod (tree);
          normal_est.setRadiusSearch (0.1);
          normal_est.compute (normals_src);

          normal_est.setInputCloud (tgt);
          pcl::search::KdTree<pcl::PointXYZ>::Ptr treeE (new pcl::search::KdTree<pcl::PointXYZ> ());
          normal_est.setSearchMethod (treeE);
          normal_est.compute (normals_tgt);
        }
        void estimateVFH (const PointCloud<Normal>::Ptr &normals_src, const PointCloud<Normal>::Ptr &normals_tgt,
                           const PointCloud<PointXYZ>::Ptr &keypoints_src, const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
                           PointCloud<VFHSignature308> &vfh_src, PointCloud<VFHSignature308> &vfh_tgt)
        {
            pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
            vfh.setInputCloud (keypoints_src);
            vfh.setInputNormals (normals_src);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            vfh.setSearchMethod (tree);
            vfh.compute (vfh_src);

            cout << vfh_src << endl;
            vfh.setInputCloud (keypoints_tgt);
            vfh.setInputNormals (normals_tgt);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr treee (new pcl::search::KdTree<pcl::PointXYZ> ());
            vfh.setSearchMethod (treee);
            vfh.compute (vfh_tgt);

        }

        void estimateFPFH (const PointCloud<Normal>::Ptr &normals_src, const PointCloud<Normal>::Ptr &normals_tgt,
                           const PointCloud<PointXYZ>::Ptr &keypoints_src, const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
                           PointCloud<FPFHSignature33> &fpfhs_src, PointCloud<FPFHSignature33> &fpfhs_tgt)
        {
          FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
          fpfh_est.setInputCloud (keypoints_src);
          fpfh_est.setInputNormals (normals_src);
          fpfh_est.setRadiusSearch (5);
          fpfh_est.setSearchSurface (keypoints_src);
          fpfh_est.compute (fpfhs_src);

          fpfh_est.setInputCloud (keypoints_tgt);
          fpfh_est.setInputNormals (normals_tgt);
          fpfh_est.setSearchSurface (keypoints_tgt);
          fpfh_est.compute (fpfhs_tgt);
        }

        void findCorrespondences (const PointCloud<FPFHSignature33>::Ptr &fpfh_src, const PointCloud<FPFHSignature33>::Ptr &fpfh_tgt,
                                  Correspondences &all_correspondences)
        {
          CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
          est.setInputSource (fpfh_src);
          est.setInputTarget (fpfh_tgt);
          est.determineCorrespondences (all_correspondences);
        }

        void rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences, const PointCloud<PointXYZ>::Ptr &keypoints_src, const PointCloud<PointXYZ>::Ptr &keypoints_tgt, Correspondences &remaining_correspondences)
        {
          CorrespondenceRejectorDistance rej;
          rej.setInputSource<PointXYZ> (keypoints_src);
          rej.setInputTarget<PointXYZ> (keypoints_tgt);
          rej.setMaximumDistance (50);
          rej.getRemainingCorrespondences (*all_correspondences, remaining_correspondences);
        }
        void estimateICP(PointCloud<PointXYZ>::Ptr &outptr, PointCloud<PointXYZ>::Ptr &tgt, PointCloud<PointXYZ>::Ptr &Final)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(outptr);
            icp.setInputTarget(tgt);
            icp.align(*Final);

            std::cout << "has converged: " << icp.hasConverged() << "\nscore: " << icp.getFitnessScore() << std::endl;
            cout << "ICP transform : " << icp.getFinalTransformation() << endl;
            savePCDFileBinary ("/home/iq9/nagendra/fpfh_pipeline/icp_source_transformed.pcd", *Final);
        }


};

class AffineEstimation
{
    public:
    void rejectBadCorrespondencesAligned (const CorrespondencesPtr &all_correspondences, const PointCloud<PointXYZ>::Ptr &keypoints_src, const PointCloud<PointXYZ>::Ptr &keypoints_tgt, Correspondences &remaining_correspondences)
    {
      CorrespondenceRejectorDistance rej;
      rej.setInputSource<PointXYZ> (keypoints_src);
      rej.setInputTarget<PointXYZ> (keypoints_tgt);
      rej.setMaximumDistance (1.0);
      rej.getRemainingCorrespondences (*all_correspondences, remaining_correspondences);
    }
    void estimateAffine(PointCloud<PointXYZ>::Ptr &Final, PointCloud<PointXYZ>::Ptr &tgt,
                        CorrespondencesPtr &good_correspondences)
    {
        Initializations in;

        int n = Final->size();
        int m = tgt->size();
        Mat source(n, 3, CV_32F), target(m, 3, CV_32F);

        in.getMat(Final, tgt, source, target);

        int gc = good_correspondences->size();
        Mat corrSrs(gc, 3, CV_32F), corrTgt(gc, 3, CV_32F);

        in.getCorrPC (gc, source, target, corrSrs, corrTgt, *good_correspondences);

        Mat affineArray;
        std::vector<uchar> inliers;
        int out;
        double ransacThreshold=3.0; double confidence=0.99;

        out = cv::estimateAffine3D(corrSrs, corrTgt, affineArray, inliers, ransacThreshold, confidence); // Estimate Pose transformation using estimateAffine3D()

        cout << "number of affine inliers : " << inliers.size() << endl;
        cv::vconcat(affineArray, cv::Mat::zeros(1,4,affineArray.type()), affineArray);
        affineArray.at<double>(3,3) = 1.0;

        cout << endl << affineArray << endl;

        Eigen::Matrix4f affineTransform;

        cv::cv2eigen(affineArray, affineTransform);

        PointCloud<PointXYZ>::Ptr outputAffine;
        outputAffine.reset(new PointCloud<PointXYZ>);

        transformPointCloud (*Final, *outputAffine, affineTransform);

        savePCDFileBinary ("/home/iq9/nagendra/fpfh_pipeline/affine_transformed.pcd", *outputAffine);
    }
};

class Visualize
{
    public:
        void visualizeBeforeAffine (pcl::visualization::PCLVisualizer::Ptr &viewer,
                          const PointCloud<PointXYZ>::Ptr &src, const PointCloud<PointXYZ>::Ptr &tgt,
                          const CorrespondencesPtr &good_correspondences)
        {
            viewer->setBackgroundColor (0, 0, 0);
            viewer->addCoordinateSystem (1.0);
            pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color (src, 0, 255, 0);
            viewer->addPointCloud<pcl::PointXYZ> (src, source_color, "source cloud");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (tgt, 0, 0, 255);
            viewer->addPointCloud<pcl::PointXYZ> (tgt, target_color, "target cloud");
            for (unsigned long i = 0; i < good_correspondences->size (); ++i)
                  {
                    std::stringstream ss_line;
                    ss_line << "correspondence_line " << i;
                    pcl::PointXYZ src_point = src->at (good_correspondences->at (i).index_query);
                    pcl::PointXYZ tgt_point = tgt->at (good_correspondences->at (i).index_match);
                    viewer->addLine<pcl::PointXYZ> (src_point, tgt_point, 255, 0, 0, ss_line.str ());
                  }
        }

        void visualizeAfterAffine (pcl::visualization::PCLVisualizer::Ptr &viewer,
                          const PointCloud<PointXYZ>::Ptr &Final, const PointCloud<PointXYZ>::Ptr &tgt,
                          const CorrespondencesPtr &good_correspondences)
        {
            viewer->setBackgroundColor (0, 0, 0);
            viewer->addCoordinateSystem (1.0);
            pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color (Final, 0, 255, 0);
            viewer->addPointCloud<pcl::PointXYZ> (Final, final_color, "final cloud");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (tgt, 0, 0, 255);
            viewer->addPointCloud<pcl::PointXYZ> (tgt, target_color, "tgt cloud");
            for (unsigned long i = 0; i < good_correspondences->size (); ++i)
                  {
                    std::stringstream ss_line;
                    ss_line << "correspondence_" << i;
                    pcl::PointXYZ src_point = Final->at (good_correspondences->at (i).index_query);
                    pcl::PointXYZ tgt_point = tgt->at (good_correspondences->at (i).index_match);

                    viewer->addLine<pcl::PointXYZ> (src_point, tgt_point, 255, 0, 0, ss_line.str ());

                  }
        }
};
