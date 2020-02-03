#include "correspondence_pipeline.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"

using namespace ppf_match_3d;

class ComputeTransformations
{
    public:
        void computeTransformation (pcl::visualization::PCLVisualizer::Ptr &viewer, pcl::visualization::PCLVisualizer::Ptr &viewer2,
                                    PointCloud<PointXYZ>::Ptr &src, PointCloud<PointXYZ>::Ptr &tgt)
        {
            CorrespondencePipeline cp;
            AffineEstimation af;
            Initializations in;
            Visualize vz;

            PointCloud<PointXYZ>::Ptr tgtptr, output, outptr, Final;

            Eigen::Matrix4f transform;

            tgtptr.reset(new PointCloud<PointXYZ>);
            output.reset(new PointCloud<PointXYZ>);
            outptr.reset(new PointCloud<PointXYZ>);
            Final.reset(new PointCloud<PointXYZ>);

            PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>), normals_tgt (new PointCloud<Normal>);
            PointCloud<Normal>::Ptr normals_src2 (new PointCloud<Normal>), normals_tgt2 (new PointCloud<Normal>);

            PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src (new PointCloud<FPFHSignature33>), fpfh_tgt (new PointCloud<FPFHSignature33>);
            PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src2 (new PointCloud<FPFHSignature33>), fpfh_tgt2 (new PointCloud<FPFHSignature33>);

            pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_src (new pcl::PointCloud<pcl::VFHSignature308> ());
            pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_tgt (new pcl::PointCloud<pcl::VFHSignature308> ());

            CorrespondencesPtr all_correspondences (new Correspondences), good_correspondences (new Correspondences);
            CorrespondencesPtr all_correspondences2 (new Correspondences), good_correspondences2 (new Correspondences);

            in.loadPCDFiles (src, tgt);
            in.randomSample (src, tgt);
            tgtptr = tgt->makeShared();
            cout << tgt;
            tgt->clear();
            in.transformPC (tgtptr, tgt);
            cp.estimateNormals (src, tgt, *normals_src, *normals_tgt);
            cp.estimateFPFH(normals_src, normals_tgt, src, tgt, *fpfh_src, *fpfh_tgt); // Compute FPFH features at each keypoint
            cp.findCorrespondences (fpfh_src, fpfh_tgt, *all_correspondences);// Find correspondences between keypoints in FPFH space
            cp.rejectBadCorrespondences (all_correspondences, src, tgt, *good_correspondences); // Reject correspondences based on their XYZ distance
            vz.visualizeBeforeAffine (viewer, src, tgt, good_correspondences);

            TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
            trans_est.estimateRigidTransformation (*src, *tgt, *all_correspondences, transform); // Obtain the best transformation between the two sets of keypoints given the remaining correspondences

            cout << transform << endl;

            transformPointCloud (*src, *output, transform);
            savePCDFileBinary ("/home/iq9/nagendra/fpfh_pipeline/source_transformed.pcd", *output);

            outptr = output->makeShared();
            cp.estimateICP(outptr, tgt, Final);

            cp.estimateNormals (Final, tgt, *normals_src2, *normals_tgt2);
            cp.estimateFPFH(normals_src2, normals_tgt2, Final, tgt, *fpfh_src2, *fpfh_tgt2);
            cp.findCorrespondences (fpfh_src2, fpfh_tgt2, *all_correspondences2);
            af.rejectBadCorrespondencesAligned (all_correspondences2, Final, tgt, *good_correspondences2);

            af.estimateAffine(Final, tgt, good_correspondences2);
            vz.visualizeAfterAffine (viewer2, Final, tgt, good_correspondences2);
        }

};

class SurfaceMatch
{
    public:
        void getSourceTarget()
        {
            Initializations in;

            cv::Mat pc, pcTest;
            PointCloud<PointXYZ>::Ptr src, tgt, tgtptr;

            src.reset(new PointCloud<PointXYZ>);
            tgt.reset(new PointCloud<PointXYZ>);
            tgtptr.reset(new PointCloud<PointXYZ>);

            loadPCDFile ("/home/iq9/nagendra/fpfh_pipeline/office1_keypoints.pcd", *src);
            loadPCDFile ("/home/iq9/nagendra/fpfh_pipeline/office1_keypoints.pcd", *tgt);

            in.randomSample (src, tgt);

            tgtptr = tgt->makeShared();
            tgt->clear();

            in.transformPC (tgtptr, tgt);

            int n = src->size();
            int m = tgt->size();
            Mat source(n, 3, CV_32F), target(m, 3, CV_32F);

            in.getMat(src, tgt, source, target);

            cv::Vec3d viewpoint(0, 0, 0);
            cv::ppf_match_3d::computeNormalsPC3d(source, pc, 6, false, viewpoint);
            cv::ppf_match_3d::computeNormalsPC3d(target, pcTest, 6, false, viewpoint);

            writePLY(pc, "/home/iq9/nagendra/fpfh_pipeline/source.ply");
            writePLY(pcTest, "/home/iq9/nagendra/fpfh_pipeline/target.ply");

        }

        cv::Mat shuffleRows(const cv::Mat &matrix){
            std::vector<int> seeds;

            for(int cont = 0; cont < matrix.rows; cont++)
                seeds.push_back(cont);

            cv::randShuffle(seeds);

            cv::Mat output;
            for (int cont = 0; cont < matrix.rows; cont++)
                output.push_back(matrix.row(seeds[cont]));

            return output;
        }


        void transformSource()
        {
            Mat pc = loadPLYSimple("/home/iq9/nagendra/fpfh_pipeline/source.ply", 1);
            Mat pcTest = loadPLYSimple("/home/iq9/nagendra/fpfh_pipeline/target.ply", 1);

            cout << "Training..." << endl;
            int64 tick1 = cv::getTickCount();
            ppf_match_3d::PPF3DDetector detector(0.025, 0.05, 30); //0.025, 0.05
            detector.trainModel(pc);
            int64 tick2 = cv::getTickCount();
            cout << endl << "Training complete in "
                 << (double)(tick2-tick1)/ cv::getTickFrequency()
                 << " sec" << endl << "Loading model..." << endl;

           // Match the model to the scene and get the pose
            cout << endl << "Starting matching..." << endl;
            vector<Pose3DPtr> results;
            tick1 = cv::getTickCount();
            detector.match(pcTest, results, 1.0/2.0); //1.0/40.0, 0.05
            tick2 = cv::getTickCount();
            cout << endl << "PPF Elapsed Time " <<
                 (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;

            cout << "Total number of results : " << results.size() << endl;
            // Get only first N results
            int N = 5; //2
            vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);

            // Create an instance of ICP
            ICP icp(200, 0.005f, 2.5f, 6); //100, 0.005f, 2.5f, 8
            int64 t1 = cv::getTickCount();

            // Register for all selected poses
            cout << endl << "Performing ICP on " << N << " poses..." << endl;
            icp.registerModelToScene(pc, pcTest, resultsSub);
            int64 t2 = cv::getTickCount();

            cout << endl << "ICP Elapsed Time " <<
                 (t2-t1)/cv::getTickFrequency() << " sec" << endl;

            cout << "Poses: " << endl;
            Mat newPose, affineInliers, pct_points, pcTest_points, prevPose;
            // debug first five poses
            for (size_t i=0; i<resultsSub.size(); i++)
            {
                Pose3DPtr result = resultsSub[i];
                cout << "Pose Result " << i << endl;
                result->printPose();
                if (i==0) //0
                {
                    Mat pct = transformPCPose(pc, result->pose);
                    pct_points = pct.colRange(0,3);
                    pcTest_points = pcTest.colRange(0,3);
                    pct_points.convertTo(pct_points, CV_64F);
                    pcTest_points.convertTo(pcTest_points, CV_64F);

                    //ppf_match_3d::addNoisePC(justAffineSrc, 1.0);

                    estimateAffine3D(pct_points, pcTest_points, newPose, affineInliers, 3);
                    cv::vconcat(newPose, cv::Mat::zeros(1,4,newPose.type()), newPose);
                    newPose.at<double>(3,3) = 1.0;
                    prevPose = Mat(result->pose);
                    cout << "Pose used for transformation : " << newPose * prevPose  << endl;
                    cout << "Affine Pose : " << endl << newPose << endl;
                    cout << "Number of inliers : " << affineInliers.size() << endl;
                    //cv::invert(newPose, newPose);
                    pct = transformPCPose(pct, newPose);
                    writePLY(pct, "/home/iq9/nagendra/fpfh_pipeline/source_transformed.ply");
                }
            }
        }
};
