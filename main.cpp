#include "transformation.hpp"
#include <thread>

using namespace cv;

int main ()
{
  PointCloud<PointXYZ>::Ptr src, tgt;
  ComputeTransformations ct;

  src.reset(new PointCloud<PointXYZ>);
  tgt.reset(new PointCloud<PointXYZ>);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Correspondences"));
  pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("Correspondence"));

  ct.computeTransformation (viewer, viewer2, src, tgt);

  while (!viewer->wasStopped())
     {
       viewer->spinOnce (100);
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
     }
  while (!viewer2->wasStopped())
    {
    viewer2->spinOnce (100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  /*SurfaceMatch sf;

  sf.getSourceTarget();

  sf.transformSource();*/
}
