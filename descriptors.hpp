/*cv::Mat shuffleRows(const cv::Mat &matrix){
    std::vector<int> seeds;

    for(int cont = 0; cont < matrix.rows; cont++)
        seeds.push_back(cont);

    cv::randShuffle(seeds);

    cv::Mat output;
    for (int cont = 0; cont < matrix.rows; cont++)
        output.push_back(matrix.row(seeds[cont]));

    return output;
}*/

/*PointCloud<PointXYZ>::Ptr DownsampledSrc;
DownsampledSrc.reset(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr DownsampledTgt;
DownsampledTgt.reset(new PointCloud<PointXYZ>);*/

/*//std::vector<pcl::PointXYZ> data = src->points;
int n = src->size();
Mat source(n, 3, CV_32F), target(n, 3, CV_32F);
int i = 0;
for (pcl::PointCloud<pcl::PointXYZ>::iterator it = src->begin(); it != src->end(); it++)
{
    source.at<float>(i,0) = it->x;
    source.at<float>(i,1) = it->y;
    source.at<float>(i,2) = it->z;
    i++;
}
cout << n << " and " << i;
i = 0;
for (pcl::PointCloud<pcl::PointXYZ>::iterator it = tgt->begin(); it != tgt->end(); it++)
{
    target.at<float>(i,0) = it->x;
    target.at<float>(i,1) = it->y;
    target.at<float>(i,2) = it->z;
    i++;
}
int gc = good_correspondences->size();
Mat corrSrs(m, 3, CV_32F), corrTgt(m, 3, CV_32F);
i = 0;
for(i = 0; i < m; i++)
{
    corrSrs.at<float>(i,0) = source.at<float>(good_correspondences->at (i).index_query, 0);
    corrTgt.at<float>(i,0) = target.at<float>(good_correspondences->at (i).index_match, 0);
    corrSrs.at<float>(i,1) = source.at<float>(good_correspondences->at (i).index_query, 1);
    corrTgt.at<float>(i,1) = target.at<float>(good_correspondences->at (i).index_match, 1);
    corrSrs.at<float>(i,2) = source.at<float>(good_correspondences->at (i).index_query, 2);
    corrTgt.at<float>(i,2) = target.at<float>(good_correspondences->at (i).index_match, 2);
}

Mat affineArray;
std::vector<uchar> inliers;
int output;
double ransacThreshold=3.0; double confidence=0.99;

output = cv::estimateAffine3D(corrSrs, corrTgt, affineArray, inliers, ransacThreshold, confidence); // Estimate Pose transformation using estimateAffine3D()

cout << "Type : " << corrSrs << endl;
cout << "number of affine inliers : " << inliers.size() << endl;
cv::vconcat(affineArray, cv::Mat::zeros(1,4,affineArray.type()), affineArray);
affineArray.at<double>(3,3) = 1.0;

cout << endl << affineArray << endl;

affineArray.convertTo(affineArray, corrSrs.type());

cv::cv2eigen(affineArray, transform);*/

    //estimateVFH(normals_src, normals_tgt, src, tgt, *vfh_src, *vfh_tgt);
