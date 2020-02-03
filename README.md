# sparse_PC_pose_estimation

# Classes: 
## 1) ComputeTransformations:
### Pipeline:

* Random sample the source and target point cloud respectively.
* Transform the target point cloud.
* Estimate normals for both the source and target point cloud.
* Estimate feature descriptors for the two point clouds (FPFH / NARF / SHOT / PFH / VFH).
* Find correspondences between the source and targets keypoints feature descriptors.
* Reject bad correspondences based on distance threshold.
* Visualize the correspondence between source and target. 
* Estimate rigit transformation between source and target based on the found correspondences.
* Transform the source point cloud using the transformation found.
* Estimate pose between the partially aligned transformed source and target with ICP algorithm.
* Transform the transformed source point cloud using the transformation found with ICP.
* Estimate corespondences between the transformed point cloud using ICP estimation and the target cloud.
* Estimate the Affine transformation between the two point clouds.
* Visualize the correspondence between the two point clouds.


## 2) SurfaceMatch
### Pipeline:

* Random sample the source and target point cloud respectively.
* Transform the target point cloud.
* Estimate normals for both the source and target point cloud.
* Convert the point cloud to PLY file format.
* Detect PPF feature descriptots.
* Train the source point cloud using PPF3DDetector.
* Match the target point cloud using PPF3DDetector.
* Select N poses from the matches found and use it as initial pose for the ICP algorithm.
* Estimate transformation pose using ICP.
* Select the best pose and transform the point cloud.
* Estimate the affine transformation between the two pint clouds.
* Transform the source point cloud and store it as sourced_transformed.ply
