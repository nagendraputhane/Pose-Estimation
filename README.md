# sparse_PC_pose_estimation

# Classes: 
## 1) ComputeTransformations:
### Pipeline:
```
* Random sample the source and target point cloud respectively.
b) Transform the target point cloud.
c) Estimate normals for both the source and target point cloud.
d) Estimate feature descriptors for the two point clouds (FPFH / NARF / SHOT / PFH / VFH).
e) Find correspondences between the source and targets keypoints feature descriptors.
f) Reject bad correspondences based on distance threshold.
g) Visualize the correspondence between source and target. 
h) Estimate rigit transformation between source and target based on the found correspondences.
i) Transform the source point cloud using the transformation found.
j) Estimate pose between the partially aligned transformed source and target with ICP algorithm.
k) Transform the transformed source point cloud using the transformation found with ICP.
l) Estimate corespondences between the transformed point cloud using ICP estimation and the target cloud.
m) Estimate the Affine transformation between the two point clouds.
n) Visualize the correspondence between the two point clouds.
```

## 2) SurfaceMatch
### Pipeline:
```
a) Random sample the source and target point cloud respectively.
b) Transform the target point cloud.
c) Estimate normals for both the source and target point cloud.
d) Convert the point cloud to PLY file format.
e) Detect PPF feature descriptots.
f) Train the source point cloud using PPF3DDetector.
g) Match the target point cloud using PPF3DDetector.
h) Select N poses from the matches found and use it as initial pose for the ICP algorithm.
i) Estimate transformation pose using ICP.
j) Select the best pose and transform the point cloud.
k) Estimate the affine transformation between the two pint clouds.
l) Transform the source point cloud and store it as sourced_transformed.ply
```
