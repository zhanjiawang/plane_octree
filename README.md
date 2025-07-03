# plane_octree
This is an algorithm that constructs octree and extracts planar features at the same time. It can quickly extract plane features from point clouds. It takes about 0.017 seconds to actually process 80064 point clouds.

## Usage
#### 1. Requirement
```
1. cmake
2. PCL
```

#### 2. Build
```
cd ${YOUR_PROJECT_DIR}
mdkir build
cd build
cmake ..
make
```

#### 3. Run
```
./plane_octree 0.pcd
#result
point_cloud size: 80064
octree_resolution: 0.5 point_cloud_span: 35.845 octree_depth: 8
create_octree_time: 0.008615
plane_extraction_time: 0.007911
```
