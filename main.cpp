#include "plane_octree.hpp"

int main(int argc, char *argv[]) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *point_cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
    return (-1);
  }
  std::cout << "point_cloud size: " << (*point_cloud).size() << std::endl;

  PlaneOctree plane_octree(
      0.5, 20, 0.00025, 2.0,
      0.25); //五个参数分别是：八叉树的分辨率，有效的叶子节点的点数量，判断叶子节点为平面的最小特征值阈值，进行平面融合时的两平面的法向量夹角阈值，进行平面融合时的平面中心到另一平面的距离阈值

  //构造八叉树
  plane_octree.PlaneOctreeCreate(
      point_cloud,
      true); //两个参数分别是：输入的点云，和是否对八叉树构建的结果进行可视化
  
  std::vector<std::shared_ptr<OctreePlane>> octree_planes_get;
  plane_octree.OctreePlaneExtraction(
      octree_planes_get,true); //两个参数分别是：存放提取的平面特征的数据，和是否对平面提取的结果进行可视化

  return 0;
}