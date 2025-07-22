#ifndef PLANE_OCTREE_HPP_
#define PLANE_OCTREE_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    ColorHandlerT;

typedef struct OctreePoint {
  float x;
  float y;
  float z;
} OctreePoint;

typedef struct OctreePointCloud {
  int r;
  int g;
  int b;
  std::vector<OctreePoint> octree_point_cloud;
} OctreePointCloud;

typedef struct OctreePlane {
  bool is_allocate = false;
  int plane_size = 0;
  Eigen::Vector3f plane_center = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f plane_normal = Eigen::Vector3f(0, 0, 0);
  std::shared_ptr<OctreePointCloud> plane_data = nullptr;
} OctreePlane;

typedef struct OctreeNode {
  int depth = 1;
  Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);
  std::shared_ptr<OctreePointCloud> data = nullptr;
  std::vector<std::shared_ptr<OctreePlane>> octree_planes;
  std::vector<std::shared_ptr<OctreeNode>> octree_ptr =
      std::vector<std::shared_ptr<OctreeNode>>(8, nullptr);
} OctreeNode;

class PlaneOctree {
public:
  PlaneOctree(float octree_resolution, int vaild_leaf_node_size,
              float plane_judgment_threshold,
              float plane_fusion_angle_threshold,
              float plane_fusion_distance_threshold) {
    this->octree_resolution_ = octree_resolution;
    this->vaild_leaf_node_size_ = vaild_leaf_node_size;
    this->plane_judgment_threshold_ = plane_judgment_threshold;
    this->plane_fusion_angle_threshold_ = plane_fusion_angle_threshold;
    this->plane_fusion_distance_threshold_ = plane_fusion_distance_threshold;
  }
  ~PlaneOctree() {}

  //计算两法向量的夹角
  float CalculateAngel(Eigen::Vector3f plane_normal_i,
                       Eigen::Vector3f plane_normal_j) {
    float plane_normal_i_dot_plane_normal_j =
        plane_normal_i.dot(plane_normal_j);
    float cos_theta = plane_normal_i_dot_plane_normal_j /
                      ((plane_normal_i.norm()) * (plane_normal_j.norm()));
    float theta = 180 * acos(cos_theta) / M_PI;
    return theta;
  }

  //计算平面中心到另一平面的距离
  float CalculateDistance(Eigen::Vector3f plane_center_i,
                          Eigen::Vector3f plane_center_j,
                          Eigen::Vector3f plane_normal_j) {
    float plane_a = plane_normal_j[0];
    float plane_b = plane_normal_j[1];
    float plane_c = plane_normal_j[2];
    float plane_d =
        -(plane_a * plane_center_j[0] + plane_b * plane_center_j[1] +
          plane_c * plane_center_j[2]);
    float distance =
        fabs(plane_a * plane_center_i[0] + plane_b * plane_center_i[1] +
             plane_c * plane_center_i[2] + plane_d) /
        sqrt(pow(plane_a, 2) + pow(plane_b, 2) + pow(plane_c, 2));
    return distance;
  }

  //构建八叉树
  void PlaneOctreeCreate(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                         bool view) {
    clock_t start_create_octree = clock();

    //八叉树八个节点的索引
    std::vector<std::vector<std::vector<int>>> direction_vector(
        3, std::vector<std::vector<int>>(3, std::vector<int>(3, -1)));
    direction_vector[0][2][2] = 0;
    direction_vector[2][2][2] = 1;
    direction_vector[0][0][2] = 2;
    direction_vector[2][0][2] = 3;
    direction_vector[0][2][0] = 4;
    direction_vector[2][2][0] = 5;
    direction_vector[0][0][0] = 6;
    direction_vector[2][0][0] = 7;

    //确定输入点云的跨度
    float point_cloud_min_x = FLT_MAX;
    float point_cloud_max_x = -FLT_MAX;
    float point_cloud_min_y = FLT_MAX;
    float point_cloud_max_y = -FLT_MAX;
    float point_cloud_min_z = FLT_MAX;
    float point_cloud_max_z = -FLT_MAX;
    for (auto &point : (*point_cloud)) {
      if (point.x < point_cloud_min_x) {
        point_cloud_min_x = point.x;
      }
      if (point.x > point_cloud_max_x) {
        point_cloud_max_x = point.x;
      }
      if (point.y < point_cloud_min_y) {
        point_cloud_min_y = point.y;
      }
      if (point.y > point_cloud_max_y) {
        point_cloud_max_y = point.y;
      }
      if (point.z < point_cloud_min_z) {
        point_cloud_min_z = point.z;
      }
      if (point.z > point_cloud_max_z) {
        point_cloud_max_z = point.z;
      }
    }
    float point_cloud_length = fabs(point_cloud_min_x) > fabs(point_cloud_max_x)
                                   ? fabs(point_cloud_min_x)
                                   : fabs(point_cloud_max_x);
    float point_cloud_width = fabs(point_cloud_min_y) > fabs(point_cloud_max_y)
                                  ? fabs(point_cloud_min_y)
                                  : fabs(point_cloud_max_y);
    float point_cloud_height = fabs(point_cloud_min_z) > fabs(point_cloud_max_z)
                                   ? fabs(point_cloud_min_z)
                                   : fabs(point_cloud_max_z);
    float point_cloud_length_width_max = point_cloud_length > point_cloud_width
                                             ? point_cloud_length
                                             : point_cloud_width;
    float point_cloud_length_width_height_max =
        point_cloud_length_width_max > point_cloud_height
            ? point_cloud_length_width_max
            : point_cloud_height;

    // std::cout << "point_cloud_min_x: " << point_cloud_min_x
    //           << " point_cloud_max_x: " << point_cloud_max_x
    //           << " point_cloud_min_y: " << point_cloud_min_y
    //           << " point_cloud_max_y: " << point_cloud_max_y
    //           << " point_cloud_min_z: " << point_cloud_min_z
    //           << " point_cloud_max_z: " << point_cloud_max_z
    //           << " point_cloud_length_width_height_max: "
    //           << point_cloud_length_width_height_max << std::endl;

    //确定八叉树所需要的深度
    octree_resolution_ = octree_resolution_ / 2.0;
    while ((octree_resolution_ * pow(2, octree_depth_)) <
           point_cloud_length_width_height_max) {
      octree_depth_++;
    }

    std::cout << "octree_resolution: " << (octree_resolution_ * 2.0)
              << " point_cloud_span: " << point_cloud_length_width_height_max
              << " octree_depth: " << octree_depth_ << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_octree_view(
        new pcl::PointCloud<pcl::PointXYZRGB>);
      
    point_cloud_octree_data_ = std::make_shared<OctreeNode>();
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;
    //把每一个点加入八叉树的同时构建八叉树
    for (auto &point : *point_cloud) {
      point_cloud_octree_ptr = point_cloud_octree_data_;
      while (point_cloud_octree_ptr->depth <= octree_depth_) {
        int direction_x = point.x > point_cloud_octree_ptr->center[0] ? 1 : -1;
        int direction_y = point.y > point_cloud_octree_ptr->center[1] ? 1 : -1;
        int direction_z = point.z > point_cloud_octree_ptr->center[2] ? 1 : -1;
        int direction_index =
            direction_vector[direction_x + 1][direction_y + 1][direction_z + 1];
        //如果还没有构建此节点那么就构建此节点
        if (point_cloud_octree_ptr->octree_ptr[direction_index] == nullptr) {
          point_cloud_octree_ptr->octree_ptr[direction_index] =
              std::make_shared<OctreeNode>();
          point_cloud_octree_ptr->octree_ptr[direction_index]->depth =
              point_cloud_octree_ptr->depth + 1;
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[0] =
              point_cloud_octree_ptr->center[0] +
              (direction_x * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[1] =
              point_cloud_octree_ptr->center[1] +
              (direction_y * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[2] =
              point_cloud_octree_ptr->center[2] +
              (direction_z * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          //如果到了最深的深度也就是叶子节点那么就把点数据放入叶子节点
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->data ==
                nullptr) {
              point_cloud_octree_ptr->octree_ptr[direction_index]->data =
                  std::make_shared<OctreePointCloud>();
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->r =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->g =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->b =
                  rand() % 255;
            } else {
              OctreePoint odtree_point;
              odtree_point.x = point.x;
              odtree_point.y = point.y;
              odtree_point.z = point.z;
              point_cloud_octree_ptr->octree_ptr[direction_index]
                  ->data->octree_point_cloud.push_back(odtree_point);

              if (view) {
                pcl::PointXYZRGB point_xyzrgb;
                point_xyzrgb.x = point.x;
                point_xyzrgb.y = point.y;
                point_xyzrgb.z = point.z;
                point_xyzrgb.r =
                    point_cloud_octree_ptr->octree_ptr[direction_index]
                        ->data->r;
                point_xyzrgb.g =
                    point_cloud_octree_ptr->octree_ptr[direction_index]
                        ->data->g;
                point_xyzrgb.b =
                    point_cloud_octree_ptr->octree_ptr[direction_index]
                        ->data->b;
                (*point_cloud_octree_view).push_back(point_xyzrgb);
              }
            }
          }
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        } else {
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->data ==
                nullptr) {
              point_cloud_octree_ptr->octree_ptr[direction_index]->data =
                  std::make_shared<OctreePointCloud>();
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->r =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->g =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->b =
                  rand() % 255;
            } else {
              OctreePoint odtree_point;
              odtree_point.x = point.x;
              odtree_point.y = point.y;
              odtree_point.z = point.z;
              point_cloud_octree_ptr->octree_ptr[direction_index]
                  ->data->octree_point_cloud.push_back(odtree_point);
              if (view) {
                pcl::PointXYZRGB point_xyzrgb;
                point_xyzrgb.x = point.x;
                point_xyzrgb.y = point.y;
                point_xyzrgb.z = point.z;
                point_xyzrgb.r =
                    point_cloud_octree_ptr->octree_ptr[direction_index]
                        ->data->r;
                point_xyzrgb.g =
                    point_cloud_octree_ptr->octree_ptr[direction_index]
                        ->data->g;
                point_xyzrgb.b =
                    point_cloud_octree_ptr->octree_ptr[direction_index]
                        ->data->b;
                (*point_cloud_octree_view).push_back(point_xyzrgb);
              }
            }
          }
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        }
      }
    }

    clock_t end_create_octree = clock();
    double create_octree_time =
        ((double)(end_create_octree - start_create_octree) /
         (double)CLOCKS_PER_SEC);
    std::cout << "create_octree_time: " << create_octree_time << std::endl;

    if (view) {
      pcl::visualization::PCLVisualizer point_cloud_octree_viewer(
          "point_cloud_octree_viewer");
      point_cloud_octree_viewer.setBackgroundColor(255, 255, 255);
      point_cloud_octree_viewer.addPointCloud(point_cloud_octree_view,
                                              "point_cloud_octree_view");
      point_cloud_octree_viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
          "point_cloud_octree_view");
      point_cloud_octree_viewer.spin();
    }
  }

  //判断八叉树的叶子节点是否是平面
  void OctreePlaneJudgment(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr leaf_node_plane_view, bool view) {
    if (point_cloud_octree_ptr->data != nullptr) {
      int left_node_point_cloud_size =
          point_cloud_octree_ptr->data->octree_point_cloud.size();
      if (left_node_point_cloud_size > vaild_leaf_node_size_) {
        Eigen::Vector3f centroid(0, 0, 0);
        for (int i = 0; i < left_node_point_cloud_size; i++) {
          Eigen::Vector3f point(
              point_cloud_octree_ptr->data->octree_point_cloud[i].x,
              point_cloud_octree_ptr->data->octree_point_cloud[i].y,
              point_cloud_octree_ptr->data->octree_point_cloud[i].z);
          centroid += point;
        }
        centroid /= left_node_point_cloud_size;

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (int i = 0; i < left_node_point_cloud_size; i++) {
          Eigen::Vector3f point(
              point_cloud_octree_ptr->data->octree_point_cloud[i].x,
              point_cloud_octree_ptr->data->octree_point_cloud[i].y,
              point_cloud_octree_ptr->data->octree_point_cloud[i].z);
          Eigen::Vector3f centered = point - centroid;
          covariance += centered * centered.transpose();
        }
        covariance /= left_node_point_cloud_size;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

        Eigen::Vector3f eigenvalues = solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = solver.eigenvectors();

        Eigen::Vector3f normal = eigenvectors.col(0);

        if (eigenvalues[0] < plane_judgment_threshold_) {
          int r = point_cloud_octree_ptr->data->r;
          int g = point_cloud_octree_ptr->data->g;
          int b = point_cloud_octree_ptr->data->b;
          std::shared_ptr<OctreePlane> octree_plane =
              std::make_shared<OctreePlane>();
          octree_plane->is_allocate = false;
          octree_plane->plane_size = left_node_point_cloud_size;
          octree_plane->plane_center = centroid;
          octree_plane->plane_normal = normal;
          if (octree_plane->plane_data == nullptr) {
            octree_plane->plane_data = std::make_shared<OctreePointCloud>();
            octree_plane->plane_data->r = r;
            octree_plane->plane_data->g = g;
            octree_plane->plane_data->b = b;
          }
          for (int i = 0; i < left_node_point_cloud_size; i++) {
            OctreePoint odtree_point;
            odtree_point.x =
                point_cloud_octree_ptr->data->octree_point_cloud[i].x;
            odtree_point.y =
                point_cloud_octree_ptr->data->octree_point_cloud[i].y;
            odtree_point.z =
                point_cloud_octree_ptr->data->octree_point_cloud[i].z;
            octree_plane->plane_data->octree_point_cloud.push_back(
                odtree_point);

            if (view) {
              pcl::PointXYZRGB point_xyzrgb;
              point_xyzrgb.x =
                  point_cloud_octree_ptr->data->octree_point_cloud[i].x;
              point_xyzrgb.y =
                  point_cloud_octree_ptr->data->octree_point_cloud[i].y;
              point_xyzrgb.z =
                  point_cloud_octree_ptr->data->octree_point_cloud[i].z;
              point_xyzrgb.r = r;
              point_xyzrgb.g = g;
              point_xyzrgb.b = b;
              (*leaf_node_plane_view).push_back(point_xyzrgb);
            }
          }
          point_cloud_octree_ptr->octree_planes.push_back(octree_plane);
        }
      }
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreePlaneJudgment(point_cloud_octree_ptr->octree_ptr[i],
                            leaf_node_plane_view, view);
      }
    }
  }

  //融合八叉树不同深度上的平面特征
  void OctreePlaneFusion(std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
                         int fusion_depth) {
    if (point_cloud_octree_ptr->depth == fusion_depth) {
      std::vector<std::shared_ptr<OctreePlane>> octree_node_planes;
      for (int i = 0; i < 8; i++) {
        if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
          if (point_cloud_octree_ptr->octree_ptr[i]->octree_planes.size() > 0) {
            for (int j = 0;
                 j <
                 point_cloud_octree_ptr->octree_ptr[i]->octree_planes.size();
                 j++) {
              octree_node_planes.push_back(
                  point_cloud_octree_ptr->octree_ptr[i]->octree_planes[j]);
            }
          }
        }
      }
      for (int i = 0; i < octree_node_planes.size(); i++) {
        if (octree_node_planes[i]->is_allocate == false) {
          bool have_fusion = true;
          while (have_fusion) {
            have_fusion = false;
            for (int j = 0; j < octree_node_planes.size(); j++) {
              if (i != j && octree_node_planes[j]->is_allocate == false) {
                float angle_ij =
                    CalculateAngel(octree_node_planes[i]->plane_normal,
                                   octree_node_planes[j]->plane_normal);
                float distance_i =
                    CalculateDistance(octree_node_planes[i]->plane_center,
                                      octree_node_planes[j]->plane_center,
                                      octree_node_planes[j]->plane_normal);
                float distance_j =
                    CalculateDistance(octree_node_planes[j]->plane_center,
                                      octree_node_planes[i]->plane_center,
                                      octree_node_planes[i]->plane_normal);

                if ((angle_ij < plane_fusion_angle_threshold_ ||
                     angle_ij > (180 - plane_fusion_angle_threshold_)) &&
                    distance_i < plane_fusion_distance_threshold_ &&
                    distance_j < plane_fusion_distance_threshold_) {
                  have_fusion = true;
                  octree_node_planes[j]->is_allocate = true;
                  for (int k = 0;
                       k < octree_node_planes[j]
                               ->plane_data->octree_point_cloud.size();
                       k++) {
                    OctreePoint odtree_point;
                    odtree_point.x = octree_node_planes[j]
                                         ->plane_data->octree_point_cloud[k]
                                         .x;
                    odtree_point.y = octree_node_planes[j]
                                         ->plane_data->octree_point_cloud[k]
                                         .y;
                    odtree_point.z = octree_node_planes[j]
                                         ->plane_data->octree_point_cloud[k]
                                         .z;
                    octree_node_planes[i]
                        ->plane_data->octree_point_cloud.push_back(
                            odtree_point);
                  }
                  octree_node_planes[i]->plane_center =
                      ((octree_node_planes[i]->plane_size *
                        octree_node_planes[i]->plane_center) +
                       (octree_node_planes[j]->plane_size *
                        octree_node_planes[j]->plane_center)) /
                      (octree_node_planes[i]->plane_size +
                       octree_node_planes[j]->plane_size);
                  if (angle_ij < plane_fusion_angle_threshold_) {
                    octree_node_planes[i]->plane_normal =
                        (octree_node_planes[i]->plane_size *
                         octree_node_planes[i]->plane_normal) +
                        (octree_node_planes[j]->plane_size *
                         octree_node_planes[j]->plane_normal);
                  } else {
                    octree_node_planes[i]->plane_normal =
                        (octree_node_planes[i]->plane_size *
                         octree_node_planes[i]->plane_normal) -
                        (octree_node_planes[j]->plane_size *
                         octree_node_planes[j]->plane_normal);
                  }
                  octree_node_planes[i]->plane_normal.normalize();
                  octree_node_planes[i]->plane_size =
                      octree_node_planes[i]->plane_size +
                      octree_node_planes[j]->plane_size;
                }
              }
            }
          }
        }
      }
      for (int i = 0; i < octree_node_planes.size(); i++) {
        if (octree_node_planes[i]->is_allocate == false) {
          point_cloud_octree_ptr->octree_planes.push_back(
              octree_node_planes[i]);
        }
      }
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->depth < fusion_depth &&
          point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreePlaneFusion(point_cloud_octree_ptr->octree_ptr[i], fusion_depth);
      }
    }
  }

  //提取八叉树的平面特征
  void OctreePlaneExtraction(
      std::vector<std::shared_ptr<OctreePlane>> &octree_planes_get, bool view) {
    clock_t start_plane_extraction = clock();

    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr leaf_node_plane_view(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    OctreePlaneJudgment(point_cloud_octree_ptr, leaf_node_plane_view, view);

    //在八叉树不同深度上进行平面特征的融合
    for (int depth = octree_depth_; depth > 0; depth--) {
      point_cloud_octree_ptr = point_cloud_octree_data_;
      OctreePlaneFusion(point_cloud_octree_ptr, depth);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr octree_plane_view(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < point_cloud_octree_data_->octree_planes.size(); i++) {
      octree_planes_get.push_back(point_cloud_octree_data_->octree_planes[i]);

      int r = point_cloud_octree_data_->octree_planes[i]->plane_data->r;
      int g = point_cloud_octree_data_->octree_planes[i]->plane_data->g;
      int b = point_cloud_octree_data_->octree_planes[i]->plane_data->b;
      for (int j = 0; j < point_cloud_octree_data_->octree_planes[i]
                              ->plane_data->octree_point_cloud.size();
           j++) {
        if (view) {
          pcl::PointXYZRGB point_xyzrgb;
          point_xyzrgb.x = point_cloud_octree_data_->octree_planes[i]
                               ->plane_data->octree_point_cloud[j]
                               .x;
          point_xyzrgb.y = point_cloud_octree_data_->octree_planes[i]
                               ->plane_data->octree_point_cloud[j]
                               .y;
          point_xyzrgb.z = point_cloud_octree_data_->octree_planes[i]
                               ->plane_data->octree_point_cloud[j]
                               .z;
          point_xyzrgb.r = r;
          point_xyzrgb.g = g;
          point_xyzrgb.b = b;
          (*octree_plane_view).push_back(point_xyzrgb);
        }
      }
    }

    clock_t end_plane_extraction = clock();
    double plane_extraction_time =
        ((double)(end_plane_extraction - start_plane_extraction) /
         (double)CLOCKS_PER_SEC);
    std::cout << "plane_extraction_time: " << plane_extraction_time
              << std::endl;

    if (view) {
      pcl::visualization::PCLVisualizer leaf_node_plane_viewer(
          "leaf_node_plane_viewer");
      leaf_node_plane_viewer.setBackgroundColor(255, 255, 255);
      leaf_node_plane_viewer.addPointCloud(leaf_node_plane_view,
                                           "leaf_node_plane_view");
      leaf_node_plane_viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
          "leaf_node_plane_view");
      leaf_node_plane_viewer.spin();

      pcl::visualization::PCLVisualizer octree_plane_viewer(
          "octree_plane_viewer");
      octree_plane_viewer.setBackgroundColor(255, 255, 255);
      octree_plane_viewer.addPointCloud(octree_plane_view, "octree_plane_view");
      octree_plane_viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
          "octree_plane_view");
      octree_plane_viewer.spin();
    }
  }

  std::shared_ptr<OctreeNode> point_cloud_octree_data_;

private:
  float octree_resolution_ = 0.5; //八叉树的分辨率
  int octree_depth_ = 1; //八叉树的初始深度，后续根据输入点云的跨度重新计算
  int vaild_leaf_node_size_ = 20; //有效的叶子节点的点数量
  float plane_judgment_threshold_ =
      0.00025; //判断叶子节点为平面的最小特征值阈值
  float plane_fusion_angle_threshold_ =
      2.0; //进行平面融合时的两平面的法向量夹角阈值
  float plane_fusion_distance_threshold_ =
      0.25; //进行平面融合时的平面中心到另一平面的距离阈值
};

#endif
