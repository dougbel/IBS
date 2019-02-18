#ifndef DEFINTIIONS_IT
#define DEFINTIIONS_IT


#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT> PointCloudT;    //TODO parece ser el mismo tipo de puntos
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;


#define DIM 3  // TODO is it necesary? 


struct point_region_hull { // TODO estas son las definiciones de regiones asociadas al point cloud parent_id
    int parent_id;  //TODO position asociated point in original cloud
    int region_id;
};



#endif
