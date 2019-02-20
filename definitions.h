#ifndef DEFINTIIONS_IT
#define DEFINTIIONS_IT


#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;    //TODO parece ser el mismo tipo de puntos
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;


#define DIM 3  // TODO is it necesary? 


struct point_region_hull { // TODO estas son las definiciones de regiones asociadas al point cloud parent_id
    int parent_id;  //TODO position asociated point in original cloud
    int region_id;
    
    bool operator<(const point_region_hull& a) const {
        return region_id < a.region_id;
    }
    
    bool operator==(const point_region_hull& a) const
    {
        return (region_id == a.region_id);
    }
};



struct ibsPointType{
    PCL_ADD_POINT4D;
    int parent_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(ibsPointType,
                                  (float, x,x)
                                  (float, y,y)
                                  (float, z,z)
                                  (int, parent_id, parent_id))



typedef pcl::PointCloud<ibsPointType> PointCloudIbs;

#endif
