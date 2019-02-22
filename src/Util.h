#ifndef UTIL
#define UTIL


#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>


class Util{
public:
    static   pcl::PointCloud<pcl::PointXYZ>::Ptr sphereExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in , pcl::PointXYZ point_pivot, float radio){
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchRad;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr cIndices (new pcl::PointIndices);
        
        std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
        std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points
        
        kdtreeSearchRad.setInputCloud( cloud_in );
        kdtreeSearchRad.radiusSearch (point_pivot, radio, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        
        
        cIndices->indices=pointIdxRadiusSearch;
        
        extract.setInputCloud( cloud_in );
        extract.setIndices( cIndices );
        extract.setNegative (false);
        extract.filter (*cloud_out);
        
        return cloud_out;
    }
};


#endif
