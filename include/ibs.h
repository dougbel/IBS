#ifndef IBS_H
#define IBS_H

#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "util.h"
#include "voronoi.h"
#include "definitions.h"


/**
 * @todo write docs
 */
class IBS
{
public:
    
    /**
     * Constructor
     *
     * @param pointCloud TODO
     */
    IBS (pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object);
    
    void calculate();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr getIBS();
    
    void setROI ( pcl::PointXYZ pivotPoint, float radio );
        
    bool isSet_ROI();
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr queryObjectCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr bigCloud;
    
    
    
private:
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibs;
    
    pcl::PointXYZ center_ROI;
    float radio_ROI;
    bool is_ROI;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr soi_sceneCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr soi_queryObjectCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr soi_bigCloud;
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr sphereExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in , pcl::PointXYZ point_pivot, float radio);

};

#endif // IBS_H
