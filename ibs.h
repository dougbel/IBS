#ifndef IBS_H
#define IBS_H



#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl/common/io.h>


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
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr queryObjectCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr bigCloud;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibs;
    

};

#endif // IBS_H
