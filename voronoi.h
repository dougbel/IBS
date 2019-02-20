#ifndef VORONOI_H
#define VORONOI_H


#include <vector>

#include <pcl/common/common.h>

#include <libqhullcpp/Qhull.h>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>


#include "definitions.h"

/**
 * @todo write docs
 */
class Voronoi
{  
public:
    
    
    Voronoi (pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud );
    
    void calculate();
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr bigCloud;   //TODO cambiarle el nombre a wholeCloud
    
    std::vector< pcl::PointXYZ > vornoiVertex;
    
    std::vector< point_region_hull > regionsVoronoi;
    
protected:
    orgQhull::Coordinates cloud_coord;  //TODO change name qhullCloudCoord
    
};

#endif // VORONOI_H
