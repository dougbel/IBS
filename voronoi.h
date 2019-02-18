#ifndef VORONOI_H
#define VORONOI_H


#include <vector>

#include <pcl/common/common.h>

#include <libqhullcpp/Qhull.h>

#include <boost/tokenizer.hpp>


#include "definitions.h"

/**
 * @todo write docs
 */
class Voronoi
{  
public:
    
    
    Voronoi (PointCloud::Ptr pointCloud );
    
    void calculate();
    
    
    PointCloud::Ptr bigCloud;   //TODO cambiarle el nombre a wholeCloud
    
    std::vector< PointT > vornoiVertex;
    
    std::vector< point_region_hull > regionsVoronoi;
    
protected:
    orgQhull::Coordinates cloud_coord;  //TODO change name qhullCloudCoord
    
};

#endif // VORONOI_H
