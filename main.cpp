#include <iostream>

#include <pcl/io/pcd_io.h>

#include <StopWatch.h>


#include "voronoi.h"



int main(int argc, char *argv[])
{
    StopWatch sw;
    
    
    PointCloudT::Ptr cloud_tmp (new PointCloudT);
    std::string filename = "scene-wood-table.pcd";
   
    pcl::io::loadPCDFile(filename, *cloud_tmp);
    
    Voronoi voronoi(cloud_tmp);
    
    voronoi.calculate();
    
    std::cout <<"tiempo "<< sw.ElapsedUs()<<""<<std::endl;
    
    std::cout <<"tiempo "<< sw.ElapsedMs()<<""<<std::endl;
    
    std::cout <<"tiempo "<< sw.ElapsedSec()<<""<<std::endl;

    
    
    return EXIT_SUCCESS;
}
