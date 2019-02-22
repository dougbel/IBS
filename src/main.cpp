
#include "Util.h"
#include "ibs.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <StopWatch.h>


int main(int argc, char *argv[])
{
    StopWatch sw;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_scene = "../test/Place/sceney_object.pcd";
    pcl::io::loadPCDFile(file_scene, *cloud_scene);
    
    /*Voronoi voronoi(cloud_scene);
    voronoi.calculate();*/
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_object = "../test/Place/query_object.pcd";
    pcl::io::loadPCDFile(file_object, *cloud_object);
    
    
    sw.Restart();
    IBS ibs(cloud_scene,cloud_object);
    ibs.calculate();
    
  
    
    std::cout << "tiempo " << sw.ElapsedUs() << std::endl;
    std::cout << "tiempo " << sw.ElapsedMs() << std::endl;
    std::cout << "tiempo " << sw.ElapsedSec() << std::endl;

    
    
    return EXIT_SUCCESS;
}
