#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include <StopWatch.h>


#include "voronoi.h"
#include "ibs.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in , pcl::PointXYZ point_pivot, float radio){
    
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

int main(int argc, char *argv[])
{
    StopWatch sw;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_scene = "scene-wood-table.pcd";
    pcl::io::loadPCDFile(file_scene, *cloud_scene);
    
    /*Voronoi voronoi(cloud_scene);
    voronoi.calculate();*/
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_object = "bowl_place.pcd";
    pcl::io::loadPCDFile(file_object, *cloud_object);
    
    pcl::PointXYZ min,max,middlePoint;
    pcl::getMinMax3D(*cloud_object,min,max);
    
    middlePoint.x=(max.x+min.x)/2;
    middlePoint.y=(max.y+min.y)/2;
    middlePoint.z=(max.z+min.z)/2;
    
    float radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    
    IBS ibs(cloud_scene,cloud_object);
    ibs.calculate();
    
    pcl::io::savePCDFile("ibs.pcd",*ibs.ibs);
    
//     
//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchRad;
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     pcl::PointIndices::Ptr cIndices (new pcl::PointIndices);
//     std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
//     std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points
//     
//     kdtreeSearchRad.setInputCloud(ibs.ibs);
//     kdtreeSearchRad.radiusSearch (middlePoint, radio, pointIdxRadiusSearch, pointRadiusSquaredDistance);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//     
//     cIndices->indices=pointIdxRadiusSearch;
//     
//     extract.setInputCloud (ibs.ibs);
//     extract.setIndices(cIndices);
//     extract.setNegative (false);
//     extract.filter (*cloud_cluster);
// 
//     pcl::io::savePCDFile("ibs_clusterized.pcd",*cloud_cluster);
    
    
    ////////////////////////////////
    
   /* 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene_clustered = clusterExtraction(cloud_scene, middlePoint,radio);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_clustered = clusterExtraction(cloud_object, middlePoint,radio);    
    
    
    IBS ibs2(cloud_scene_clustered, cloud_object_clustered);
    ibs2.calculate();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibs_clustered = clusterExtraction(ibs2.ibs, middlePoint,radio); 
    
    pcl::io::savePCDFile("cloud_scene_clustered.pcd", *cloud_scene_clustered);
    pcl::io::savePCDFile("cloud_object_clustered.pcd",*cloud_object_clustered);
    pcl::io::savePCDFile("ibs_clustered.pcd",*ibs_clustered);
    
    
    std::cout << "Middle point: " << middlePoint.x << " " << middlePoint.y<< " " << middlePoint.z << endl;
    std::cout << "Radio " << radio << std::endl;*/
    
    std::cout << "tiempo " << sw.ElapsedUs() << std::endl;
    std::cout << "tiempo " << sw.ElapsedMs() << std::endl;
    std::cout << "tiempo " << sw.ElapsedSec() << std::endl;

    
    
    return EXIT_SUCCESS;
}
