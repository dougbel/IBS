#include <iostream>
#include <assert.h>  

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include "voronoi.h"
#include "ibs.h"

class IBS_Distance{
public:
    pcl::PointCloud<pcl::PointXYZ> cloudA;
    pcl::PointCloud<pcl::PointXYZ> cloudB;
    float a_b_distance;
    float b_a_distance;

    IBS_Distance (pcl::PointCloud<pcl::PointXYZ> &cloud_a, pcl::PointCloud<pcl::PointXYZ> &cloud_b){
        this->cloudA = cloud_a;
        this->cloudB = cloud_b;
        this->a_b_distance = 0;
        this->b_a_distance = 0;
    }

    void compute ()
    {

        // compare A to B
        pcl::search::KdTree<pcl::PointXYZ> tree_b;
        tree_b.setInputCloud (cloudB.makeShared ());
        for (size_t i = 0; i < cloudA.points.size (); ++i)
        {
            std::vector<int> indices (1);
            std::vector<float> sqr_distances (1);

            tree_b.nearestKSearch (cloudA.points[i], 1, indices, sqr_distances);
            
            a_b_distance += sqr_distances[0];
        }

        // compare B to A
        pcl::search::KdTree<pcl::PointXYZ> tree_a;
        tree_a.setInputCloud (cloudA.makeShared ());
        for (size_t i = 0; i < cloudB.points.size (); ++i)
        {
            std::vector<int> indices (1);
            std::vector<float> sqr_distances (1);

            tree_a.nearestKSearch (cloudB.points[i], 1, indices, sqr_distances);
            
            b_a_distance += sqr_distances[0];
        }

        
    }
};


int main(int argc, char *argv[])
{

    
    
    //precalculated IBS
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibs_whole (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_ibs_whole = "../../test/Place/ibs.pcd";
    pcl::io::loadPCDFile(file_ibs_whole, *ibs_whole);
    
    //full scene point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_scene = "../../test/Place/scene_object.pcd";
    pcl::io::loadPCDFile(file_scene, *cloud_scene);
    
    //query object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_object = "../../test/Place/query_object.pcd";
    pcl::io::loadPCDFile(file_object, *cloud_object);
    
    
    pcl::PointXYZ min,max,middlePoint;
    pcl::getMinMax3D(*cloud_object,min,max);
    
    middlePoint.x=(max.x+min.x)/2;
    middlePoint.y=(max.y+min.y)/2;
    middlePoint.z=(max.z+min.z)/2;
    
    float radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  WHOLE IBS CALCULATION WITH COMPLETE SCENE
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING test complete IBS calculation " << endl;
    
    IBS ibs(cloud_scene,cloud_object);
    ibs.calculate();
    
    IBS_Distance distanceIBS_whole(*ibs.getIBS(), *ibs_whole);
    distanceIBS_whole.compute();    
    
    std::cout << "Distance: [" << "A->B: "  <<  distanceIBS_whole.a_b_distance << ", B->A: "  << distanceIBS_whole.b_a_distance  << endl;
    
    if (distanceIBS_whole.a_b_distance >= 0.001 ){
        std::cout << "ERROR! A is not a subconjunction of B" <<endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << endl << "SUCESS!" <<endl<<endl<<endl;
    }
        
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  WHOLE IBS CALCULATION WITH Region of Interest (ROI)
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    std::cout << "STARTING test, IBS with segmentation calculation " << endl<< endl;
    

    IBS ibs_roi (cloud_scene,cloud_object);
    ibs_roi.setROI (middlePoint, radio);
    ibs_roi.calculate();
    
    IBS_Distance distanceIBS(*ibs_roi.getIBS(), *ibs_whole);
    distanceIBS.compute();
    
    
    std::cout << "Distance: [" << "A->B: "  <<  distanceIBS.a_b_distance << ", B->A: "  << distanceIBS.b_a_distance  << endl;
    
    if (distanceIBS.a_b_distance != 0 ){
        std::cout << "ERROR! A is not a subconjunction of B" <<endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << endl << "SUCESS!" <<endl<<endl<<endl;
    }
    
    
    
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    //////  CLUSTERING POINT CLOUD BEFORE IBS CALCULATION
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
     std::cout << "STARTING test, IBS with pre-segmentation calculation " << endl<< endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene_filtered = Util::sphereExtraction(cloud_scene, middlePoint,radio);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_filtered = Util::sphereExtraction(cloud_object, middlePoint,radio);    
    
    
    IBS ibs_clouds_prefiltered ( cloud_scene_filtered, cloud_object_filtered );
    ibs_clouds_prefiltered.calculate();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibs_filtered = Util::sphereExtraction( ibs_clouds_prefiltered.getIBS(), middlePoint,radio); 
    
    
    pcl::io::savePCDFile("cloud_scene_clustered.pcd", *cloud_scene_filtered );
    pcl::io::savePCDFile("cloud_object_clustered.pcd",*cloud_object_filtered );
    pcl::io::savePCDFile("ibs_clouds_prefiltered.pcd",*ibs_clouds_prefiltered.getIBS()); //this is originally calculated 
    pcl::io::savePCDFile("ibs_clouds_prefiltered_filtered.pcd",*ibs_filtered );          //this is filtered after calculation
    
    
    IBS_Distance distanceIBS_clouds_prefiltered (*ibs_filtered, *ibs_whole);
    distanceIBS_clouds_prefiltered.compute();
    
    std::cout << "Distance: [" << "A->B: "  <<  distanceIBS_clouds_prefiltered.a_b_distance << ", B->A: "  << distanceIBS_clouds_prefiltered.b_a_distance  << endl;
    
     if ( distanceIBS_clouds_prefiltered.a_b_distance >= 0.001 ){
        std::cout << "ERROR! A is not a subconjunction of B" <<endl;
        return EXIT_FAILURE;
    }
    else{
        std::cout << endl << "SUCESS!" <<endl<<endl;
    }
    
    
    
    return EXIT_SUCCESS;
}
