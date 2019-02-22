#include "ibs.h"


IBS::IBS ( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
    is_ROI = false;
    
    sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    queryObjectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    bigCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    ibs.reset(new pcl::PointCloud<pcl::PointXYZ>);
     
     // to compute voronoi diagram over all points
    this->sceneCloud        = scene;
    this->queryObjectCloud  = object;
    
    pcl::copyPointCloud(*queryObjectCloud,*bigCloud);
    *bigCloud += *sceneCloud;
    
    
    this->soi_queryObjectCloud = this->queryObjectCloud;
    this->soi_sceneCloud = this->sceneCloud;
    this->soi_bigCloud = this->bigCloud;
    

}


void IBS::calculate(){
    
    Voronoi voronoi(this->soi_bigCloud);
    voronoi.calculate();

    std::vector<point_region_hull> regions_cloud_object;
    std::vector<point_region_hull> regions_cloud_scene;
    
    for (const point_region_hull t : voronoi.regionsVoronoi){
        if(t.parent_id < this->soi_queryObjectCloud->size())
            regions_cloud_object.push_back(t);
        else 
            regions_cloud_scene.push_back(t);
    }

    // At this point there are two sets of vertices ids
    // Ids comming from query object and scene object
    // We need to find commong ids and the points associated to
    // those vertices form the Bisector surface
    std::sort( regions_cloud_object.begin(), regions_cloud_object.end() );
    //vertices_one.resize( std::distance(vertices_one.begin(), std::unique( vertices_one.begin(), vertices_one.end() )   ) );
    regions_cloud_object.erase( std::unique( regions_cloud_object.begin(), regions_cloud_object.end() ), regions_cloud_object.end() );
    
    std::sort( regions_cloud_scene.begin(), regions_cloud_scene.end() );
    //vertices_two.resize( std::distance(vertices_two.begin(), std::unique( vertices_two.begin(), vertices_two.end() )   ) );
    regions_cloud_scene.erase( std::unique( regions_cloud_scene.begin(), regions_cloud_scene.end() ), regions_cloud_scene.end() );
    
    std::vector<point_region_hull> toSearch(regions_cloud_object);
    toSearch.insert(toSearch.end(),regions_cloud_scene.begin(),regions_cloud_scene.end());
    std::sort(toSearch.begin(),toSearch.end());
    
    // By now same ids (comming from different pointclouds) should be
    // together (one after the other) in the same container
    // Just need to loop through the container and detect 
    // same ids adjacent to each other.
    int k=0;
    
    while(!voronoi.vornoiVertex.empty())
    {
        if(k>=(toSearch.size()-1))
            break;
        int one=toSearch.at(k).region_id;
        int two=toSearch.at(k+1).region_id;
        if(one==two)    //Same ids
        {
            Eigen::Vector4f coord(voronoi.vornoiVertex.at(one).x, voronoi.vornoiVertex.at(one).y, voronoi.vornoiVertex.at(one).z,1);
            pcl::PointXYZ point(coord[0],coord[1],coord[2]);
            ibs->push_back(point); //A vertex (PCL point) is pushed to the vector
            k=k+2;
        }
        else
            k=k+1;
    }
    
    return;
}

void IBS::setROI ( pcl::PointXYZ pivotPoint, float radio ) {
    
    this->center_ROI = pivotPoint;
    this->radio_ROI = radio;
    
    this->soi_bigCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    this->soi_sceneCloud = Util::sphereExtraction(sceneCloud , pivotPoint, radio);
    this->soi_queryObjectCloud = Util::sphereExtraction(queryObjectCloud , pivotPoint, radio);
    
    pcl::copyPointCloud( *this->soi_queryObjectCloud, *this->soi_bigCloud );
    
    *this->soi_bigCloud += *this->soi_sceneCloud;
    
    this->is_ROI = true;
}




pcl::PointCloud<pcl::PointXYZ>::Ptr 
IBS::getIBS(){
    
    if(this->is_ROI )
        return Util::sphereExtraction(this->ibs, center_ROI, radio_ROI );
    else 
        return this->ibs;
    
}
