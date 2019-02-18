#include "voronoi.h"


Voronoi::Voronoi ( PointCloudT::Ptr pointCloud )
{
    
    // to compute voronoi diagram over all points
    bigCloud.reset(new PointCloudT);
    *bigCloud += *pointCloud;
    
    vornoiVertex.clear();
    regionsVoronoi.clear();
    
    // Loop through the cloud coordinates and copy them into a Coordinates object/array
    double aux[3];
    for(int i=0;i<bigCloud->size();i++)
    {
        aux[0] =bigCloud->at(i).x;
        aux[1] =bigCloud->at(i).y;
        aux[2] =bigCloud->at(i).z;
        cloud_coord.append(DIM,aux);
    }

    
}

void Voronoi::calculate()
{
    
     orgQhull::Qhull qhull;
    
    #if DEBUG
    std::cout<<" Runing Qhull Voronoi"<<endl;
    #endif

    // Call the lib with -v flag to get voronoi diagram
    qhull.runQhull("",DIM,int(bigCloud->size()),&cloud_coord.at(0),"v");
    // Only way to read output data (that I find) is to read the output string and parse everything
    qhull.outputQhull("o");
    
    #if DEBUG    
    std::cout<<"Done ... Reading output "<<endl;
    #endif
    

    //Start reading the output string and parse it

    assert(qhull.hasQhullMessage());
    
    std::string line;
    std::istringstream qhull_stream (qhull.qhullMessage());
    
    
    //First line is point dimension, don't use it now but could be useful
    std::getline( qhull_stream,line);   
    //Second line is:  #vetex #inputs 1
    std::getline( qhull_stream,line);
    
    //tokenizer to deal with Qhull output
    boost::tokenizer<> tok(line);
    boost::tokenizer<>::iterator iterator = tok.begin();
    
    
    
    //read vertex
    int total_vertices;
    
    total_vertices = std::atoi( (*iterator).c_str() );
    
    vornoiVertex.reserve(total_vertices);
    
    int num_vertex=0;
    
    while( num_vertex < total_vertices)
    {  
        std::getline( qhull_stream,line);
        
        
        iterator = tok.begin();
        
        pcl::PointXYZ vertex(0,0,0);
        vertex.x = std::atof((*iterator ).c_str());
        iterator++;
        vertex.y = std::atof((*iterator ).c_str());
        iterator++;
        vertex.z = std::atof((*iterator ).c_str());
                
        vornoiVertex.push_back( vertex);
        num_vertex++;
    }
    
    #if DEBUG
    std::cout <<"num vertex: " << total_vertices << " " << this->vornoiVertex.size() <<std::endl;
    #endif
    
    //Get id of voronoi vertices comprising each region of point cloud
    int num_point_in_cloud=0;
 
    
    //Get id of voronoi vertices comprising each region of scene cloud
    regionsVoronoi.reserve( bigCloud->size() * 6 ); //guess the size of vector for better performance
    
    while( std::getline( qhull_stream,line) )
    {  
        
        tok = boost::tokenizer<>(line);
        
        iterator = tok.begin();

        for( ++iterator; iterator!=tok.end();++iterator ){
            point_region_hull a = { num_point_in_cloud, std::atoi((*iterator ).c_str()) };
            regionsVoronoi.push_back(a);
        }
        num_point_in_cloud++;

    }
    
    #if DEBUG
    std::cout << "cloud size: " << bigCloud->size() << " " << num_point_in_cloud << std::endl;
    #endif
    
    qhull.clearQhullMessage();
    
}
