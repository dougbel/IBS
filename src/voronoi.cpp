#include "voronoi.h"


Voronoi::Voronoi ( pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud )
{
    
    // to compute voronoi diagram over all points
    bigCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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
    
    
    #if !NDEBUG
    std::cout<<"Runing Qhull Voronoi"<<endl;
    #endif

    // Call the lib with -v flag to get voronoi diagram
    qhull.runQhull("",DIM,int(bigCloud->size()),&cloud_coord.at(0),"v");
    // Only way to read output data (that I find) is to read the output string and parse everything
    qhull.outputQhull("o");
    
    #if !NDEBUG
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
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(line, sep);
    boost::tokenizer<boost::char_separator<char> >::iterator iterator = tok.begin();
    
    //read vertex
    int total_vertices;
    
    total_vertices = std::stoi(*iterator);
    
    vornoiVertex.reserve(total_vertices);
    
    int num_vertex=0;
    
    while( num_vertex < total_vertices)
    {  
        std::getline( qhull_stream,line);
        
        tok =boost::tokenizer<boost::char_separator<char> >(line, sep);
        iterator = tok.begin();
        
        pcl::PointXYZ vertex(0,0,0);
        vertex.x = std::stod(*iterator );
        iterator++;
        vertex.y = std::stod(*iterator );
        iterator++;
        vertex.z = std::stod(*iterator );
                
        vornoiVertex.push_back( vertex);
        num_vertex++;
    }
    
    #if !NDEBUG
    std::cout <<"num vertex: " << total_vertices << " " << this->vornoiVertex.size() <<std::endl;
    assert(total_vertices == this->vornoiVertex.size());
    #endif
    
    //Get id of voronoi vertices comprising each region of point cloud
    int num_point_in_cloud=0;
 
    
    //Get id of voronoi vertices comprising each region of scene cloud
    regionsVoronoi.reserve( bigCloud->size() * 6 ); //guess the size of vector for better performance
    
    while( std::getline( qhull_stream,line) )
    {  
        
        tok =boost::tokenizer<boost::char_separator<char> >(line, sep);
        iterator = tok.begin();
        
        for( ++iterator; iterator!=tok.end();iterator++ ){
            point_region_hull a = { num_point_in_cloud, std::stoi(*iterator ) };
            regionsVoronoi.push_back(a);
        }
        num_point_in_cloud++;

    }
    
    #if !NDEBUG
    std::cout << "cloud size: " << bigCloud->size() << " " << num_point_in_cloud << " voronoi regions: " << regionsVoronoi.size() << std::endl;
    assert(bigCloud->size() == num_point_in_cloud);
    #endif
    
    qhull.clearQhullMessage();
    
}
