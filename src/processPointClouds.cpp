// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered( new pcl::PointCloud<PointT> );

    //First, just applying a simple voxel filter
    pcl::VoxelGrid< PointT > sor;
    sor.setInputCloud( cloud );
    sor.setLeafSize( filterRes, filterRes, filterRes );
    sor.filter( *cloudFiltered );

    //Second, applygin the "cropbox" to "focus" n the important areas of the image
    pcl::CropBox< PointT > cropboxFilter(true);
    cropboxFilter.setMax( maxPoint );
    cropboxFilter.setMin( minPoint );
    cropboxFilter.setInputCloud( cloudFiltered );
    cropboxFilter.filter( *cloudFiltered );


    
    //Third, We will create and apply a third cropbox to remove the "car" from the pontcloud
    std::vector< int > carIndices;

    pcl::CropBox< PointT > egoCarFilter(true);
    egoCarFilter.setMin( Eigen::Vector4f( -1.5, -1.7, -1, 1 ) );
    egoCarFilter.setMax( Eigen::Vector4f( 2.6, 1.7, -0.4, 1 ) );
    egoCarFilter.setInputCloud( cloudFiltered );
    egoCarFilter.filter( carIndices );

    //Car Indices now contains all the indices of the poiints that are on the car. Let's now remove those points from the pointcloud

    pcl::PointIndices::Ptr egoCarIndices( new pcl::PointIndices() );
    
    for( int i : carIndices ) {
        egoCarIndices->indices.push_back( i );
    }

    pcl::ExtractIndices<PointT> carIndicesExtractor;
    carIndicesExtractor.setInputCloud (cloudFiltered);
    carIndicesExtractor.setIndices (egoCarIndices);
    carIndicesExtractor.setNegative (true);
    carIndicesExtractor.filter( *cloudFiltered );


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    //Removing the part of the cloud considered to be part of an plane
    //Filtering Object - Probably the object that will separate the inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    typename pcl::PointCloud<PointT>::Ptr cloud_p( new pcl::PointCloud<PointT>() ), cloud_f( new pcl::PointCloud<PointT>() );

    //Copying the plane part to the cloud_p pointer
    extract.filter (*cloud_p);

    extract.setNegative( true );
    extract.filter (*cloud_f);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold )
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers( new pcl::PointIndices() );
    
    if( cloud->size() > 0 ) {
        //For some reason, we receive a cloud with size zero, and, if that happens, we get an error at our ransac method regarding a division by zero
        std::unordered_set<int> indices = RansacPlane( cloud, maxIterations, distanceThreshold );

        if (indices.size () == 0)
        {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        //We need to transform our indices int an PCL TYPE Index for later extraction
        for( int i : indices ) {
            inliers->indices.push_back( i );
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud< PointT >::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize )
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    typename std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> cluster_indices;

    KdTree* tree = new KdTree;
    
    //This could be better if we make an Kd Tree teample type, but going "easy" for now
    std::vector< float > point;
    point.resize( 3 );

    for (int i=0; i< cloud->size(); i++) {
        point[0] = (*cloud)[i].x;
        point[1] = (*cloud)[i].y;
        point[2] = (*cloud)[i].z;
        tree->insert(point,i); 
    }

    std::vector<std::vector<int>> clustersIndices = euclideanCluster( cloud, tree, clusterTolerance, minSize, maxSize );   
    
     for( std::vector<int> c : clustersIndices ) {        
        pcl::PointIndices newCluster;
        for( int i : c ) {
            newCluster.indices.push_back( i );
        }
        cluster_indices.push_back(newCluster);
    }

    delete tree;


    for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it )
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT> );

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back ( cloud->points[*pit] ); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back( cloud_cluster );
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {

  std::unordered_set<int> inliersResult;
  srand(time(NULL));  

  std::unordered_set<int> partialResult;

  double bestA, bestB, bestC, bestD; 
  double distance;

  double A, B, C, D;

  for( int i = 0; i < maxIterations; ++i ) {
    
    partialResult.clear();
    
    while( partialResult.size() < 3 ) {
      partialResult.insert( std::rand() % cloud->size() );  
    }

    auto iter = partialResult.begin();

    auto p1 = cloud->points[*iter];
    ++iter;
    auto p2 = cloud->points[*iter];
    ++iter;
    auto p3 = cloud->points[*iter];

    A = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
    B = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
    C = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
    D = -1.0*(A*p1.x + B*p1.y + C*p1.z );

    for( int j = 0; j < cloud->size(); ++j ) {
      distance = distancetoPlane( A, B, C, D, cloud->points[j] );

      if( distance < distanceTol ) {
        partialResult.insert( j );
      }
    }

    if( partialResult.size() > inliersResult.size() ) {
      inliersResult = partialResult;
    }

   }

  // For max iterations 

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers
  
  return inliersResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity( int index, std::vector<int> & cluster, std::vector< bool > &processedPoints, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol ) {

    processedPoints[index] = true;
    cluster.push_back( index );

    const PointT cloudPoint  = (*cloud)[ index ];
    std::vector< float > p( 3, 0.0 );       
    p[0] = cloudPoint.x;
    p[1] = cloudPoint.y;
    p[2] = cloudPoint.z;

    std::vector<int> nearbyPoints = tree->search( p,  distanceTol );

    for( int j = 0; j < nearbyPoints.size(); ++j ) {
        if( !processedPoints[nearbyPoints[j]] ) {
            proximity( nearbyPoints[j], cluster, processedPoints, cloud, tree, distanceTol );
        }      
    }

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster( typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize )
{
    std::vector< std::vector<int> > clusters;

    std::vector<bool> processedPoints( cloud->size(), false);

    for( int i = 0; i < cloud->size(); ++i ) {
        if( !processedPoints[i] ) {
            std::vector<int> newCluster;
            proximity( i, newCluster, processedPoints, cloud, tree, distanceTol );

            if( newCluster.size() > minSize ) {
                clusters.push_back( newCluster );    
            }

            
        }
    }

    return clusters;
}