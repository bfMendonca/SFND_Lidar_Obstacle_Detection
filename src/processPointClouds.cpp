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


    //Solution based on the tutorial http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    //Actually the segmenting class    
    pcl::SACSegmentation<PointT> seg;

    //Coefficientes for the solution. It contains. roughtly, the parameters for the extraction that will be performed
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients () );
    //Class to store the indices of the points that are, actually, inliers. Those are the points that are supposed to belong 
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices () );
    
    // Optional - Don't know what it means, look at the docs latter
    seg.setOptimizeCoefficients (true);

    //Setting the parameters for the segmentation
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations ( maxIterations );
    seg.setDistanceThreshold ( distanceThreshold );

    //Setting the input cloud to the segmentation class
    seg.setInputCloud( cloud );

    //Performing the segmentation
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
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

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree< PointT >::Ptr tree (new pcl::search::KdTree< PointT >);
    tree->setInputCloud (cloud);

    //Setting properties for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction< PointT > ec;
    ec.setClusterTolerance ( clusterTolerance ); // 2cm
    ec.setMinClusterSize ( minSize );
    ec.setMaxClusterSize ( maxSize );
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


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