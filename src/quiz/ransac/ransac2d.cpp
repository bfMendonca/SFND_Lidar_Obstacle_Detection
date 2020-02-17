/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

double distanceToLine( double A, double B, double C, pcl::PointXYZ& p ) {
  return fabs( A*p.x + B*p.y + C )/sqrt( pow(A,2) + pow(B,2) );
}

double distancetoPlane( double A, double B, double C, double D, pcl::PointXYZ &p ) {
  return fabs( A*p.x + B*p.y + C*p.z + D )/(sqrt( pow(A,2) + pow(B,2) +pow(C,2) ) );
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
  double p1Rand = 0;
  double p2Rand = 0;

  size_t p1Index;
  size_t p2Index;

  double A, B, C;
  size_t inliers;
  std::unordered_set<int> partialResult;

  double bestA, bestB, bestC;
  size_t bestInliers = 0;

  double distance;

  for( int i = 0; i < maxIterations; ++i ) {
    inliers = 0;
    partialResult.clear();

    p1Rand = static_cast< double >( std::rand() )/static_cast< double >( RAND_MAX );    
    p1Index = round( p1Rand*cloud->size() );

    do {
      p2Rand = static_cast< double >( std::rand() )/static_cast< double >( RAND_MAX );    
      p2Index = round( p2Rand*cloud->size() );
    }while( p1Index == p2Index );

    partialResult.insert( p1Index );
    partialResult.insert( p2Index );
    
    //Okay,now let's get the line parameters A, B and C
    pcl::PointXYZ p1, p2;

    p1 = (*cloud)[p1Index];
    p2 = (*cloud)[p2Index];

    A = p1.y - p2.y;
    B = p2.x - p1.x;
    C = p1.x*p2.y - p2.x*p1.y;

    for( size_t j = 0; j < cloud->size(); ++j ) {
      if( j == p1Index || j == p2Index ) {
        continue;
      }

      distance = distanceToLine( A, B, C, (*cloud)[j] );

      if( distance < distanceTol ) {
        ++inliers;
        partialResult.insert( j );
      }
    }

    if( inliers > bestInliers ) {
      bestA = A;
      bestB = B;
      bestC = C;
      bestInliers = inliers;
      inliersResult = partialResult;
    }

   }
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
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

    double x1, y1, z1, x2, y2, z2, x3, y3, z3;

    x1 = p1.x;
    y1 = p1.y;
    z1 = p1.z;

    x2 = p2.x;
    y2 = p2.y;
    z2 = p2.z;

    x3 = p3.x;
    y3 = p3.y;
    z3 = p3.z;

    A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
    B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
    C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    D = -1.0*(A*x1 + B*y1 + C*z1 );

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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

  //Creating 3d data for plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
