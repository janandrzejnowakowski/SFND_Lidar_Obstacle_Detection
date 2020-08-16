/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

    int max_inliers = 0;
    double a, b, c, d, distance, denominator;

    std::random_device random_device;
    std::mt19937 engine{random_device()};
    std::uniform_int_distribution<int> dist(0, cloud->points.size() - 1);

    while (maxIterations--) {
        std::vector<pcl::PointXYZ> p {};
        std::unordered_set<int> currentInliers;

        while (p.size() < 3) {
            int i = dist(engine);
            p.push_back(cloud->points[i]);
            currentInliers.insert(i);
        }

        a = (p[1].y - p[0].y)*(p[2].z - p[0].z) - (p[1].z - p[0].z)*(p[2].y - p[0].y);
        b = (p[1].z - p[0].z)*(p[2].x - p[0].x) - (p[1].x - p[0].x)*(p[2].z - p[0].z);
        c = (p[1].x - p[0].x)*(p[2].y - p[0].y) - (p[1].y - p[0].y)*(p[2].x - p[0].x);
        d = -(a*p[0].x + b*p[0].y + c*p[0].y);
//        std::cout << "------------------" << std::endl;
        denominator = std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));

//        std::cout << "denominator: " << denominator << std::endl;
//        std::cout << "First point: " << std::abs(a * cloud->points[0].x + b * cloud->points[0].y + c * cloud->points[0].z + d) / denominator << std::endl;
        for (int j = 0; j < cloud->points.size(); ++j) {
            pcl::PointXYZ curr_point = cloud->points[j];
            distance = std::abs(a * curr_point.x + b * curr_point.y + c * curr_point.z + d) / denominator;
            if (distance < distanceTol) {
                currentInliers.insert(j);
            }
        }
        if (currentInliers.size() > max_inliers) {
            max_inliers = currentInliers.size();
            inliersResult = currentInliers;
        }
    }
//    std::cout << "First point coords: " << cloud->points[0].x << ", " << cloud->points[0].y << ", " << cloud->points[0].z << std::endl;
//
//    std::cout << "Inliers: " << inliersResult.size() << std::endl;

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);

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
