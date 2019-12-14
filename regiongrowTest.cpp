#include <iostream>
#include <string>



#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/common/time.h>
#include <pcl/surface/mls.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

using namespace std;
using namespace pcl;

int main()
{

	pcl::StopWatch time;
	//参考点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//对齐点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>);


	////创建mesh对象
	//pcl::PolygonMesh mesh;
	////读取polygon文件，obj格式读取为mesh
	//pcl::io::loadPolygonFile("chaijie-65-rotatest.obj", mesh);
	//
	//
	//
	////将mesh格式转换为PointCloud格式 方便读取
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud_align);
	////转存为可读取的PCD文件格式
	//pcl::io::savePCDFileASCII("chaijie-65-rotatest.pcd", *cloud_align);

	pcl::io::loadPCDFile("y-19.pcd", *cloud);
	
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setKSearch(200);
	//normal_estimator.compute(*normals);

	//创建mls对象
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	//   pcl::MovingLeastSquares<point,point> mls;

	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true); //设置为true则在平滑过程中采用多项式拟合来提高精度
	mls.setPolynomialOrder(4); //MLS拟合的阶数，默认是2
	mls.setSearchMethod(tree);
	mls.setSearchRadius(5);  //搜索半径

	mls.process(*mls_points);

	normals->resize(mls_points->size());
	for (int i = 0; i < mls_points->size(); i++)
	{
		normals->points[i].normal_x = mls_points->points[i].normal_x;
		normals->points[i].normal_y = mls_points->points[i].normal_y;
		normals->points[i].normal_z = mls_points->points[i].normal_z;
		normals->points[i].curvature = mls_points->points[i].curvature;
	}

	/*坐标过滤器
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
*/
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(300);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(200);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(60.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(2);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;

	int n = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto temp : clusters)
	{
		cloud_save->width = temp.indices.size();
		cloud_save->height = 1;
		cloud_save->points.resize(cloud_save->width * cloud_save->height);
		cloud_save->is_dense = true;
		for (auto pit = temp.indices.begin(); pit != temp.indices.end(); ++pit)
		{
			cloud_save->points.push_back(cloud->points[*pit]);
		}
		cloud_save->width = cloud_save->points.size();
		cloud_save->height = 1;
		cloud_save->is_dense = false;
		string filename = "./test/region-" + to_string(n) + ".pcd";
		pcl::io::savePCDFileASCII(filename, *cloud_save);
		n++;
	}

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("myRegistration"));  //定义窗口共享指针
	

	viewer->addPointCloud(colored_cloud, "colored_cloud");
	viewer->setBackgroundColor(1, 1, 1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}



	






	return 0;
}