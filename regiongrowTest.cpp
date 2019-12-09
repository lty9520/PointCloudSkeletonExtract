#include <iostream>
#include <string>



#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/common/time.h>

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
	//�ο�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>);


	////����mesh����
	//pcl::PolygonMesh mesh;
	////��ȡpolygon�ļ���obj��ʽ��ȡΪmesh
	//pcl::io::loadPolygonFile("chaijie-65-rotatest.obj", mesh);
	//
	//
	//
	////��mesh��ʽת��ΪPointCloud��ʽ �����ȡ
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud_align);
	////ת��Ϊ�ɶ�ȡ��PCD�ļ���ʽ
	//pcl::io::savePCDFileASCII("chaijie-65-rotatest.pcd", *cloud_align);

	pcl::io::loadPCDFile("z-33.pcd", *cloud);
	
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setKSearch(100);
	normal_estimator.compute(*normals);
	/*���������
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
*/
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(100);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(10);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(0.5);

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

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("myRegistration"));  //���崰�ڹ���ָ��
	

	viewer->addPointCloud(colored_cloud, "colored_cloud");
	viewer->setBackgroundColor(1, 1, 1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}



	






	return 0;
}