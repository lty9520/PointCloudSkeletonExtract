#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>			//边界提取头文件
#include <direct.h>
#include <imagehlp.h>
#include <time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

using namespace pcl;
using namespace std;

#define MAX_COLOR_NUMBER 256
#define random(x) (rand()%x)
#define COLOR_NUM 3


vector<int> CreateRandomNums(int min, int max, int num)
{
	vector<int> res;
	res.clear();
	if (max - min + 1 < num)
		return res;
	

	srand(time(0));

	for (auto i{ 0 }; i < num; i++)
	{
		while (true)
		{
			auto temp{ rand() % (max + 1 - min) + min };
			auto iter{ find(res.begin(), res.end(), temp) };
			if (res.end() == iter)
			{
				res.push_back(temp);
				break;
			}
		}
	}
	return res;
}

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> random_Color(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	vector<int> rgb = CreateRandomNums(0, 255, COLOR_NUM);
	int r = rgb[0];
	int g = rgb[1];
	int b = rgb[2];

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rand_Color(cloud, r, g, b);
	
	return rand_Color;
}

// 如果这个函数返回的是真，这这个候选点将会被加入聚类中
bool
customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// Do whatever you want here.做你想做的条件的筛选
	if (squaredDistance > 0.0000001)  //如果候选点的Y的值小于种子点的Y值（就是之前被选择为聚类的点），则不满足条件，返回假
		return false;

	return true;
}

int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	////pushback
	////cloud->push_back(PointXYZ(0,0,0));
	//
	////创建mesh对象
	//pcl::PolygonMesh mesh;
	////读取polygon文件，obj格式读取为mesh
	////pcl::io::loadPolygonFile("zhengti-dy.obj", mesh);
	//pcl::io::loadPolygonFilePLY("zhengti-moxing.ply", mesh);
	//
	////初始化结果存储点云final
	//pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	////将mesh格式转换为PointCloud格式 方便读取
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	////转存为可读取的PCD文件格式
	//pcl::io::savePCDFileASCII("zhengti-moxing.pcd", *cloud);
	//
	////可输出点的数量
	//cout << cloud->size() << endl;
	//cout << "OK!";



	//加载点云文件
	pcl::io::loadPCDFile("z-33.pcd", *cloud);

	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	std::vector<pcl::PointIndices> cluster_indices;
	/*传统欧式距离分割*/
	
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ec.setInputCloud(cloud);
	ec.setClusterTolerance(0.01);                     // 设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(10);                 //设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(25000);               //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                    //设置点云的搜索机制
	ec.extract(cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
										   //迭代访问点云索引cluster_indices,直到分割处所有聚类
	

	/*条件欧式距离分割
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	clustering.setClusterTolerance(0.01);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	//设置每次检测一对点云时的函数
	clustering.setConditionFunction(&customCondition);
	//std::vector<pcl::PointIndices> clusters;
	clustering.segment(cluster_indices);
*/
	cout << "number of cluster is :" << cluster_indices.size() << endl;
	

	//创建viewer对象
	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("原始点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");


	//vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster;
	int n = 0;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		temp->width = cluster_indices[n].indices.size();
		temp->height = 1;
		temp->points.resize(temp->width * temp->height);
		temp->is_dense = true;
		
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			//设置保存点云的属性问题
			temp->points.push_back(cloud->points[*pit]); //*
		temp->width = temp->points.size();
		temp->height = 1;
		temp->is_dense = true;
		//cloud_cluster.push_back(temp);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rand_color =  random_Color(temp);
		string cloudid = "cloud" + to_string(n);
		viewer_ori->addPointCloud(temp, rand_color, cloudid);
		n++;
	}
	

	
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
	
	//viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer_ori->addCoordinateSystem(1);

	while (!viewer_ori->wasStopped())
	{

		viewer_ori->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}





	cin.get();
	return 0;
}