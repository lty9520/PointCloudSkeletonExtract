#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���ͷ�ļ�
#include <pcl/sample_consensus/model_types.h>   //ģ�Ͷ���ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ�����ͷ�ļ�
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>			//�߽���ȡͷ�ļ�
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

// �������������ص����棬�������ѡ�㽫�ᱻ���������
bool
customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// Do whatever you want here.����������������ɸѡ
	if (squaredDistance > 0.0000001)  //�����ѡ���Y��ֵС�����ӵ��Yֵ������֮ǰ��ѡ��Ϊ����ĵ㣩�����������������ؼ�
		return false;

	return true;
}

int main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	////pushback
	////cloud->push_back(PointXYZ(0,0,0));
	//
	////����mesh����
	//pcl::PolygonMesh mesh;
	////��ȡpolygon�ļ���obj��ʽ��ȡΪmesh
	////pcl::io::loadPolygonFile("zhengti-dy.obj", mesh);
	//pcl::io::loadPolygonFilePLY("zhengti-moxing.ply", mesh);
	//
	////��ʼ������洢����final
	//pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	////��mesh��ʽת��ΪPointCloud��ʽ �����ȡ
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	////ת��Ϊ�ɶ�ȡ��PCD�ļ���ʽ
	//pcl::io::savePCDFileASCII("zhengti-moxing.pcd", *cloud);
	//
	////������������
	//cout << cloud->size() << endl;
	//cout << "OK!";



	//���ص����ļ�
	pcl::io::loadPCDFile("z-33.pcd", *cloud);

	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	std::vector<pcl::PointIndices> cluster_indices;
	/*��ͳŷʽ����ָ�*/
	
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //ŷʽ�������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ec.setInputCloud(cloud);
	ec.setClusterTolerance(0.01);                     // ���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(10);                 //����һ��������Ҫ�����ٵĵ���ĿΪ100
	ec.setMaxClusterSize(25000);               //����һ��������Ҫ��������ĿΪ25000
	ec.setSearchMethod(tree);                    //���õ��Ƶ���������
	ec.extract(cluster_indices);           //�ӵ�������ȡ���࣬������������������cluster_indices��
										   //�������ʵ�������cluster_indices,ֱ���ָ���о���
	

	/*����ŷʽ����ָ�
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	clustering.setClusterTolerance(0.01);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	//����ÿ�μ��һ�Ե���ʱ�ĺ���
	clustering.setConditionFunction(&customCondition);
	//std::vector<pcl::PointIndices> clusters;
	clustering.segment(cluster_indices);
*/
	cout << "number of cluster is :" << cluster_indices.size() << endl;
	

	//����viewer����
	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("ԭʼ����"));
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
			//���ñ�����Ƶ���������
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