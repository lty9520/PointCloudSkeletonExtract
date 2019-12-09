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
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

using namespace pcl;
using namespace std;




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
	pcl::io::loadPCDFile("zhengti-moxing.pcd", *cloud);

	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;


	bool disable_transform = false;

	//�趨�ᾧ����
	float voxel_resolution = 0.008f;
	float seed_resolution = 0.001f;
	float color_importance = 0.02f;
	float spatial_importance = 0.04f;
	float normal_importance = 1.0f;

	//���ɽᾧ��
	pcl::SupervoxelClustering<PointXYZ> super(voxel_resolution, seed_resolution);
	//�͵�����ʽ�й�
	if (disable_transform)
		super.setUseSingleCameraTransform(false);
	//������Ƽ��ᾧ����
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	//����ᾧ�ָ����������һ��ӳ���
	std::map <uint32_t, pcl::Supervoxel<PointXYZ>::Ptr > supervoxel_clusters;
	super.extract(supervoxel_clusters);
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	//CPC����
	int max_cuts;
	int cutting_min_segments;
	double cutting_min_score;
	bool locally_constrained;
	bool directed_cutting;
	bool clean_cutting;
	uint32_t ransac_iterations = 2;

	//����CPC�ָ���
	pcl::CPCSegmentation<PointXYZ> seg;
	pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud_arg(new pcl::PointCloud<pcl::PointXYZL>);
	//pcl::CPCSegmentation<PointXYZ>::CPCSegmentation seg;
	//���볬�������
	seg.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	//���÷ָ����
	seg.setCutting(max_cuts = 10,
		cutting_min_segments = 0,
		cutting_min_score = 0.001,
		locally_constrained = true,
		directed_cutting = true,
		clean_cutting = false);
	seg.setRANSACIterations(ransac_iterations);
	seg.segment();
	seg.relabelCloud(*labeled_cloud_arg);

	//����viewer����
	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("ԭʼ����"));
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> red(labeled_cloud_arg, 255, 0, 0);
	viewer_ori->addPointCloud(labeled_cloud_arg, red, "cloud");
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