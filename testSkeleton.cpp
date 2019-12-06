#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
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

using namespace pcl;
using namespace std;


int main()
{
	//��ʼ��PointCloud����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid(new pcl::PointCloud<pcl::PointXYZ>);

	//���ص����ļ�
	pcl::io::loadPCDFile("x-1.pcd", *cloud);

	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);

	//auto p = PointXYZ(centroid[0], centroid[1], centroid[2]);

	cloud_centroid->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));

	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_all(new pcl::visualization::PCLVisualizer("skeleton����"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
	viewer_all->addPointCloud(cloud, red, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_centroid, 0, 255, 0);
	viewer_all->addPointCloud(cloud_centroid, green, "skeleton");
	viewer_all->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "skeleton");
	viewer_all->addCoordinateSystem(1);

	while (!viewer_all->wasStopped())
	{

		viewer_all->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}



	cin.get();
	return 0;
}