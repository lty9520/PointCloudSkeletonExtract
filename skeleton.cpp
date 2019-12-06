#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
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

using namespace pcl;
using namespace std;



//求数组最大值
double
max_array(double *array, int n)//array 为int类型的数组的指针 ，n为数组元素个数
{
	double max = array[0];
	for (int i = 0; i < n; i++)
	{
		if (max < array[i])
		{
			max = array[i];
		}
	}
	return max;
}

//求数组最小值
double
min_array(double *array, int n)//array 为int类型的数组的指针 ，n为数组元素个数
{
	double min = array[0];
	for (int i = 0; i < n; i++)
	{
		if (min > array[i])
		{
			min = array[i];
		}
	}
	return min;
}


//计算点云密度（点云间平均间距）
double
computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;

}




/**
**n->片数
**max_x ->切取方向最大值
**min_x ->切取方向最小值
**y->点云对应坐标集合
**z->点云对应坐标集合
**x->点云对应坐标集合
**cloud_in->输入点云
**size->点云大小
**res_cloud->点云密度
**cut_n->当前操作片

**/
pcl::PointCloud<pcl::PointXYZ>::Ptr cutPointCloud(int n, double max_x, double min_x, double* y, double* z, double* x,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, int cut_n, string filename, int mode_flag,
	//pcl::PointCloud<pcl::Boundary>& boundaries,
	double res_cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_boundaries
	)
{
	n = n - 1;

	double pitch = (max_x - min_x) / n;
	double * x_pitch = new  double[n];
	double * y_cuted = new double[size];
	double * z_cuted = new double[size];

	x_pitch[0] = min_x;
	//计算切片区间
	for (int b = 1; b < n + 2; b++) {
		x_pitch[b] = x_pitch[b - 1] + pitch;
		cout << "x_pitch[" << b << "] = " << x_pitch[b - 1] << endl;
	}
	//存储切片区间中点
	double * x_cuted = new double[n];
	for (int i = 0; i < n + 1; i++) {
		x_cuted[i] = x_pitch[i] + 0.5 * pitch;
	}
	//切片
	for (int k = cut_n; k < cut_n + 1; k++) {
		cout << "k =" << k << endl;
		//切片内点云数量
		int u = 0;

		for (int v = 0; v < size; v++) {

			if (x[v] >= x_pitch[k] && x[v] <= x_pitch[k + 1]) {
				y_cuted[u] = y[v];
				z_cuted[u] = z[v];
				u++;
			}
			continue;

		}
		cout << "number:" << u << endl;

		cloud_in->width = u;
		cloud_in->height = 1;
		cloud_in->points.resize(cloud_in->width * cloud_in->height);
		cloud_in->is_dense = true;

		switch (mode_flag)
		{
		//x轴切(x = x = x,y = y = y, z = z = z)
		case 0:
		//获取切片点云
			for (int i = 0; i < u; i++) {
				cloud_in->points[i].x = x_cuted[k];
				cloud_in->points[i].z = y_cuted[i];
				cloud_in->points[i].y = z_cuted[i];
			}
			break;
			
		//y轴切(y = x = y,z = y = z, x = z = x)
		case 1:
			//获取切片点云
			for (int i = 0; i < u; i++) {
				cloud_in->points[i].y = x_cuted[k];
				cloud_in->points[i].x = y_cuted[i];
				cloud_in->points[i].z = z_cuted[i];
			}
			break;
		//z轴切(z = x = z,y = y = y, x = z = x)
		case 2:
			//获取切片点云
			for (int i = 0; i < u; i++) {
				cloud_in->points[i].z = x_cuted[k];
				cloud_in->points[i].y = y_cuted[i];
				cloud_in->points[i].x = z_cuted[i];
			}
			break;
			
		default:
			break;
		}
		

		cloud_in->width = u;
		cloud_in->points.resize(cloud_in->width * cloud_in->height);

		cout << "point cloud_cuted has:" << cloud_in->points.size() << "data points" << endl;
		//计算切片后点云密度
		res_cloud = computeCloudResolution(cloud_in);
		std::cout << "cloud resolution = " << res_cloud << endl;

		
		
		
		if (cloud_in->size() == 0)
		{
			cout << "this picth has no data!" << endl;
		}
		else
		{
			pcl::io::savePCDFileASCII(filename, *cloud_in);
		}
		
		//boundaryEstimation(boundaries, cloud_in, res_cloud, cloud_boundaries);
	}

	
	return cloud_in;

}

//得到文件路径的目录(must with '/')
string GetPathDir(string filePath)
{
	string dirPath = filePath;
	size_t p = filePath.find_last_of('/');
	if (p != -1)
	{
		dirPath.erase(p);
	}
	return dirPath;
}

//创建多级目录
void CreateMultiLevel(string dir)
{
	if (_access(dir.c_str(), 00) == 0)
	{
		return;
	}

	list <string> dirList;
	dirList.push_front(dir);

	string curDir = GetPathDir(dir);
	while (curDir != dir)
	{
		if (_access(curDir.c_str(), 00) == 0)
		{
			break;
		}

		dirList.push_front(curDir);

		dir = curDir;
		curDir = GetPathDir(dir);
	}

	for (auto it : dirList)
	{
		_mkdir(it.c_str());
	}
}


//void normalEstimation(pcl::PointCloud<pcl::Normal>::Ptr& cloud_normal,
//	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input) {
//
//	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>nor;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	nor.setInputCloud(cloud_input);
//	nor.setNumberOfThreads(50);
//	nor.setRadiusSearch(0.01);
//	nor.setSearchMethod(tree);
//	//cout << "wojinliale" << endl;
//	nor.compute(*cloud_normal);
//}


int main()
{

	DWORD start_time = GetTickCount(); //开始计时
									   //初始化PointCloud对象
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
	pcl::io::loadPCDFile("zhengti-moxing.pcd", *cloud);
	
	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	//求点云size大小
	int size = cloud->points.size();
	double* x = new double[size];
	double* y = new double[size];
	double* z = new double[size];

	for (int i = 0; i < size; i++) {

		x[i] = cloud->points[i].x;
		y[i] = cloud->points[i].y;
		z[i] = cloud->points[i].z;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}
	
	//求y的最大值
	double max_y = max_array(y, size);
	//求y的最小值
	double min_y = min_array(y, size);
	//求z的最大值
	double max_z = max_array(z, size);
	//求z的最小值
	double min_z = min_array(z, size);
	//初始化边界对象
	//pcl::PointCloud<pcl::Boundary> boundaries;
	//初始化切片点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
	//初始化gujia点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_skeleton(new pcl::PointCloud <pcl::PointXYZ>);
	//初始化存储边界对象的点云
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries(new pcl::PointCloud <pcl::PointXYZ>);
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);

	//片数
	int n = 50;

	//点云密度
	double res_cloud = 0.0;

	//切片数
	int cut_n_3d = 0;
	int cut_n_2d = 0;

	for (cut_n_3d = 0; cut_n_3d < n - 1; cut_n_3d++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_2d(new pcl::PointCloud <pcl::PointXYZ>);
		string filepath_3d = "./cut_out_3d/z-" + to_string(cut_n_3d + 1);
		if (0 != access(filepath_3d.c_str(), 0))
		{
			// if this folder not exist, create a new one.
			//int flag_3d = mkdir(filepath_3d.c_str());   // 返回 0 表示创建成功，-1 表示失败
										 //换成 ::_mkdir  ::_access 也行，不知道什么意思
			CreateMultiLevel(filepath_3d);
		}
		string filename_3d = filepath_3d  + "/z-" + to_string(cut_n_3d + 1) + ".pcd";
		
		//z轴点云切片(3d->2d)
		cloud_cuted_3d = cutPointCloud(n, max_z, min_z, y, x, z, cloud_cuted_3d, size, cut_n_3d, filename_3d, 2, res_cloud);

		//创建存储点云重心的对象
		Eigen::Vector4f centroid;
		//计算重心点
		pcl::compute3DCentroid(*cloud_cuted_3d, centroid);
		//添加重心到骨架中
		cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));


		int size_2d = cloud_cuted_3d->size();
		double* x_2d = new double[size_2d];
		double* y_2d = new double[size_2d];
		double* z_2d = new double[size_2d];
		for (int i = 0; i < size_2d; i++) {

			x_2d[i] = cloud_cuted_3d->points[i].x;
			y_2d[i] = cloud_cuted_3d->points[i].y;
			z_2d[i] = cloud_cuted_3d->points[i].z;
			//fout << x[i];
			//fout << "\n";
			//fout << flush;
		}
		//求x的最大值
		double max_x = max_array(x_2d, size_2d);
		//求x的最小值
		double min_x = min_array(x_2d, size_2d);
		double res_cloud_2d = 0.0;
		/*
		for (cut_n_2d = 0; cut_n_2d < n - 1; cut_n_2d++)
		{

			string filepath_2d = filepath_3d + "/cut_2d/x";
			if (0 != access(filepath_2d.c_str(), 0))
			{
				// if this folder not exist, create a new one.
				CreateMultiLevel(filepath_2d);   // 返回 0 表示创建成功，-1 表示失败
											  //换成 ::_mkdir  ::_access 也行，不知道什么意思
			}
			string filename_2d = filepath_2d + "/x-" + to_string(cut_n_2d + 1) + ".pcd";
			//x轴点云切片(2d->1d)
			cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, y_2d, x_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 0, res_cloud_2d);
			//x轴点云切片(2d->1d)
			//cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, x_2d, y_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 1, res_cloud_2d);

			//创建存储点云重心的对象
			Eigen::Vector4f centroid;
			//计算重心点
			pcl::compute3DCentroid(*cloud_cuted_2d, centroid);
			//添加重心到骨架中
			cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
		}
		*/
	}

	pcl::io::savePCDFileASCII("cloud_skeleton.pcd", *cloud_skeleton);

	//创建viewer对象
	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("原始点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
	viewer_ori->addPointCloud(cloud, red, "cloud");
	viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer_ori->addCoordinateSystem(1);

	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cut(new pcl::visualization::PCLVisualizer("qiepian点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color_handler(cloud, 255, 0, 0);
	//viewer_cut->addPointCloud(cloud, red, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_skeleton, 0, 255, 0);
	viewer_cut->addPointCloud(cloud_skeleton, green, "cut");
	//viewer_cut->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cut");
	viewer_cut->addCoordinateSystem(1);

	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_all(new pcl::visualization::PCLVisualizer("skeleton点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color_handler(cloud, 255, 0, 0);
	viewer_all->addPointCloud(cloud, red, "cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_cuted, 0, 255, 0);
	viewer_all->addPointCloud(cloud_skeleton, green, "skeleton");
	viewer_all->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "skeleton");
	viewer_all->addCoordinateSystem(1);

	while (!viewer_ori->wasStopped())
	{

		viewer_ori->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	while (!viewer_cut->wasStopped())
	{

		viewer_cut->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	while (!viewer_all->wasStopped())
	{

		viewer_all->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	cin.get();
	return 0;
}