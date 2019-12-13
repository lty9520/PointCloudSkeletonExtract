#include <windows.h>
#include <windef.h>
#include <wincrypt.h>
#include <iostream>
#include <direct.h>
#include <imagehlp.h>
#include <iostream>
#include <fstream>
#include <iosfwd>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
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
#include <pcl/segmentation/region_growing.h>



using namespace pcl;
using namespace std;



//���������ֵ
double
max_array(double *array, int n)//array Ϊint���͵������ָ�� ��nΪ����Ԫ�ظ���
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

//��������Сֵ
double
min_array(double *array, int n)//array Ϊint���͵������ָ�� ��nΪ����Ԫ�ظ���
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


//��������ܶȣ����Ƽ�ƽ����ࣩ
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
**n->Ƭ��
**max_x ->��ȡ�������ֵ
**min_x ->��ȡ������Сֵ
**y->���ƶ�Ӧ���꼯��
**z->���ƶ�Ӧ���꼯��
**x->���ƶ�Ӧ���꼯��
**cloud_in->�������
**size->���ƴ�С
**res_cloud->�����ܶ�
**cut_n->��ǰ����Ƭ

**/
pcl::PointCloud<pcl::PointXYZ>::Ptr cutPointCloud(int n, double max_x, double min_x, double* y, double* z, double* x,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, int cut_n, string filename, int mode_flag,
	//pcl::PointCloud<pcl::Boundary>& boundaries,
	double res_cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_boundaries
	, ofstream& _my_log
	)
{
	n = n - 1;

	double pitch = (max_x - min_x) / n;
	double * x_pitch = new  double[n];
	double * y_cuted = new double[size];
	double * z_cuted = new double[size];

	x_pitch[0] = min_x;
	//������Ƭ����
	for (int b = 1; b < n + 2; b++) {
		x_pitch[b] = x_pitch[b - 1] + pitch;
		_my_log << "x_pitch[" << b << "] = " << x_pitch[b - 1] << endl;
	}
	//�洢��Ƭ�����е�
	double * x_cuted = new double[n];
	for (int i = 0; i < n + 1; i++) {
		x_cuted[i] = x_pitch[i] + 0.5 * pitch;
	}
	//��Ƭ
	for (int k = cut_n; k < cut_n + 1; k++) {
		_my_log << "k =" << k << endl;
		//��Ƭ�ڵ�������
		int u = 0;

		for (int v = 0; v < size; v++) {

			if (x[v] >= x_pitch[k] && x[v] <= x_pitch[k + 1]) {
				y_cuted[u] = y[v];
				z_cuted[u] = z[v];
				u++;
			}
			continue;

		}
		_my_log << "number:" << u << endl;

		cloud_in->width = u;
		cloud_in->height = 1;
		cloud_in->points.resize(cloud_in->width * cloud_in->height);
		cloud_in->is_dense = true;

		switch (mode_flag)
		{
		//x����(x = x = x,y = y = y, z = z = z)
		case 0:
		//��ȡ��Ƭ����
			for (int i = 0; i < u; i++) {
				cloud_in->points[i].x = x_cuted[k];
				cloud_in->points[i].z = y_cuted[i];
				cloud_in->points[i].y = z_cuted[i];
			}
			break;
			
		//y����(y = x = y,z = y = z, x = z = x)
		case 1:
			//��ȡ��Ƭ����
			for (int i = 0; i < u; i++) {
				cloud_in->points[i].y = x_cuted[k];
				cloud_in->points[i].x = y_cuted[i];
				cloud_in->points[i].z = z_cuted[i];
			}
			break;
		//z����(z = x = z,y = y = y, x = z = x)
		case 2:
			//��ȡ��Ƭ����
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

		std::cout << "point cloud_cuted has:" << cloud_in->points.size() << "data points" << endl;
		//������Ƭ������ܶ�
		res_cloud = computeCloudResolution(cloud_in);
		_my_log << "cloud resolution = " << res_cloud << endl;

		
		
		
		if (cloud_in->size() == 0)
		{
			_my_log << "this picth has no data!" << endl;
		}
		else
		{
			pcl::io::savePCDFileASCII(filename, *cloud_in);
		}
		
		//boundaryEstimation(boundaries, cloud_in, res_cloud, cloud_boundaries);
	}

	
	return cloud_in;

}


vector <pcl::PointIndices> regionGrow(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in)
{


	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setInputCloud(cloud_in);
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
	reg.setMinClusterSize(500);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(20);
	reg.setInputCloud(cloud_in);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(60.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(5);

	vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	//std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

	return clusters;

}

//�õ��ļ�·����Ŀ¼(must with '/')
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

//�����༶Ŀ¼
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
//	//std::cout << "wojinliale" << endl;
//	nor.compute(*cloud_normal);
//}


//************************************
// Method:    CalcRatio
// FullName:  CalcRatio
// Access:    public 
// Returns:   double			��һ������ڶ�����������
// Qualifier:
// Parameter: double max_first	��һ��������ֵ
// Parameter: double min_first	��һ�������Сֵ
// Parameter: double max_second	�ڶ���������ֵ
// Parameter: double min_second	�ڶ��������Сֵ
//************************************
double CalcRatio(double max_first, double min_first, double max_second, double min_second)
{

	double dis_x_2d = max_first - min_first;
	double dis_y_2d = max_second - min_second;

	return abs(dis_x_2d) / abs(dis_y_2d);

}



//************************************
// Method:    CreateSkeleton
// FullName:  CreateSkeleton
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_skeleton	�Ǽ��ߵ���
// Parameter: int n	��Ƭ��
// Parameter: string direction	��Ƭ���򣨴������ļ�����
// Parameter: double max_z	��Ƭ�������ֵ
// Parameter: double min_z	��Ƭ������Сֵ
// Parameter: double * y	ԭʼ����yֵ����
// Parameter: double * z	ԭʼ����zֵ����
// Parameter: double * x	ԭʼ����xֵ����
// Parameter: int size		���ƴ�С
// Parameter: int mode		��Ƭ����x->0, y->1, z->2)
//************************************
void CreateSkeleton(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
					, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_skeleton
					, int n
					//, string direction
					//, double max_z, double min_z
					//, double* y, double* z, double* x
					, int size 
					, int mode
					, ofstream& _my_log
	)
{

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
	//��x�����ֵ
	double max_x = max_array(x, size);
	//��x����Сֵ
	double min_x = min_array(x, size);
	//��y�����ֵ
	double max_y = max_array(y, size);
	//��y����Сֵ
	double min_y = min_array(y, size);
	//��z�����ֵ
	double max_z = max_array(z, size);
	//��z����Сֵ
	double min_z = min_array(z, size);

	/****************Initialize************************/
	//��Ƭ����
	string direction = "";
	//�����ܶ�
	double res_cloud = 0.0;
	//��ǰ��Ƭ��
	int cut_n_3d = 0;
	int cut_n_2d = 0;
	//������
	int coun = 0;
	//��Ƭ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
	//�洢·��
	string filepath_3d = "";
	//�洢����
	string filename_3d = "";
	//��Ƭ����size
	int size_2d = 0;
	//��Ƭ�����ܶ�
	double res_cloud_2d = 0.0;
	//��Ƭ���Ƽ���ֵ
	//��z�����ֵ
	double max_z_2d = 0.0;
	//��z����Сֵ	   
	double min_z_2d = 0.0;
	//��x�����ֵ	   
	double max_x_2d = 0.0;
	//��x����Сֵ	   
	double min_x_2d = 0.0;
	//��y�����ֵ	   
	double max_y_2d = 0.0;
	//��y����Сֵ	   
	double min_y_2d = 0.0;
	//��������������
	double ratio_ = 0.0;

	/********Execution**********/
	switch (mode)
	{
		/**************X����Ǽ�*****************/
	case 0:
		direction = "x";

		_my_log << "/*****************X******************/" << endl;
		
		for (cut_n_3d = 0; cut_n_3d < n - 1; cut_n_3d++)
		{
			
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_2d(new pcl::PointCloud <pcl::PointXYZ>);

			filepath_3d = "./cut_out_3d/" + direction;
			if (0 != access(filepath_3d.c_str(), 0))
			{
				// if this folder not exist, create a new one.
				//int flag_3d = mkdir(filepath_3d.c_str());   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
				//���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
				CreateMultiLevel(filepath_3d);
			}
			filename_3d = filepath_3d + "/" + direction + "-" + to_string(cut_n_3d + 1) + ".pcd";

			//z�������Ƭ(3d->2d)
			cloud_cuted_3d = cutPointCloud(n, max_x, min_x, z, y, x, cloud_cuted_3d, size, cut_n_3d, filename_3d, mode, res_cloud, _my_log);

			size_2d = cloud_cuted_3d->size();
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

			
			//��z�����ֵ
			max_z_2d = max_array(z_2d, size_2d);
			//��z����Сֵ
			min_z_2d = min_array(z_2d, size_2d);
			////��x�����ֵ
			//max_x_2d = max_array(x_2d, size_2d);
			////��x����Сֵ
			//min_x_2d = min_array(x_2d, size_2d);
			//��y�����ֵ
			max_y_2d = max_array(y_2d, size_2d);
			//��y����Сֵ
			min_y_2d = min_array(y_2d, size_2d);

			//double dis_x_2d = max_x_2d - min_x_2d;
			//double dis_y_2d = max_y_2d - min_y_2d;
			//double ratio_ = abs(dis_x_2d) / abs(dis_y_2d);
			

			ratio_ = CalcRatio(max_y_2d, min_y_2d, max_z_2d, min_z_2d);

			//switch (mode)
			//{
			//	//xΪ��Ƭ����y/z
			//case 0:
			//	ratio_ = CalcRatio(max_y_2d, min_y_2d, max_z_2d, min_z_2d);
			//	//yΪ��Ƭ����x/z
			//case 1:
			//	ratio_ = CalcRatio(max_x_2d, min_x_2d, max_z_2d, min_z_2d);
			//	//zΪ��Ƭ����x/y
			//case 2:
			//	ratio_ = CalcRatio(max_x_2d, min_x_2d, max_y_2d, min_y_2d);
			//}

			_my_log << "ratio of 1 by 2 = " << ratio_ << endl;

			vector <pcl::PointIndices> cluster_2d = regionGrow(cloud_cuted_3d);

			if (cluster_2d.size() != 1 || (cluster_2d.size() == 1 && abs(ratio_ - 1.0) < 0.01))
			{
				coun++;
				std::cout << "number of clusters is : " << cluster_2d.size() << endl;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_centroid(new pcl::PointCloud <pcl::PointXYZ>);
				for (auto it : cluster_2d)
				{
					cloud_temp_centroid->clear();
					for (auto pit = it.indices.begin(); pit != it.indices.end(); ++pit)
					{
						cloud_temp_centroid->points.push_back(cloud_cuted_3d->points[*pit]);
					}
					cloud_temp_centroid->width = cloud_temp_centroid->points.size();
					cloud_temp_centroid->height = 1;
					cloud_temp_centroid->is_dense = true;
					//�����洢�������ĵĶ���
					Eigen::Vector4f centroid;
					//�������ĵ�
					pcl::compute3DCentroid(*cloud_temp_centroid, centroid);
					//������ĵ��Ǽ���
					cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
				}

			}
			else
				continue;


			/*
			for (cut_n_2d = 0; cut_n_2d < n - 1; cut_n_2d++)
			{

			string filepath_2d = filepath_3d + "/cut_2d/x";
			if (0 != access(filepath_2d.c_str(), 0))
			{
			// if this folder not exist, create a new one.
			CreateMultiLevel(filepath_2d);   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
			//���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
			}
			string filename_2d = filepath_2d + "/x-" + to_string(cut_n_2d + 1) + ".pcd";
			//x�������Ƭ(2d->1d)
			cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, y_2d, x_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 0, res_cloud_2d);
			//x�������Ƭ(2d->1d)
			//cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, x_2d, y_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 1, res_cloud_2d);

			//�����洢�������ĵĶ���
			Eigen::Vector4f centroid;
			//�������ĵ�
			pcl::compute3DCentroid(*cloud_cuted_2d, centroid);
			//������ĵ��Ǽ���
			cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
			}
			*/
		}

		break;

		/**************Y����Ǽ�*****************/
	case 1:
		direction = "y";

		_my_log << "/*************************Y**************************/" << endl;

		for (cut_n_3d = 0; cut_n_3d < n - 1; cut_n_3d++)
		{
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_2d(new pcl::PointCloud <pcl::PointXYZ>);

			filepath_3d = "./cut_out_3d/" + direction;
			if (0 != access(filepath_3d.c_str(), 0))
			{
				// if this folder not exist, create a new one.
				//int flag_3d = mkdir(filepath_3d.c_str());   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
				//���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
				CreateMultiLevel(filepath_3d);
			}
			filename_3d = filepath_3d + "/" + direction + "-" + to_string(cut_n_3d + 1) + ".pcd";

			//z�������Ƭ(3d->2d)
			cloud_cuted_3d = cutPointCloud(n, max_y, min_y, x, z, y, cloud_cuted_3d, size, cut_n_3d, filename_3d, mode, res_cloud, _my_log);

			size_2d = cloud_cuted_3d->size();
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

			
			//��z�����ֵ
			max_z_2d = max_array(z_2d, size_2d);
			//��z����Сֵ
			min_z_2d = min_array(z_2d, size_2d);
			//��x�����ֵ
			max_x_2d = max_array(x_2d, size_2d);
			//��x����Сֵ
			min_x_2d = min_array(x_2d, size_2d);
			////��y�����ֵ
			//max_y_2d = max_array(y_2d, size_2d);
			////��y����Сֵ
			//min_y_2d = min_array(y_2d, size_2d);

			//double dis_x_2d = max_x_2d - min_x_2d;
			//double dis_y_2d = max_y_2d - min_y_2d;
			//double ratio_ = abs(dis_x_2d) / abs(dis_y_2d);

			ratio_ = CalcRatio(max_x_2d, min_x_2d, max_z_2d, min_z_2d);

			//switch (mode)
			//{
			//	//xΪ��Ƭ����y/z
			//case 0:
			//	ratio_ = CalcRatio(max_y_2d, min_y_2d, max_z_2d, min_z_2d);
			//	//yΪ��Ƭ����x/z
			//case 1:
			//	ratio_ = CalcRatio(max_x_2d, min_x_2d, max_z_2d, min_z_2d);
			//	//zΪ��Ƭ����x/y
			//case 2:
			//	ratio_ = CalcRatio(max_x_2d, min_x_2d, max_y_2d, min_y_2d);
			//}

			_my_log << "ratio of 1 by 2 = " << ratio_ << endl;

			vector <pcl::PointIndices> cluster_2d = regionGrow(cloud_cuted_3d);

			if (cluster_2d.size() != 1 || (cluster_2d.size() == 1 && abs(ratio_ - 1.0) < 0.01))
			{
				coun++;
				std::cout << "number of clusters is : " << cluster_2d.size() << endl;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_centroid(new pcl::PointCloud <pcl::PointXYZ>);
				for (auto it : cluster_2d)
				{
					cloud_temp_centroid->clear();
					for (auto pit = it.indices.begin(); pit != it.indices.end(); ++pit)
					{
						cloud_temp_centroid->points.push_back(cloud_cuted_3d->points[*pit]);
					}
					cloud_temp_centroid->width = cloud_temp_centroid->points.size();
					cloud_temp_centroid->height = 1;
					cloud_temp_centroid->is_dense = true;
					//�����洢�������ĵĶ���
					Eigen::Vector4f centroid;
					//�������ĵ�
					pcl::compute3DCentroid(*cloud_temp_centroid, centroid);
					//������ĵ��Ǽ���
					cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
				}

			}
			else
				continue;


			/*
			for (cut_n_2d = 0; cut_n_2d < n - 1; cut_n_2d++)
			{

			string filepath_2d = filepath_3d + "/cut_2d/x";
			if (0 != access(filepath_2d.c_str(), 0))
			{
			// if this folder not exist, create a new one.
			CreateMultiLevel(filepath_2d);   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
			//���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
			}
			string filename_2d = filepath_2d + "/x-" + to_string(cut_n_2d + 1) + ".pcd";
			//x�������Ƭ(2d->1d)
			cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, y_2d, x_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 0, res_cloud_2d);
			//x�������Ƭ(2d->1d)
			//cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, x_2d, y_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 1, res_cloud_2d);

			//�����洢�������ĵĶ���
			Eigen::Vector4f centroid;
			//�������ĵ�
			pcl::compute3DCentroid(*cloud_cuted_2d, centroid);
			//������ĵ��Ǽ���
			cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
			}
			*/
		}

		break;

		/**************Z����Ǽ�*****************/
	case 2:
		direction = "z";

		_my_log << "/******************************Z**********************************/" << endl;

		for (cut_n_3d = 0; cut_n_3d < n - 1; cut_n_3d++)
		{
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_2d(new pcl::PointCloud <pcl::PointXYZ>);

			filepath_3d = "./cut_out_3d/" + direction;
			if (0 != access(filepath_3d.c_str(), 0))
			{
				// if this folder not exist, create a new one.
				//int flag_3d = mkdir(filepath_3d.c_str());   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
				//���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
				CreateMultiLevel(filepath_3d);
			}
			filename_3d = filepath_3d + "/" + direction + "-" + to_string(cut_n_3d + 1) + ".pcd";

			//z�������Ƭ(3d->2d)
			cloud_cuted_3d = cutPointCloud(n, max_z, min_z, y, x, z, cloud_cuted_3d, size, cut_n_3d, filename_3d, mode, res_cloud, _my_log);

			size_2d = cloud_cuted_3d->size();
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

			
			////��z�����ֵ
			//max_z_2d = max_array(z_2d, size_2d);
			////��z����Сֵ
			//min_z_2d = min_array(z_2d, size_2d);
			//��x�����ֵ
			max_x_2d = max_array(x_2d, size_2d);
			//��x����Сֵ
			min_x_2d = min_array(x_2d, size_2d);
			//��y�����ֵ
			max_y_2d = max_array(y_2d, size_2d);
			//��x����Сֵ
			min_y_2d = min_array(y_2d, size_2d);

			//double dis_x_2d = max_x_2d - min_x_2d;
			//double dis_y_2d = max_y_2d - min_y_2d;
			//double ratio_ = abs(dis_x_2d) / abs(dis_y_2d);

			ratio_ = CalcRatio(max_x_2d, min_x_2d, max_y_2d, min_y_2d);

			//switch (mode)
			//{
			//	//xΪ��Ƭ����y/z
			//case 0:
			//	ratio_ = CalcRatio(max_y_2d, min_y_2d, max_z_2d, min_z_2d);
			//	//yΪ��Ƭ����x/z
			//case 1:
			//	ratio_ = CalcRatio(max_x_2d, min_x_2d, max_z_2d, min_z_2d);
			//	//zΪ��Ƭ����x/y
			//case 2:
			//	ratio_ = CalcRatio(max_x_2d, min_x_2d, max_y_2d, min_y_2d);
			//}

			_my_log << "ratio of 1 by 2 = " << ratio_ << endl;

			vector <pcl::PointIndices> cluster_2d = regionGrow(cloud_cuted_3d);

			if (cluster_2d.size() != 1 || (cluster_2d.size() == 1 && abs(ratio_ - 1.0) < 0.01))
			{
				coun++;
				std::cout << "number of clusters is : " << cluster_2d.size() << endl;
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_centroid(new pcl::PointCloud <pcl::PointXYZ>);
				for (auto it : cluster_2d)
				{
					cloud_temp_centroid->clear();
					for (auto pit = it.indices.begin(); pit != it.indices.end(); ++pit)
					{
						cloud_temp_centroid->points.push_back(cloud_cuted_3d->points[*pit]);
					}
					cloud_temp_centroid->width = cloud_temp_centroid->points.size();
					cloud_temp_centroid->height = 1;
					cloud_temp_centroid->is_dense = true;
					//�����洢�������ĵĶ���
					Eigen::Vector4f centroid;
					//�������ĵ�
					pcl::compute3DCentroid(*cloud_temp_centroid, centroid);
					//������ĵ��Ǽ���
					cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
				}

			}
			else
				continue;


			/*
			for (cut_n_2d = 0; cut_n_2d < n - 1; cut_n_2d++)
			{

			string filepath_2d = filepath_3d + "/cut_2d/x";
			if (0 != access(filepath_2d.c_str(), 0))
			{
			// if this folder not exist, create a new one.
			CreateMultiLevel(filepath_2d);   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
			//���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
			}
			string filename_2d = filepath_2d + "/x-" + to_string(cut_n_2d + 1) + ".pcd";
			//x�������Ƭ(2d->1d)
			cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, y_2d, x_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 0, res_cloud_2d);
			//x�������Ƭ(2d->1d)
			//cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, x_2d, y_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 1, res_cloud_2d);

			//�����洢�������ĵĶ���
			Eigen::Vector4f centroid;
			//�������ĵ�
			pcl::compute3DCentroid(*cloud_cuted_2d, centroid);
			//������ĵ��Ǽ���
			cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
			}
			*/
		}
		break;
	}
	


}


int main()
{

	//DWORD start_time = GetTickCount(); //��ʼ��ʱ
									   //��ʼ��PointCloud����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	////pushback
	////cloud->push_back(PointXYZ(0,0,0));
	//
	////����mesh����
	//pcl::PolygonMesh mesh;
	////��ȡpolygon�ļ���obj��ʽ��ȡΪmesh
	//pcl::io::loadPolygonFile("zhengti-moxing-xie.obj", mesh);
	////pcl::io::loadPolygonFilePLY("zhengti-moxing.ply", mesh);
	//
	////��ʼ������洢����final
	//pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	////��mesh��ʽת��ΪPointCloud��ʽ �����ȡ
	//pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	////ת��Ϊ�ɶ�ȡ��PCD�ļ���ʽ
	//pcl::io::savePCDFileASCII("zhengti-moxing-xie.pcd", *cloud);
	//
	////������������
	//std::cout << cloud->size() << endl;
	//std::cout << "OK!";



	//���ص����ļ�
	pcl::io::loadPCDFile("zhengti-moxing-xie.pcd", *cloud);

	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	//�����size��С
	int size = cloud->points.size();
	
	//��ʼ���߽����
	//pcl::PointCloud<pcl::Boundary> boundaries;
	//��ʼ����Ƭ����
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
	//��ʼ��gujia����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_skeleton(new pcl::PointCloud <pcl::PointXYZ>);
	//��ʼ���洢�߽����ĵ���
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries(new pcl::PointCloud <pcl::PointXYZ>);
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);

	//Ƭ��
	int n = 80;

	//�����ܶ�
	double res_cloud = 0.0;

	//��ǰ��Ƭ��
	int cut_n_3d = 0;
	int cut_n_2d = 0;


	int coun = 0;


	/*
	for (cut_n_3d = 0; cut_n_3d < n - 1; cut_n_3d++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_3d(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted_2d(new pcl::PointCloud <pcl::PointXYZ>);
		string filepath_3d = "./cut_out_3d/z-" + to_string(cut_n_3d + 1);
		if (0 != access(filepath_3d.c_str(), 0))
		{
			// if this folder not exist, create a new one.
			//int flag_3d = mkdir(filepath_3d.c_str());   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
										 //���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
			CreateMultiLevel(filepath_3d);
		}
		string filename_3d = filepath_3d  + "/z-" + to_string(cut_n_3d + 1) + ".pcd";

		//z�������Ƭ(3d->2d)
		cloud_cuted_3d = cutPointCloud(n, max_z, min_z, y, x, z, cloud_cuted_3d, size, cut_n_3d, filename_3d, 2, res_cloud);

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

		double res_cloud_2d = 0.0;
		//��x�����ֵ
		double max_x_2d = max_array(x_2d, size_2d);
		//��x����Сֵ
		double min_x_2d = min_array(x_2d, size_2d);
		//��y�����ֵ
		double max_y_2d = max_array(y_2d, size_2d);
		//��x����Сֵ
		double min_y_2d = min_array(y_2d, size_2d);

		double dis_x_2d = max_x_2d - min_x_2d;
		double dis_y_2d = max_y_2d - min_y_2d;
		double ratio_xy = abs(dis_x_2d) / abs(dis_y_2d);

		std::cout << "ratio of x by y = " << ratio_xy << endl;

		vector <pcl::PointIndices> cluster_2d = regionGrow(cloud_cuted_3d);

		if (cluster_2d.size() != 1 || (cluster_2d.size() == 1 && abs(ratio_xy - 1.0) < 0.01))
		{
			coun++;
			std::cout << "number of clusters is : " << cluster_2d.size() << endl;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_centroid(new pcl::PointCloud <pcl::PointXYZ>);
			for (auto it : cluster_2d)
			{
				cloud_temp_centroid->clear();
				for (auto pit = it.indices.begin(); pit != it.indices.end(); ++pit)
				{
					cloud_temp_centroid->points.push_back(cloud_cuted_3d->points[*pit]);
				}
				cloud_temp_centroid->width = cloud_temp_centroid->points.size();
				cloud_temp_centroid->height = 1;
				cloud_temp_centroid->is_dense = true;
				//�����洢�������ĵĶ���
				Eigen::Vector4f centroid;
				//�������ĵ�
				pcl::compute3DCentroid(*cloud_temp_centroid, centroid);
				//������ĵ��Ǽ���
				cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
			}



		}
		else
			continue;





		
		for (cut_n_2d = 0; cut_n_2d < n - 1; cut_n_2d++)
		{

			string filepath_2d = filepath_3d + "/cut_2d/x";
			if (0 != access(filepath_2d.c_str(), 0))
			{
				// if this folder not exist, create a new one.
				CreateMultiLevel(filepath_2d);   // ���� 0 ��ʾ�����ɹ���-1 ��ʾʧ��
											  //���� ::_mkdir  ::_access Ҳ�У���֪��ʲô��˼
			}
			string filename_2d = filepath_2d + "/x-" + to_string(cut_n_2d + 1) + ".pcd";
			//x�������Ƭ(2d->1d)
			cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, y_2d, x_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 0, res_cloud_2d);
			//x�������Ƭ(2d->1d)
			//cloud_cuted_2d = cutPointCloud(n, max_x, min_x, z_2d, x_2d, y_2d, cloud_cuted_2d, size_2d, cut_n_2d, filename_2d, 1, res_cloud_2d);

			//�����洢�������ĵĶ���
			Eigen::Vector4f centroid;
			//�������ĵ�
			pcl::compute3DCentroid(*cloud_cuted_2d, centroid);
			//������ĵ��Ǽ���
			cloud_skeleton->push_back(PointXYZ(centroid[0], centroid[1], centroid[2]));
		}

	}
	*/


	ofstream _my_log("_my_log.txt");
	
	//x��Ǽ�
	CreateSkeleton(cloud, cloud_skeleton, n, size, 0, _my_log);
	int num1 = cloud_skeleton->size();
	std::cout << "skeleton after x has " << num1 << "points" << endl;
	//y��Ǽ�
	CreateSkeleton(cloud, cloud_skeleton, n, size, 1, _my_log);
	int num2 = cloud_skeleton->size();
	std::cout << "skeleton after y has " << num2 << "points" << endl;
	//z��Ǽ�
	CreateSkeleton(cloud, cloud_skeleton, n, size, 2, _my_log);
	int num3 = cloud_skeleton->size();


	_my_log << "skeleton after x has " << num1 << "points" << endl;
	_my_log << "skeleton after y has " << num2 - num1 << "points" << endl;
	_my_log << "skeleton after z has " << num3 - num2 - num1 << "points" << endl;

	
	

	pcl::io::savePCDFileASCII("cloud_skeleton.pcd", *cloud_skeleton);
	std::cout << "number" << coun << endl;

	_my_log << "The run time is " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	_my_log.close();

	//����viewer����
	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("ԭʼ����"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 185, 185);
	viewer_ori->addPointCloud(cloud, "cloud");
	viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer_ori->addCoordinateSystem(1);

	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cut(new pcl::visualization::PCLVisualizer("qiepian����"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color_handler(cloud, 255, 0, 0);
	//viewer_cut->addPointCloud(cloud, red, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_skeleton, 0, 255, 0);
	viewer_cut->addPointCloud(cloud_skeleton, green, "cut");
	//viewer_cut->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cut");
	viewer_cut->addCoordinateSystem(1);

	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_all(new pcl::visualization::PCLVisualizer("skeleton����"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color_handler(cloud, 255, 0, 0);
	viewer_all->addPointCloud(cloud, red, "cloud");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_cuted, 0, 255, 0);
	viewer_all->addPointCloud(cloud_skeleton, green, "skeleton");
	viewer_all->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "skeleton");
	viewer_all->setPointCloudSelected(true);
	viewer_all->addCoordinateSystem(0.5);

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