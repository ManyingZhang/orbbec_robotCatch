#include <QtNetwork/QUdpSocket>
#include <QCoreApplication>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <astra/astra.hpp>

# define PI 3.1415926

using namespace Eigen;
using namespace std;
using namespace cv;

int WriteData(string fileName, Mat1d & matData)
{
	int retVal = 0;

	// 打开文件 
	ofstream outFile(fileName.c_str(), ios_base::out); //按新建或覆盖方式写入 
	if (!outFile.is_open())
	{
		cout << "打开文件失败" << endl;
		retVal = -1;
		return (retVal);
	}

	// 检查矩阵是否为空 
	if (matData.empty())
	{
		cout << "矩阵为空" << endl;
		retVal = 1;
		return (retVal);
	}

	// 写入数据 
	for (int r = 0; r < matData.rows; r++)
	{
		for (int c = 0; c < matData.cols; c++)
		{
			double data = matData[r][c]; //读取数据，at<type> - type 是矩阵元素的具体数据格式 
			outFile << data << "\t"; //每列数据用 tab 隔开 
		}
		outFile << endl; //换行 
	}

	return (retVal);
}
//求解手眼标定矩阵
int ComputeRT_3D(std::vector<Vec3d>& M, std::vector<Vec3d>& D, Mat1d & R, Mat1d & T){}

void eyetohand()
{
	int num = 4;
	FILE *f1 = fopen("Robotxyz.txt", "r");
	vector<Vec3d> M;
	for (int i = 0; i < num; i++)
	{
		Vec3d Temp1;
		float x1, y1, z1;
		fscanf_s(f1, "%f %f %f\n", &x1, &y1, &z1);
		Temp1[0] = x1; Temp1[1] = y1; Temp1[2] = z1;
		M.push_back(Temp1);
	}
	fclose(f1);

	FILE *f2 = fopen("PCLxyz.txt", "r");
	vector<Vec3d> D;
	for (int i = 0; i < num; i++)
	{
		Vec3d Temp2;
		float x2, y2, z2;
		fscanf_s(f2, "%f %f %f\n", &x2, &y2, &z2);
		Temp2[0] = x2; Temp2[1] = y2; Temp2[2] = z2;
		D.push_back(Temp2);
	}
	fclose(f2);

	Mat1d R;
	Mat1d T;
	ComputeRT_3D(M, D, R, T);

	Mat1d RT = Mat1d::eye(4, 4);
	R.copyTo(RT(Rect(0, 0, 3, 3)));
	RT(0, 3) = T[0][0];
	RT(1, 3) = T[1][0];
	RT(2, 3) = T[2][0];
	cout << RT << endl;
	system("pause");
	RT = RT.inv();
	WriteData("ethRTinv.txt", RT);
}
//拟合平面
void get2RandomPlaneR(std::vector<cv::Vec4f>& inCloud, cv::Vec4f &Plane, double th){}
void Calpoint(const std::vector<cv::Vec4f> ptCloud, cv::Vec3f &centerPt)
{
	int ptNum = ptCloud.size();

	for (int i = 0; i < ptNum; i++) {
		Vec3f temp;
		temp[0] = ptCloud[i][0];
		temp[2] = ptCloud[i][2];
		temp[1] = ptCloud[i][1];
		centerPt += temp;
	}
	centerPt = centerPt / ptNum;
}
void RigidTransPtCloud(std::vector<cv::Vec4f> &srcV, Matrix4f trsM)
{
	for (unsigned int i = 0; i < srcV.size(); i++)
	{
		float nX = trsM(0, 0) * srcV[i][0] + trsM(0, 1) * srcV[i][1] + trsM(0, 2) * srcV[i][2] + trsM(0, 3);
		float nY = trsM(1, 0) * srcV[i][0] + trsM(1, 1) * srcV[i][1] + trsM(1, 2) * srcV[i][2] + trsM(1, 3);
		float nZ = trsM(2, 0) * srcV[i][0] + trsM(2, 1) * srcV[i][1] + trsM(2, 2) * srcV[i][2] + trsM(2, 3);
		srcV[i][0] = nX;
		srcV[i][1] = nY;
		srcV[i][2] = nZ;
	}
}
cv::Vec3d CrossProduct_3Pt(cv::Vec3d OA, cv::Vec3d OB)
{
	//OA OB向量的叉乘
	double x, y, z;
	x = OA[1] * OB[2] - OA[2] * OB[1];
	y = OA[2] * OB[0] - OA[0] * OB[2];
	z = OA[0] * OB[1] - OA[1] * OB[0];
	cv::Vec3d C(x, y, z);
	return C;
}
double Size_Pt(cv::Vec3d p)
{
	//向量的模（3个点）
	return sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}
void readPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	astra::initialize();
	astra::StreamSet streamSet;
	astra::StreamReader reader = streamSet.create_reader();
	reader.stream<astra::PointStream>().start();
	//Stores the maximum number of frames we're going to process in the loop
	const int maxFramesToProcess = 100;
	//Sentinel to count the number of frames that we've processed
	int count = 0;
	astra::Frame frame = reader.get_latest_frame();
	const auto depthFrame = frame.get<astra::PointFrame>();
	const int frameIndex = depthFrame.frame_index();
	size_t m = 10000;
	for (size_t i = 0; i < 640 * 480 - 1; i++) 
	{

		astra::Vector3f pixelValue = depthFrame.data()[i];
		pcl::PointXYZ point;
		point.x = pixelValue.x;
		point.y = pixelValue.y;
		point.z = pixelValue.z;
		if (pixelValue.z < 580 || pixelValue.z>630 || pixelValue.y < -150)continue;
		cloud->push_back(point);
	}

	cout << "采集完毕" << endl;
	
}
void segPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector <pcl::PointIndices> &clusters)
{
	/*pcl::PointCloud<pcl::PointXYZ>Yun;*/
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(3000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	/*std::vector <pcl::PointIndices> clusters;*/
	reg.extract(clusters);

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	//pcl::visualization::CloudViewer viewer("Cluster viewer");
	//viewer.showCloud(colored_cloud);
	//while (!viewer.wasStopped());
}
bool findElipse(Mat1b grayM, Point2f &center)
{
	Mat1b edgeM;
	Canny(grayM, edgeM, 60, 120);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat1b tmpM = edgeM.clone();
	findContours(tmpM, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	if (contours.size() < 1)
		return false;
	size_t maxS = 0;
	int m0 = 0;
	for (int m = 0; m < contours.size(); m++)
	{
		if (contours[m].size() > maxS)
		{
			maxS = contours[m].size();
			m0 = m;
		}
	}
	if (maxS < 6)
		return false;
	RotatedRect box;
	box = cv::fitEllipse(contours[m0]);
 	Mat3b temp1 = Mat3b::zeros(tmpM.rows, tmpM.cols);
	cv::drawContours(temp1, contours, 0, cv::Scalar(255, 0, 0));
	cv::ellipse(temp1, box, cv::Scalar(0, 255, 0));
	cv::imshow("123", temp1);
	waitKey(0);
 	center = box.center;
}

void work()
{
	//通信
	QUdpSocket xx;
	std::cout << xx.bind(QHostAddress("192.168.0.15"), 60000) << std::endl;
	//读取手眼标定矩阵
	Matrix4f ethM;
	std::ifstream fin;
	fin.open("ethM.txt");
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			fin >> ethM(i, j);
		}
	}
	fin.close();

	//参数设置
	int N = 0;
	int Num = 4;
	int num;
	cv::Vec3f centerPt;
	cv::Vec4f planeParm;
	cv::Vec4f planeParmnew;
	std::vector<cv::Vec4f> ptCloud;//匹配用的点集
	Vec4f p;

	while (1)
	{
	//for (size_t i = 0; i < Num; i++)
	//{
		//获取点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		readPoint(cloud);

		//点云操作 滤波 分割 配准
		
		//分割及转换	
		
		std::vector <pcl::PointIndices> clusters;
		segPoint(cloud,clusters);
		pcl::PointCloud<pcl::PointXYZ>Yun;
		Yun = *cloud;
		if ( Yun.size() > 1000)
		{
			size_t maxC = 0;
			for (int m = 0; m < clusters.size(); m++)
			{
				if (clusters[m].indices.size() > maxC)
				{
					maxC = clusters[m].indices.size();
					N = m;
				}
			}
			num = clusters[N].indices.size();
			int counter = 0;
			while (counter < num)
			{
				size_t ind = clusters[N].indices[counter];

				p[0] = Yun.points[ind].x;
				p[1] = Yun.points[ind].y;
				p[2] = Yun.points[ind].z;
				p[3] = 255;

				ptCloud.push_back(p);
				counter++;
				//if (counter % 10 == 0)
				//	std::cout << std::endl;
			}
			Yun.clear();
			cloud->empty();
			RigidTransPtCloud(ptCloud, ethM);

			//求平面法向量（3D映射2D）
			get2RandomPlaneR(ptCloud, planeParm, 3);
			if (planeParm[2] < 0)
				planeParm = -planeParm;
			//计算抓取点
			Calpoint(ptCloud, centerPt);
			ptCloud.clear();

			//求解抓取姿态
			cv::Vec3f Vec_N;
			Vec_N[0] = planeParm[0]; Vec_N[1] = planeParm[1]; Vec_N[2] = planeParm[2];
			//Vec_N = -Vec_N;
			cv::Vec3f Vec_Base = cv::Vec3f(0, 0, 1);
			cv::Vec3f Vec_new = CrossProduct_3Pt(Vec_N, Vec_Base);
			Mat1f M = Mat1f::zeros(3, 3);
			Rodrigues(Vec_new, M);

			float Angle_O, Angle_A, Angle_T;
			Angle_A = atan2(sqrt(M[0][2] * M[0][2] + M[1][2] * M[1][2]), M[2][2]);

			if (abs(Angle_A - 0.0) < 1E-2)
			{
				Angle_O = 0;
				Angle_T = atan2(-M[0][1], M[0][0]);
			}
			else if (abs(Angle_A - PI) < 1E-2)
			{
				Angle_O = 0;
				Angle_T = atan2(M[0][1], -M[0][0]);
			}
			else
			{
				Angle_O = atan2(M[1][2] / sin(Angle_A), M[0][2] / sin(Angle_A));
				Angle_T = atan2(M[2][1] / sin(Angle_A), -M[2][0] / sin(Angle_A));
			}

			Angle_O = Angle_O *180.0 / PI;
			Angle_A = 180 - (Angle_A *180.0 / PI);
			Angle_T = Angle_T *180.0 / PI;
			std::cout << "x" << '\t' << "y" << '\t' << "z" << "O" << '\t' << "A" << '\t' << "T" << '\t' << std::endl;
			std::cout << centerPt[0] << '\t' << centerPt[1] << '\t' << centerPt[2] << '\t' << Angle_O << '\t' << Angle_A << '\t' << Angle_T << '\t' << std::endl;

			cv::Vec6f buf(centerPt[0], centerPt[1], centerPt[2], Angle_O, Angle_A, Angle_T);
			char BUF[100];
			sprintf(BUF, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,4,0", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

			//传给机器人BUF
			//xx.writeDatagram(BUF, sizeof(BUF), QHostAddress("192.168.0.2"), 60003);
			cout << "OK" << endl;
			getchar();
		}

		else
		{
			cout << "没有找到木块" << endl;
			break;
		}

	}
}

int main()
{
	//eyetohand();

	work();

	return 0;
}
