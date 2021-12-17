


#include "CPanoramaStitching.h"



CPanoramaStitching::CPanoramaStitching()
{

	m_fOpeningAngle = 40.0; 

	loadCameraParam();

	m_pcloud_stitch.reset(new pcl::PointCloud<PointType>);

}


CPanoramaStitching::~CPanoramaStitching()
{

}


void CPanoramaStitching::loadCameraParam()
{
	//  相机矩阵 [{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}]
	//camera_matrixa18 = (cv::Mat_<double>(3, 3) << 610.7208, 0.0, 645.3167, 0, 610.8133, 483.2261, 0, 0, 1);
	//camera_matrixa20 = (cv::Mat_<double>(3, 3) << 596.5926, 0.0, 629.46, 0, 596.9962, 458.5713, 0, 0, 1);
	camera_matrixa18 = (cv::Mat_<double>(3, 3) <<
		m_intrinsics18[0], m_intrinsics18[3], m_intrinsics18[6],
		m_intrinsics18[1], m_intrinsics18[4], m_intrinsics18[7],
		m_intrinsics18[2], m_intrinsics18[5], m_intrinsics18[8]);
	camera_matrixa20 = (cv::Mat_<double>(3, 3) <<
		m_intrinsics20[0], m_intrinsics20[3], m_intrinsics20[6],
		m_intrinsics20[1], m_intrinsics20[4], m_intrinsics20[7],
		m_intrinsics20[2], m_intrinsics20[5], m_intrinsics20[8]);

	//  畸变系数向量 (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]]),默认值为0
	//distCoeffs18 = (cv::Mat_<double >(5, 1) << -0.3094, 0.0992, 0.000014154, -0.00034282, -0.0143);
	//distCoeffs20 = (cv::Mat_<double >(5, 1) << -0.2827, 0.0781, -0.0011, 0.00076695, -0.0093);
	distCoeffs18 = (cv::Mat_<double >(5, 1) <<
		m_distorts18[0], m_distorts18[1], m_distorts18[3], m_distorts18[4], m_distorts18[2]);
	distCoeffs20 = (cv::Mat_<double >(5, 1) <<
		m_distorts20[0], m_distorts20[1], m_distorts20[3], m_distorts20[4], m_distorts20[2]);

	m_fRadius18 = m_intrinsics18[0];
	m_fRadius20 = m_intrinsics20[0];
}


void CPanoramaStitching::loadRotateMatrix(int angle)
{
	switch (angle)
	{
	case 20:
		rotateMatrixInitial20();
		pvecRotation = &vecRotation20;
		break;
	case 30:
		rotateMatrixInitial30();
		pvecRotation = &vecRotation30;
		break;
	case 40:
		rotateMatrixInitial40();
		pvecRotation = &vecRotation40;
		break;
	case 49:
		rotateMatrixInitial49();
		pvecRotation = &vecRotation49;
		break;
	case 55:
		rotateMatrixInitial55();
		pvecRotation = &vecRotation55;
		break;
	default:
		break;
	}
}


void CPanoramaStitching::rotateMatrixInitial20()
{

}


void CPanoramaStitching::rotateMatrixInitial30()
{
	vecRotation30.resize(IMAGE_NUM);
	for (int index = 0; index < IMAGE_NUM; ++index) {
		vecRotation30[index].resize(MATRIX_ROWS);
		for (int row = 0; row < MATRIX_ROWS; ++row) {
			vecRotation30[index][row].resize(MATRIX_COLS);
		}
	}
	vecRotation30[0][0][0] = 1.0248;
	vecRotation30[0][0][1] = 0.0280;
	vecRotation30[0][0][2] = -0.0316;
	vecRotation30[0][1][0] = 0.0057;
	vecRotation30[0][1][1] = 0.9187;
	vecRotation30[0][1][2] = 0.5059;
	vecRotation30[0][2][0] = 0.0245;
	vecRotation30[0][2][1] = -0.5155;
	vecRotation30[0][2][2] = 0.8505;

	vecRotation30[1][0][0] = 0.9362;
	vecRotation30[1][0][1] = -0.0266;
	vecRotation30[1][0][2] = 0.4380;
	vecRotation30[1][1][0] = -0.0447;
	vecRotation30[1][1][1] = 1.0137;
	vecRotation30[1][1][2] = 0.2290;
	vecRotation30[1][2][0] = -0.4396;
	vecRotation30[1][2][1] = -0.2347;
	vecRotation30[1][2][2] = 0.8623;

	vecRotation30[2][0][0] = 0.9210;
	vecRotation30[2][0][1] = 0.0494;
	vecRotation30[2][0][2] = 0.4210;
	vecRotation30[2][1][0] = 0.0471;
	vecRotation30[2][1][1] = 0.9710;
	vecRotation30[2][1][2] = -0.2355;
	vecRotation30[2][2][0] = -0.4220;
	vecRotation30[2][2][1] = 0.2311;
	vecRotation30[2][2][2] = 0.8747;

	vecRotation30[3][0][0] = 1.0158;
	vecRotation30[3][0][1] = -0.0097;
	vecRotation30[3][0][2] = -0.0134;
	vecRotation30[3][1][0] = 0.0037;
	vecRotation30[3][1][1] = 0.8693;
	vecRotation30[3][1][2] = -0.4685;
	vecRotation30[3][2][0] = 0.0087;
	vecRotation30[3][2][1] = 0.4602;
	vecRotation30[3][2][2] = 0.8834;

	vecRotation30[4][0][0] = 0.9341;
	vecRotation30[4][0][1] = -0.0635;
	vecRotation30[4][0][2] = -0.3658;
	vecRotation30[4][1][0] = -0.0486;
	vecRotation30[4][1][1] = 0.9635;
	vecRotation30[4][1][2] = -0.2329;
	vecRotation30[4][2][0] = 0.3590;
	vecRotation30[4][2][1] = 0.2171;
	vecRotation30[4][2][2] = 0.9015;

	vecRotation30[5][0][0] = 0.8818;
	vecRotation30[5][0][1] = 0.0050;
	vecRotation30[5][0][2] = -0.4821;
	vecRotation30[5][1][0] = 0.0260;
	vecRotation30[5][1][1] = 1.0158;
	vecRotation30[5][1][2] = 0.1359;
	vecRotation30[5][2][0] = 0.4688;
	vecRotation30[5][2][1] = -0.1469;
	vecRotation30[5][2][2] = 0.8599;

}


void CPanoramaStitching::rotateMatrixInitial40()
{
	vecRotation40.resize(IMAGE_NUM);
	for (int index = 0; index < IMAGE_NUM; ++index) {
		vecRotation40[index].resize(MATRIX_ROWS);
		for (int row = 0; row < MATRIX_ROWS; ++row) {
			vecRotation40[index][row].resize(MATRIX_COLS);
		}
	}
	vecRotation40[0][0][0] = 1.0248;
	vecRotation40[0][0][1] = 0.0302;
	vecRotation40[0][0][2] = -0.0264;
	vecRotation40[0][1][0] = 0.0026;
	vecRotation40[0][1][1] = 0.8190;
	vecRotation40[0][1][2] = 0.6585;
	vecRotation40[0][2][0] = 0.0308;
	vecRotation40[0][2][1] = -0.6639;
	vecRotation40[0][2][2] = 0.7456;

	vecRotation40[1][0][0] = 0.8516;
	vecRotation40[1][0][1] = -0.0967;
	vecRotation40[1][0][2] = 0.5573;
	vecRotation40[1][1][0] = -0.0842;
	vecRotation40[1][1][1] = 0.9832;
	vecRotation40[1][1][2] = 0.3032;
	vecRotation40[1][2][0] = -0.5660;
	vecRotation40[1][2][1] = -0.2814;
	vecRotation40[1][2][2] = 0.7788;

	vecRotation40[2][0][0] = 0.8439;
	vecRotation40[2][0][1] = 0.1004;
	vecRotation40[2][0][2] = 0.5470;
	vecRotation40[2][1][0] = 0.0847;
	vecRotation40[2][1][1] = 0.9598;
	vecRotation40[2][1][2] = -0.3075;
	vecRotation40[2][2][0] = -0.5488;
	vecRotation40[2][2][1] = 0.3033;
	vecRotation40[2][2][2] = 0.7785;

	vecRotation40[3][0][0] = 1.0085;
	vecRotation40[3][0][1] = -0.0095;
	vecRotation40[3][0][2] = -0.0121;
	vecRotation40[3][1][0] = -0.0037;
	vecRotation40[3][1][1] = 0.7633;
	vecRotation40[3][1][2] = -0.6064;
	vecRotation40[3][2][0] = 0.0072;
	vecRotation40[3][2][1] = 0.6002;
	vecRotation40[3][2][2] = 0.7937;

	vecRotation40[4][0][0] = 0.8657;
	vecRotation40[4][0][1] = -0.1067;
	vecRotation40[4][0][2] = -0.4934;
	vecRotation40[4][1][0] = -0.0922;
	vecRotation40[4][1][1] = 0.9378;
	vecRotation40[4][1][2] = -0.3038;
	vecRotation40[4][2][0] = 0.4893;
	vecRotation40[4][2][1] = 0.2879;
	vecRotation40[4][2][2] = 0.8167;

	vecRotation40[5][0][0] = 0.7778;
	vecRotation40[5][0][1] = 0.0375;
	vecRotation40[5][0][2] = -0.6001;
	vecRotation40[5][1][0] = 0.0478;
	vecRotation40[5][1][1] = 0.9974;
	vecRotation40[5][1][2] = 0.2111;
	vecRotation40[5][2][0] = 0.5651;
	vecRotation40[5][2][1] = -0.2066;
	vecRotation40[5][2][2] = 0.7817;
}


void CPanoramaStitching::rotateMatrixInitial49()
{
	vecRotation49.resize(IMAGE_NUM);
	for (int index = 0; index < IMAGE_NUM; ++index) {
		vecRotation49[index].resize(MATRIX_ROWS);
		for (int row = 0; row < MATRIX_ROWS; ++row) {
			vecRotation49[index][row].resize(MATRIX_COLS);
		}
	}
	vecRotation49[0][0][0] = 1.0148;
	vecRotation49[0][0][1] = 0.0281;
	vecRotation49[0][0][2] = -0.0307;
	vecRotation49[0][1][0] = 0.0067;
	vecRotation49[0][1][1] = 0.7006;
	vecRotation49[0][1][2] = 0.7761;
	vecRotation49[0][2][0] = 0.0320;
	vecRotation49[0][2][1] = -0.7838;
	vecRotation49[0][2][2] = 0.6358;

	vecRotation49[1][0][0] = 0.7797;
	vecRotation49[1][0][1] = -0.1280;
	vecRotation49[1][0][2] = 0.6665;
	vecRotation49[1][1][0] = -0.1316;
	vecRotation49[1][1][1] = 0.9625;
	vecRotation49[1][1][2] = 0.3643;
	vecRotation49[1][2][0] = -0.6795;
	vecRotation49[1][2][1] = -0.3671;
	vecRotation49[1][2][2] = 0.6524;

	vecRotation49[2][0][0] = 0.7749;
	vecRotation49[2][0][1] = 0.1411;
	vecRotation49[2][0][2] = 0.6474;
	vecRotation49[2][1][0] = 0.1380;
	vecRotation49[2][1][1] = 0.9326;
	vecRotation49[2][1][2] = -0.3593;
	vecRotation49[2][2][0] = -0.6470;
	vecRotation49[2][2][1] = 0.3587;
	vecRotation49[2][2][2] = 0.6739;

	vecRotation49[3][0][0] = 1.0076;
	vecRotation49[3][0][1] = -0.0167;
	vecRotation49[3][0][2] = -0.0118;
	vecRotation49[3][1][0] = -0.0177;
	vecRotation49[3][1][1] = 0.6810;
	vecRotation49[3][1][2] = -0.7198;
	vecRotation49[3][2][0] = 0.0057;
	vecRotation49[3][2][1] = 0.7161;
	vecRotation49[3][2][2] = 0.6937;

	vecRotation49[4][0][0] = 0.7862;
	vecRotation49[4][0][1] = -0.1432;
	vecRotation49[4][0][2] = -0.5904;
	vecRotation49[4][1][0] = -0.1348;
	vecRotation49[4][1][1] = 0.9119;
	vecRotation49[4][1][2] = -0.3578;
	vecRotation49[4][2][0] = 0.5835;
	vecRotation49[4][2][1] = 0.3494;
	vecRotation49[4][2][2] = 0.7250;

	vecRotation49[5][0][0] = 0.7118;
	vecRotation49[5][0][1] = 0.0616;
	vecRotation49[5][0][2] = -0.7074;
	vecRotation49[5][1][0] = 0.1205;
	vecRotation49[5][1][1] = 0.9909;
	vecRotation49[5][1][2] = 0.2731;
	vecRotation49[5][2][0] = 0.7053;
	vecRotation49[5][2][1] = -0.3021;
	vecRotation49[5][2][2] = 0.6489;


}


void CPanoramaStitching::rotateMatrixInitial55()
{
	vecRotation55.resize(IMAGE_NUM);
	for (int index = 0; index < IMAGE_NUM; ++index) {
		vecRotation55[index].resize(MATRIX_ROWS);
		for (int row = 0; row < MATRIX_ROWS; ++row) {
			vecRotation55[index][row].resize(MATRIX_COLS);
		}
	}
	vecRotation55[0][0][0] = 1.0199;
	vecRotation55[0][0][1] = 0.0385;
	vecRotation55[0][0][2] = -0.0204;
	vecRotation55[0][1][0] = 0.0111;
	vecRotation55[0][1][1] = 0.4961;
	vecRotation55[0][1][2] = 0.9176;
	vecRotation55[0][2][0] = 0.0290;
	vecRotation55[0][2][1] = -0.9250;
	vecRotation55[0][2][2] = 0.4264;

	vecRotation55[1][0][0] = 0.6612;
	vecRotation55[1][0][1] = -0.2074;
	vecRotation55[1][0][2] = 0.8051;
	vecRotation55[1][1][0] = -0.2038;
	vecRotation55[1][1][1] = 0.9206;
	vecRotation55[1][1][2] = 0.4446;
	vecRotation55[1][2][0] = -0.8564;
	vecRotation55[1][2][1] = -0.4478;
	vecRotation55[1][2][2] = 0.4242;

	vecRotation55[2][0][0] = 0.6371;
	vecRotation55[2][0][1] = 0.2274;
	vecRotation55[2][0][2] = 0.7735;
	vecRotation55[2][1][0] = 0.2303;
	vecRotation55[2][1][1] = 0.8973;
	vecRotation55[2][1][2] = -0.4283;
	vecRotation55[2][2][0] = -0.7709;
	vecRotation55[2][2][1] = 0.4226;
	vecRotation55[2][2][2] = 0.4819;

	vecRotation55[3][0][0] = 0.9973;
	vecRotation55[3][0][1] = -0.0181;
	vecRotation55[3][0][2] = -0.0125;
	vecRotation55[3][1][0] = -0.0235;
	vecRotation55[3][1][1] = 0.4980;
	vecRotation55[3][1][2] = -0.8577;
	vecRotation55[3][2][0] = 0.0067;
	vecRotation55[3][2][1] = 0.8482;
	vecRotation55[3][2][2] = 0.5147;

	vecRotation55[4][0][0] = 0.6515;
	vecRotation55[4][0][1] = -0.2150;
	vecRotation55[4][0][2] = -0.7131;
	vecRotation55[4][1][0] = -0.2049;
	vecRotation55[4][1][1] = 0.8690;
	vecRotation55[4][1][2] = -0.4299;
	vecRotation55[4][2][0] = 0.7112;
	vecRotation55[4][2][1] = 0.4186;
	vecRotation55[4][2][2] = 0.5487;

	vecRotation55[5][0][0] = 0.5432;
	vecRotation55[5][0][1] = 0.1360;
	vecRotation55[5][0][2] = -0.8222;
	vecRotation55[5][1][0] = 0.2033;
	vecRotation55[5][1][1] = 0.9647;
	vecRotation55[5][1][2] = 0.3470;
	vecRotation55[5][2][0] = 0.8309;
	vecRotation55[5][2][1] = -0.4047;
	vecRotation55[5][2][2] = 0.4439;

}


void CPanoramaStitching::ImageStitch(const char* imgpath)
{
	float opening_angle_arc = -m_fOpeningAngle * M_PI / 180;
	float angleX, angleY, angleZ, x, y, z;
	for (int index = 0; index <= 6; ++index) {
		pcl::PointCloud<PointType>::Ptr cloud;
		cloud.reset(new pcl::PointCloud<PointType>);

		std::string strimg = std::string(imgpath) + std::to_string(m_FILE_INDEX[index]) + ".bmp";
		int result = GenerateCloudFromImage(strimg.c_str(), cloud);
		if (result != 0) {
			return;
		}

		if (index == 0) {
			// 图像尺寸的统一
			for (int i = 0; i < cloud->width; ++i) {
				float factor = m_fRadius20 / m_fRadius18;
				cloud->points[i].x = cloud->points[i].x * factor;
				cloud->points[i].y = cloud->points[i].y * factor;
				cloud->points[i].z = cloud->points[i].z * factor;
			}
		}

		//  0. 计算几何变换参数
		if (index == 0) {
			angleX = -M_PI_2;
			angleY = M_PI_2 + M_PI / 3.0;
			angleZ = 0.0;
			y = -m_fRadius18;
			x = 0.0;
			z = 0.0;
		}
		else {
			angleX = opening_angle_arc;
			angleY = -(M_PI / 3.0) * (index - 1);
			angleZ = 0.0;
			y = m_fRadius18 * sin(opening_angle_arc);
			x = -m_fRadius18 * sin(angleY);
			z = -m_fRadius18 * cos(angleY);
		}

		//  1. 旋转
		Eigen::Affine3f angleX_transform = Eigen::Affine3f::Identity();
		angleX_transform.rotate(Eigen::AngleAxisf(angleX, Eigen::Vector3f::UnitX()));
		pcl::transformPointCloud(*cloud, *cloud, angleX_transform);
		Eigen::Affine3f angleY_transform = Eigen::Affine3f::Identity();
		angleY_transform.rotate(Eigen::AngleAxisf(angleY, Eigen::Vector3f::UnitY()));
		pcl::transformPointCloud(*cloud, *cloud, angleY_transform);
		Eigen::Affine3f angleZ_transform = Eigen::Affine3f::Identity();
		angleZ_transform.rotate(Eigen::AngleAxisf(angleZ, Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*cloud, *cloud, angleZ_transform);

		//  2. 平移
		Eigen::Affine3f pos_transform = Eigen::Affine3f::Identity();
		pos_transform.translation() << x, y, z;
		pcl::transformPointCloud(*cloud, *cloud, pos_transform);

		//  3.投影
		SphereProject(cloud, cloud, m_fRadius18);

		*m_pcloud_stitch += *cloud;
		qDebug("load image %d finished\n", index);
	}
}


void CPanoramaStitching::ImageStitchUseRotateMatrix(const char* imgpath, int angle)
{
	m_nOpenAngleSelect = angle;

	loadRotateMatrix(angle);

	float angleX, angleY, angleZ, x, y, z;
	for (int index = 0; index <= 6; ++index) {
		pcl::PointCloud<PointType>::Ptr cloud;
		cloud.reset(new pcl::PointCloud<PointType>);

		//  1. 读图，矫正畸变，转换为点云坐标
		std::string strimg = std::string(imgpath) + std::to_string(m_FILE_INDEX[index]) + ".bmp";
		OpencvImgReadUndistort(index, strimg.c_str(), cloud);
		//qDebug("%d before rotate: x = %f, y = %f, z = %f", index, cloud->points.at(0).x, cloud->points.at(0).y, cloud->points.at(0).z);

		// 所有点云均首先变换至1#初始位置
		if (index != 0) {
			Eigen::Affine3f initial_transform = Eigen::Affine3f::Identity();
			initial_transform.rotate(Eigen::AngleAxisf(-(M_PI_2 + (M_PI / 3.0) * (index - 1)), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud, *cloud, initial_transform);
		}

		//  2.投影至球面
		SphereProject(cloud, cloud, m_fRadius18);

		//  3.乘以旋转矩阵
		for (int i = 0; i < cloud->points.size(); ++i) {
			float nx = cloud->points.at(i).x;
			float ny = cloud->points.at(i).y;
			float nz = cloud->points.at(i).z;
			if (index == 0) {
				// 图像尺寸的统一
				float factor = m_fRadius20 / m_fRadius18;
				cloud->points[i].x = nx * factor;
				cloud->points[i].y = -ny * factor;
				cloud->points[i].z = nz * factor;
			}
			else {
				cloud->points[i].x =
					pvecRotation->at(index - 1)[0][0] * nx +
					pvecRotation->at(index - 1)[0][1] * ny +
					pvecRotation->at(index - 1)[0][2] * nz;
				cloud->points[i].y = -(
					pvecRotation->at(index - 1)[1][0] * nx +
					pvecRotation->at(index - 1)[1][1] * ny +
					pvecRotation->at(index - 1)[1][2] * nz);
				cloud->points[i].z =
					pvecRotation->at(index - 1)[2][0] * nx +
					pvecRotation->at(index - 1)[2][1] * ny +
					pvecRotation->at(index - 1)[2][2] * nz;
			}
		}

		*m_pcloud_stitch += *cloud;
		qDebug("load image %d finished", index);
	}
}


void CPanoramaStitching::ImageStitchUseRotateMatrix(const char* imgpath, int serialindex, int angle)
{
	m_nOpenAngleSelect = angle;

	loadRotateMatrix(angle);

	float angleX, angleY, angleZ, x, y, z;
	for (int index = 0; index <= 6; ++index) {
		pcl::PointCloud<PointType>::Ptr cloud;
		cloud.reset(new pcl::PointCloud<PointType>);

		//  1. 读图，矫正畸变，转换为点云坐标
		std::string strimg = 
			std::string(imgpath) + 
			std::to_string(m_FILE_INDEX[index]) + 
			"_" + 
			std::to_string(serialindex) + 
			".bmp";
		
		OpencvImgReadUndistort(index, strimg.c_str(), cloud);
		//qDebug("%d before rotate: x = %f, y = %f, z = %f", index, cloud->points.at(0).x, cloud->points.at(0).y, cloud->points.at(0).z);

		// 所有点云均首先变换至1#初始位置
		if (index != 0) {
			Eigen::Affine3f initial_transform = Eigen::Affine3f::Identity();
			initial_transform.rotate(Eigen::AngleAxisf(-(M_PI_2 + (M_PI / 3.0) * (index - 1)), Eigen::Vector3f::UnitZ()));
			pcl::transformPointCloud(*cloud, *cloud, initial_transform);
		}

		//  2.投影至球面
		SphereProject(cloud, cloud, m_fRadius18);

		//  3.乘以旋转矩阵
		for (int i = 0; i < cloud->points.size(); ++i) {
			float nx = cloud->points.at(i).x;
			float ny = cloud->points.at(i).y;
			float nz = cloud->points.at(i).z;
			if (index == 0) {
				// 图像尺寸的统一
				float factor = m_fRadius20 / m_fRadius18;
				cloud->points[i].x = nx * factor;
				cloud->points[i].y = -ny * factor;
				cloud->points[i].z = nz * factor;
			}
			else {
				cloud->points[i].x =
					pvecRotation->at(index - 1)[0][0] * nx +
					pvecRotation->at(index - 1)[0][1] * ny +
					pvecRotation->at(index - 1)[0][2] * nz;
				cloud->points[i].y = -(
					pvecRotation->at(index - 1)[1][0] * nx +
					pvecRotation->at(index - 1)[1][1] * ny +
					pvecRotation->at(index - 1)[1][2] * nz);
				cloud->points[i].z =
					pvecRotation->at(index - 1)[2][0] * nx +
					pvecRotation->at(index - 1)[2][1] * ny +
					pvecRotation->at(index - 1)[2][2] * nz;
			}
		}

		*m_pcloud_stitch += *cloud;
		qDebug("load image %d finished", index);
	}
}


int CPanoramaStitching::OpencvImgReadUndistort(int index, const char* imgname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	cv::Mat src = cv::imread(imgname, CV_8UC1);
	if (src.empty()) {
		qDebug("Read Image Error!");
		return -1;
	}
	cv::Mat undistor;
	if (index == 0)
		cv::undistort(src, undistor, camera_matrixa20, distCoeffs20);
	else
		cv::undistort(src, undistor, camera_matrixa18, distCoeffs18);

	int width = undistor.cols;
	int height = undistor.rows;

	// 要将图像坐标转换为相机坐标
	pcl::PointXYZ center(width / 2 + 0.5f, height / 2 + 0.5f, 0);
	for (int j = 0; j < height; ++j) {
		uchar* pdata = undistor.ptr<uchar>(height - j - 1);
		for (int i = 0; i < width; ++i) {
			pcl::PointXYZRGB point;
			point.y = (float)(i + 1) - center.x;
			point.x = -(float)(j + 1) + center.y;
			if (index == 0) {
				point.z = m_fRadius20;
				pdata[i] = pdata[i];
				//if (pdata[i] < 255)
				//	pdata[i] = pdata[i];
			}
			else {
				point.z = m_fRadius18;
				if (index == 6) {
					pdata[i] = pdata[i];
					//if (pdata[i] < 255)
					//	pdata[i] = pdata[i];
				}
			}

			point.r = pdata[i];
			point.g = pdata[i];
			point.b = pdata[i];
			cloud->push_back(point);
		}
	}
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;
	return 0;
}


int CPanoramaStitching::GenerateCloudFromImage(const char* imgname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	int img_width = 1280;
	int img_height = 960;

	// 打开图像文件, 读取像素值
	std::unique_ptr<unsigned char[]> pbuffer(new unsigned char[img_width * img_height]());
	//std::string imgname = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\60\\6.bmp";
	std::ifstream inFile;
	inFile.open(imgname, std::ios::in | std::ios::binary);
	if (!inFile.is_open()) {
		printf("cannot open image %s\n", imgname);
		return -1;
	}
	inFile.seekg(1078, inFile.beg);   // 输入文件偏移1078字节：54 + 1024
	inFile.read((char*)&pbuffer[0], img_width * img_height);
	if (pbuffer == nullptr) {
		printf("read pixel value error: %d\n", GetLastError());
		return -1;
	}

	//读取pcd文件。如果没有指定文件，则创建样本云点
	//pcl::PointCloud<PointType>::Ptr cloud;
	//cloud.reset(new pcl::PointCloud<PointType>);
	pcl::PointXYZ center(img_width / 2 + 0.5f, img_height / 2 + 0.5f, 0);
	for (int j = 0; j < img_height; ++j) {
		for (int i = 0; i < img_width; ++i) {
			pcl::PointXYZRGB point;
			point.x = (float)i - center.x;
			point.y = (float)j - center.y;
			point.z = 0.0f - center.z;
			point.r = pbuffer[j * img_width + i];
			point.g = pbuffer[j * img_width + i];
			point.b = pbuffer[j * img_width + i];
			cloud->push_back(point);
		}
	}
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;
	return 0;
}


void CPanoramaStitching::SaveCloudFile(const char* cloudname)
{
	pcl::PCDWriter writer;
	if (!m_pcloud_stitch->empty()) {
		//writer.write(cloudname, *m_pcloud_stitch);
		pcl::io::savePCDFileBinaryCompressed(cloudname, *m_pcloud_stitch);
		//pcl::io::savePCDFileASCII(cloudname, *cloud);
	}

}


void CPanoramaStitching::SaveCloudFile(const char* cloudname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	//std::string strout = "E:\\检测与装备事业部\\4_国防科大柔性阵列\\to_fang\\60\\6.pcd";
	pcl::PCDWriter writer;
	if (!cloud->empty()) {
		writer.write(cloudname, *cloud);
		//pcl::io::savePCDFileBinaryCompressed(cloudname, cloud);
		//pcl::io::savePCDFileASCII(cloudname, *cloud);
	}
}


void CPanoramaStitching::PointProject(PointType input, PointType *output, float radius)
{
	float factor_k = radius / std::sqrt(std::pow(input.x, 2) + std::pow(input.y, 2) + std::pow(input.z, 2));
	output->x = input.x * factor_k;
	output->y = input.y * factor_k;
	output->z = input.z * factor_k;
	//output->z = std::sqrt(std::pow(radius, 2) - std::pow(input.x, 2) - std::pow(input.y, 2));
}


void CPanoramaStitching::SphereProject(pcl::PointCloud<PointType>::Ptr cloud,
	pcl::PointCloud<PointType>::Ptr projected_cloud, float radius)
{
	for (int index = 0; index < cloud->width; ++index) {
		PointType projected_point;

		PointProject(cloud->at(index), &projected_point, radius);
		projected_cloud->at(index).x = projected_point.x;
		projected_cloud->at(index).y = projected_point.y;
		projected_cloud->at(index).z = projected_point.z;
		//printf("output:x=%f, y=%f, z=%f\n", cloud->at(index).x, cloud->at(index).y, cloud->at(index).z);

	}
}


