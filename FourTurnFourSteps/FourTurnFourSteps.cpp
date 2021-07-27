﻿// FourTurnFourSteps.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//


#include <iostream>
#include <fstream>
#include <time.h>
#include <random>
#include <algorithm>
#include <cmath>
#include <vector>

#include "fftw3.h"
#include "HalconCpp.h"
#include "HDevThread.h"


#define PI 3.1415926

using namespace HalconCpp;


int m_nrows = 700;
int m_ncolumns = 1180;
clock_t m_clock_start;
clock_t m_clock_finish;
const int m_phasecycle = 21;
double m_dtime_consuming;


std::unique_ptr<double[]> img1(new double[m_nrows*m_ncolumns]());
std::unique_ptr<double[]> img2(new double[m_nrows*m_ncolumns]());
std::unique_ptr<double[]> img3(new double[m_nrows*m_ncolumns]());
std::unique_ptr<double[]> img4(new double[m_nrows*m_ncolumns]());



int ImageDCT(double* parryin, double* parryout, int rows, int columns)
{
	/*
	*fftw_complex 是FFTW自定义的复数类
	*引入<complex>则会使用STL的复数类
	*/
	const int N = 2 * rows;
	const int M = 2 * columns;
	// 1.定义 DCT 的输入与输出 ，real型
	//double* input_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);
	double* input_arry = parryin;
	double* dct_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);
	double* idct_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);

	// 拷贝像素数据
	//for (int i = 0; i < rows*columns; i++)
	//	input_arry[i] = vecin.at(i);

	// 2.定义plan，包含序列长度(行，列)、输入序列、输出序列、变换方向、变换模式
	//   DCT可认为实偶对称数据的DFT，  REDFTab
	fftw_plan plan_dct = fftw_plan_r2r_2d(rows, columns, input_arry, dct_arry, FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE);
	fftw_plan plan_idct = fftw_plan_r2r_2d(rows, columns, dct_arry, idct_arry, FFTW_REDFT01, FFTW_REDFT01, FFTW_ESTIMATE);

	// 3. 执行plan，对于每个plan，应当"一次定义 多次使用"，同一plan的运算速度极快
	//    DCT
	//clock_t start, finish;
	//start = clock();
	fftw_execute_r2r(plan_dct, input_arry, dct_arry);
	//finish = clock();
	//std::cout << "DCT total time(s) = " << (double)(finish - start) / 1000.0 << std::endl;

	//    IDCT
	//  解相位
	int R = 2; // lamda
	double fenzi = 0.0;
	double top_left = dct_arry[0];
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			fenzi = (2 + 16 * R)*(std::cos(PI * i / rows) + std::cos(PI * j / columns)) -
				2 * R*(std::cos(2 * PI * i / rows) + std::cos(2 * PI * j / columns)) -
				8 * R*std::cos(PI * i / rows) * std::cos(PI * j / columns) -
				20 * R - 4;
			dct_arry[i * columns + j] = dct_arry[i * columns + j] / fenzi;
		}
	}
	dct_arry[0] = top_left;

	//start = clock();
	fftw_execute_r2r(plan_idct, dct_arry, idct_arry);
	//finish = clock();
	//std::cout << "IDCT total time(s) = " << (double)(finish - start) / 1000.0 << std::endl;
	// 结果处理
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			//vecout.push_back(idct_arry[i * columns + j] / (N * M));
			//vecout->at(i*columns + j) = idct_arry[i * columns + j] / (N * M);
			parryout[i*columns + j] = idct_arry[i * columns + j] / (N * M);
		}
	}

	// 4.销毁plan
	fftw_destroy_plan(plan_dct);
	fftw_destroy_plan(plan_idct);
	//fftw_free(input_arry);
	fftw_free(dct_arry);
	fftw_free(idct_arry);

	return 0;
}

// 均值
double average(double* arry, int len)
{
	double sum = 0.0;
	for (int i = 0; i < len; i++) {
		sum += *(arry + i);
	}
	return sum / len;
}

// 方差
double variance(double* arry, int len)
{
	double sum = 0.0;
	double ave = average(arry, len);
	for (int i = 0; i < len; i++) {
		sum += std::pow(*(arry + i) - ave, 2);
	}
	return sum / len;
}

// 标准差
double stdeviation(double* arry, int len)
{
	return std::sqrt(variance(arry, len));
}

// 计算包裹相位
void calWrapPhase(double* pinput, double* poutput)
{
	if (m_phasecycle <= 0) {
		// error code 
		return;
	}
	// 动态数组
	double* arry_sub = new double[m_phasecycle]();
	int half_cycle = m_phasecycle / 2;
	double kmeans = 2.0 / 255.0;
	double stddev = 0.0;
	double iiii = 0.0;
	for (int i = 0; i < m_nrows; i++) {
		for (int j = 0; j < m_ncolumns; j++) {
			std::vector<double> vecsub;
			if (j <= half_cycle) {
				for (int k = 0; k < m_phasecycle; k++)
					arry_sub[k] = *(pinput + i * m_ncolumns + k);
			}
			else if (j >= m_ncolumns - half_cycle) {
				for (int k = 0; k < m_phasecycle; k++)
					arry_sub[k] = *(pinput + i * m_ncolumns + (m_ncolumns - m_phasecycle) + k);
			}
			else {
				for (int k = 0; k < m_phasecycle; k++)
					arry_sub[k] = *(pinput + i * m_ncolumns + (j - half_cycle) + k);
			}
			// value = (gray - mean)/std, 之后归一化;
			stddev = stdeviation(arry_sub, m_phasecycle);
			if (stddev != 0)
				iiii = (*(pinput + i * m_ncolumns + j) - average(arry_sub, m_phasecycle) - 127.5) / stddev * kmeans;
			else
				iiii = iiii;
			//output.push_back(iiii);
			poutput[i*m_ncolumns + j] = iiii;
		}
	}

	delete[] arry_sub;
}

// 解包裹
int imgUnwrapping(double* pinput, double* poutput, double threshold)
{
	//正弦条纹图像相位展开（解包裹）
	int state = 0;
	//1.计算梯度
	// 包裹相位沿x方向的斜率（梯度）
	//std::vector<double> vecdx, vecdy, vecpossion;
	std::unique_ptr<double[]> arrydx(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arrydy(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arrypossion(new double[m_nrows*m_ncolumns]());
	double dx = 0.0;
	for (int i = 0; i < m_nrows; i++) {
		for (int j = 0; j < m_ncolumns; j++) {
			if (j < m_ncolumns - 1) {
				//dx = input.at(i * m_ncolumns + j + 1) - input.at(i * m_ncolumns + j);
				dx = pinput[i*m_ncolumns + j + 1] - pinput[i*m_ncolumns + j];
			}
			else
				dx = 0.0;

			if (dx > threshold)
				dx = dx - 2 * PI;
			else if (dx < -threshold)
				dx = dx + 2 * PI;

			//vecdx.push_back(dx);
			arrydx[i*m_ncolumns + j] = dx;
		}
	}
	// 包裹相位沿y方向的斜率（梯度）
	double dy = 0.0;
	for (int i = 0; i < m_nrows; i++) {
		for (int j = 0; j < m_ncolumns; j++) {
			if (i < m_nrows - 1)
				//dy = input.at((i + 1) * m_ncolumns + j) - input.at(i * m_ncolumns + j);
				dy = pinput[(i + 1)*m_ncolumns + j] - pinput[i*m_ncolumns + j];
			else
				dy = 0.0;

			if (dy > threshold)
				dy = dy - 2 * PI;
			else if (dy < -threshold)
				dy = dy + 2 * PI;

			//vecdy.push_back(dy);
			arrydy[i*m_ncolumns + j] = dy;
		}
	}

	//2.离散泊松方程
	double possion = 0.0;
	for (int i = 0; i < m_nrows; i++) {
		for (int j = 0; j < m_ncolumns; j++) {
			if (i == 0 && j == 0)
				//possion = vecdx.at(0) + vecdy.at(0);
				possion = arrydx[0] + arrydy[0];
			else if (i == 0 && j != 0)
				//possion = (vecdx.at(j) - vecdx.at(j - 1)) + vecdy.at(j);
				possion = arrydx[j] - arrydx[j - 1] + arrydy[j];
			else if (i != 0 && j == 0)
				//possion = vecdx.at(i * m_ncolumns) + (vecdy.at(i * m_ncolumns) - vecdy.at((i - 1) * m_ncolumns));
				possion = arrydx[i*m_ncolumns] + arrydy[i*m_ncolumns] - arrydy[(i - 1)*m_ncolumns];
			else
				//possion = vecdx.at(i * m_ncolumns + j) - vecdx.at(i * m_ncolumns + j - 1) +
				//vecdy.at(i * m_ncolumns + j) - vecdy.at((i - 1) * m_ncolumns + j);
				possion = arrydx[i*m_ncolumns + j] - arrydx[i*m_ncolumns + j - 1] +
				arrydy[i*m_ncolumns + j] - arrydy[(i - 1)*m_ncolumns + j];

			//vecpossion.push_back(possion);
			arrypossion[i*m_ncolumns + j] = possion;
		}
	}

	//3. DCT变换   二维离散余弦变换
	//4. 解出相位值
	//5. IDCT变换    离散余弦逆变换, 得到相位的最小二乘解
	state = ImageDCT(&arrypossion[0], poutput, m_nrows, m_ncolumns);

	return state;
}


void readImage(std::wstring img_path)
{
	HImage  ho_Image, ho_Source1, ho_Source2, ho_Source3, ho_Source4;

	//参考图像，标定
	ReadImage(&ho_Source1, (HTuple)img_path.c_str() + "9.bmp");
	ReadImage(&ho_Source2, (HTuple)img_path.c_str() + "10.bmp");
	ReadImage(&ho_Source3, (HTuple)img_path.c_str() + "11.bmp");
	ReadImage(&ho_Source4, (HTuple)img_path.c_str() + "12.bmp");
	HalconCpp::HString hs_type;
	Hlong hl_width, hl_height;
	void* psource1 = ho_Source1.GetImagePointer1(&hs_type, &hl_width, &hl_height);
	void* psource2 = ho_Source2.GetImagePointer1(&hs_type, &hl_width, &hl_height);
	void* psource3 = ho_Source3.GetImagePointer1(&hs_type, &hl_width, &hl_height);
	void* psource4 = ho_Source4.GetImagePointer1(&hs_type, &hl_width, &hl_height);

	const int rows = (int)hl_height;
	const int columns = (int)hl_width;
	double v1, v2, v3, v4;
	for (int i = 0; i < rows*columns; i++) {
		v1 = (double)*(static_cast<unsigned char*>(psource1) + i);
		v2 = (double)*(static_cast<unsigned char*>(psource2) + i);
		v3 = (double)*(static_cast<unsigned char*>(psource3) + i);
		v4 = (double)*(static_cast<unsigned char*>(psource4) + i);

		img1[i] = v1;
		img2[i] = v2;
		img3[i] = v3;
		img4[i] = v4;

	}
}


int action(double* img1, double* img2, double* img3, double* img4,
	int width, int height, double* phase)
{
	if (!img1 || !img2 || !img3 || !img4 || !phase) {
		// error code : pointer is null

		return -1;
	}
	if (!width || !height || width % 4 != 0) {
		// error code : wrong number, width must be div by 4
		return -1;
	}

	//m_nrows = height;
	//m_ncolumns = width;

	//std::vector<double> vecPhaseSource, vecPhaseUnwrapS;
	std::unique_ptr<double[]> arryphase_wraped(new double[m_nrows*m_ncolumns]());

	//std::vector<double> vec_i1, vec_i2, vec_i3, vec_i4;
	std::unique_ptr<double[]> arryv1(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arryv2(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arryv3(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arryv4(new double[m_nrows*m_ncolumns]());
	calWrapPhase(img1, &arryv1[0]);
	calWrapPhase(img2, &arryv2[0]);
	calWrapPhase(img3, &arryv3[0]);
	calWrapPhase(img4, &arryv4[0]);

	double i1 = 0.0, i2 = 0.0, i3 = 0.0, i4 = 0.0, wrappedphase = 0.0;
	for (int i = 0; i < m_nrows*m_ncolumns; i++) {
		i1 = arryv1[i];
		i2 = arryv2[i];
		i3 = arryv3[i];
		i4 = arryv4[i];

		if (i4 > i2) {
			if (i1 > i3)
				wrappedphase = std::atan((i4 - i2) / (i1 - i3));
			else if (i1 < i3)
				wrappedphase = std::atan((i4 - i2) / (i1 - i3)) + PI;
			else
				wrappedphase = 1 / 2 * PI;
		}
		else if (i4 < i2) {
			if (i1 > i3)
				wrappedphase = std::atan((i4 - i2) / (i1 - i3)) + 2 * PI;
			else if (i1 < i3)
				wrappedphase = std::atan((i4 - i2) / (i1 - i3)) + PI;
			else
				wrappedphase = 3 / 2 * PI;
		}
		else {
			if (i1 > i3)
				wrappedphase = 2 * PI;
			else
				wrappedphase = PI;
		}

		arryphase_wraped[i] = wrappedphase;
	}

	//相位解包裹
	//std::unique_ptr<double[]> arryphase_unwrap(new double[m_nrows*m_ncolumns]());
	try {
		imgUnwrapping(&arryphase_wraped[0], phase, PI);
	}
	catch (const char* errmsg) {
		// error code
		std::cerr << errmsg << std::endl;
	}

	return 0;
}


int main(int argc, char *argv[])
{
	int ret = 0;

	try
	{
		SetSystem("use_window_thread", "true");

		// Default settings used in HDevelop (can be omitted)
		SetSystem("width", 512);
		SetSystem("height", 512);

		std::wstring def_path = L"E:\\检测与装备事业部\\国防科大三维扫描\\算法开发\\扫描仪拍摄测试图\\Test210629_2\\def\\";
		std::wstring ref_path = L"E:\\检测与装备事业部\\国防科大三维扫描\\算法开发\\扫描仪拍摄测试图\\Test210629_2\\ref\\";

		std::unique_ptr<double[]> phaseR(new double[m_nrows*m_ncolumns]());
		std::unique_ptr<double[]> phaseD(new double[m_nrows*m_ncolumns]());


		int state = 0;

		readImage(ref_path);
		m_clock_start = clock();
		state = action(&img1[0], &img2[0], &img3[0], &img4[0], m_ncolumns, m_nrows, &phaseR[0]);
		m_clock_finish = clock();
		m_dtime_consuming = (m_clock_finish - m_clock_start) / 1000.0;
		std::cout << "R unwrap time = " << m_dtime_consuming << std::endl;


		readImage(def_path);
		m_clock_start = clock();
		for (int index = 0; index < 1; index++) {
			state = action(&img1[0], &img2[0], &img3[0], &img4[0], m_ncolumns, m_nrows, &phaseD[0]);
		}
		m_clock_finish = clock();
		m_dtime_consuming = (m_clock_finish - m_clock_start) / 1000.0;
		std::cout << "D unwrap time = " << m_dtime_consuming << std::endl;



		//计算被测物体相位
		std::ofstream outfile;
		outfile.open("test.txt", std::ios::out);
		if (!outfile.is_open()) {
			std::cout << "open file failure!\n";
			return -1;
		}
		//std::vector<double> vecPhase;
		double phaseX = 0.0, phaseY = 0.0, phaseZ = 0.0;
		for (int i = 0; i < m_nrows; i++) {
			for (int j = 0; j < m_ncolumns; j++) {
				//phaseZ = vecPhaseUnwrapM.at(i*m_ncolumns + j) - vecPhaseUnwrapS.at(i*m_ncolumns + j);
				phaseZ = phaseD[i*m_ncolumns + j] - phaseR[i*m_ncolumns + j];
				phaseX = (double)(j + 1)*255.0 / m_ncolumns;
				phaseY = (double)(i + 1)*255.0 / m_nrows;
				outfile << phaseX << " " << phaseY << " " << phaseZ << "\n";
			}
		}
		outfile.close();


	}
	catch (HException &exception)
	{
		fprintf(stderr, "  Error #%u in %s: %s\n", exception.ErrorCode(),
			(const char *)exception.ProcName(),
			(const char *)exception.ErrorMessage());
		ret = 1;
	}

	std::cout << "\nFinished!!" << std::endl;
	//std::cin.get();

	return ret;
}