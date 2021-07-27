

#include "CPhaseCaculate.h"
#include "fftw3.h"

#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>
#include <cmath>


#define PI 3.1415926



CPhaseCaculate::CPhaseCaculate()
{
	m_nrows = 0;
	m_ncolumns = 0;
	m_dfactor_x = 1.0;
	m_dfactor_y = 1.0;
	m_dfactor_z = 1.0;
	m_dtime_consuming = 0.0;
	m_clock_start = 0;
	m_clock_finish = 0;

}


CPhaseCaculate::~CPhaseCaculate()
{

}


void CPhaseCaculate::errorProcess()
{

}


int CPhaseCaculate::action(double* img1, double* img2, double* img3, double* img4, 
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

	m_nrows = height;
	m_ncolumns = width;
	m_clock_start = clock();

	std::unique_ptr<double[]> arryphase_wraped(new double[m_nrows*m_ncolumns]());
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
	m_clock_finish = clock();
	m_dtime_consuming = (m_clock_finish - m_clock_start) / 1000.0;

	int pause = 0;
}


void CPhaseCaculate::setCalibrateParam(double k1, double k2, double k3)
{
	m_dfactor_x = k1;
	m_dfactor_y = k2;
	m_dfactor_z = k3;

}


int CPhaseCaculate::ImageDCT(double* parryin, double* parryout, int rows, int columns)
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
double CPhaseCaculate::average(double* arry, int len)
{
	double sum = 0.0;
	for (int i = 0; i < len; i++) {
		sum += *(arry + i);
	}
	return sum / len;
}

// 方差
double CPhaseCaculate::variance(double* arry, int len)
{
	double sum = 0.0;
	double ave = average(arry, len);
	for (int i = 0; i < len; i++) {
		sum += std::pow(*(arry + i) - ave, 2);
	}
	return sum / len;
}

// 标准差
double CPhaseCaculate::stdeviation(double* arry, int len)
{
	return std::sqrt(variance(arry, len));
}

// 计算包裹相位
void CPhaseCaculate::calWrapPhase(double* pinput, double* poutput)
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
int CPhaseCaculate::imgUnwrapping(double* pinput, double* poutput, double threshold)
{
	//正弦条纹图像相位展开（解包裹）
	int state = 0;
	//1.计算梯度
	// 包裹相位沿x方向的斜率（梯度）
	std::unique_ptr<double[]> arrydx(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arrydy(new double[m_nrows*m_ncolumns]());
	std::unique_ptr<double[]> arrypossion(new double[m_nrows*m_ncolumns]());
	double dx = 0.0;
	for (int i = 0; i < m_nrows; i++) {
		for (int j = 0; j < m_ncolumns; j++) {
			if (j < m_ncolumns - 1) {
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
				dy = pinput[(i + 1)*m_ncolumns + j] - pinput[i*m_ncolumns + j];
			else
				dy = 0.0;

			if (dy > threshold)
				dy = dy - 2 * PI;
			else if (dy < -threshold)
				dy = dy + 2 * PI;

			arrydy[i*m_ncolumns + j] = dy;
		}
	}

	//2.离散泊松方程
	double possion = 0.0;
	for (int i = 0; i < m_nrows; i++) {
		for (int j = 0; j < m_ncolumns; j++) {
			if (i == 0 && j == 0)
				possion = arrydx[0] + arrydy[0];
			else if (i == 0 && j != 0)
				possion = arrydx[j] - arrydx[j - 1] + arrydy[j];
			else if (i != 0 && j == 0)
				possion = arrydx[i*m_ncolumns] + arrydy[i*m_ncolumns] - arrydy[(i - 1)*m_ncolumns];
			else
				possion = arrydx[i*m_ncolumns + j] - arrydx[i*m_ncolumns + j - 1] +
				arrydy[i*m_ncolumns + j] - arrydy[(i - 1)*m_ncolumns + j];

			arrypossion[i*m_ncolumns + j] = possion;
		}
	}

	//3. DCT变换   二维离散余弦变换
	//4. 解出相位值
	//5. IDCT变换    离散余弦逆变换, 得到相位的最小二乘解
	state = ImageDCT(&arrypossion[0], poutput, m_nrows, m_ncolumns);

	return state;
}



