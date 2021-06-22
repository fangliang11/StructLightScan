// Unwrapping.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

/*
#include <iostream>
#include <math.h>
#include<cstdio>
#include <iomanip>
#include<algorithm>

using namespace std;
#define PI 3.1415926

void DCT(double ** input, double ** output, int row, int col);
void IDCT(double ** input, double ** output, int row, int col);

int main()
{
	int i = 0;
	int j = 0;
	int u = 0;
	int v = 0;

	const int rows = 3;
	const int cols = 3;

	double inputdata[rows][cols] = {
		{89,101,114},
		{97,115,131},
		{114,134,159},
	};

	double outputdata[rows][cols];

	DCT((double**)inputdata, (double**)outputdata, rows, cols);
	IDCT((double**)outputdata, (double**)inputdata, rows, cols);

	system("pause");
	return 0;
}

// DCT - Discrete Cosine Transform
void DCT(double ** input, double ** output, int row, int col)
{
	cout << "Test in DCT" << endl;
	double ALPHA, BETA;
	int u = 0;
	int v = 0;
	int i = 0;
	int j = 0;

	for (u = 0; u < row; u++)
	{
		for (v = 0; v < col; v++)
		{
			if (u == 0)
			{
				ALPHA = sqrt(1.0 / row);
			}
			else
			{
				ALPHA = sqrt(2.0 / row);
			}

			if (v == 0)
			{
				BETA = sqrt(1.0 / col);
			}
			else
			{
				BETA = sqrt(2.0 / col);
			}

			double tmp = 0.0;
			for (i = 0; i < row; i++)
			{
				for (j = 0; j < col; j++)
				{
					tmp += *((double*)input + col * i + j) * cos((2 * i + 1)*u*PI / (2.0 * row)) * cos((2 * j + 1)*v*PI / (2.0 * col));
				}
			}
			*((double*)output + col * u + v) = ALPHA * BETA * tmp;
		}
	}

	cout << "The result of DCT:" << endl;
	for (int m = 0; m < row; m++)
	{
		for (int n = 0; n < col; n++)
		{
			cout << setw(8) << *((double*)output + col * m + n) << " \t";
		}
		cout << endl;
	}
}

// Inverse DCT
void IDCT(double ** input, double ** output, int row, int col)
{
	cout << "Test in IDCT" << endl;
	double ALPHA, BETA;
	int u = 0;
	int v = 0;
	int i = 0;
	int j = 0;

	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			double tmp = 0.0;
			for (u = 0; u < row; u++)
			{
				for (v = 0; v < col; v++)
				{
					if (u == 0)
					{
						ALPHA = sqrt(1.0 / row);
					}
					else
					{
						ALPHA = sqrt(2.0 / row);
					}
					if (v == 0)
					{
						BETA = sqrt(1.0 / col);
					}
					else
					{
						BETA = sqrt(2.0 / col);
					}
					tmp += ALPHA * BETA * *((double*)input + col * u + v)* cos((2 * i + 1)*u*PI / (2.0 * row)) * cos((2 * j + 1)*v*PI / (2.0 * col));
				}
			}
			*((double*)output + col * i + j) = tmp;
		}
	}

	cout << "The result of IDCT:" << endl;
	for (int m = 0; m < row; m++)
	{
		for (int n = 0; n < col; n++)
		{
			cout << setw(8) << *((double*)output + col * m + n) << "\t";
		}
		cout << endl;
	}
}

*/


#include <iostream>
#include <time.h>
#include <stdio.h>
#include <random>
#include <algorithm>
#include <cmath>
#include <comdef.h>

#include "fftw3.h"
#include "HalconCpp.h"
#include "HDevThread.h"

#include "opencv2/opencv.hpp"
//#pragma comment(lib, "libfftw3-3.lib")

//实部与虚部
#define REAL 0
#define IMAG 1

#define PI 3.1415926

//using namespace HalconCpp;
using namespace cv;

int main() {
	std::wstring imgname = L"E:\\检测与装备事业部\\国防科大三维扫描\\算法开发\\秦司益数据右\\def\\0.bmp";
	HalconCpp::HImage test_img;
	HalconCpp::ReadImage(&test_img, (HalconCpp::HTuple)(imgname.c_str()));
	HalconCpp::HString hs_type;
	Hlong hl_width, hl_height;
	void* ptr = test_img.GetImagePointer1(&hs_type, &hl_width, &hl_height);
	if (ptr == nullptr) {
		std::cout << "read image error\n";
		return -1;
	}

	unsigned char* pdata = static_cast<unsigned char*>(ptr);
	const int width = (const int)hl_width;
	const int height = (const int)hl_height;

	/*
	*fftw_complex 是FFTW自定义的复数类
	*引入<complex>则会使用STL的复数类
	*/
	const int rows = height;
	const int columns = width;
	const int N = 2 * rows;
	const int M = 2 * columns;
	// 1.定义 DCT 的输入与输出 ，real型
	double* input_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);
	double* dct_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);
	double* idct_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);

	// 拷贝像素数据
	const int wide_step = ((width * 8 / 8 + 3) / 4) * 4;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			input_arry[i * width + j] = (double)pdata[i * wide_step + j];
		}
	}
	//std::vector<double> vecSource1(input_arry, input_arry + height * wide_step);


	// 2.定义plan，包含序列长度(行，列)、输入序列、输出序列、变换方向、变换模式
	//   DCT可认为实偶对称数据的DFT，  REDFTab
	fftw_plan plan_dct = fftw_plan_r2r_2d(rows, columns, input_arry, dct_arry, FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE);
	fftw_plan plan_idct = fftw_plan_r2r_2d(rows, columns, dct_arry, idct_arry, FFTW_REDFT01, FFTW_REDFT01, FFTW_ESTIMATE);

	// 3. 执行plan，对于每个plan，应当"一次定义 多次使用"，同一plan的运算速度极快
	//    DCT
	fftw_execute_r2r(plan_dct, input_arry, dct_arry);
	std::cout << "\nDCT2D:" << std::endl;
	clock_t start, finish;
	double* dct_conv_array = (double *)fftw_malloc(sizeof(double) * rows * columns);
	start = clock();
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			dct_conv_array[i * columns + j] = dct_arry[i * columns + j] / (std::sqrt(N) * std::sqrt(M));
			if (i == 0)
				dct_conv_array[i * columns + j] = dct_conv_array[i * columns + j] / std::sqrt(2.0);
			if (j == 0)
				dct_conv_array[i * columns + j] = dct_conv_array[i * columns + j] / std::sqrt(2.0);

			std::cout << dct_conv_array[i * columns + j] << "  ";
			if (j == 9)
				std::cout << "\n";
		}
	}
	finish = clock();
	std::cout << "DCT total time(s) = " << (double)(finish - start)/1000.0 << std::endl;

	//    IDCT
	//  解相位
	int R = 2; // lamda
	double fenzi = 0.0;
	double top_left = dct_arry[0];
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			//  方法1
			//fenzi = 2 * (std::cos(PI * i / rows) + std::cos(PI * j / columns) - 2);

			//  方法2
			fenzi = (2 + 16 * R)*(std::cos(PI * i / rows) + std::cos(PI * j / columns)) -
				2 * R*(std::cos(2 * PI * i / rows) + std::cos(2 * PI * j / columns)) -
				8 * R*std::cos(PI * i / rows) * std::cos(PI * j / columns) - 
				20 * R - 4;
			dct_arry[i * columns + j] = dct_arry[i * columns + j] / fenzi;
		}
	}
	dct_arry[0] = top_left;

	fftw_execute_r2r(plan_idct, dct_arry, idct_arry);
	std::cout << "\nIDCT2D:" << std::endl;
	double* idct_conv_array = (double *)fftw_malloc(sizeof(double) * rows * columns);
	start = clock();
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			idct_conv_array[i * columns + j] = idct_arry[i * columns + j] / (N * M);

			std::cout << idct_conv_array[i * columns + j] << "  ";
			if (j == 9)
				std::cout << "\n";
		}
	}
	finish = clock();
	std::cout << "IDCT total time(s) = " << (double)(finish - start) / 1000.0 << std::endl;


	// 4.销毁plan
	fftw_destroy_plan(plan_dct);
	fftw_destroy_plan(plan_idct);
	fftw_free(input_arry);
	fftw_free(dct_arry);
	fftw_free(idct_arry);
	fftw_free(dct_conv_array);
	fftw_free(idct_conv_array);

	std::cout << "\nPress Enter to exit..." << std::endl;
	std::cin.get();

	return 0;
}




int ImageDCT(int nRefRow, int nRefCol, float** pRefImg)
{
	std::wstring imgname = L"E:\\检测与装备事业部\\国防科大三维扫描\\算法开发\\秦司益数据右\\def\\0.bmp";
	HalconCpp::HImage test_img;
	HalconCpp::ReadImage(&test_img, (HalconCpp::HTuple)(imgname.c_str()));
	HalconCpp::HString hs_type;
	Hlong hl_width, hl_height;
	void* ptr = test_img.GetImagePointer1(&hs_type, &hl_width, &hl_height);
	if (ptr == nullptr) {
		std::cout << "read image error\n";
		return -1;
	}

	unsigned char* pdata = static_cast<unsigned char*>(ptr);
	const int width = (const int)hl_width;
	const int height = (const int)hl_height;

	/*
	*fftw_complex 是FFTW自定义的复数类
	*引入<complex>则会使用STL的复数类
	*/
	const int rows = height;
	const int columns = width;
	const int N = 2 * rows;
	const int M = 2 * columns;
	// 1.定义 DCT 的输入与输出 ，real型
	double* input_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);
	double* dct_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);
	double* idct_arry = (double *)fftw_malloc(sizeof(double) * rows * columns);

	// 拷贝像素数据
	const int wide_step = ((width * 8 / 8 + 3) / 4) * 4;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			input_arry[i * width + j] = (double)pdata[i * wide_step + j];
		}
	}

	// 2.定义plan，包含序列长度(行，列)、输入序列、输出序列、变换方向、变换模式
	//   DCT可认为实偶对称数据的DFT，  REDFTab
	fftw_plan plan_dct = fftw_plan_r2r_2d(rows, columns, input_arry, dct_arry, FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE);
	fftw_plan plan_idct = fftw_plan_r2r_2d(rows, columns, dct_arry, idct_arry, FFTW_REDFT01, FFTW_REDFT01, FFTW_ESTIMATE);

	// 3. 执行plan，对于每个plan，应当"一次定义 多次使用"，同一plan的运算速度极快
	//    DCT
	fftw_execute_r2r(plan_dct, input_arry, dct_arry);
	std::cout << "\nDCT2D:" << std::endl;
	clock_t start, finish;
	double* dct_conv_array = (double *)fftw_malloc(sizeof(double) * rows * columns);
	start = clock();
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			dct_conv_array[i * columns + j] = dct_arry[i * columns + j] / (std::sqrt(N) * std::sqrt(M));
			if (i == 0)
				dct_conv_array[i * columns + j] = dct_conv_array[i * columns + j] / std::sqrt(2.0);
			if (j == 0)
				dct_conv_array[i * columns + j] = dct_conv_array[i * columns + j] / std::sqrt(2.0);

			std::cout << dct_conv_array[i * columns + j] << "  ";
			if (j == 9)
				std::cout << "\n";
		}
	}
	finish = clock();
	std::cout << "DCT total time(s) = " << (double)(finish - start) / 1000.0 << std::endl;

	//    IDCT
	//  解相位
	int R = 2; // lamda
	double fenzi = 0.0;
	double top_left = dct_arry[0];
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			//  方法1
			//fenzi = 2 * (std::cos(PI * i / rows) + std::cos(PI * j / columns) - 2);

			//  方法2
			fenzi = (2 + 16 * R)*(std::cos(PI * i / rows) + std::cos(PI * j / columns)) -
				2 * R*(std::cos(2 * PI * i / rows) + std::cos(2 * PI * j / columns)) -
				8 * R*std::cos(PI * i / rows) * std::cos(PI * j / columns) -
				20 * R - 4;
			dct_arry[i * columns + j] = dct_arry[i * columns + j] / fenzi;
		}
	}
	dct_arry[0] = top_left;

	fftw_execute_r2r(plan_idct, dct_arry, idct_arry);
	std::cout << "\nIDCT2D:" << std::endl;
	double* idct_conv_array = (double *)fftw_malloc(sizeof(double) * rows * columns);
	start = clock();
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			idct_conv_array[i * columns + j] = idct_arry[i * columns + j] / (N * M);

			std::cout << idct_conv_array[i * columns + j] << "  ";
			if (j == 9)
				std::cout << "\n";
		}
	}
	finish = clock();
	std::cout << "IDCT total time(s) = " << (double)(finish - start) / 1000.0 << std::endl;


	// 4.销毁plan
	fftw_destroy_plan(plan_dct);
	fftw_destroy_plan(plan_idct);
	fftw_free(input_arry);
	fftw_free(dct_arry);
	fftw_free(idct_arry);
	fftw_free(dct_conv_array);
	fftw_free(idct_conv_array);

	return 0;
}

void testDftWithMatlab()
{
	const int rows = 3;
	const int columns = 4;
	// N = 2 * n
	const int N = 2 * rows;
	const int M = 2 * columns;
	//fftw_complex inputArry[rows * columns];
	//fftw_complex outputArry[rows * columns];
	double inputArry[rows * columns];
	double dctArry[rows * columns];
	double idctArry[rows * columns];

	double testArry[rows * columns] = {
		89.56,101.39,114.18, 243.07,
		97.34,115.93,131.72, 8.93,
		114.54,134.37,159.25, 66.66 };

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			//inputArry[i][REAL] = testArry[i * columns + j];
			//inputArry[i][IMAG] = 0;
			inputArry[i * columns + j] = testArry[i * columns + j];
		}
	}

	//定义plan，包含序列长度、输入序列、输出序列、变换方向、变换模式
	// DCT可认为实偶对称数据的DFT，  REDFTab
	fftw_plan plan = fftw_plan_r2r_2d(rows, columns, inputArry, dctArry, FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE);
	//对于每个plan，应当"一次定义 多次使用"，同一plan的运算速度极快
	fftw_execute(plan);

	std::cout << "\nDCT2D:" << std::endl;
	double dctConvertArry[rows * columns];
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			dctConvertArry[i * columns + j] = dctArry[i * columns + j] / (std::sqrt(N) * std::sqrt(M));
			if (i == 0)
				dctConvertArry[i * columns + j] = dctConvertArry[i * columns + j] / std::sqrt(2.0);
			if (j == 0)
				dctConvertArry[i * columns + j] = dctConvertArry[i * columns + j] / std::sqrt(2.0);

			std::cout << dctConvertArry[i * columns + j] << "  ";
			if (j == columns - 1)
				std::cout << "\n";
		}
	}

	//IDCT
	fftw_plan plan2 = fftw_plan_r2r_2d(rows, columns, dctArry, idctArry, FFTW_REDFT01, FFTW_REDFT01, FFTW_ESTIMATE);
	fftw_execute(plan2);

	std::cout << "\nIDCT2D:" << std::endl;
	double idctConvertArry[rows * columns];
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			idctConvertArry[i * columns + j] = idctArry[i * columns + j] / (N * M);

			std::cout << idctConvertArry[i * columns + j] << "  ";
			if (j == columns - 1)
				std::cout << "\n";
		}
	}

	//销毁plan
	fftw_destroy_plan(plan);
	fftw_destroy_plan(plan2);
}

void ImageFFT()
{
	/* load original image */
	Mat img_src = imread("E:\\检测与装备事业部\\国防科大三维扫描\\算法开发\\秦司益数据右\\def\\0.bmp", IMREAD_GRAYSCALE);

	/* create new image for FFT & IFFT result */
	Mat img_fft = img_src.clone();
	Mat img_ifft = img_src.clone();
	std::cout << img_src.size;

	/* get image properties */
	int width = img_src.cols;
	int height = img_src.rows;
	int step = width;
	uchar *img_src_data = (uchar *)img_src.data;
	uchar *img_fft_data = (uchar *)img_fft.data;
	uchar *img_ifft_data = (uchar *)img_ifft.data;

	/* initialize arrays for fftw operations */
	fftw_complex *data_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * width * height);
	fftw_complex *fft = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * width * height);
	fftw_complex *ifft = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * width * height);

	/* create plans */
	fftw_plan plan_f = fftw_plan_dft_2d(height, width, data_in, fft, FFTW_FORWARD, FFTW_ESTIMATE);
	fftw_plan plan_b = fftw_plan_dft_2d(height, width, fft, ifft, FFTW_BACKWARD, FFTW_ESTIMATE);

	int i, j, k;
	/* load img_src's data to fftw input */
	for (i = 0, k = 0; i < height; ++i) {
		for (j = 0; j < width; ++j) {
			// method 1: 输入数据乘以(-1)^（i+j），即可中心化
			//data_in[k][0] = (double) img_src_data[i * step + j];
			data_in[k][0] = pow(-1, i + j) * (double)img_src_data[i * step + j];
			data_in[k][1] = 0.0;
			k++;
		}
	}

	/* perform FFT */
	fftw_execute(plan_f);

	/* perform IFFT */
	fftw_execute(plan_b);

	/* normalize FFT result */
	double maxx = 0.0, minn = 10000000000.0;
	for (i = 0; i < width * height; ++i) {
		//fft[i][0] = log(sqrt(fft[i][0] * fft[i][0] + fft[i][1] * fft[i][1]));
		fft[i][0] = log(sqrt(fft[i][0] * fft[i][0] + fft[i][1] * fft[i][1]) + 1); //plus 1 to avoid log(0)
		maxx = fft[i][0] > maxx ? fft[i][0] : maxx;
		minn = fft[i][0] < minn ? fft[i][0] : minn;
	}

	for (i = 0; i < width * height; ++i) {
		fft[i][0] = 255.0 * (fft[i][0] - minn) / (maxx - minn);
	}

	/* copy FFT result to img_fft's data */
	int i0, j0;
	for (i = 0, k = 0; i < height; ++i) {
		for (j = 0; j < width; ++j) {
			if (i < height / 2)
				i0 = i + height / 2;
			else
				i0 = i - height / 2;
			if (j < width / 2)
				j0 = j + width / 2;   // method 2
			else
				j0 = j - width / 2;

			img_fft_data[i * step + j] = (uchar)fft[/*k++*/i0 * width + j0][0];
		}
	}

	/* normalize IFFT result */
	for (i = 0; i < width * height; ++i) {
		ifft[i][0] /= width * height;
	}

	/* copy IFFT result to img_ifft's data */
	for (i = 0, k = 0; i < height; ++i) {
		for (j = 0; j < width; ++j) {
			img_ifft_data[i * step + j] = (uchar)ifft[k++][0];
		}
	}

	/* display images */
	namedWindow("original_image", 1);
	namedWindow("FFT", 1);
	namedWindow("IFFT", 1);

	imshow("original_image", img_src);
	imshow("FFT", img_fft);
	imshow("IFFT", img_ifft);

	waitKey(0);

	/* free memory */
	destroyWindow("original_image");
	destroyWindow("FFT");
	destroyWindow("IFFT");

	fftw_destroy_plan(plan_f);
	fftw_destroy_plan(plan_b);
	fftw_free(data_in);
	fftw_free(fft);
	fftw_free(ifft);
}