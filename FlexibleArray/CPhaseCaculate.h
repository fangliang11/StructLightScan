#pragma once


#include <vector>
#include <time.h>



class CPhaseCaculate
{
public:
	CPhaseCaculate();
	~CPhaseCaculate();

public:
	int action(double* img1, double* img2, double* img3, double* img4,
		int width, int height, double* phase);
	void setCalibrateParam(double k1, double k2, double k3);

protected:
	const int m_phasecycle = 23;
	double m_dtime_consuming;

	void errorProcess();

private:
	int m_nrows;
	int m_ncolumns;
	double m_dfactor_x;
	double m_dfactor_y;
	double m_dfactor_z;
	clock_t m_clock_start;
	clock_t m_clock_finish;

	double* m_pimg1 = nullptr;
	double* m_pimg2 = nullptr;
	double* m_pimg3 = nullptr;
	double* m_pimg4 = nullptr;

	int ImageDCT(double* parryin, double* parryout, int rows, int columns);
	double average(double* arry, int len);
	double variance(double* arry, int len);
	double stdeviation(double* arry, int len);
	void calWrapPhase(double* pinput, double* poutput);
	int imgUnwrapping(double* pinput, double* poutput, double threshold);

};

