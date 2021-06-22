#pragma once



class CFDCT
{
public:
	CFDCT();
	~CFDCT();

public:
	bool FDCT(double ** input, double ** output, int row, int col);
	bool IFDCT(double ** input, double ** output, int row, int col);

private:


};

