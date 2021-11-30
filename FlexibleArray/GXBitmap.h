//------------------------------------------------------------------------
/**
\file		GXBitmap.h
\brief		此类主要用于图像的显示和存储，图像显示和存储可以自适应黑白彩色相机，
图像存储可以存储为Bmp、Raw，对图像显示和存储进行了声明

*/
//------------------------------------------------------------------------
#pragma once

#include "GalaxyIncludes.h"

class CGXBitmap
{
public:
	///构造函数
	CGXBitmap(CGXDevicePointer& objCGXDevicePointer,HWND hWnd);

	///析构函数
	~CGXBitmap(void);
	
	 ///显示图像
	 void Show(CImageDataPointer& objCImageDataPointer);

	 //显示图像及帧率
	 void Show(CImageDataPointer& objCImageDataPointer,char* strDeviceSNFPS);

	 ///图像处理后并显示图像
	 void ShowImageProcess(CImageProcessConfigPointer& objCfg,CImageDataPointer& objCImageDataPointer);

	 /// 存储Bmp图像
	 void SaveBmp(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath);

	 /// 存储Raw图像
	 void SaveRaw(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath);

	 ///通过GX_PIXEL_FORMAT_ENTRY获取最优Bit位
	 GX_VALID_BIT_LIST GetBestValudBit(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry);
private:
	///判断PixelFormat是否为8位
	bool __IsPixelFormat8(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry);

	///为彩色相机图像显示准备资源
	void __ColorPrepareForShowImg();

	///为黑白相机图像显示准备资源
	void __MonoPrepareForShowImg();

	///判断是否兼容
	bool __IsCompatible(BITMAPINFO *pBmpInfo, uint64_t nWidth, uint64_t nHeight);

	///更新Bitmap的信息
	void __UpdateBitmap(CImageDataPointer& objCImageDataPointer);

	///将m_pBufferRGB中图像显示到界面
	void __DrawImg(BYTE* pBuffer);

	///将m_pBufferRGB中图像和帧率显示到界面
	void __DrawImg(BYTE* pBuffer, char* strDeviceSNFPS);

	///计算宽度所占的字节数
	int64_t __GetStride(int64_t nWidth, bool bIsColor);


private:
	//CWnd*              m_pWnd;                           ///<显示图像窗口(控件)指针
	HWND               m_hWnd;
	bool               m_bIsColor ;                      ///<是否支持彩色相机
	int64_t            m_nImageHeight;                   ///<原始图像高
	int64_t            m_nImageWidth;                    ///<原始图像宽
	BITMAPINFO         *m_pBmpInfo;	                     ///<BITMAPINFO 结构指针，显示图像时使用
	char               m_chBmpBuf[2048];	             ///<BIMTAPINFO 存储缓冲区，m_pBmpInfo即指向此缓冲区
	HDC                m_hDC;                            ///<绘制图像DC句柄
	BYTE               *m_pImageBuffer;                  ///<保存翻转后的图像用于显示
private:
	CGXBitmap& operator=(const CGXBitmap&);
	CGXBitmap(const CGXBitmap&);
};

