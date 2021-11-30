//------------------------------------------------------------------------
/**
\file		GXBitmap.h
\brief		������Ҫ����ͼ�����ʾ�ʹ洢��ͼ����ʾ�ʹ洢��������Ӧ�ڰײ�ɫ�����
ͼ��洢���Դ洢ΪBmp��Raw����ͼ����ʾ�ʹ洢����������

*/
//------------------------------------------------------------------------
#pragma once

#include "GalaxyIncludes.h"

class CGXBitmap
{
public:
	///���캯��
	CGXBitmap(CGXDevicePointer& objCGXDevicePointer,HWND hWnd);

	///��������
	~CGXBitmap(void);
	
	 ///��ʾͼ��
	 void Show(CImageDataPointer& objCImageDataPointer);

	 //��ʾͼ��֡��
	 void Show(CImageDataPointer& objCImageDataPointer,char* strDeviceSNFPS);

	 ///ͼ�������ʾͼ��
	 void ShowImageProcess(CImageProcessConfigPointer& objCfg,CImageDataPointer& objCImageDataPointer);

	 /// �洢Bmpͼ��
	 void SaveBmp(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath);

	 /// �洢Rawͼ��
	 void SaveRaw(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath);

	 ///ͨ��GX_PIXEL_FORMAT_ENTRY��ȡ����Bitλ
	 GX_VALID_BIT_LIST GetBestValudBit(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry);
private:
	///�ж�PixelFormat�Ƿ�Ϊ8λ
	bool __IsPixelFormat8(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry);

	///Ϊ��ɫ���ͼ����ʾ׼����Դ
	void __ColorPrepareForShowImg();

	///Ϊ�ڰ����ͼ����ʾ׼����Դ
	void __MonoPrepareForShowImg();

	///�ж��Ƿ����
	bool __IsCompatible(BITMAPINFO *pBmpInfo, uint64_t nWidth, uint64_t nHeight);

	///����Bitmap����Ϣ
	void __UpdateBitmap(CImageDataPointer& objCImageDataPointer);

	///��m_pBufferRGB��ͼ����ʾ������
	void __DrawImg(BYTE* pBuffer);

	///��m_pBufferRGB��ͼ���֡����ʾ������
	void __DrawImg(BYTE* pBuffer, char* strDeviceSNFPS);

	///��������ռ���ֽ���
	int64_t __GetStride(int64_t nWidth, bool bIsColor);


private:
	//CWnd*              m_pWnd;                           ///<��ʾͼ�񴰿�(�ؼ�)ָ��
	HWND               m_hWnd;
	bool               m_bIsColor ;                      ///<�Ƿ�֧�ֲ�ɫ���
	int64_t            m_nImageHeight;                   ///<ԭʼͼ���
	int64_t            m_nImageWidth;                    ///<ԭʼͼ���
	BITMAPINFO         *m_pBmpInfo;	                     ///<BITMAPINFO �ṹָ�룬��ʾͼ��ʱʹ��
	char               m_chBmpBuf[2048];	             ///<BIMTAPINFO �洢��������m_pBmpInfo��ָ��˻�����
	HDC                m_hDC;                            ///<����ͼ��DC���
	BYTE               *m_pImageBuffer;                  ///<���淭ת���ͼ��������ʾ
private:
	CGXBitmap& operator=(const CGXBitmap&);
	CGXBitmap(const CGXBitmap&);
};

