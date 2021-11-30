//------------------------------------------------------------------------
/**
\file		GXBitmap.cpp
\brief		������Ҫ����ͼ�����ʾ�ʹ洢��ͼ����ʾ�ʹ洢��������Ӧ�ڰײ�ɫ�����
ͼ��洢���Դ洢ΪBmp��Raw����ͼ����ʾ�ʹ洢������ʵ��

*/
//------------------------------------------------------------------------

#include "GXBitmap.h"


//---------------------------------------------------------------------------------
/**
\brief   ���캯��
\param   objCGXDevicePointer ͼ���豸ָ��
\param   pWnd ����ָ��
\return  ��
*/
//----------------------------------------------------------------------------------
CGXBitmap::CGXBitmap(CGXDevicePointer& objCGXDevicePointer, HWND hWnd)
:m_hWnd(hWnd)
,m_hDC(NULL)
,m_bIsColor(false)
,m_nImageHeight(0)
,m_nImageWidth(0)
,m_pBmpInfo(NULL)
,m_pImageBuffer(NULL)
{
	if ((objCGXDevicePointer.IsNull())||(NULL == hWnd))
	{
		throw std::runtime_error("Argument is error");
	}

	//HWND hWnd = pWnd->m_hWnd;
	if (!::IsWindow(hWnd))
	{
		throw std::runtime_error("The HWND must be form");
	}

	m_hDC  = ::GetDC(hWnd);
	memset(m_chBmpBuf,0,sizeof(m_chBmpBuf));
	gxstring strValue = "";

	//���ͼ���ȡ��߶ȵ�

	m_nImageWidth = (int64_t)objCGXDevicePointer->GetRemoteFeatureControl()->GetIntFeature("Width")->GetValue();
	m_nImageHeight = (int64_t)objCGXDevicePointer->GetRemoteFeatureControl()->GetIntFeature("Height")->GetValue();

	//��ȡ�Ƿ�Ϊ��ɫ���
	if (objCGXDevicePointer->GetRemoteFeatureControl()->IsImplemented("PixelColorFilter"))
	{
		strValue = objCGXDevicePointer->GetRemoteFeatureControl()->GetEnumFeature("PixelColorFilter")->GetValue();

		if ("None" != strValue)
		{
			m_bIsColor = true;
		}
	}

	if (m_bIsColor)
	{
		__ColorPrepareForShowImg();
	} 
	else
	{
		__MonoPrepareForShowImg();
	}
}

//---------------------------------------------------------------------------------
/**
\brief   ��������

\return  ��
*/
//----------------------------------------------------------------------------------
CGXBitmap::~CGXBitmap(void)
{
	//�ͷ�pDC
	::ReleaseDC(m_hWnd, m_hDC);

	if (m_pImageBuffer != NULL)
	{
		delete m_pImageBuffer;
		m_pImageBuffer = NULL;
	}
}

//----------------------------------------------------------------------------------
/**
\brief     �ж�PixelFormat�Ƿ�Ϊ8λ
\param     emPixelFormatEntry ͼ�����ݸ�ʽ
\return    trueΪ8Ϊ���ݣ�falseΪ��8λ����
*/
//----------------------------------------------------------------------------------
bool CGXBitmap::__IsPixelFormat8(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry)
{
	bool bIsPixelFormat8 = false;
	const unsigned  PIXEL_FORMATE_BIT = 0x00FF0000;  ///<�����뵱ǰ�����ݸ�ʽ����������õ���ǰ������λ��
	unsigned uiPixelFormatEntry = (unsigned)emPixelFormatEntry;
	if ((uiPixelFormatEntry & PIXEL_FORMATE_BIT) == GX_PIXEL_8BIT)
	{
		bIsPixelFormat8 = true;
	}
	return bIsPixelFormat8;
}


//----------------------------------------------------------------------------------
/**
\brief     ͨ��GX_PIXEL_FORMAT_ENTRY��ȡ����Bitλ
\param     emPixelFormatEntry ͼ�����ݸ�ʽ
\return    ����Bitλ
*/
//----------------------------------------------------------------------------------
GX_VALID_BIT_LIST CGXBitmap::GetBestValudBit(GX_PIXEL_FORMAT_ENTRY emPixelFormatEntry)
{
	GX_VALID_BIT_LIST emValidBits = GX_BIT_0_7;
	switch (emPixelFormatEntry)
	{
	case GX_PIXEL_FORMAT_MONO8:
	case GX_PIXEL_FORMAT_BAYER_GR8:
	case GX_PIXEL_FORMAT_BAYER_RG8:
	case GX_PIXEL_FORMAT_BAYER_GB8:
	case GX_PIXEL_FORMAT_BAYER_BG8:
		{
			emValidBits = GX_BIT_0_7;
			break;
		}
	case GX_PIXEL_FORMAT_MONO10:
	case GX_PIXEL_FORMAT_BAYER_GR10:
	case GX_PIXEL_FORMAT_BAYER_RG10:
	case GX_PIXEL_FORMAT_BAYER_GB10:
	case GX_PIXEL_FORMAT_BAYER_BG10:
		{
			emValidBits = GX_BIT_2_9;
			break;
		}
	case GX_PIXEL_FORMAT_MONO12:
	case GX_PIXEL_FORMAT_BAYER_GR12:
	case GX_PIXEL_FORMAT_BAYER_RG12:
	case GX_PIXEL_FORMAT_BAYER_GB12:
	case GX_PIXEL_FORMAT_BAYER_BG12:
		{
			emValidBits = GX_BIT_4_11;
			break;
		}
	case GX_PIXEL_FORMAT_MONO14:
		{
			//��ʱû�����������ݸ�ʽ������
			break;
		}
	case GX_PIXEL_FORMAT_MONO16:
	case GX_PIXEL_FORMAT_BAYER_GR16:
	case GX_PIXEL_FORMAT_BAYER_RG16:
	case GX_PIXEL_FORMAT_BAYER_GB16:
	case GX_PIXEL_FORMAT_BAYER_BG16:
		{
			//��ʱû�����������ݸ�ʽ������
			break;
		}
	default:
		break;
	}
	return emValidBits;
}

//---------------------------------------------------------------------------------
/**
\brief   Ϊ��ɫ���ͼ����ʾ׼����Դ

\return  ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::__ColorPrepareForShowImg()
{
	//--------------------------------------------------------------------
	//---------------------------��ʼ��bitmapͷ---------------------------
	m_pBmpInfo								= (BITMAPINFO *)m_chBmpBuf;
	m_pBmpInfo->bmiHeader.biSize			= sizeof(BITMAPINFOHEADER);
	m_pBmpInfo->bmiHeader.biWidth			= (LONG)m_nImageWidth;
	m_pBmpInfo->bmiHeader.biHeight			= (LONG)m_nImageHeight;

	m_pBmpInfo->bmiHeader.biPlanes			= 1;
	m_pBmpInfo->bmiHeader.biBitCount        = 24;
	m_pBmpInfo->bmiHeader.biCompression		= BI_RGB;
	m_pBmpInfo->bmiHeader.biSizeImage		= 0;
	m_pBmpInfo->bmiHeader.biXPelsPerMeter	= 0;
	m_pBmpInfo->bmiHeader.biYPelsPerMeter	= 0;
	m_pBmpInfo->bmiHeader.biClrUsed			= 0;
	m_pBmpInfo->bmiHeader.biClrImportant	= 0;
}

//---------------------------------------------------------------------------------
/**
\brief   Ϊ�ڰ����ͼ����ʾ׼����Դ

\return  ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::__MonoPrepareForShowImg()
{
	//---------------------------------------------------------------------
	//----------------------��ʼ��bitmapͷ---------------------------------
	m_pBmpInfo								= (BITMAPINFO *)m_chBmpBuf;
	m_pBmpInfo->bmiHeader.biSize			= sizeof(BITMAPINFOHEADER);
	m_pBmpInfo->bmiHeader.biWidth			= (LONG)m_nImageWidth;
	m_pBmpInfo->bmiHeader.biHeight			= (LONG)m_nImageHeight;	

	m_pBmpInfo->bmiHeader.biPlanes			= 1;
	m_pBmpInfo->bmiHeader.biBitCount		= 8; // �ڰ�ͼ��Ϊ8
	m_pBmpInfo->bmiHeader.biCompression		= BI_RGB;
	m_pBmpInfo->bmiHeader.biSizeImage		= 0;
	m_pBmpInfo->bmiHeader.biXPelsPerMeter	= 0;
	m_pBmpInfo->bmiHeader.biYPelsPerMeter	= 0;
	m_pBmpInfo->bmiHeader.biClrUsed			= 0;
	m_pBmpInfo->bmiHeader.biClrImportant	= 0;

	// �ڰ�ͼ����Ҫ��ʼ����ɫ��
	for(int i=0;i<256;i++)
	{
		m_pBmpInfo->bmiColors[i].rgbBlue	=i;
		m_pBmpInfo->bmiColors[i].rgbGreen	=i;
		m_pBmpInfo->bmiColors[i].rgbRed		=i;
		m_pBmpInfo->bmiColors[i].rgbReserved=i;
	}

	//Ϊ������ת���ͼ�����ݷ���ռ�
	if (m_pImageBuffer != NULL)
	{
		delete m_pImageBuffer;
		m_pImageBuffer = NULL;
	}

	m_pImageBuffer = new BYTE[(size_t)(m_nImageWidth * m_nImageHeight)];
	if (m_pImageBuffer == NULL)
	{
		throw std::runtime_error("Fail to allocate memory");
	}
}

//----------------------------------------------------------------------------------
/**
\brief     �ж��Ƿ����
\param     pBmpInfo BITMAPINFOָ��
\param     nWidth ͼ���
\param     nHeight ͼ���
\return    trueΪһ����false��һ��
*/
//----------------------------------------------------------------------------------
bool CGXBitmap::__IsCompatible(BITMAPINFO *pBmpInfo, uint64_t nWidth, uint64_t nHeight)
{
	if (pBmpInfo == NULL
		|| pBmpInfo->bmiHeader.biHeight != nHeight
		|| pBmpInfo->bmiHeader.biWidth != nWidth
		)
	{
		return false;
	}
	return true;
}

//----------------------------------------------------------------------------------
/**
\brief     ���ͼ���Ƿ�ı䲢����Buffer��Ϊͼ����ʾ׼����Դ
\param     objCImageDataPointer  ͼ�����ݶ���
\return    ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::__UpdateBitmap(CImageDataPointer& objCImageDataPointer)
{
	if (!__IsCompatible(m_pBmpInfo, objCImageDataPointer->GetWidth(), objCImageDataPointer->GetHeight()))
	{
		m_nImageWidth = objCImageDataPointer->GetWidth();
		m_nImageHeight = objCImageDataPointer->GetHeight();
		if (m_bIsColor)
		{
			__ColorPrepareForShowImg();
		} 
		else
		{
			__MonoPrepareForShowImg();
		}
	}
}

//---------------------------------------------------------------------------------
/**
\brief   ��m_pBufferRGB��ͼ����ʾ������
\param   pBuffer  ͼ������Bufferָ��
\return  ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::__DrawImg(BYTE* pBuffer)
{
	int nWndWidth  = 0;
	int nWndHeight = 0;

	// Ϊ��ͼ��׼��
	RECT objRect;
	//m_pWnd->GetClientRect(&objRect);	
	GetWindowRect(m_hWnd, &objRect);
	nWndWidth  = objRect.right - objRect.left;
	nWndHeight = objRect.bottom - objRect.top;

	// ������ø���䣬����ͼ�����ˮ��
	::SetStretchBltMode(m_hDC, COLORONCOLOR);
	::StretchDIBits(m_hDC,
		0,						
		0,
		nWndWidth,
		nWndHeight,
		0,
		0,
		(int)m_nImageWidth,
		(int)m_nImageHeight,
		pBuffer,
		m_pBmpInfo,
		DIB_RGB_COLORS,
		SRCCOPY
		);
}

//---------------------------------------------------------------------------------
/**
\brief   ��m_pBufferRGB��ͼ����ʾ������
\param   pBuffer         ͼ������Bufferָ��
\param   strDeviceSNFPS  �豸֡�����к�
\return  ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::__DrawImg(BYTE* pBuffer, char* strDeviceSNFPS)
{
	
	int nWndWidth  = 0;
	int nWndHeight = 0;

	// Ϊ��ͼ��׼��
	RECT objRect;
	//m_pWnd->GetClientRect(&objRect);
	GetWindowRect(m_hWnd, &objRect);
	nWndWidth  = objRect.right - objRect.left;
	nWndHeight = objRect.bottom - objRect.top;

	HDC      objMemDC = ::CreateCompatibleDC(m_hDC);
	HBITMAP  objMemBmp= CreateCompatibleBitmap(m_hDC, nWndWidth, nWndHeight);
	::SelectObject(objMemDC,objMemBmp);

	// ������ø���䣬����ͼ�����ˮ��
	::SetStretchBltMode(objMemDC, COLORONCOLOR);
	::StretchDIBits(objMemDC,
		0,						
		0,
		nWndWidth,
		nWndHeight,
		0,
		0,
		(int)m_nImageWidth,
		(int)m_nImageHeight,
		pBuffer,
		m_pBmpInfo,
		DIB_RGB_COLORS,
		SRCCOPY
		);

	TextOut(objMemDC,0,0,(LPCWSTR)strDeviceSNFPS,(int)strlen(strDeviceSNFPS));
	StretchBlt(m_hDC,
		0,
		0,
		nWndWidth,
		nWndHeight,
		objMemDC,
		0,
		0,
		nWndWidth,
		nWndHeight,
		SRCCOPY);

	::DeleteDC(objMemDC);
	DeleteObject(objMemBmp);
}
//----------------------------------------------------------------------------------
/**
\brief     ��������ռ���ֽ���
\param     nWidth  ͼ����
\param     bIsColor  �Ƿ��ǲ�ɫ���
\return    ͼ��һ����ռ���ֽ���
*/
//----------------------------------------------------------------------------------
int64_t CGXBitmap::__GetStride(int64_t nWidth, bool bIsColor)
{
	return bIsColor ? nWidth * 3 : nWidth;
}

//----------------------------------------------------------------------------------
/**
\brief     ������ʾͼ��
\param     objCImageDataPointer  ͼ�����ݶ���
\return    ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::Show(CImageDataPointer& objCImageDataPointer)
{
	GX_VALID_BIT_LIST emValidBits = GX_BIT_0_7;
	BYTE* pBuffer = NULL;

	if (objCImageDataPointer.IsNull())
	{
		throw std::runtime_error("NULL pointer dereferenced");
	}

	//���ͼ���Ƿ�ı䲢����Buffer
	__UpdateBitmap(objCImageDataPointer);

	emValidBits = GetBestValudBit(objCImageDataPointer->GetPixelFormat());
	if (m_bIsColor)
	{
		pBuffer = (BYTE*)objCImageDataPointer->ConvertToRGB24(emValidBits, GX_RAW2RGB_NEIGHBOUR, true);
		__DrawImg(pBuffer);
	}
	else
	{
		if (__IsPixelFormat8(objCImageDataPointer->GetPixelFormat()))
		{
			pBuffer = (BYTE*)objCImageDataPointer->GetBuffer();
		}
		else
		{
			pBuffer = (BYTE*)objCImageDataPointer->ConvertToRaw8(emValidBits);
		}

		// �ڰ������Ҫ��ת���ݺ���ʾ
		for(int i =0;i <m_nImageHeight;i++)
		{
			memcpy(m_pImageBuffer+i*m_nImageWidth, pBuffer+(m_nImageHeight-i-1)*m_nImageWidth,(size_t)m_nImageWidth);
		}

		__DrawImg(m_pImageBuffer);
	}
	
}

//----------------------------------------------------------------------------------
/**
\brief     ������ʾͼ��
\param     objCImageDataPointer  ͼ�����ݶ���
\param     strDeviceSNFPS        ͼ��֡�����к�
\return    ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::Show(CImageDataPointer& objCImageDataPointer,char* strDeviceSNFPS)
{
	GX_VALID_BIT_LIST emValidBits = GX_BIT_0_7;
	BYTE* pBuffer = NULL;

	if (objCImageDataPointer.IsNull())
	{
		throw std::runtime_error("NULL pointer dereferenced");
	}

	//���ͼ���Ƿ�ı䲢����Buffer
	__UpdateBitmap(objCImageDataPointer);

	emValidBits = GetBestValudBit(objCImageDataPointer->GetPixelFormat());
	if (m_bIsColor)
	{
		pBuffer = (BYTE*)objCImageDataPointer->ConvertToRGB24(emValidBits, GX_RAW2RGB_NEIGHBOUR, true);
		__DrawImg(pBuffer,strDeviceSNFPS);
	}
	else
	{
		if (__IsPixelFormat8(objCImageDataPointer->GetPixelFormat()))
		{
			pBuffer = (BYTE*)objCImageDataPointer->GetBuffer();
		}
		else
		{
			pBuffer = (BYTE*)objCImageDataPointer->ConvertToRaw8(emValidBits);
		}

		// �ڰ������Ҫ��ת���ݺ���ʾ
		for(int i =0;i <m_nImageHeight;i++)
		{
			memcpy(m_pImageBuffer + i * m_nImageWidth, pBuffer + (m_nImageHeight - i -1) * m_nImageWidth,(size_t)m_nImageWidth);
		}

		__DrawImg(m_pImageBuffer,strDeviceSNFPS);
	}
	
}

//----------------------------------------------------------------------------------
/**
\brief     ����ͼ�������ʾͼ��
\param     objCfg  ͼ������ڲ�������
\param     objCImageDataPointer  ͼ�����ݶ���
\return    ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::ShowImageProcess(CImageProcessConfigPointer& objCfg,CImageDataPointer& objCImageDataPointer)
{
	if ((objCfg.IsNull())||(objCImageDataPointer.IsNull()))
	{
		throw std::runtime_error("NULL pointer dereferenced");
	}

	//���ͼ���Ƿ�ı䲢����Buffer
	__UpdateBitmap(objCImageDataPointer);

	BYTE* pBuffer = (BYTE*)objCImageDataPointer->ImageProcess(objCfg);

	if (m_bIsColor)
	{
		__DrawImg(pBuffer);
	}
	else
	{
		// �ڰ������Ҫ��ת���ݺ���ʾ
		for(int i =0;i <m_nImageHeight;i++)
		{
			memcpy(m_pImageBuffer + i * m_nImageWidth, pBuffer + (m_nImageHeight - i -1) * m_nImageWidth,(size_t)m_nImageWidth);
		}

		__DrawImg(m_pImageBuffer);
	}
}

//----------------------------------------------------------------------------------
/**
\brief     �洢Bmpͼ��
\param     objCImageDataPointer  ͼ�����ݶ���
\param     strFilePath  ��ʾͼ���ļ���
\return    ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::SaveBmp(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath)
{
	GX_VALID_BIT_LIST emValidBits = GX_BIT_0_7;
	BYTE* pBuffer = NULL;

	if ((objCImageDataPointer.IsNull())||(strFilePath == ""))
	{
		throw std::runtime_error("Argument is error");
	}

	//���ͼ���Ƿ�ı䲢����Buffer
	__UpdateBitmap(objCImageDataPointer);

	emValidBits = GetBestValudBit(objCImageDataPointer->GetPixelFormat());

	if (m_bIsColor)
	{
		pBuffer = (BYTE*)objCImageDataPointer->ConvertToRGB24(emValidBits, GX_RAW2RGB_NEIGHBOUR, true);
	}
	else
	{
		if (__IsPixelFormat8(objCImageDataPointer->GetPixelFormat()))
		{
			pBuffer = (BYTE*)objCImageDataPointer->GetBuffer();
		}
		else
		{
			pBuffer = (BYTE*)objCImageDataPointer->ConvertToRaw8(emValidBits);
		}
		// �ڰ������Ҫ��ת���ݺ���ʾ
		for(int i =0;i < m_nImageHeight;i++)
		{
			memcpy(m_pImageBuffer + i * m_nImageWidth, pBuffer + (m_nImageHeight - i -1) * m_nImageWidth,(size_t)m_nImageWidth);
		}
		pBuffer = m_pImageBuffer;
	}

	DWORD		         dwImageSize = (DWORD)(__GetStride(m_nImageWidth,m_bIsColor) * m_nImageHeight);
	BITMAPFILEHEADER     stBfh	     = {0};
	DWORD		         dwBytesRead = 0;

	stBfh.bfType	= (WORD)'M' << 8 | 'B';			 //�����ļ�����
	stBfh.bfOffBits = m_bIsColor ?sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER)
		:sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + (256 * 4);	//�����ļ�ͷ��СtrueΪ��ɫ,falseΪ�ڰ�
	stBfh.bfSize	= stBfh.bfOffBits + dwImageSize; //�ļ���С

	DWORD dwBitmapInfoHeader = m_bIsColor ?sizeof(BITMAPINFOHEADER)
		:sizeof(BITMAPINFOHEADER) + (256 * 4);	//����BitmapInfoHeader��СtrueΪ��ɫ,falseΪ�ڰ�

	//�����ļ�
	HANDLE hFile = ::CreateFile((LPCWSTR)strFilePath.c_str(),
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,														
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	if (hFile == INVALID_HANDLE_VALUE) 
	{
		throw std::runtime_error("Handle is invalid");
	}

	::WriteFile(hFile, &stBfh, sizeof(BITMAPFILEHEADER), &dwBytesRead, NULL);
	::WriteFile(hFile, m_pBmpInfo, dwBitmapInfoHeader, &dwBytesRead, NULL); //�ڰ׺Ͳ�ɫ����Ӧ
	::WriteFile(hFile, pBuffer, dwImageSize, &dwBytesRead, NULL);

	CloseHandle(hFile);
}

//----------------------------------------------------------------------------------
/**
\brief     �洢Rawͼ��
\param     objCImageDataPointer  ͼ�����ݶ���
\param     strFilePath  ��ʾͼ���ļ���
\return    ��
*/
//----------------------------------------------------------------------------------
void CGXBitmap::SaveRaw(CImageDataPointer& objCImageDataPointer,const std::string& strFilePath)
{
	if ((objCImageDataPointer.IsNull())||(strFilePath == ""))
	{
		throw std::runtime_error("Argument is error");
	}

	//���ͼ���Ƿ�ı䲢����Buffer
	__UpdateBitmap(objCImageDataPointer);

	DWORD   dwImageSize = (DWORD)objCImageDataPointer->GetPayloadSize();  // д���ļ��ĳ���
	DWORD   dwBytesRead = 0;                // �ļ���ȡ�ĳ���

	BYTE* pbuffer = (BYTE*)objCImageDataPointer->GetBuffer();
	if (!m_bIsColor)
	{
		// �ڰ������Ҫ��ת���ݺ���ʾ
		for(int i =0;i < m_nImageHeight;i++)
		{
			memcpy(m_pImageBuffer + i * m_nImageWidth, pbuffer + (m_nImageHeight - i -1) * m_nImageWidth,(size_t)m_nImageWidth);
		}
		pbuffer = m_pImageBuffer;
	}

	// �����ļ�
	HANDLE hFile = ::CreateFile((LPCWSTR)strFilePath.c_str(),
		GENERIC_WRITE,
		FILE_SHARE_READ,
		NULL,
		CREATE_ALWAYS,														
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	if (hFile == INVALID_HANDLE_VALUE)   // ����ʧ���򷵻�
	{
		throw std::runtime_error("Handle is invalid");
	}
	else                                 // ����Rawͼ��          
	{ 
		::WriteFile(hFile, pbuffer, dwImageSize, &dwBytesRead, NULL);
		CloseHandle(hFile);
	}
}