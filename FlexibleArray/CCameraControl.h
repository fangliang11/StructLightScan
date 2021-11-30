#pragma once


#include <QMessageBox>
#include <QString>

//#include <afxwin.h>
#include "GalaxyIncludes.h"
#include "GXBitmap.h"

class CCameraControl
{
	//---------------------------------------------------------------------------------
	/**
	\brief   用户继承采集事件处理类
	*/
	//----------------------------------------------------------------------------------
	class CSampleCaptureEventHandler :public ICaptureEventHandler
	{
		//---------------------------------------------------------------------------------
		/**
		\brief   采集回调函数
		\param   objImageDataPointer      图像处理参数
		\param   pFrame                   用户参数

		\return  无
		*/
		//----------------------------------------------------------------------------------
		void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
		{
			try
			{
				CCameraControl* pcameraControl = (CCameraControl*)pUserParam;

				//显示图像
				pcameraControl->m_pBitmap->Show(objImageDataPointer);

				uint64_t timestamp = objImageDataPointer->GetTimeStamp();

				//判断是否需要保存图像
				if (pcameraControl->m_bCheckSaveBmp == true) {
					pcameraControl->SavePicture(objImageDataPointer);

				}
			}
			catch (CGalaxyException)
			{
				//do nothing
			}
			catch (std::exception)
			{
				//do nothing
			}
		}
	};
	// 构造
public:
	CCameraControl(HWND  HImgWnd);
	~CCameraControl();

	int initialThisClass();

	bool openCamera();
	bool closeCamera();
	void acquireImages();
	void stopAcquireImages();
	void setCameraTriggeModel(double delay);
	void doTriggerSoftwareOnce();
	void setCameraExposure(int flag, double value);
	void setUserOutput(int index, bool flag);
	bool getUserOutputStatue();

private:
	double m_dEditShutterValue;
	double m_dEditGainValue;
	double m_dEditBalanceRatioValue;
	BOOL   m_bCheckSaveBmp;



	/// 初始化设备参数
	void __InitParam();


	/// 保存图像为BMP格式
	void SavePicture(CImageDataPointer& objImageDataPointer);

	CGXBitmap*  m_pBitmap;                  ///< 保存图像指针

private:

	GxIAPICPP::gxdeviceinfo_vector    m_vectorDeviceInfo;


	CGXDevicePointer                  m_objDevicePtr;             ///< 设备句柄
	CGXStreamPointer                  m_objStreamPtr;             ///< 设备流
	CGXFeatureControlPointer          m_objFeatureControlPtr;     ///< 属性控制器
	CGXFeatureControlPointer          m_objStreamFeatureControlPtr; ///< 流层控制器对象
	//CWnd*                             m_pWnd;                     ///< 窗口指针
	HWND                              m_phWnd;
	CSampleCaptureEventHandler*       m_pSampleCaptureEventHandle;///< 回调函数指针

	bool                              m_bIsOpen;                  ///< 设备打开标识
	bool                              m_bIsSnap;                  ///< 设备采集标识
	bool                              m_bColorFilter;             ///< 是否支持彩色相机
	bool                              m_bTriggerMode;             ///< 是否支持触发模式
	bool                              m_bTriggerSource;           ///< 是否支持触发源
	bool                              m_bTriggerActive;           ///< 是否支持触发极性
	bool                              m_bBalanceWhiteAuto;        ///< 是否支持自动白平衡
	bool                              m_bBalanceWhiteRatioSelect; ///< 是否支持白平衡通道选择
	double                            m_dShutterValueMax;         ///< 曝光时间最大值
	double                            m_dShutterValueMin;         ///< 曝光时间最小值
	double                            m_dGainValueMax;            ///< 增益最大值
	double                            m_dGainValueMin;            ///< 增益最小值
	double                            m_dBalanceWhiteRatioMax;    ///< 自动白平衡系数最大值
	double                            m_dBalanceWhiteRatioMin;    ///< 自动白平衡系数最小值
	int                               m_nTriggerModeOld;          ///< 记录触发模式
	int                               m_nTriggerSourceOld;        ///< 记录触发源
	int                               m_nTriggerActiveOld;        ///< 记录触发极性
	int                               m_nBalanceWhiteAutoOld;     ///< 记录自动白平衡
	int                               m_nBanlanceWhiteRatioOld;   ///< 记录自动白平衡系数
	gxstring                          m_strBalanceWhiteAutoMode;  ///< 记录自动白平衡方式
	std::string                       m_strSavePath;              ///< 图像保存路径
};
