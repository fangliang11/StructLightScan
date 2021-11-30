// camera control for daheng

#include <exception>

#include "CCameraControl.h"
//#include "FileVersion.h"



CCameraControl::CCameraControl(HWND HImgWnd) : 
	m_bIsOpen(false)
	, m_bIsSnap(false)
	, m_bColorFilter(false)
	, m_bTriggerMode(false)
	, m_bTriggerSource(false)
	, m_bTriggerActive(false)
	, m_bBalanceWhiteAuto(false)
	, m_bBalanceWhiteRatioSelect(false)
	, m_strSavePath("")
	, m_strBalanceWhiteAutoMode("Off")
	, m_pSampleCaptureEventHandle(NULL)
	, m_pBitmap(NULL)
	, m_bCheckSaveBmp(FALSE)
	, m_dEditShutterValue(0)
	, m_dEditGainValue(0)
	, m_dEditBalanceRatioValue(0)
	, m_nTriggerModeOld(0)
	, m_nTriggerSourceOld(0)
	, m_nTriggerActiveOld(0)
	, m_nBalanceWhiteAutoOld(0)
	, m_nBanlanceWhiteRatioOld(0)
	, m_dShutterValueMax(0)
	, m_dShutterValueMin(0)
	, m_dGainValueMax(0)
	, m_dGainValueMin(0)
	, m_dBalanceWhiteRatioMax(0)
	, m_dBalanceWhiteRatioMin(0)
{
	

	//m_pWnd = CWnd::FromHandle(HImgWnd);
	m_phWnd = HImgWnd;

}

CCameraControl::~CCameraControl()
{

	//判断是否停止采集
	if (m_bIsSnap)
	{
		//发送停采命令
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

		//关闭流层通道
		m_objStreamPtr->StopGrab();

		//注销采集回调
		m_objStreamPtr->UnregisterCaptureCallback();

		m_bIsSnap = false;
	}

	try
	{
		//判断是否关闭设备
		if (m_bIsOpen)
		{
			//关闭流对象
			m_objStreamPtr->Close();

			//关闭设备
			m_objDevicePtr->Close();

			m_bIsOpen = false;
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}
	catch (std::exception)
	{
		//do noting
	}

	try
	{
		//释放设备资源
		IGXFactory::GetInstance().Uninit();
	}
	catch (CGalaxyException)
	{
		//do noting
	}
	catch (std::exception)
	{
		//do noting
	}

	if (m_pSampleCaptureEventHandle != NULL)
	{
		delete m_pSampleCaptureEventHandle;
		m_pSampleCaptureEventHandle = NULL;
	}

	if (m_pBitmap != NULL)
	{
		delete m_pBitmap;
		m_pBitmap = NULL;
	}

}


int CCameraControl::initialThisClass()
{

	try
	{
		//初始化设备
		IGXFactory::GetInstance().Init();

		//将窗口指针指向picture控件
		//m_pWnd = GetDlgItem(IDC_SHOW_PICTURE_STATIC);
		m_pSampleCaptureEventHandle = new CSampleCaptureEventHandler();


		//枚举设备
		//GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
		IGXFactory::GetInstance().UpdateDeviceList(1000, m_vectorDeviceInfo);

		//判断枚举到的设备是否大于零，如果不是则弹框提示
		if (m_vectorDeviceInfo.size() <= 0)
		{
			QMessageBox msg(QMessageBox::Warning, "Information", "未发现设备!", QMessageBox::Ok);
			return -1;
		}

			   
		//获取可执行程序的当前路径
		char    strFileName[MAX_PATH] = { 0 };
		std::string  strSavePath = "";
		size_t  nPos = 0;

		GetModuleFileName(NULL, (LPWSTR)strFileName, MAX_PATH);
		strSavePath = strFileName;
		nPos = strSavePath.find_last_of('\\');
		m_strSavePath = strSavePath.substr(0, nPos);
		m_strSavePath = m_strSavePath + "\\SinImagesSaved";



	}
	catch (CGalaxyException& e)
	{
		if (m_pSampleCaptureEventHandle != NULL)
		{
			delete m_pSampleCaptureEventHandle;
			m_pSampleCaptureEventHandle = NULL;
		}

		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return -2;
	}
	catch (std::exception& e)
	{
		if (m_pSampleCaptureEventHandle != NULL)
		{
			delete m_pSampleCaptureEventHandle;
			m_pSampleCaptureEventHandle = NULL;
		}

		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return -3;
	}

	return 0;
}

//初始化设备参数
void CCameraControl::__InitParam()
{
	bool bBalanceWhiteAutoRead = false;         ///< 白平衡是否可读

	//设置采集模式为连续采集模式
	m_objFeatureControlPtr->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

	//是否支持触发模式选择
	m_bTriggerMode = m_objFeatureControlPtr->IsImplemented("TriggerMode");
	if (m_bTriggerMode)
	{
		//设置触发模式关
		m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off");
	}

	//是否支持Bayer格式
	m_bColorFilter = m_objFeatureControlPtr->IsImplemented("PixelColorFilter");

	//是否支持触发源选择
	m_bTriggerSource = m_objFeatureControlPtr->IsImplemented("TriggerSource");

	//是否支持触发极性选择
	m_bTriggerActive = m_objFeatureControlPtr->IsImplemented("TriggerActivation");

	//是否支持自动白平衡
	m_bBalanceWhiteAuto = m_objFeatureControlPtr->IsImplemented("BalanceWhiteAuto");

	//白平衡是否可读
	bBalanceWhiteAutoRead = m_objFeatureControlPtr->IsReadable("BalanceWhiteAuto");

	//如果支持且可读，则获取设备当前白平衡模式
	if (m_bBalanceWhiteAuto)
	{
		if (bBalanceWhiteAutoRead)
		{
			m_strBalanceWhiteAutoMode = m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")
				->GetValue();
		}
	}

	//是否支持自动白平衡通道选择
	m_bBalanceWhiteRatioSelect = m_objFeatureControlPtr->IsImplemented("BalanceRatioSelector");

	//获取曝光时间、增益及自动白平衡系数的最大值和最小值
	m_dShutterValueMax = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMax();
	m_dShutterValueMin = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMin();
	m_dGainValueMax = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMax();
	m_dGainValueMin = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMin();
	m_dBalanceWhiteRatioMax = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMax();
	m_dBalanceWhiteRatioMin = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMin();
}


bool CCameraControl::openCamera()
{
	bool bIsDeviceOpen = false;         ///< 设备是否打开标志
	bool bIsStreamOpen = false;         ///< 设备流是否打开标志

	try
	{

		//打开设备, 默认打开设备列表中的第一台相机
		m_objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(m_vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
		bIsDeviceOpen = true;
		m_objFeatureControlPtr = m_objDevicePtr->GetRemoteFeatureControl();

		//判断画图对象是否为空
		if (m_pBitmap != NULL)
		{
			delete m_pBitmap;
			m_pBitmap = NULL;
		}

		//为画图对象分配内存
		m_pBitmap = new CGXBitmap(m_objDevicePtr, m_phWnd);


		//判断设备流是否大于零，如果大于零则打开流
		int nStreamCount = m_objDevicePtr->GetStreamCount();

		if (nStreamCount > 0)
		{
			m_objStreamPtr = m_objDevicePtr->OpenStream(0);
			m_objStreamFeatureControlPtr = m_objStreamPtr->GetFeatureControl();
			bIsStreamOpen = true;
		}
		else
		{
			throw std::exception("未发现设备流!");
		}

		// 建议用户在打开网络相机之后，根据当前网络环境设置相机的流通道包长值，
		// 以提高网络相机的采集性能,设置方法参考以下代码。
		GX_DEVICE_CLASS_LIST objDeviceClass = m_objDevicePtr->GetDeviceInfo().GetDeviceClass();
		if (GX_DEVICE_CLASS_GEV == objDeviceClass)
		{
			// 判断设备是否支持流通道数据包功能
			if (true == m_objFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
			{
				// 获取当前网络环境的最优包长值
				int nPacketSize = m_objStreamPtr->GetOptimalPacketSize();
				// 将最优包长值设置为当前设备的流通道包长值
				m_objFeatureControlPtr->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
			}
		}

		//初始化设备参数
		__InitParam();

		m_bIsOpen = true;

	}
	catch (CGalaxyException& e)
	{
		//判断设备流是否已打开
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//判断设备是否已打开
		if (bIsDeviceOpen)
		{
			m_objDevicePtr->Close();
		}

		if (m_pBitmap != NULL)
		{
			delete m_pBitmap;
			m_pBitmap = NULL;
		}

		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return false;
	}
	catch (std::exception& e)
	{
		//判断设备流是否已打开
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//判断设备是否已打开
		if (bIsDeviceOpen)
		{
			m_objDevicePtr->Close();
		}

		if (m_pBitmap != NULL)
		{
			delete m_pBitmap;
			m_pBitmap = NULL;
		}

		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return false;
	}

	return true;
}


bool CCameraControl::closeCamera()
{
	//失去焦点
	//m_phWnd.SetFocus();

	try
	{
		//判断是否已停止采集
		if (m_bIsSnap)
		{
			//发送停采命令
			m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

			//关闭流层采集
			m_objStreamPtr->StopGrab();

			//注销采集回调
			m_objStreamPtr->UnregisterCaptureCallback();
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	try
	{
		//关闭流对象
		m_objStreamPtr->Close();

	}
	catch (CGalaxyException)
	{
		//do noting
	}
	try
	{
		//关闭设备
		m_objDevicePtr->Close();
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	m_bIsOpen = false;
	m_bIsSnap = false;

	if (m_pBitmap != NULL)
	{
		delete m_pBitmap;
		m_pBitmap = NULL;
	}

	return true;
}


void CCameraControl::acquireImages()
{
	try
	{
		//设置Buffer处理模式
		m_objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("OldestFirst");

		//注册回调函数
		m_objStreamPtr->RegisterCaptureCallback(m_pSampleCaptureEventHandle, this);

		//开启流层通道
		m_objStreamPtr->StartGrab();

		//发送开采命令
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
		m_bIsSnap = true;

	}
	catch (CGalaxyException& e)
	{
		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return;
	}
	catch (std::exception& e)
	{
		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return;
	}

}


void CCameraControl::stopAcquireImages()
{
	try
	{
		//发送停采命令
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

		//关闭流层通道
		m_objStreamPtr->StopGrab();

		//注销采集回调
		m_objStreamPtr->UnregisterCaptureCallback();
		m_bIsSnap = false;

	}
	catch (CGalaxyException& e)
	{
		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return;
	}
	catch (std::exception& e)
	{
		QMessageBox msg(QMessageBox::Warning, "Error", e.what(), QMessageBox::Ok);
		return;
	}

}


void CCameraControl::setCameraTriggeModel(double delay)
{
	//触发类型选择： FrameStart
	m_objFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");

	//触发模式开关  On   Off
	m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("On");

	//触发源选择： Software    Line0    Line2    Line3
	m_objFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Line0");

	//触发极性：  FallingEdge    RisingEdge
	m_objFeatureControlPtr->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");

	//触发延迟:(以毫秒为单位)    double
	m_objFeatureControlPtr->GetFloatFeature("TriggerDelay")->SetValue(delay);


}


void CCameraControl::doTriggerSoftwareOnce()
{
	if (!m_bIsOpen)
		return;

	if (!m_bTriggerMode)
		return;

	if (!m_bTriggerSource)
		return;

	std::string strText = m_objFeatureControlPtr->GetEnumFeature("TriggerSource")->GetValue();
	if (strText.compare("Software")) {
		//发送软触发命令(在触发模式开启时有效)
		m_objFeatureControlPtr->GetCommandFeature("TriggerSoftware")->Execute();

	}

}

// 曝光时间调整
void CCameraControl::setCameraExposure(int flag, double value)
{
	//  0 固定曝光时间，   1 自动曝光
	if (flag == 0) {

		m_objFeatureControlPtr->GetEnumFeature("ExposureMode")->SetValue("Timed");

		//曝光时间：(单位微秒)  Min:20.0000 Max:1000000.0000
		m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(value);
	}
	else if (flag == 1) {
		m_objFeatureControlPtr->GetEnumFeature("ExposureMode")->SetValue("Timed");

		// 自动曝光：  Off    Continuous    Once
		m_objFeatureControlPtr->GetEnumFeature("ExposureAuto")->SetValue("Continuous");

		//自动曝光最小值
		m_objFeatureControlPtr->GetFloatFeature("AutoExposureTimeMin")->SetValue(20.0000);

		//自动曝光最大值
		m_objFeatureControlPtr->GetFloatFeature("AutoExposureTimeMax")->SetValue(1000000.0000);

		//自动调节感兴趣区域宽度
		m_objFeatureControlPtr->GetIntFeature("AAROIWidth")->SetValue(1440);

		//自动调节感兴趣区域高度
		m_objFeatureControlPtr->GetIntFeature("AAROIHeight")->SetValue(1080);

		//自动调节感兴趣区域X坐标
		m_objFeatureControlPtr->GetIntFeature("AAROIOffsetX")->SetValue(0);

		//自动调节感兴趣区域Y坐标
		m_objFeatureControlPtr->GetIntFeature("AAROIOffsetY")->SetValue(0);

		//期望灰度值
		m_objFeatureControlPtr->GetIntFeature("ExpectedGrayValue")->SetValue(120);
	}

}


void CCameraControl::setUserOutput(int index, bool flag)
{
	if (index < 0)
		return;

	switch (index)
	{
	case 0:
		m_objFeatureControlPtr->GetEnumFeature("UserOutputSelector")->SetValue("UserOutput0");
		m_objFeatureControlPtr->GetBoolFeature("UserOutputValue")->SetValue(flag);
		break;
	case 1:
		m_objFeatureControlPtr->GetEnumFeature("UserOutputSelector")->SetValue("UserOutput1");
		m_objFeatureControlPtr->GetBoolFeature("UserOutputValue")->SetValue(flag);
		break;
	case 2:
		m_objFeatureControlPtr->GetEnumFeature("UserOutputSelector")->SetValue("UserOutput2");
		m_objFeatureControlPtr->GetBoolFeature("UserOutputValue")->SetValue(flag);
		break;
	default:
		break;
	}
}


bool CCameraControl::getUserOutputStatue()
{
	return m_objFeatureControlPtr->GetBoolFeature("UserOutputValue")->GetValue();
}


void CCameraControl::SavePicture(CImageDataPointer& objImageDataPointer)
{
	try
	{
		SYSTEMTIME   sysTime;                   ///< 系统时间
		const char* strFilePath = m_strSavePath.c_str();
		strFilePath = m_strSavePath.c_str();
		
		//创建保存图像的文件夹
		BOOL bRet = CreateDirectory((LPCWSTR)strFilePath, NULL);

		//获取当前时间为图像保存的默认名称
		GetLocalTime(&sysTime);
		QString qstrFileName = QString("%1\\%2_%3_%4_%5_%6_%7_%8.bmp")
			.arg(strFilePath)
			.arg(sysTime.wYear)
			.arg(sysTime.wMonth)
			.arg(sysTime.wDay)
			.arg(sysTime.wHour)
			.arg(sysTime.wMinute)
			.arg(sysTime.wSecond)
			.arg(sysTime.wMilliseconds);

		//保存图像为BMP
		m_pBitmap->SaveBmp(objImageDataPointer, qstrFileName.toLocal8Bit().constData());
	}
	catch (std::exception)
	{
		//由于存图是在线程回调中实现的，如果存图抛出异常。采集线程将终止，为了避免线程终止,存图将对异常不做处理
		return;

	}

}

