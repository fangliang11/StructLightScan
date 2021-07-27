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

	//�ж��Ƿ�ֹͣ�ɼ�
	if (m_bIsSnap)
	{
		//����ͣ������
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

		//�ر�����ͨ��
		m_objStreamPtr->StopGrab();

		//ע���ɼ��ص�
		m_objStreamPtr->UnregisterCaptureCallback();

		m_bIsSnap = false;
	}

	try
	{
		//�ж��Ƿ�ر��豸
		if (m_bIsOpen)
		{
			//�ر�������
			m_objStreamPtr->Close();

			//�ر��豸
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
		//�ͷ��豸��Դ
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
		//��ʼ���豸
		IGXFactory::GetInstance().Init();

		//������ָ��ָ��picture�ؼ�
		//m_pWnd = GetDlgItem(IDC_SHOW_PICTURE_STATIC);
		m_pSampleCaptureEventHandle = new CSampleCaptureEventHandler();


		//ö���豸
		//GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
		IGXFactory::GetInstance().UpdateDeviceList(1000, m_vectorDeviceInfo);

		//�ж�ö�ٵ����豸�Ƿ�����㣬��������򵯿���ʾ
		if (m_vectorDeviceInfo.size() <= 0)
		{
			QMessageBox msg(QMessageBox::Warning, "Information", "δ�����豸!", QMessageBox::Ok);
			return -1;
		}

			   
		//��ȡ��ִ�г���ĵ�ǰ·��
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

//��ʼ���豸����
void CCameraControl::__InitParam()
{
	bool bBalanceWhiteAutoRead = false;         ///< ��ƽ���Ƿ�ɶ�

	//���òɼ�ģʽΪ�����ɼ�ģʽ
	m_objFeatureControlPtr->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

	//�Ƿ�֧�ִ���ģʽѡ��
	m_bTriggerMode = m_objFeatureControlPtr->IsImplemented("TriggerMode");
	if (m_bTriggerMode)
	{
		//���ô���ģʽ��
		m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off");
	}

	//�Ƿ�֧��Bayer��ʽ
	m_bColorFilter = m_objFeatureControlPtr->IsImplemented("PixelColorFilter");

	//�Ƿ�֧�ִ���Դѡ��
	m_bTriggerSource = m_objFeatureControlPtr->IsImplemented("TriggerSource");

	//�Ƿ�֧�ִ�������ѡ��
	m_bTriggerActive = m_objFeatureControlPtr->IsImplemented("TriggerActivation");

	//�Ƿ�֧���Զ���ƽ��
	m_bBalanceWhiteAuto = m_objFeatureControlPtr->IsImplemented("BalanceWhiteAuto");

	//��ƽ���Ƿ�ɶ�
	bBalanceWhiteAutoRead = m_objFeatureControlPtr->IsReadable("BalanceWhiteAuto");

	//���֧���ҿɶ������ȡ�豸��ǰ��ƽ��ģʽ
	if (m_bBalanceWhiteAuto)
	{
		if (bBalanceWhiteAutoRead)
		{
			m_strBalanceWhiteAutoMode = m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")
				->GetValue();
		}
	}

	//�Ƿ�֧���Զ���ƽ��ͨ��ѡ��
	m_bBalanceWhiteRatioSelect = m_objFeatureControlPtr->IsImplemented("BalanceRatioSelector");

	//��ȡ�ع�ʱ�䡢���漰�Զ���ƽ��ϵ�������ֵ����Сֵ
	m_dShutterValueMax = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMax();
	m_dShutterValueMin = m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMin();
	m_dGainValueMax = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMax();
	m_dGainValueMin = m_objFeatureControlPtr->GetFloatFeature("Gain")->GetMin();
	m_dBalanceWhiteRatioMax = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMax();
	m_dBalanceWhiteRatioMin = m_objFeatureControlPtr->GetFloatFeature("BalanceRatio")->GetMin();
}


bool CCameraControl::openCamera()
{
	bool bIsDeviceOpen = false;         ///< �豸�Ƿ�򿪱�־
	bool bIsStreamOpen = false;         ///< �豸���Ƿ�򿪱�־

	try
	{

		//���豸, Ĭ�ϴ��豸�б��еĵ�һ̨���
		m_objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(m_vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
		bIsDeviceOpen = true;
		m_objFeatureControlPtr = m_objDevicePtr->GetRemoteFeatureControl();

		//�жϻ�ͼ�����Ƿ�Ϊ��
		if (m_pBitmap != NULL)
		{
			delete m_pBitmap;
			m_pBitmap = NULL;
		}

		//Ϊ��ͼ��������ڴ�
		m_pBitmap = new CGXBitmap(m_objDevicePtr, m_phWnd);


		//�ж��豸���Ƿ�����㣬��������������
		int nStreamCount = m_objDevicePtr->GetStreamCount();

		if (nStreamCount > 0)
		{
			m_objStreamPtr = m_objDevicePtr->OpenStream(0);
			m_objStreamFeatureControlPtr = m_objStreamPtr->GetFeatureControl();
			bIsStreamOpen = true;
		}
		else
		{
			throw std::exception("δ�����豸��!");
		}

		// �����û��ڴ��������֮�󣬸��ݵ�ǰ���绷�������������ͨ������ֵ��
		// �������������Ĳɼ�����,���÷����ο����´��롣
		GX_DEVICE_CLASS_LIST objDeviceClass = m_objDevicePtr->GetDeviceInfo().GetDeviceClass();
		if (GX_DEVICE_CLASS_GEV == objDeviceClass)
		{
			// �ж��豸�Ƿ�֧����ͨ�����ݰ�����
			if (true == m_objFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
			{
				// ��ȡ��ǰ���绷�������Ű���ֵ
				int nPacketSize = m_objStreamPtr->GetOptimalPacketSize();
				// �����Ű���ֵ����Ϊ��ǰ�豸����ͨ������ֵ
				m_objFeatureControlPtr->GetIntFeature("GevSCPSPacketSize")->SetValue(nPacketSize);
			}
		}

		//��ʼ���豸����
		__InitParam();

		m_bIsOpen = true;

	}
	catch (CGalaxyException& e)
	{
		//�ж��豸���Ƿ��Ѵ�
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//�ж��豸�Ƿ��Ѵ�
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
		//�ж��豸���Ƿ��Ѵ�
		if (bIsStreamOpen)
		{
			m_objStreamPtr->Close();
		}

		//�ж��豸�Ƿ��Ѵ�
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
	//ʧȥ����
	//m_phWnd.SetFocus();

	try
	{
		//�ж��Ƿ���ֹͣ�ɼ�
		if (m_bIsSnap)
		{
			//����ͣ������
			m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

			//�ر�����ɼ�
			m_objStreamPtr->StopGrab();

			//ע���ɼ��ص�
			m_objStreamPtr->UnregisterCaptureCallback();
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	try
	{
		//�ر�������
		m_objStreamPtr->Close();

	}
	catch (CGalaxyException)
	{
		//do noting
	}
	try
	{
		//�ر��豸
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
		//����Buffer����ģʽ
		m_objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("OldestFirst");

		//ע��ص�����
		m_objStreamPtr->RegisterCaptureCallback(m_pSampleCaptureEventHandle, this);

		//��������ͨ��
		m_objStreamPtr->StartGrab();

		//���Ϳ�������
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
		//����ͣ������
		m_objFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

		//�ر�����ͨ��
		m_objStreamPtr->StopGrab();

		//ע���ɼ��ص�
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
	//��������ѡ�� FrameStart
	m_objFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");

	//����ģʽ����  On   Off
	m_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("On");

	//����Դѡ�� Software    Line0    Line2    Line3
	m_objFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Line0");

	//�������ԣ�  FallingEdge    RisingEdge
	m_objFeatureControlPtr->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");

	//�����ӳ�:(�Ժ���Ϊ��λ)    double
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
		//������������(�ڴ���ģʽ����ʱ��Ч)
		m_objFeatureControlPtr->GetCommandFeature("TriggerSoftware")->Execute();

	}

}

// �ع�ʱ�����
void CCameraControl::setCameraExposure(int flag, double value)
{
	//  0 �̶��ع�ʱ�䣬   1 �Զ��ع�
	if (flag == 0) {

		m_objFeatureControlPtr->GetEnumFeature("ExposureMode")->SetValue("Timed");

		//�ع�ʱ�䣺(��λ΢��)  Min:20.0000 Max:1000000.0000
		m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(value);
	}
	else if (flag == 1) {
		m_objFeatureControlPtr->GetEnumFeature("ExposureMode")->SetValue("Timed");

		// �Զ��ع⣺  Off    Continuous    Once
		m_objFeatureControlPtr->GetEnumFeature("ExposureAuto")->SetValue("Continuous");

		//�Զ��ع���Сֵ
		m_objFeatureControlPtr->GetFloatFeature("AutoExposureTimeMin")->SetValue(20.0000);

		//�Զ��ع����ֵ
		m_objFeatureControlPtr->GetFloatFeature("AutoExposureTimeMax")->SetValue(1000000.0000);

		//�Զ����ڸ���Ȥ������
		m_objFeatureControlPtr->GetIntFeature("AAROIWidth")->SetValue(1440);

		//�Զ����ڸ���Ȥ����߶�
		m_objFeatureControlPtr->GetIntFeature("AAROIHeight")->SetValue(1080);

		//�Զ����ڸ���Ȥ����X����
		m_objFeatureControlPtr->GetIntFeature("AAROIOffsetX")->SetValue(0);

		//�Զ����ڸ���Ȥ����Y����
		m_objFeatureControlPtr->GetIntFeature("AAROIOffsetY")->SetValue(0);

		//�����Ҷ�ֵ
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
		SYSTEMTIME   sysTime;                   ///< ϵͳʱ��
		const char* strFilePath = m_strSavePath.c_str();
		strFilePath = m_strSavePath.c_str();
		
		//��������ͼ����ļ���
		BOOL bRet = CreateDirectory((LPCWSTR)strFilePath, NULL);

		//��ȡ��ǰʱ��Ϊͼ�񱣴��Ĭ������
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

		//����ͼ��ΪBMP
		m_pBitmap->SaveBmp(objImageDataPointer, qstrFileName.toLocal8Bit().constData());
	}
	catch (std::exception)
	{
		//���ڴ�ͼ�����̻߳ص���ʵ�ֵģ������ͼ�׳��쳣���ɼ��߳̽���ֹ��Ϊ�˱����߳���ֹ,��ͼ�����쳣��������
		return;

	}

}

