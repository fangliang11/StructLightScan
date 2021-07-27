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
	\brief   �û��̳вɼ��¼�������
	*/
	//----------------------------------------------------------------------------------
	class CSampleCaptureEventHandler :public ICaptureEventHandler
	{
		//---------------------------------------------------------------------------------
		/**
		\brief   �ɼ��ص�����
		\param   objImageDataPointer      ͼ�������
		\param   pFrame                   �û�����

		\return  ��
		*/
		//----------------------------------------------------------------------------------
		void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
		{
			try
			{
				CCameraControl* pcameraControl = (CCameraControl*)pUserParam;

				//��ʾͼ��
				pcameraControl->m_pBitmap->Show(objImageDataPointer);

				uint64_t timestamp = objImageDataPointer->GetTimeStamp();

				//�ж��Ƿ���Ҫ����ͼ��
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
	// ����
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



	/// ��ʼ���豸����
	void __InitParam();


	/// ����ͼ��ΪBMP��ʽ
	void SavePicture(CImageDataPointer& objImageDataPointer);

	CGXBitmap*  m_pBitmap;                  ///< ����ͼ��ָ��

private:

	GxIAPICPP::gxdeviceinfo_vector    m_vectorDeviceInfo;


	CGXDevicePointer                  m_objDevicePtr;             ///< �豸���
	CGXStreamPointer                  m_objStreamPtr;             ///< �豸��
	CGXFeatureControlPointer          m_objFeatureControlPtr;     ///< ���Կ�����
	CGXFeatureControlPointer          m_objStreamFeatureControlPtr; ///< �������������
	//CWnd*                             m_pWnd;                     ///< ����ָ��
	HWND                              m_phWnd;
	CSampleCaptureEventHandler*       m_pSampleCaptureEventHandle;///< �ص�����ָ��

	bool                              m_bIsOpen;                  ///< �豸�򿪱�ʶ
	bool                              m_bIsSnap;                  ///< �豸�ɼ���ʶ
	bool                              m_bColorFilter;             ///< �Ƿ�֧�ֲ�ɫ���
	bool                              m_bTriggerMode;             ///< �Ƿ�֧�ִ���ģʽ
	bool                              m_bTriggerSource;           ///< �Ƿ�֧�ִ���Դ
	bool                              m_bTriggerActive;           ///< �Ƿ�֧�ִ�������
	bool                              m_bBalanceWhiteAuto;        ///< �Ƿ�֧���Զ���ƽ��
	bool                              m_bBalanceWhiteRatioSelect; ///< �Ƿ�֧�ְ�ƽ��ͨ��ѡ��
	double                            m_dShutterValueMax;         ///< �ع�ʱ�����ֵ
	double                            m_dShutterValueMin;         ///< �ع�ʱ����Сֵ
	double                            m_dGainValueMax;            ///< �������ֵ
	double                            m_dGainValueMin;            ///< ������Сֵ
	double                            m_dBalanceWhiteRatioMax;    ///< �Զ���ƽ��ϵ�����ֵ
	double                            m_dBalanceWhiteRatioMin;    ///< �Զ���ƽ��ϵ����Сֵ
	int                               m_nTriggerModeOld;          ///< ��¼����ģʽ
	int                               m_nTriggerSourceOld;        ///< ��¼����Դ
	int                               m_nTriggerActiveOld;        ///< ��¼��������
	int                               m_nBalanceWhiteAutoOld;     ///< ��¼�Զ���ƽ��
	int                               m_nBanlanceWhiteRatioOld;   ///< ��¼�Զ���ƽ��ϵ��
	gxstring                          m_strBalanceWhiteAutoMode;  ///< ��¼�Զ���ƽ�ⷽʽ
	std::string                       m_strSavePath;              ///< ͼ�񱣴�·��
};
