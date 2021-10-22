#pragma once

// QT
#include <QtWidgets/QMainWindow>
#include <QSplitter>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMdiArea>
#include <QMessageBox>
#include <QTreeView>
#include <QLabel>
#include <QTextEdit>
#include <QStandardItemModel>
#include <QStyleFactory>
#include <QDebug>
#include <QCloseEvent>

#include "ui_StructScan.h"


class CCalibrationWnd;
class CImageWnd;
class CPlotWnd;
class CPointCloudWnd;
class CRoi;
class CDBRoot;
class CCameraControl;
class CPhaseCaculate;

class StructScan : public QMainWindow
{
    Q_OBJECT

public:
    StructScan(QWidget *parent = Q_NULLPTR);
	~StructScan();



private:
    Ui::StructScanClass ui;
	QLabel *m_img;

	//QMdiArea *m_mdiArea = nullptr;
	CPointCloudWnd *m_pCloud = nullptr;
	CImageWnd *m_pimagewnd = nullptr;
	CDBRoot *m_dbroot;

	CCalibrationWnd *m_pcalibwnd = nullptr;
	
	CCameraControl* m_pcamera = nullptr;
	CPhaseCaculate* m_pcaculate = nullptr;

	void closeEvent(QCloseEvent *event);
	void InitialConnection();
	void InitialStatusBar();
	void InitialDockWidget(int width);
	void InitialProjectTree();
	void InitialPropertyTree();

signals:
	void signalOpenPCL();
	void signalSelect();
	void signalDelete();
	void signalAdd();
	void signalFilter();
	void signalMesh();
	void signalSurfaceRebuild();

private slots:
	void systemOnline();
	void systemOffline();
	void onActionProjectNewClicked();
	void onActionProjectSaveClicked();
	void onActionProjectOpenFileClicked();
	void onActionProjectOpenCalibrationClicked();
	void onActionSetupSystemClicked();
	void onActionSetupCameraClicked();
	void onActionSetupProjectorClicked();
	void onActionStartClicked();
	void onActionStopClicked();
	void onActionPointCloudSelectClicked();
	void onActionPointCloudDeleteClicked();
	void onActionPointCloudAddClicked();
	void onActionPointCloudFilterClicked();
	void onActionPointCloudMeshClicked();
	void onActionViewPlotClicked();
	void onActionView2DClicked();
	void onActionView3DClicked();
	void onActionViewFrontClicked();
	void onActionViewBackClicked();
	void onActionViewLeftClicked();
	void onActionViewRightClicked();
	void onActionViewTopClicked();
	void onActionViewBottomClicked();
	void onRootClickedRefrushImgWnd();


};
