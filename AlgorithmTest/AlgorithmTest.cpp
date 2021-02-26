/*
演示各种不同的鼠标交互模式
*/
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkVolume16Reader.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkOutlineFilter.h"
#include "vtkCamera.h"
#include "vtkProperty.h"
#include "vtkPolyDataNormals.h"
#include "vtkContourFilter.h"
#include "vtkMarchingCubes.h"
#include "vtkDICOMImageReader.h"
#include "vtkImageCast.h"
#include "vtkDecimatePro.h"
#include "vtkStripper.h"
#include "vtkImageShrink3D.h"
#include "vtkSmoothPolyDataFilter.h"
#include "vtkTriangleFilter.h"
#include "vtkFeatureEdges.h"
#include "vtkPolyDataWriter.h"
#include "vtkImageData.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkInteractorStyleUnicam.h"
#include "vtkInteractorStyleUser.h"
#include "vtkInteractorStyleTrackballActor.h"
#include "vtkInteractorStyleTerrain.h"
#include "vtkInteractorStyleSwitch.h"
#include "vtkInteractorStyleFlight.h"

int main()
{
	vtkRenderer *aRenderer = vtkRenderer::New();
	vtkRenderWindow *renWin = vtkRenderWindow::New();
	renWin->AddRenderer(aRenderer);
	///////////////////////////////////////////////////////////////////////////
	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);
	// 总共10种交互方式 下面是八种
	//   vtkInteractorStyleTrackballCamera *style = //常用的方式 移动摄像机
	//     vtkInteractorStyleTrackballCamera::New();
	//   iren->SetInteractorStyle(style);
	// vtkInteractorStyleTrackballActor *style = //移动对象
	//    vtkInteractorStyleTrackballActor::New();
	// iren->SetInteractorStyle(style);
	// For a 3-button mouse, the left button is for rotation, the right button for zooming,
	//the middle button for panning, and ctrl + left button for spinning.
	//(With fewer mouse buttons, ctrl + shift + left button is for zooming,
	//and shift + left button is for panning.)

	//   vtkInteractorStyleUnicam *style = //只有放大和平移的功能
	//     vtkInteractorStyleUnicam::New();
	//   iren->SetInteractorStyle(style);

	//   vtkInteractorStyleUser *style =   //没有鼠标响应 主要用于用户自定义的操作
	//     vtkInteractorStyleUser::New();
	//   iren->SetInteractorStyle(style);
	//   vtkInteractorStyleTerrain *style =
	//     vtkInteractorStyleTerrain::New();
	//   iren->SetInteractorStyle(style);
	/*
	vtkInteractorStyleSwitch *style =
	vtkInteractorStyleSwitch::New();
	iren->SetInteractorStyle(style);
	*/
	//   vtkInteractorStyleFlight *style =
	//     vtkInteractorStyleFlight::New();
	//   iren->SetInteractorStyle(style);
	// vtkInteractorStyleRubberBandZoom *style=vtkInteractorStyleRubberBandZoom::New();
	// iren->SetInteractorStyle(style);
	// vtkInteractorStyleSwitch *style=vtkInteractorStyleSwitch::New();
	//     iren->SetInteractorStyle(style);
	//     vtkInteractorStyleJoystickCamera *style=vtkInteractorStyleJoystickCamera::New();
	// iren->SetInteractorStyle(style);
	// vtkInteractorStyleJoystickActor *style=vtkInteractorStyleJoystickActor::New();
	// iren->SetInteractorStyle(style);

	///////////////////////////////////////////////////////////////////////////
	vtkDICOMImageReader   *reader = vtkDICOMImageReader::New();
	reader->SetDataByteOrderToLittleEndian();
	reader->SetDirectoryName("D://1//3");
	vtkImageData *imageData = vtkImageData::New();
	imageData = reader->GetOutput();
	imageData->SetOrigin(.0, .0, .0);
	//imageData->SetDimensions(64,64,93);
	vtkImageShrink3D *shrink = vtkImageShrink3D::New();//二次采样
	shrink->SetInput((vtkDataObject *)reader->GetOutput());
	shrink->SetShrinkFactors(4, 4, 1);//采样因子
	vtkMarchingCubes *skinExtractor = vtkMarchingCubes::New();
	skinExtractor->SetInputConnection(shrink->GetOutputPort());
	skinExtractor->SetValue(0, 200);//计算体素的等值面 可以提取多个等值面 0~200表示第一个等值面的值为200
	vtkDecimatePro *deci = vtkDecimatePro::New();//减少mesh的三角面片
	deci->SetInputConnection(skinExtractor->GetOutputPort());
	deci->SetTargetReduction(0.3);//将原先的三角面片减少到原来的百分之七十
	// Specify the desired reduction in the total number of polygons (e.g., if TargetReduction is set to 0.9, this filter will try to reduce the data set to 10% of its original size). Because of various constraints, this level of reduction may not be realized. If you want to guarantee a particular reduction, you must turn off PreserveTopology, turn on SplitEdges and BoundaryVertexDeletion, and set the MaximumError to VTK_DOUBLE_MAX (these ivars are initialized this way when the object is instantiated).

	//adjust point positions using Laplacian smoothing
	vtkSmoothPolyDataFilter *smooth = vtkSmoothPolyDataFilter::New();
	smooth->SetInputConnection(deci->GetOutputPort());
	smooth->SetNumberOfIterations(100);//设置Laplace平滑的迭代次数
	vtkPolyDataNormals *skinNormals = vtkPolyDataNormals::New();//compute normals for polygonal mesh
	skinNormals->SetInputConnection(smooth->GetOutputPort());
	skinNormals->SetFeatureAngle(60.0);//Specify the angle that defines a sharp edge. If the difference in angle across neighboring polygons is greater than this value, the shared edge is considered "sharp".
	////////////////////////////////////////////////////////////////////////////////////////
	// vtkTriangleFilter *triangle_filter=vtkTriangleFilter::New();//create triangle polygons from input polygons and triangle strips
	// triangle_filter->SetInput( skinNormals->GetOutput());
	vtkFeatureEdges *edges_extractor = vtkFeatureEdges::New();//extract boundary, non-manifold, and/or sharp edges from polygonal data
	edges_extractor->SetInput(skinNormals->GetOutput());
	edges_extractor->ColoringOff();//Turn on/off the coloring of edges by type.
	edges_extractor->BoundaryEdgesOn();//Turn on/off the extraction of boundary edges
	edges_extractor->ManifoldEdgesOn();//Turn on/off the extraction of manifold edges.
	edges_extractor->NonManifoldEdgesOn();//Turn on/off the extraction of non-manifold edges.
	vtkStripper *stripper = vtkStripper::New();
	stripper->SetInput(skinNormals->GetOutput());
	/*
	vtkPolyDataWriter *wSP=vtkPolyDataWriter::New();
	wSP->SetInput(stripper->GetOutput());
	wSP->SetFileName("E://CT/aaaa.VTK");
	wSP->Write();
	wSP->Delete();
	*///保存为VTK格式
	vtkPolyDataMapper *skinMapper = vtkPolyDataMapper::New();
	skinMapper->SetInput(stripper->GetOutput());
	skinMapper->ScalarVisibilityOff();//Turn on/off flag to control whether the symbol's scalar data is used to color the symbol. If off, the color of the vtkLegendBoxActor is used.

	vtkActor *skin = vtkActor::New();
	skin->SetMapper(skinMapper);
	skin->GetProperty()->SetDiffuseColor(1, 0.49, 0.25);
	skin->GetProperty()->SetSpecular(.3);
	skin->GetProperty()->SetSpecularPower(20);
	vtkCamera *aCamera = vtkCamera::New();
	aCamera->SetViewUp(0, 0, -1);
	aCamera->SetPosition(0, 1, 0);//Set/Get the position of the camera in world coordinates. The default position is (0,0,1).
	aCamera->SetFocalPoint(0, 0, 0);
	aCamera->ComputeViewPlaneNormal();
	aRenderer->AddActor(skin);
	aRenderer->SetActiveCamera(aCamera);
	aRenderer->ResetCamera();//Automatically set up the camera based on the visible actors. The camera will reposition itself to view the center point of the actors, and move along its initial view plane normal (i.e., vector defined from camera position to focal point) so that all of the actors can be seen.
	aCamera->Dolly(1.5);// Move the position of the camera along the direction of projection. Moving towards the focal point (e.g., greater than 1) is a dolly-in, moving away from the focal point (e.g., less than 1) is a dolly-out.
	aRenderer->SetBackground(1, 1, 1);
	renWin->SetSize(640, 480);
	aRenderer->ResetCameraClippingRange(); // Reset the camera clipping range based on the bounds of the visible actors. This ensures that no props are cut off

	iren->Initialize();
	iren->Start();
	shrink->Delete();
	smooth->Delete();
	deci->Delete();
	stripper->Delete();
	skinExtractor->Delete();
	skinNormals->Delete();
	skinMapper->Delete();
	skin->Delete();
	aCamera->Delete();
	iren->Delete();
	renWin->Delete();
	aRenderer->Delete();
	reader->Delete();
	return 0;
}