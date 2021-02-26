
#include "CloudPointDisplayQtWidget.h"
#include <QtWidgets/QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);


#include <iostream>
#include <thread>

std::thread *workthread;
QVTKOpenGLNativeWidget *widget;

void runThread()
{

	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "loop once" << std::endl;
	}

}


int main(int argc, char* argv[])
{
	QApplication a(argc, argv);

	QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
	QVTKOpenGLNativeWidget *widget = new QVTKOpenGLNativeWidget();

	vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();

	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();

	sphereActor->SetMapper(sphereMapper);
	sphereActor->GetProperty()->SetColor(colors->GetColor4d("Tomato").GetData());

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(sphereActor);
	renderer->SetBackground(colors->GetColor3d("SteelBlue").GetData());

	vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetWindowName("RenderWindowNoUIFile");

	widget->SetRenderWindow(renderWindow);
	widget->resize(800, 600);
	widget->show();



	workthread = new std::thread(runThread);

	return a.exec();
}

