#ifndef MANIPULATORMODEL_H
#define MANIPULATORMODEL_H

#include "KUKAControl/definitions.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <assert.h>
#include <vtkMath.h>
#include <vtkPlanes.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkOpenGLRenderer.h>
#include <vtkPlane.h>
#include <vtkAxisActor.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <QTimer>
#include <QThread>
#include <QSlider>
#include <QGridLayout>
#include <QTextEdit>
#include <vtkVertexGlyphFilter.h>

class ManipulatorModel
{
public:
    ManipulatorModel(vtkTransform *gloCOr,QVTKWidget *vtk,vtkRenderer * render );
    void updateModelStates(Vector7d &joints, Matrix4d &mat);
//	vtkActor *getActor();
//	void RotateX(double a);
//	void RotateY(double a);
//	void RotateZ(double a);
//	void SetPosition(double a, double b, double c);

private:
    QVTKWidget *VTKscene;
    vtkRenderer * currRender;
    vtkSmartPointer<vtkSTLReader> reader[8];
    vtkSmartPointer<vtkPolyDataMapper> mapper[8];
    vtkSmartPointer<vtkActor> actor[8];
    vtkSmartPointer<vtkTransform> Transform[9];
    vtkSmartPointer<vtkTransform> TransformBase;
    vtkSmartPointer<vtkAxesActor> localAxes;

    vtkSmartPointer<vtkPoints> Points;
    vtkSmartPointer<vtkActor> lineActor;
    vtkSmartPointer<vtkPolyDataMapper> pointMapper;
    vtkSmartPointer<vtkPolyData> polydata ;
    vtkSmartPointer<vtkPolyData> polyDataPoints;
};

#endif // NEWACTOR_H

