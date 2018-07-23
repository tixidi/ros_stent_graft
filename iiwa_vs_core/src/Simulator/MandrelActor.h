#ifndef MANDRELACTOR_H
#define MANDRELACTOR_H

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkProperty.h>
#include <vtkMatrix4x4.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkPolyDataMapper.h>
#include "KUKAControl/definitions.h"

class MandrelActor
{
public:
    MandrelActor(vtkSmartPointer<vtkOpenGLRenderer> renderer);
    void setRobotTipPosture(vtkSmartPointer<vtkMatrix4x4> mat );
private:
    vtkSmartPointer<vtkActor>     mandrelActor;
    vtkSmartPointer<vtkAxesActor> mandrelAxis[8];
    Matrix4d markerTrans[8];

    Matrix4d VTK2EigenMatrix(vtkSmartPointer<vtkMatrix4x4> mat );
    vtkSmartPointer<vtkMatrix4x4> eigen2VTKMatrix(Matrix4d& mat );
};

#endif // NEWACTOR_H

