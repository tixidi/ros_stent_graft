#ifndef KUKAGRAPHIC_H
#define KUKAGRAPHIC_H

#include "KUKAControl/definitions.h"
#include "Simulator/manipulatorModel.h"
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
#include <vtkSphereSource.h>
#include "MandrelActor.h"


class KukaGraphic :public QVTKWidget
{
    Q_OBJECT
public:
    KukaGraphic();
private:
    ManipulatorModel *manipulator1;
    ManipulatorModel *manipulator2;

    MandrelActor *mandrel;
//    vtkSmartPointer<vtkAxesActor> mandrelAxis[8];
//    vtkSmartPointer<vtkMatrix4x4>  MarkCoor[8];

    //vtkSmartPointer<vtkActor> toolActor;
    void assembleKukaGraphic();
    vtkSmartPointer<vtkOpenGLRenderer> renderer;

public slots:
    void updateModelStates(kukaInformation a);
};

#endif
