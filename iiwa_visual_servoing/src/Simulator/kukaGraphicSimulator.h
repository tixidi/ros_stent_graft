#ifndef KUKAGRAPHICSIMULATOR_H
#define KUKAGRAPHICSIMULATOR_H
#include "definitions.h"
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkStructuredPoints.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkInformation.h>
#include <vtkPlaneSource.h>
#include <vtkLightActor.h>
#include <vtkFrameBufferObject.h>
#include <vtkImageSinusoidSource.h>
#include <vtkImageData.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>
#include <vtkCamera.h>
#include <vtkCameraPass.h>
#include <vtkLightsPass.h>
#include <vtkSequencePass.h>
#include <vtkOpaquePass.h>
#include <vtkDepthPeelingPass.h>
#include <vtkTranslucentPass.h>
#include <vtkVolumetricPass.h>
#include <vtkOverlayPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkConeSource.h>
#include <vtkPlaneSource.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkInformation.h>
#include <vtkProperty.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <assert.h>
#include <vtkMath.h>
#include <vtkFrustumSource.h>
#include <vtkPlanes.h>
#include <vtkActorCollection.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkOpenGLRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkLightKit.h>
#include <vtkContourFilter.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkReverseSense.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkCellArray.h>
#include <vtkPlane.h>
#include <vtkPLYWriter.h>
#include <vtkPolyLine.h>
#include <vtkAxisActor.h>
#include <vtkCubeAxesActor.h>
#include <vtkSphereSource.h>
#include <vtkPolygon.h>
#include "newActor.h"
#include <vtkCylinderSource.h>
#include <vtkSTLReader.h>
class KukaGraphicSimulator : public QWidget
{
    Q_OBJECT
public :
    KukaGraphicSimulator();
private:
    QVTKWidget *vtkDispaly;
    vtkSmartPointer<vtkRenderWindow> renderWindow ;
    vtkSmartPointer<vtkRenderer> renderer;
    void PrepareVTKScene();
    NewActor *data[8];
    vtkActor *actor[8];
    vtkSmartPointer<vtkTransform> Transform[20];
    vtkSmartPointer<vtkTransform> TransformBase;
public slots:
    void  sliderSlot1(double a);
    void  sliderSlot2(double a);
    void  sliderSlot3(double a);
    void  sliderSlot4(double a);
    void  sliderSlot5(double a);
    void  sliderSlot6(double a);
    void  sliderSlot7(double a);

    //void  sliderSlot8(double a);
};

#endif
