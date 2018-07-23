#include "KukaGraphic.h"

//double aa=22;
//double bb=34;
// double aa = -790;
// double bb = 1240;
//double aa = 31.6;
//double bb = 49.6;
double aa = -43;
double bb = 1519;

//double bb=829.14/1000.;
//double aa=539.7/1000;
KukaGraphic::KukaGraphic()
{      
    assembleKukaGraphic();

    //myWi->resize(1500,700);
//    QGridLayout *layout =new QGridLayout();
//    layout->addWidget(this,     0,0, 10, 10 );
//    myWi->setLayout(layout);
//    myWi->show();
    vtkSmartPointer<vtkTransform> glo1=vtkSmartPointer<vtkTransform>::New();
    vtkSmartPointer<vtkTransform> glo2=vtkSmartPointer<vtkTransform>::New();
//    glo2->Translate( -aa*0.025*1000,bb*0.025*1000,0);
    glo2->Translate( aa,bb,0);
    //glo2->Translate(aa,bb,0);
    manipulator1=new ManipulatorModel(glo1,this,renderer);
    manipulator2=new ManipulatorModel(glo2,this,renderer);
}

void KukaGraphic::assembleKukaGraphic()
{
    this->resize(600,800);
    // Setup renderer
    renderer =vtkSmartPointer<vtkOpenGLRenderer>::New();
    renderer->SetBackground(0.0/255.0,0.0/255.0,100.0/255.0);

    this->GetRenderWindow()->AddRenderer( renderer);
    this->GetRenderWindow();
    this->show();
    //draw ground
    vtkSmartPointer<vtkPlaneSource> rectangleSource = vtkSmartPointer<vtkPlaneSource>::New();
    rectangleSource->SetOrigin(-3000.0,  3000.0,-0.0);
    rectangleSource->SetPoint1( 3000.0,  3000.0,-0.0);
    rectangleSource->SetPoint2(-3000.0, -3000.0,-0.0);
    rectangleSource->SetResolution(100,100);

    vtkSmartPointer<vtkPolyDataMapper> rectangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    rectangleMapper->SetInputConnection(rectangleSource->GetOutputPort());
    rectangleMapper->SetScalarVisibility(0);

    vtkSmartPointer<vtkActor> rectangleActor = vtkSmartPointer<vtkActor>::New();
    rectangleActor->SetMapper(rectangleMapper);
    rectangleActor->SetVisibility(1);
    rectangleActor->GetProperty()->SetColor(1.,1.,1.);
    rectangleActor->SetPosition(0.0,0.0,-0.0);
    renderer->AddViewProp(rectangleActor);

    //draw coordinates
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(300.0,300.0,300.0);
    axes->AxisLabelsOff ();
    renderer->AddActor(axes);

    //set view angle
    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(-2400.,  -1000., 700.);
    camera->SetFocalPoint(-500, 500., 400);
    camera->SetViewUp(0,0,1 );
    camera->SetClippingRange(100,6000);
    renderer->SetActiveCamera(camera);

    //add Mandrel
    mandrel =new MandrelActor(renderer);
}

void KukaGraphic::updateModelStates (kukaInformation info)
{
    //update Manipulator
    manipulator1->updateModelStates(info.jointAngles1,info.gloHomoMatrix1);
    manipulator2->updateModelStates(info.jointAngles2,info.gloHomoMatrix2);
    //update mandrel
    vtkSmartPointer<vtkMatrix4x4>  localMat=vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
    localMat->SetElement(i,j,info.gloHomoMatrix1(i,j));
    localMat->SetElement(0,3,info.gloHomoMatrix1(0,3)*1000);
    localMat->SetElement(1,3,info.gloHomoMatrix1(1,3)*1000);
    localMat->SetElement(2,3,info.gloHomoMatrix1(2,3)*1000);
    mandrel->setRobotTipPosture(localMat);
}



