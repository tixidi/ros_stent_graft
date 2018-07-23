#include"kukaGraphicSimulator.h"

KukaGraphicSimulator::KukaGraphicSimulator()
{
    PrepareVTKScene();

    TransformBase = vtkSmartPointer<vtkTransform>::New();
    TransformBase->Identity();
    TransformBase->Translate(0,0,0.);

    data[0]=new NewActor("../source/kukaModel/1.STL");
    actor[0]=vtkSmartPointer<vtkActor>::New();
    actor[0]=data[0]->getActor();
    Transform[0] = vtkSmartPointer<vtkTransform>::New();
    Transform[0]->SetInput(TransformBase);
    actor[0]->SetUserTransform(Transform[0]);

    data[1]=new NewActor("../source/kukaModel/2.STL");
    actor[1]=vtkSmartPointer<vtkActor>::New();
    actor[1]=data[1]->getActor();
    Transform[1] = vtkSmartPointer<vtkTransform>::New();
    Transform[1]->SetInput(Transform[0]);
    actor[1]->SetUserTransform(Transform[1]);


    data[2]=new NewActor("../source/kukaModel/3.STL");
    actor[2]=vtkSmartPointer<vtkActor>::New();
    actor[2]=data[2]->getActor();
    Transform[2] = vtkSmartPointer<vtkTransform>::New();
    Transform[2]->SetInput(Transform[1]);
    actor[2]->SetUserTransform(Transform[2]);


    data[3]=new NewActor("../source/kukaModel/4.STL");
    actor[3]=vtkSmartPointer<vtkActor>::New();
    actor[3]=data[3]->getActor();
    Transform[3] = vtkSmartPointer<vtkTransform>::New();
    Transform[3]->SetInput(Transform[2]);
    actor[3]->SetUserTransform(Transform[3]);

    data[4]=new NewActor("../source/kukaModel/5.STL");
    actor[4]=vtkSmartPointer<vtkActor>::New();
    actor[4]=data[4]->getActor();
    Transform[4] = vtkSmartPointer<vtkTransform>::New();
    Transform[4]->SetInput(Transform[3]);
    actor[4]->SetUserTransform(Transform[4]);

    data[5]=new NewActor("../source/kukaModel/6.STL");
    actor[5]=vtkSmartPointer<vtkActor>::New();
    actor[5]=data[5]->getActor();
    Transform[5] = vtkSmartPointer<vtkTransform>::New();
    Transform[5]->SetInput(Transform[4]);
    actor[5]->SetUserTransform(Transform[5]);

    data[6]=new NewActor("../source/kukaModel/7.STL");
    actor[6]=vtkSmartPointer<vtkActor>::New();
    actor[6]=data[6]->getActor();
    Transform[6] = vtkSmartPointer<vtkTransform>::New();
    Transform[6]->SetInput(Transform[5]);
    actor[6]->SetUserTransform(Transform[6]);


    data[7]=new NewActor("../source/kukaModel/8.STL");
    actor[7]=vtkSmartPointer<vtkActor>::New();
    actor[7]=data[7]->getActor();
    Transform[7] = vtkSmartPointer<vtkTransform>::New();
    Transform[7]->SetInput(Transform[6]);
    actor[7]->SetUserTransform(Transform[7]);

    renderer->AddActor(actor[0]);
    renderer->AddActor(actor[1]);
    renderer->AddActor(actor[2]);
    renderer->AddActor(actor[3]);
    renderer->AddActor(actor[4]);
    renderer->AddActor(actor[5]);
    renderer->AddActor(actor[6]);
    renderer->AddActor(actor[7]);
    this->update();
}

void KukaGraphicSimulator::PrepareVTKScene()
{   
    vtkDispaly= new QVTKWidget(this);
    vtkDispaly->resize(800,460);

    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetInteractor(vtkDispaly->GetInteractor());

    vtkDispaly->SetRenderWindow(renderWindow);
    renderer->SetBackground(135.0/255.0,206.0/255.0,250.0/255.0);

//    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    renderWindowInteractor->SetRenderWindow(renderWindow);

//    renderWindow->Render();
//    renderWindowInteractor->Start();


    //renderWindow->SetInteractor(widget.GetInteractor());

//    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    renderWindowInteractor->SetRenderWindow(renderWindow);

//    //renderWindow->Render();
//    renderWindowInteractor->Start();

    ////////draw ground//////////////////////////////////////////
    vtkSmartPointer<vtkPlaneSource> rectangleSource = vtkSmartPointer<vtkPlaneSource>::New();
    rectangleSource->SetOrigin(-3000.0,3000.0,-0.0);

    rectangleSource->SetPoint1( 3000.0,  3000.0,-0.0);
    rectangleSource->SetPoint2(-3000.0, -3000.0,-0.0);
    rectangleSource->SetResolution(100,100);

    vtkSmartPointer<vtkPolyDataMapper> rectangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    rectangleMapper->SetInputConnection(rectangleSource->GetOutputPort());
    rectangleMapper->SetScalarVisibility(0);

    //texture ground
    //    vtkSmartPointer<vtkJPEGReader> jPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
    //    jPEGReader->SetFileName ( "..\\Source\\knife\\board.jpg" );
    //    vtkSmartPointer<vtkTexture> texture =vtkSmartPointer<vtkTexture>::New();
    //    texture->SetInputConnection(jPEGReader->GetOutputPort());

    vtkSmartPointer<vtkActor> rectangleActor = vtkSmartPointer<vtkActor>::New();
    //rectangleActor->SetTexture(texture);
    rectangleActor->SetMapper(rectangleMapper);
    rectangleActor->SetVisibility(1);
    rectangleActor->GetProperty()->SetColor(0.0,0.0,0.0);
    rectangleActor->SetPosition(0.0,0.0,-4.0);
    //renderer->AddViewProp(rectangleActor);

    ///draw coordinates
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(1000.0,1000.0,1000.0);
    axes->AxisLabelsOn ();
    renderer->AddActor(axes);

    ///set view angle
    vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(0, -2000., 1000.);
    camera->SetFocalPoint(0, 0., 0.);
    renderer->SetActiveCamera(camera);
    this->update();
}

void KukaGraphicSimulator::sliderSlot1(double a)
{
    Transform[1]->Identity();
    Transform[1]->RotateZ(a);
    this->update();
}

void KukaGraphicSimulator::sliderSlot2(double a)
{
    Transform[2]->Identity();
    Transform[2]->Translate(0.0,0.0,+360);
    Transform[2]->RotateY(a);
    Transform[2]->Translate(0.0,0.0,-360);
    this->update();
}

void KukaGraphicSimulator::sliderSlot3(double a)
{
    Transform[3]->Identity();
    Transform[3]->RotateZ(a);
    this->update();
}

void KukaGraphicSimulator::sliderSlot4(double a)
{
    Transform[4]->Identity();
    Transform[4]->Translate(0.0,0.0,420+360);
    Transform[4]->RotateY(-a);
    Transform[4]->Translate(0.0,0.0,-420-360);
    this->update();
}

void KukaGraphicSimulator::sliderSlot5(double a)
{
    Transform[5]->Identity();
    Transform[5]->RotateZ(a);
    this->update();
}

void KukaGraphicSimulator::sliderSlot6(double a)
{
    Transform[6]->Identity();
    Transform[6]->Translate(0.0,0.0,420+360+400);
    Transform[6]->RotateY(a);
    Transform[6]->Translate(0.0,0.0,-420-360-400);
    this->update();
}

void KukaGraphicSimulator::sliderSlot7(double a)
{
    Transform[7]->Identity();
    Transform[7]->RotateZ(a);
    this->update();
}
