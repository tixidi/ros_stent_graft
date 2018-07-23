#include "manipulatorModel.h"

ManipulatorModel::ManipulatorModel(vtkTransform *gloCOr, QVTKWidget *vtk,vtkRenderer * render )
{
    VTKscene=vtk;
    currRender=render;
    //set actors and add into renderer

    //read STL files
    for (int i=0;i<8;i++)
    {
        reader[i] =vtkSmartPointer<vtkSTLReader>::New();
        char buffer[30];
        sprintf(buffer,"../source/kukaModel/kuka4/%d.STL",i+1);
        reader[i]->SetFileName(buffer);
        mapper[i] =vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper[i]->SetInputConnection(reader[i]->GetOutputPort()  );
        actor[i]=vtkSmartPointer<vtkActor>::New();
        //actor[i]->GetProperty()->FrontfaceCullingOn();
        actor[i]->SetMapper(mapper[i]);
    }
//    actor[0]->GetProperty()->SetColor(255./255., 127./255, 0.);
//    actor[2]->GetProperty()->SetColor(255./255., 127./255, 0.);
//    actor[4]->GetProperty()->SetColor(255./255., 127./255, 0.);
    //actor[6]->GetProperty()->SetColor(255./255., 127./255, 0.);
     actor[6]->GetProperty()->SetOpacity(0.2);
     actor[7]->GetProperty()->SetOpacity(0.2);
     actor[7]->GetProperty()->SetColor(255./255., 127./255, 0.);
    //actor[5]->GetProperty()->SetOpacity(0.5);
    TransformBase = vtkSmartPointer<vtkTransform>::New();
    TransformBase->Identity();
    TransformBase=gloCOr;

    //connect transformation
    Transform[0] = vtkSmartPointer<vtkTransform>::New();
    Transform[0]->SetInput(TransformBase);
    actor[0]->SetUserTransform(Transform[0]);

    Transform[1] = vtkSmartPointer<vtkTransform>::New();
    Transform[1]->SetInput(Transform[0]);
    actor[1]->SetUserTransform(Transform[1]);

    Transform[2] = vtkSmartPointer<vtkTransform>::New();
    Transform[2]->SetInput(Transform[1]);
    actor[2]->SetUserTransform(Transform[2]);

    Transform[3] = vtkSmartPointer<vtkTransform>::New();
    Transform[3]->SetInput(Transform[2]);
    actor[3]->SetUserTransform(Transform[3]);

    Transform[4] = vtkSmartPointer<vtkTransform>::New();
    Transform[4]->SetInput(Transform[3]);
    actor[4]->SetUserTransform(Transform[4]);

    Transform[5] = vtkSmartPointer<vtkTransform>::New();
    Transform[5]->SetInput(Transform[4]);
    actor[5]->SetUserTransform(Transform[5]);

    Transform[6] = vtkSmartPointer<vtkTransform>::New();
    Transform[6]->SetInput(Transform[5]);
    actor[6]->SetUserTransform(Transform[6]);

    Transform[7] = vtkSmartPointer<vtkTransform>::New();
    Transform[7]->SetInput(Transform[6]);
    actor[7]->SetUserTransform(Transform[7]);

//    Transform[8] = vtkSmartPointer<vtkTransform>::New();
//    Transform[8]->SetInput(Transform[7]);

    //set local axis
    localAxes = vtkSmartPointer<vtkAxesActor>::New();
    localAxes->SetTotalLength(130,130,130);
    localAxes->AxisLabelsOff ();
    currRender->AddActor(localAxes);

    for (int i=0;i<8;i++)
    {
      currRender->AddActor(actor[i]);
    }

    Points =vtkSmartPointer<vtkPoints>::New();
    lineActor= vtkSmartPointer<vtkActor>::New();
    pointMapper =vtkSmartPointer<vtkPolyDataMapper>::New();
    polydata=vtkSmartPointer<vtkPolyData>::New();
    polyDataPoints =  vtkSmartPointer<vtkPolyData>::New();
}

void ManipulatorModel::updateModelStates(Vector7d &joints, Matrix4d &mat)
{
    double conf[7]= {310.5,   0,    400,   0,    390,   0,      1178.5-1100.5 +10};
    double confN[7]={360.0,   0,    420,   0,    400,   0,      0};
    Transform[1]->Identity();
    Transform[1]->RotateZ(joints(0)*180/3.1415926 );

    Transform[2]->Identity();
    Transform[2]->Translate(0.0,0.0,+conf[0]);
    Transform[2]->RotateY(-joints(1)*180/3.1415926 );
    Transform[2]->Translate(0.0,0.0,-conf[0]);

    Transform[3]->Identity();
    Transform[3]->RotateZ(joints(2)*180/3.1415926 );

    Transform[4]->Identity();
    Transform[4]->Translate(0.0,0.0, conf[0]+conf[2]);
    Transform[4]->RotateY(joints(3)*180/3.1415926 );
    Transform[4]->Translate(0.0,0.0,-conf[0]-conf[2]);

    Transform[5]->Identity();
    Transform[5]->RotateZ(joints(4)*180/3.1415926 );

    Transform[6]->Identity();
    Transform[6]->Translate(0.0,0.0, conf[0]+conf[2]+conf[4]);
    Transform[6]->RotateY(-joints(5)*180/3.1415926 );
    Transform[6]->Translate(0.0,0.0,-conf[0]-conf[2]-conf[4]);

    Transform[7]->Identity();
    Transform[7]->RotateZ(joints(6)*180/3.1415926 );

    //update local axis
    vtkSmartPointer<vtkMatrix4x4>  localMat=vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
    localMat->SetElement(i,j,mat(i,j));
    localMat->SetElement(0,3,mat(0,3)*1000);
    localMat->SetElement(1,3,mat(1,3)*1000);
    localMat->SetElement(2,3,mat(2,3)*1000);
    localAxes->SetUserMatrix(localMat);

    Points->InsertNextPoint(mat(0,3)*1000.,mat(1,3)*1000.,mat(2,3)*1000. );
    polyDataPoints->SetPoints(Points);

    vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter =vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexGlyphFilter->SetInputConnection(polyDataPoints->GetProducerPort());
    vertexGlyphFilter->Update();
    polydata->ShallowCopy(vertexGlyphFilter->GetOutput());

    pointMapper->SetInputConnection(polydata->GetProducerPort());
    lineActor->SetMapper(pointMapper);
    lineActor->GetProperty()->SetPointSize(3);
    lineActor->GetProperty()->SetColor(1,0,1);
    currRender->AddActor(lineActor);
    VTKscene->update();
}
