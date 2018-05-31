#include "MandrelActor.h"

MandrelActor::MandrelActor(vtkSmartPointer<vtkOpenGLRenderer> renderer)
{ 
    vtkSmartPointer<vtkSTLReader> mandrelReader =vtkSmartPointer<vtkSTLReader>::New();
    mandrelReader->SetFileName("../source/kukaModel/mandrel.STL");
    vtkSmartPointer<vtkPolyDataMapper> mandrelMapper =vtkSmartPointer<vtkPolyDataMapper>::New();
    mandrelMapper->SetInputConnection(mandrelReader->GetOutputPort()  );

    mandrelActor=vtkSmartPointer<vtkActor>::New();
    mandrelActor->SetMapper(mandrelMapper);
    mandrelActor->SetScale(1,1,1);
    //mandrelActor->GetProperty()->SetOpacity(0.1);
    //renderer->AddActor(mandrelActor);

    for (int i=0;i<8;i++)
    {
        mandrelAxis[i]= vtkSmartPointer<vtkAxesActor>::New();
        mandrelAxis[i]->SetTotalLength(18.0,18.0,20.0);
        mandrelAxis[i]->AxisLabelsOff ();
        renderer->AddActor(mandrelAxis[i]);
    }
}
void MandrelActor::setRobotTipPosture(vtkSmartPointer<vtkMatrix4x4> mat  )
{
    mandrelActor->SetUserMatrix( mat);
    Matrix4d tipPosture=VTK2EigenMatrix( mat);
    Matrix4d markertrans=Matrix4d::Identity();
    markertrans(0,3)=24; markertrans(1,3)=0 ;markertrans(2,3)=14;

    for (int i=0;i<8;i++)
    {
        Matrix4d Rotate=Matrix4d::Identity();
        Rotate(0,0)=cos(2*3.1415926*i/8);  Rotate(0,1)= -sin(2*3.1415926*i/8);
        Rotate(1,0)=sin(2*3.1415926*i/8);  Rotate(1,1)=  cos(2*3.1415926*i/8);

//        for (int n=0;n<2;n++)
//            for (int m=0;m<2;m++)
//            {
//                markertrans(n,m)=Rotate(n,m);
//            }
        Matrix4d markerPosture=tipPosture*Rotate*markertrans;
        vtkSmartPointer<vtkMatrix4x4> markerPostureVTK=eigen2VTKMatrix(markerPosture);
        mandrelAxis[i]->SetUserMatrix( markerPostureVTK);
    }
}

Matrix4d MandrelActor::VTK2EigenMatrix(vtkSmartPointer<vtkMatrix4x4> mat )
{
     Matrix4d eigenMat;
     for (int i=0;i<4;i++)
         for (int j=0;j<4;j++)
     eigenMat(i,j)=mat->GetElement(i,j);
     return eigenMat;
}

vtkSmartPointer<vtkMatrix4x4> MandrelActor::eigen2VTKMatrix(Matrix4d& mat )
{
     vtkSmartPointer<vtkMatrix4x4> VTKMat=  vtkSmartPointer<vtkMatrix4x4> ::New();
     for (int i=0;i<4;i++)
         for (int j=0;j<4;j++)
     VTKMat->SetElement(i,j,  mat(i,j) );
     return VTKMat;
}

