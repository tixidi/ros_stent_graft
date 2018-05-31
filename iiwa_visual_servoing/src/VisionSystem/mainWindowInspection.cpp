#include "mainWindowInspection.h"

mainWindowInspection::mainWindowInspection()
{
    //this->setFixedSize(1600,400);
//    trackingApp=new VisualTrackingThread();
//    trackingApp->start(QThread::LowestPriority);
//    qRegisterMetaType<stereoImages>("stereoImages");
//    QObject::connect(trackingApp, SIGNAL(imagesReady(stereoImages )), this, SLOT(disPlayImages( stereoImages)));

    myLabelLeft= new QLabel();
    myLabelRight=new QLabel();

    cmd_yes= new QPushButton("YES");
    cmd_no= new QPushButton("NO");
    connect (cmd_yes, SIGNAL(clicked()),SIGNAL(yes_clicked()) );
    connect (cmd_no,  SIGNAL(clicked()),SIGNAL(no_clicked()));

    QGridLayout *mainLayout =new QGridLayout(this);
    mainLayout->addWidget(myLabelLeft, 0,0,10,10);
    mainLayout->addWidget(myLabelRight,0,10,10,10);
    mainLayout->addWidget(cmd_yes,10,0,1,5);
    mainLayout->addWidget(cmd_no,10,10,1,5);
//    cmd_no->hide();
//    cmd_yes->hide();
    this->setLayout(mainLayout);
}

void mainWindowInspection::disPlayImages(stereoImages Images)
{
    QImage ImageLeft= Mat2QImage( Images.frameLeft );
    QImage ImageRight= Mat2QImage(Images.frameRight );
    myLabelLeft->setPixmap(QPixmap::fromImage(ImageLeft));
    myLabelRight->setPixmap(QPixmap::fromImage(ImageRight));
//    cmd_no->show();
//    cmd_yes->show();

    this->update();
    this->show();
}

QImage mainWindowInspection::Mat2QImage(const cv::Mat_<double> &src)
{
        double scale = 255.0;
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const double *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        unsigned int color = srcrow[x] * scale;
                        destrow[x] = qRgba(color, color, color, 255);
                }
        }
        return dest;
}

QImage mainWindowInspection::Mat2QImage(const cv::Mat3b &src)
{
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const cv::Vec3b *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
                }
        }
        return dest;
}
