#include "mainwindow.h"
#include <QApplication>
#include "markerdetection.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

  //  w.show();


    Mat  img =  imread("/home/amir/markert.jpg");

  //  waitKey();

    vector <int>Dp; //the marker code

    /*1  1  1
     *1  1  1
     *1  0  1
     */

        Dp.push_back(1);
        Dp.push_back(1);
        Dp.push_back(1);

        Dp.push_back(1);
        Dp.push_back(1);
        Dp.push_back(1);

        Dp.push_back(1);
        Dp.push_back(0);
        Dp.push_back(1);
MarkerDetection a1;
a1.DetectANDmatchMarker(Dp,img,8);
    return 0;
}
