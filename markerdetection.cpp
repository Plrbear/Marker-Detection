#include "markerdetection.h"

MarkerDetection::MarkerDetection()
{

}

MarkerDetection::~MarkerDetection()
{

}

bool MarkerDetection:: intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
{
    Point2f x = (o2 - o1);
    Point2f d1 =( p1 - o1);
    Point2f d2 = (p2 - o2);

    float cross =abs( d1.x*d2.y - d1.y*d2.x);
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 =(x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}
////////////////////////////////////////////////////////////////



bool MarkerDetection::DetectANDmatchMarker(vector<int> Dp, Mat img,int dotNumber ){

    Mat H;
    Mat im_out;
    im_out.create(400,400,CV_32FC1);
     cvNamedWindow("orginal_image", CV_WINDOW_NORMAL);
     imshow("orginal_image",img);
    ////
    cout<<"C :   "<<"    ";
    for(int i=0;i<9;i++)
    cout<<Dp[i]<<"    ";
    cout<<endl;
    int in=1;//for showing C index

    /////
    Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
    Mat drawingC = Mat::zeros( img.size(), CV_8UC3 );//for showing contour

    //////////////[1]calculate Pi and N////////////////
    vector<Point2f> Pi;

    Pi.push_back(Point2f(0, 0));
    Pi.push_back(Point2f(0, PatternSide));
    Pi.push_back(Point2f(PatternSide, PatternSide));
    Pi.push_back(Point2f(PatternSide, 0));

    //////////[2]&[3] Gaussian blur+thershold//////////


    Mat imgGrayScale;

    cv::cvtColor(img,imgGrayScale,CV_BGR2GRAY);
    cv::GaussianBlur(imgGrayScale,imgGrayScale,Size(3,3),0.6,0);
    cv::threshold(imgGrayScale,imgGrayScale,150,255,CV_THRESH_BINARY);
    //////////////////////////////////[5] find contours//////////////////////////////

    vector<vector<Point> > contours;
    vector<vector<Point> > polygons;
    vector<vector<Point> > E;

    vector<Point> approx;
    vector<Vec4i> hierarchy;

       imgGrayScale = imgGrayScale.clone();
    ///////////////////////find contours
      findContours( imgGrayScale.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    ////////////////////////////////////[6] check child///////////////////////////////////
    int min_area1=110;
    double minpolygonChildArea;

      for(int i = 0; i <int( contours.size()); i++)
      {
          /////////    [7]

          if ( hierarchy[i][2]==-1 )
         continue;
         ////////  [8]
          if(int(contourArea(contours[i],false))<min_area1)
          continue;


    /////////// [9] convert to polygon

          approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.01, true);

     ///////////////  [10]
          if (approx.size() !=5 || !isContourConvex(approx) )
         continue;

    ///////////////   [11]

    E.clear();

    ///////////////////////////  [12] & [13] find child of polygon
    /// find the first child index
    vector<Point> approx2;

    int j;
    for( j=0;j<int(contours.size());j++)
    {

        if(hierarchy[j][3]!=-1)
        approxPolyDP(Mat(contours[hierarchy[j][3]]), approx2, arcLength(Mat(contours[hierarchy[j][3]]), true) * 0.01, true);

        if(hierarchy[i][2]==j && approx2.size()==5  )
    break;

    }
    minpolygonChildArea=0;
    minpolygonChildArea=0.3*contourArea(contours[j],false);

    ////////////////find next child
        for(int i=j;i>0;i=hierarchy[i][0])
           {
            if(contourArea(contours[i])>minpolygonChildArea)

           E.push_back(contours[i]);}



    ////////////////////////////////// [14]
    if(int(E.size())!=dotNumber)
    continue;


    polygons.push_back(approx);
    //////////////////////////////////


    Point P1=approx[0];
    Point P2=approx[1];//o1;
    Point Pa=approx[2];//p1;
    Point Pb=approx[3];//p2
    Point P4=approx[4];//o2
    vector<float>S;//polygon side
    S.push_back( cv::norm(P1-P2));
    S.push_back(cv::norm(P2-Pa));
    S.push_back( cv::norm(Pa-Pb));
    S.push_back( cv::norm(Pb-P4));
    S.push_back( cv::norm(P4-P1));
    ///////////find minimum side
    float min=S[0]; int minimumSide;
    for(int i=0;i<int(S.size());i++)
    {
        if(S[i]<min)
      {  min=S[i]; minimumSide=i;}


    }
    ////////calculate line equation ////////////////////
    Point2f P3;

    switch (minimumSide){

    case 0:
        intersection( P4, P1, Pa, P2,  P3);

        break;
    case 1:
        intersection( P2, P1, Pa, Pb,  P3);

        break;
    case 2:
        intersection( P2, Pa, P4, Pb,  P3);

        break;
    case 3:
        intersection( Pa, Pb, P1, P4,  P3);

        break;
    case 4:
      intersection( P2, P1, P4, Pb,  P3);

        break;


    }


    vector<Point2f> Pp;

    switch(minimumSide){
    case 0:

    Pp.push_back(P4);//p3//1
    Pp.push_back(P3);//p2//2
    Pp.push_back(Pa);//p1//3
    Pp.push_back(Pb);//p0//4
        break;

    case 1:
        Pp.push_back(P1);//p3//1
        Pp.push_back(P3);//p2//2
        Pp.push_back(Pb);//p1//3
        Pp.push_back(P4);//p0//4
        break;
    case 2:
        Pp.push_back(P2);//p3//1
        Pp.push_back(P3);//p2//2
        Pp.push_back(P4);//p1//3
        Pp.push_back(P1);//p0//4


    break;
    case 3:
        Pp.push_back(Pa);//p3//1
        Pp.push_back(P3);//p2//2
        Pp.push_back(P1);//p1//3
        Pp.push_back(P2);//p0//4
        break;
    case 4:
        Pp.push_back(Pb);//p3//1
        Pp.push_back(P3);//p2//2
        Pp.push_back(P2);//p1//3
        Pp.push_back(Pa);//p0//4
        break;
    }

     H =findHomography(Pp,Pi);


    ////////////////////////// [17]


    Point2f M;
    vector<vector<Point2f> > E3(E.size());//new E (point2f)
    ////////////////convert point  >  point2f
     for(int i=0;i<(int)E.size();i++)
    for(int j=0;j<(int)E[i].size();j++)
     {

        M={};//tmp
    M=E[i][j];
    E3[i].push_back(M);
     }


    ///////////// transform dot by H
    vector<vector<Point2f> > finalE(E.size());
    for(int i=0;i<(int)E.size();i++){
    perspectiveTransform(E3[i], finalE[i], H);

    }


    ///////////// moment
     vector<Moments> mu(finalE.size() );
     for( int i = 0; i <(int) finalE.size(); i++ )
      { mu[i] = moments( finalE[i], false ); }
     ///  Get the mass centers:

     vector<Point2f> mc(finalE.size() );

     for( int i = 0; i <(int)finalE.size(); i++ )
      { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );


     }

    //cout<<mc;


    ///////find ORDER and MATCH
     vector<int>nDp(9);
     for(int i=0;i<9;i++)
     nDp[i]=0;

     /// in this section we find the order of dots by devide pattern square into 9 equal squer

     for(int i=0;i<9;i++){
    if(i<(int)finalE.size() && mc[i].x<PatternSide/3 && mc[i].y<PatternSide/3)//[0,0]
    {    nDp[0]=1; continue;}
    if(i<(int)finalE.size() && mc[i].x<2*(PatternSide/3 ) && mc[i].x>PatternSide/3&& mc[i].y<PatternSide/3)//[0,1]
    {    nDp[1]=1; continue;}
    if(i<(int)finalE.size() && mc[i].x<3*(PatternSide/3 ) && mc[i].x>2*(PatternSide/3) && mc[i].y<PatternSide/3)//[0,2]
    {    nDp[2]=1; continue;}

    if(i<(int)finalE.size() && mc[i].x<PatternSide/3  && mc[i].y>PatternSide/3 && mc[i].y<2*(PatternSide/3) )//[1,0]
    {    nDp[3]=1; continue;}

    if(i<(int)finalE.size() && mc[i].x>(PatternSide/3) && mc[i].x<2*(PatternSide/3) && mc[i].y>PatternSide/3 && mc[i].y<2*(PatternSide/3) )//[1,1]
    {    nDp[4]=1; continue;}
    if(i<(int)finalE.size() && mc[i].x>2*(PatternSide/3) && mc[i].x<3*(PatternSide/3) && mc[i].y>PatternSide/3 && mc[i].y<2*(PatternSide/3) )//[1,2]
    {    nDp[5]=1; continue;}

    if(i<(int)finalE.size() && mc[i].x<(PatternSide/3)  && mc[i].y>2*(PatternSide/3) && mc[i].y<3*(PatternSide/3) )//[2,0]
    {    nDp[6]=1; continue;}

    if(i<(int)finalE.size() && mc[i].x>(PatternSide/3)  &&mc[i].x<2*(PatternSide/3)&& mc[i].y>2*(PatternSide/3) && mc[i].y<3*(PatternSide/3) )//[2,1]
    {    nDp[7]=1; continue;}

    if(i<(int)finalE.size() && mc[i].x>2*(PatternSide/3)  &&mc[i].x<3*(PatternSide/3)&& mc[i].y>2*(PatternSide/3) && mc[i].y<3*(PatternSide/3) )//[2,2]
    {    nDp[8]=1; continue;}

     }
     cout<<"C"<<in<<":      ";

     for(int i=0;i<9;i++)
     cout<<nDp[i]<<"    ";
     in=2;
    if(Dp==nDp)
       { cout<<"so  C=C' "<<endl;

}

    else
        cout<<"so  C != C' "<<endl;



     for( int i = 0; i<int( polygons.size()); i++ )
        {
          drawContours( drawing, polygons, i, CV_RGB(0,0,255), 2, 8, hierarchy, 0, Point() );

        }


        for( int i = 0; i<(int)E.size(); i++ )
           {
             drawContours( drawing, E, i,CV_RGB(255,0,0), 2, 8, hierarchy, 0, Point() );

           }

        for( int i = 0; i<(int)contours.size(); i++ )
           {
             drawContours( drawingC, contours, i,CV_RGB(255,0,0), 2, 8, hierarchy, 0, Point() );

           }

    }

//Print results

warpPerspective(drawing, im_out, H, im_out.size());
cvNamedWindow("TRUE_marker_without_prespective", CV_WINDOW_NORMAL);
imshow( "TRUE_marker_without_prespective",im_out);

cvNamedWindow("markers", CV_WINDOW_NORMAL);
imshow( "markers",drawing);

cvNamedWindow("contours", CV_WINDOW_NORMAL);
imshow( "contours",drawingC);
waitKey();

return true;
    }


//////////////////////////////filtering functions//////////////////////////////////////////////////////////////////////////


void  MarkerDetection::mask1dx(int n,float s,cv::Mat &mask)
{

    mask.create(1,n,CV_32FC1);
    int  m=n/2;
    for(int i =-m;i<=m;i++)
            mask.at<float>(0,i+m)=1.0/sqrt((2*s*CV_PI))*exp(-(i*i)/(2*s*s));

}
/////////////////////////////////
void  MarkerDetection::mask1dy(int n,float s,cv::Mat &mask)
{

    mask.create(n,1,CV_32FC1);
    int  m=n/2;
    for(int i =-m;i<=m;i++)
            mask.at<float>(i+m,0)=1.0/sqrt((2*s*CV_PI))*exp(-(i*i)/(2*s*s));

}
/////////////////////////////////////


void MarkerDetection::conv1x(cv::Mat input,cv::Mat &output,cv::Mat mask )
{

    output=Mat::zeros(input.size(),CV_32FC1);


    int m=mask.cols;
    cv::Mat A;
    cv::copyMakeBorder(input,A,0,0,m/2,m/2,BORDER_CONSTANT,0);
    float* md=mask.ptr<float>(0);
    for(int i=0;i<input.rows;i++)
    {
        float* ind=A.ptr<float>(i);
        float* outd=output.ptr<float>(i);
        for(int j=0;j<input.cols;j++)
        for(int k=0;k<m;k++)
        outd[j]+=md[m-k-1]*ind[j+k];


    }

}
///////////////////////////////
    void MarkerDetection::conv1y(cv::Mat input,cv::Mat &output,cv::Mat mask )
    {

        output=Mat::zeros(input.size(),CV_32FC1);


        int m=mask.rows;
        cv::Mat A;
        cv::copyMakeBorder(input,A,m/2,m/2,0,0,BORDER_CONSTANT,0);


        for(int j=0;j<input.cols;j++)
        {

            for(int i=0;i<input.rows;i++){

                float* outd=output.ptr<float>(i);

            for(int k=0;k<m;k++)
            {
                float* ind=A.ptr<float>(i+k);
            float *md=mask.ptr<float>(m-k-1);
            outd[j]+=md[0]*ind[j];

            }
        }

    }
    }

///////////////////maskder//////////////////
    void MarkerDetection::mask1derx(int n,float s,cv::Mat &mask){


        mask.create(1,n,CV_32FC1);
        int  m=n/2;
        for(int i =-m;i<=m;i++)
                mask.at<float>(0,i+m)=(-i/(s*s))*(1.0/sqrt((2*s*CV_PI)))*exp(-(i*i)/(2*s*s));



    }
//////////////////////
   void MarkerDetection::mask1dery(int n,float s,cv::Mat &mask){

        mask.create(n,1,CV_32FC1);
        int  m=n/2;
        for(int i =-m;i<=m;i++)
                mask.at<float>(i+m,0)=(-i/(s*s)*(1.0/sqrt((2*s*CV_PI)))*exp(-(i*i)/(2*s*s)));

    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


