
#include <opencv2/opencv.hpp>
#include <opencvExt/TPS.h>
struct ImgSrcDst
{
    cv::Mat img;
    std::vector<cv::Point2d> src;
    std::vector<cv::Point2d> dst;
    

};
void update(const cv::Mat& img,const std::vector<cv::Point2d>& src,const std::vector<cv::Point2d>& dst,bool coarse = false)
{
    // create thin plate spline object and put the vectors into the constructor
    TPS2d tps(src,dst);

    // warp the image to dst
    cv::Mat imgT = tps.warpImage(img,coarse? cv::INTER_NEAREST:cv::INTER_LINEAR);
    

    cv::Mat imgwmarks;
    img.copyTo(imgwmarks);
    for(int i=0;i!=src.size();++i)
    {
        cv::circle(imgwmarks,src[i],1,cv::Scalar(0,255,0),10);
        cv::circle(imgwmarks,dst[i],1,cv::Scalar(255,0,0),10);

    }
    cv::imshow("input", imgwmarks);

    //cv::setMouseCallback( "input", mouseHandler, &imgwmarks );
    cv::imshow("output",imgT);
}
void mouseHandler(int event, int x, int y, int flags, void *param)
{
    ImgSrcDst& isd = *((ImgSrcDst*)param);
    switch(event) {
        /* left button down */
    case CV_EVENT_LBUTTONDOWN:
        isd.src.push_back(cv::Point2d(double(x),double(y)));
        
        fprintf(stdout, "Left button down (%d, %d).\n", x, y);

        break;

        /* right button down */
    case CV_EVENT_LBUTTONUP:
        isd.dst.push_back(cv::Point2d(double(x),double(y)));
        update(isd.img,isd.src,isd.dst);
        break;

        /* mouse move */
    case CV_EVENT_MOUSEMOVE:
        std::vector<cv::Point2d> dstT(isd.dst);
        if(isd.src.size()!=dstT.size())
        {
            dstT.push_back(cv::Point2d(double(x),double(y)));
            //update(isd.img,isd.src,dstT,true);
        }
        
        break;
    }
    
}

void testTPS()
{
    std::string fn = "lena_color.bmp";
    
    ImgSrcDst isd;
    isd.img = cv::imread(fn);
    
    int h = isd.img.rows;
    int w = isd.img.cols;
    isd.src.push_back(cv::Point(0,0));
    isd.dst.push_back(cv::Point(0,0));

    isd.src.push_back(cv::Point(w,0));
    isd.dst.push_back(cv::Point(w,0));

    isd.src.push_back(cv::Point(0,h));
    isd.dst.push_back(cv::Point(0,h));

    isd.src.push_back(cv::Point(w,h));
    isd.dst.push_back(cv::Point(w,h));
    cv::imshow("input", isd.img);

    cv::setMouseCallback( "input", mouseHandler, &isd );

    cv::waitKey();
    cv::destroyAllWindows();
}