#include <opencv2/opencv.hpp>
#include <opencvExt/BlobDetector.h>




void testBlobDetector()
{
    std::string fn = "bw.bmp";
    cv::Mat img = cv::imread(fn,CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat lblImg;
    
    int nBlobs = BlobDetector::detect(img,lblImg);

    std::vector<BlobDetector::BlobData> blbData = BlobDetector::getBlobsData(lblImg,true,false,true);



}