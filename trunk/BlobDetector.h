/********************************************************************
created:    2013/06/19
created:    19:6:2013   9:18
filename: 	BlobDetector.h
author:		O. Menashe

purpose:	
*********************************************************************/
#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H
#include <opencv2/opencv.hpp>
#include <vector>
class BlobDetector
{
public:

   
    typedef unsigned short LabelType;
private:
    
    static LabelType privMaxLabel()    {return std::numeric_limits<LabelType>::max(); }
    template<class T>
    static int privCVtype() 
    {

        if(std::is_same<T,uchar>    ::value) return CV_8U;
        if(std::is_same<T,cv::Vec2b>::value) return CV_8UC2;
        if(std::is_same<T,cv::Vec3b>::value) return CV_8UC3;
        if(std::is_same<T,cv::Vec4b>::value) return CV_8UC4;

        if(std::is_same<T,char>     ::value) return CV_8S;


        if(std::is_same<T,ushort>   ::value) return CV_16U;
        if(std::is_same<T,cv::Vec2w>::value) return CV_16UC2;
        if(std::is_same<T,cv::Vec3w>::value) return CV_16UC3;
        if(std::is_same<T,cv::Vec4w>::value) return CV_16UC4;

        if(std::is_same<T,short>    ::value) return CV_16S;
        if(std::is_same<T,cv::Vec2s>::value) return CV_16SC2;
        if(std::is_same<T,cv::Vec3s>::value) return CV_16SC3;
        if(std::is_same<T,cv::Vec4s>::value) return CV_16SC4;

        if(std::is_same<T,int>      ::value) return CV_32S;
        if(std::is_same<T,cv::Vec2i>::value) return CV_32SC2;
        if(std::is_same<T,cv::Vec3i>::value) return CV_32SC3;
        if(std::is_same<T,cv::Vec4i>::value) return CV_32SC4;

        if(std::is_same<T,float>    ::value) return CV_32F;
        if(std::is_same<T,cv::Vec2f>::value) return CV_32FC2;
        if(std::is_same<T,cv::Vec3f>::value) return CV_32FC3;
        if(std::is_same<T,cv::Vec4f>::value) return CV_32FC4;

        if(std::is_same<T,double>   ::value) return CV_64F;
        if(std::is_same<T,cv::Vec2d>::value) return CV_64FC2;
        if(std::is_same<T,cv::Vec3d>::value) return CV_64FC3;
        if(std::is_same<T,cv::Vec4d>::value) return CV_64FC4;


        throw std::runtime_error("unknown Image type");


        //int typeSize = sizeof(T);
    }


    static void privRecPaintNeightbor(const cv::Mat& img, cv::Mat& imgOut,int y,int x,LabelType label)
    {
        if(y==-1 || x==-1 || x==img.cols || y==img.rows)
            return;
        

        if(img.at<uchar>(y,x)==0)//no data
                return;

        LabelType& outPix = imgOut.at<LabelType>(y,x);
        if(outPix!=0)//already labeled
                return;
        outPix = label;

        privRecPaintNeightbor(img,imgOut,y-1,x  ,label);//up
        privRecPaintNeightbor(img,imgOut,y  ,x+1,label);//left
        privRecPaintNeightbor(img,imgOut,y+1,x  ,label);//down
        privRecPaintNeightbor(img,imgOut,y  ,x-1,label);//down
        if(convectivity8())
        {
            privRecPaintNeightbor(img,imgOut,y-1,x-1,label);//up left
            privRecPaintNeightbor(img,imgOut,y-1,x+1,label);//up right
            privRecPaintNeightbor(img,imgOut,y+1,x-1,label);//down left
            privRecPaintNeightbor(img,imgOut,y+1,x+1,label);//down right
        }


    }

    static std::vector<cv::Point> privFindIndices(const cv::Mat& mask)
    {
        int nPix = cv::countNonZero(mask);
        std::vector<cv::Point> v(nPix);
        int cnt=0;
        for(int y=0;y!=mask.rows;++y)
        {
            for(int x=0;x!=mask.cols;++x)
            {
                if(mask.at<uchar>(y,x))
                    v[cnt++] = cv::Point(x,y);

            }
        }
        return v;
    }
 public:

// BlobDetector() {}                       //C-TOR
//~BlobDetector() {}                       //D-TOR
// BlobDetector(const BlobDetector&  obj) {}  //COPY-TOR
// BlobDetector(const BlobDetector&& obj) {}  //MOVE-TOR
// BlobDetector& operator=(const BlobDetector&& obj) {}  //ASSIGNMENT

     static bool convectivity8(int v=-1)
     {
         static bool val=false;
         if(v!=-1)
             val = v==1;
         return val;
     }


     struct BlobData
     {
         typedef std::vector<cv::Point> PtSet;
         PtSet blobPts;
         PtSet contour;
         cv::Moments moments;
         PtSet cvxHull;
         cv::RotatedRect minAreaRect;

     };
     static std::vector<BlobData> getBlobsData(const cv::Mat& labelImg,
         bool findBlobPts = true,
         bool findContour = false,
         bool findConvexHull = false,
         bool findMomments = false,
         bool findMinAreaRect = false
         )



     {
         if(findConvexHull || findMinAreaRect)
             findContour=true;

         double n;
         cv::minMaxLoc(labelImg,nullptr,&n);
         int nn = int(n);
         assert(n==nn);

         std::vector<BlobData> bdv(nn);
         if(nn==0)
             return bdv;
         for(int i=0;i!=nn;++i)
         {
             cv::Mat mask = labelImg == i+1;
             BlobData bd;
             if(findBlobPts)
                 bd.blobPts = privFindIndices(mask);
             if(findContour)
                 cv::findContours(mask,bd.contour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
             if(findMomments)
                 bd.moments = cv::moments(mask,true);
             if(findConvexHull)
                 cv::convexHull(bd.contour,bd.cvxHull);
             if(findMinAreaRect)
                 bd.minAreaRect = cv::minAreaRect(bd.contour);


             bdv[i]=bd;
         }
         return bdv;
     }

     static LabelType detect (const cv::Mat &img, cv::Mat& labelImg)
  {
      
      assert(!img.empty());
      assert(img.type()==CV_8U);
      assert(img.channels()==1);
      
      labelImg = cv::Mat(img.size(),privCVtype<LabelType>());
      labelImg = 0;
      unsigned int numPixels=cv::countNonZero(img);

      LabelType label=0;
      
      int h =  img.rows;
      int w = img.cols;

      for(int y=0;y!=h;++y)
      {
          for(int x=0;x!=w;++x)
          {
              const uchar& inpix = img.at<uchar>(y,x);
              
              if(inpix==0)
                  //no data
                  continue;
              LabelType& outPix = labelImg.at<LabelType>(y,x);

              if(outPix!=0)
                  //already labeled
                  continue;
              ++label;
              privRecPaintNeightbor(img,labelImg,y,x,label);
          }
          int nonZero = cv::countNonZero(labelImg);
          if(nonZero == numPixels)
              break;
      }
      return label;




    
   
  }

};



#endif
