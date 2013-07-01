/********************************************************************
created:    2013/07/01
created:    1:7:2013   14:12
filename: 	TPS.h
author:		O. Menashe

purpose:	
*********************************************************************/
#ifndef TPS_H
#define TPS_H
#include <opencv2/opencv.hpp>
#include <vector>
class TPS
{
    cv::Mat m_src;
    cv::Mat m_mat;

    static void privUfunc(const cv::Mat& v,cv::Mat& d)
    {
        cv::Mat r2;
        cv::reduce(v.mul(v),r2,1,CV_REDUCE_SUM);
        r2 = cv::max(r2,1e-320);
        
        int dim = v.cols;
        
        switch(dim)
        {
            case 1 :
                {
                    cv::Mat r;
                    cv::sqrt(r2,r);
                    d = r2.mul(r);
                }   break;
            case 2 :
                {
                    cv::Mat logr;
                    cv::log(r2,logr);
                    d = r2.mul(logr);
                }   break;
            case 3:
                {
                    cv::sqrt(r2,d);
                }break;
            default:
                {
                    cv::pow(r2,1-dim/2.0,d);
                    break;
                }
        }
        
        
    }

    enum {N_MAX_EVAL_POINTS = 100000};

    void privAt(const cv::Mat& pts,cv::Mat& ptsT) const
    {
        int dim = m_src.cols ;
        
        int npts = pts.rows;
        cv::Mat k_mat(m_src.rows,npts,CV_64F);
        for(int i=0;i!=npts;++i)
            privUfunc(cv::repeat(pts.row(i),m_src.rows,1)-m_src,k_mat.col(i));

        cv::Mat l_mat(npts,m_src.rows+dim+1,CV_64F);
        l_mat.colRange(0,m_src.rows) = k_mat.t();
        l_mat.col(m_src.rows) = 1;
        pts.copyTo(l_mat.colRange(m_src.rows+1,m_src.rows+dim+1));

        ptsT = l_mat * m_mat;
    }

 public:

//~TPS() {}                       //D-TOR
// TPS(const TPS&  obj) {}  //COPY-TOR
// TPS(const TPS&& obj) {}  //MOVE-TOR
// TPS& operator=(const TPS&& obj) {}  //ASSIGNMENT
// 

      TPS() {}                       //C-TOR
      TPS(const cv::Mat& src,const cv::Mat& dst){set(src,dst);}
      void set(const cv::Mat& src,const cv::Mat& dst)//input NxP points where N is the number of points and P is the poins dim
      {
          assert(src.size() == dst.size());
          int npts = src.rows;
          int dim   = src.cols;

          int ld = npts+dim+1;

          cv::Mat l_mat(ld,ld,CV_64F);

          cv::Mat k_mat(l_mat,cv::Rect(0,0,npts,npts));
          for(int i=0;i!=npts;++i)
          {
              cv::Mat rowr = cv::repeat(src.row(i),npts,1);
              privUfunc(src-rowr,k_mat.col(i));

          }
          cv::completeSymm(k_mat,true);
          cv::Mat s_mat(l_mat,cv::Rect(npts,0,dim+1,npts));
          s_mat.col(0)=1;
          src.copyTo(s_mat.colRange(1,dim+1));

          l_mat.colRange(0   ,npts).rowRange(npts,ld  ) = s_mat.t();
          l_mat.colRange(npts,ld  ).rowRange(npts,ld  )=0;
          cv::Mat d_mat(ld,dim,CV_64F);
          dst.copyTo(d_mat.rowRange(0,npts));
          d_mat.rowRange(npts,ld)=0;

          m_mat = l_mat.inv()*d_mat;
          src.copyTo(m_src);


      }

      cv::Mat operator()(const cv::Mat& pts) const //MxP points where M is number of points and P is the points dim
      {
          int dim = m_src.cols ;
          assert(pts.cols == dim);
          int npts = pts.rows;
          cv:: Mat z;
          if(npts<=N_MAX_EVAL_POINTS)
              privAt(pts,z);
          else
          {
              z = cv::Mat(pts.size(),CV_64F);
              for(int i=0;i<pts.rows;i+=N_MAX_EVAL_POINTS)
              {
                  int i_end = std::min(i+N_MAX_EVAL_POINTS,pts.rows);
                  privAt(pts.rowRange(i,i_end),z.rowRange(i,i_end));
              }

          }
          return z;
      }

      TPS operator*(const TPS& tpsB)
      {
          const TPS& tpsA = *this;

          int nA = tpsA.m_src.rows;
          int nB = tpsB.m_src.rows;

          int dim = tpsA.dim();
          assert(tpsB.dim() == dim);

          cv::Mat src(nA+nB,dim,CV_64F);
          cv::Mat dst(nA+nB,dim,CV_64F);
          src.rowRange(0,nA) = tpsA.src();
          dst.rowRange(0,nA) = tpsB(tpsA(tpsA.src()));

          src.rowRange(nA,nA+nB) = tpsA.inv()(tpsB.src());
          dst.rowRange(nA,nA+nB) = tpsB.dst();


          TPS tpsC(src,dst);
          return tpsC;
      }

      TPS inv()const
      {
          TPS tpsInv(dst(),src());
          return tpsInv;
      }
      
      int dim() const {return m_src.cols;}

      const cv::Mat& src() const {return m_src;}
      cv::Mat dst() const {return this->operator()(m_src);}
};

class TPS2d
{
    TPS m_tps;
    typedef std::vector<cv::Point2d> PtArr;

    static cv::Mat privMatFROMptArr(const PtArr& pts)
    {
        cv::Mat mat(int(pts.size()),2,CV_64F);
        for(int i=0;i!=pts.size();++i)
        {
            mat.at<double>(i,0) = pts[i].x;
            mat.at<double>(i,1) = pts[i].y;
        }
        return mat;
    }
    static PtArr privPtArrFROMmat(const cv::Mat& mat)
    {
        PtArr arr(mat.rows);
        for(int i=0;i!=arr.size();++i)
        {
            arr[i].x = mat.at<double>(i,0);
            arr[i].y = mat.at<double>(i,1);
        }
        return arr;
    }
public:
    TPS2d(){}
    TPS2d   (const PtArr& src,const PtArr& dst)  {     set(src,dst);   }
    TPS2d inv() const {return TPS2d(dst(),src());}
    void  set(const PtArr& src,const PtArr& dst)  {  m_tps.set(privMatFROMptArr(src),privMatFROMptArr(dst));  }
    PtArr operator()(const PtArr& pts) const        {  return privPtArrFROMmat(m_tps(privMatFROMptArr(pts)));   }
    PtArr src() const {return privPtArrFROMmat(m_tps.src());}
    PtArr dst() const {return privPtArrFROMmat(m_tps.dst());}
    cv::Mat warpImage(const cv::Mat& img,int intrp)
    {
        cv::Mat pts(img.rows*img.cols,2,CV_64F);
        for(int x=0;x!=img.cols;++x)
        {
            for(int y=0;y!=img.rows;++y)
            {
                int c = y + x*img.rows;
                pts.at<double>(c,0)=x;
                pts.at<double>(c,1)=y;
            }
        }
        TPS tpsInv = m_tps.inv();
        cv::Mat dstPts = tpsInv(pts);
        cv::Mat xmap(img.size(),CV_32F);
        cv::Mat ymap(img.size(),CV_32F);
        for(int c=0;c!=dstPts.rows;++c)
        {
            int x = c/img.rows;
            int y = c%img.rows;
            xmap.at<float>(y,x)=float(dstPts.at<double>(c,0));
            ymap.at<float>(y,x)=float(dstPts.at<double>(c,1));
        }
        cv::Mat outImg;
        cv::remap(img,outImg,xmap,ymap,intrp);
        return outImg;
        
    }


};


#endif
