# Introduction #

This document describes the BlobDetector API.
BlobDetector detects acts similarly to Matlab's bwlabel function - for an input of black and white image (CV\_8U) it will product an integer map such that each connected element (or blob) will receive the same value. for furthe details see: [Connected-component labeling](http://en.wikipedia.org/wiki/Blob_extraction)

# Code #
The BlobDetector is using a recursive algorithm for blob labeling. Although inefficient, it is relatively simply and bug-safe. In the future the two-pass algorithm will be used.
# API #
The BlobDetector is a static class ( namespace only) and has two function:
  * detect: for a given black-white images, produce a labeled image. return the number of blobs found
  * getBlobsData: for a given labeled image produce a vector of BlobData. the BlobData struct is filled according to flags in the function, which fills the data structure.
# Example #


```
#include <opencv2/opencv.hpp>
#include <opencvExt/BlobDetector.h>

void foo()
{
        BlobDetector::convectivity8(true);
        cv::Mat bwImg;
        ...
        ...
        ...
        cv::Mat labeled;
        
        int n = BlobDetector::detect(mask,labeled);
        std::vector<BlobData> blbs = getBlobsData(mask,true);
}
```

Add your content here.  Format your content with:
  * Text in **bold** or _italic_
  * Headings, paragraphs, and lists
  * Automatic links to other wiki pages