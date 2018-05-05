/*
// ポール検出. サンプルプログラム
//
// カメラデバイス番号は適宜変えてください.
// 
// openCV 3.41 + opencv_contrib
// opencv_contribが必要であることに注意してください.
//
// 
*/


#include "PoleDetection.h"

#include <iostream>
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2\ximgproc\disparity_filter.hpp>
using namespace cv;


#define OPENCV_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))


int main() {
    std::cout << "version: " << CV_VERSION << std::endl;

    
    PoleDetection detection;


    //string rightImg = "../LRImgs/sampleR.png";
    //string leftImg = "../LRImgs/sampleL.png";
    /*
    string rightImg = "../LRImgs/R.png";
    string leftImg = "../LRImgs/L.png";*/
    /*
        cv::VideoCapture captureL(0);
        cv::VideoCapture captureR(1);
        cv::namedWindow("CaptureL", CV_WINDOW_AUTOSIZE);
        cv::namedWindow("CaptureR", CV_WINDOW_AUTOSIZE);
    */
/*
    Mat imgR = cv::imread(rightImg, CV_8UC1);
    Mat imgL = cv::imread(leftImg, CV_8UC1);

*/
    Mat imgR;
    Mat imgL;
    
    /*
    Mat dispLMat(imgR.rows, imgR.cols, CV_8UC1);
    Mat dispRMat(imgR.rows, imgR.cols, CV_8UC1);
    Mat filteredDispMat(imgR.rows, imgR.cols, CV_8UC1);

    Mat edgesMat(imgR.rows, imgR.cols, CV_8UC1);

    Mat detectedLinesMat;

*/

    // *** Notice *********************************************
    // >> カメラデバイス番号は適宜変えてください.
    // >> 
    // ********************************************************
    cv::VideoCapture captureL(0);
    cv::VideoCapture captureR(2);


    cv::namedWindow("Right Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Left Image", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("DisparityR", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("DisparityL", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("FilteredDispMat", CV_WINDOW_AUTOSIZE);

    cv::namedWindow("DetectedLinesMat", CV_WINDOW_AUTOSIZE);


    TickMeter meter;
    meter.start();


    while (true) {
        double deltaX;

        Mat frameL, frameR;
        

        captureL >> frameL;
        captureR >> frameR;

        cvtColor(frameL, imgL, CV_RGB2GRAY);
        cvtColor(frameR, imgR, CV_RGB2GRAY);

        if (detection.CalculateDeltaX(imgL, imgR, deltaX)) {
            cout << deltaX << endl;
        }
        else {
            cout << "Not detected." << endl;
        }



        imshow("Right Image", imgR);
        imshow("Left Image", imgL);

        
        //imshow("DisparityR", detection.dispRMat);
        imshow("DisparityL", detection.dispLMat);
        imshow("FilteredDispMat", detection.filteredDispMat);
        imshow("DetectedLinesMat", detection.detectedLinesMat);
        


        if (cv::waitKey(30) >= 0) {
            break;
        }
    }

    cvDestroyAllWindows();

    return 0;


    // 以下からは, すべてコメントアウト.


    //
    //
    //    {
    //        int disp = 32;
    //        int minDisparity = 0;
    //        int numDisparities = 32;
    //        int SADWindowSize = 3;
    //        int P1 = 0;
    //        int P2 = 0;
    //        int disp12MaxDiff = 0;
    //        int preFilterCap = 0;
    //        int uniquenessRatio = 0;
    //        /*int speckleWindowSize = 200;
    //        int speckleRange = 1;*/
    //        int speckleWindowSize = 0;
    //        int speckleRange = 0;
    //        bool fullDp = false;
    //
    //        cv::Ptr<cv::StereoSGBM> ssgbm = cv::StereoSGBM::create(
    //            minDisparity,
    //            numDisparities,
    //            SADWindowSize,
    //            P1,
    //            P2, disp12MaxDiff,
    //            preFilterCap,
    //            uniquenessRatio,
    //            speckleWindowSize, speckleRange, cv::StereoSGBM::MODE_SGBM);
    //
    //        //            for (int i = 0; i < 100; i++) {
    //        //                ssgbm->compute(imgL, imgR, dispMat);
    //        //                //sgbm(imgL, imgR, dispMat);
    //        //            }
    //
    //        ssgbm->compute(imgL, imgR, dispLMat);
    //
    //
    //
    //        Ptr<StereoMatcher> rightMatcher = ximgproc::createRightMatcher(ssgbm);
    //        rightMatcher->compute(imgR, imgL, dispRMat);
    //
    //        /* ssgbm->compute*/
    //         //meter.stop();
    //         /*std::cout << "Average Time for Calculation : "
    //             << meter.getTimeMilli() / 100.0 << std::endl;*/
    //
    //
    //        Ptr<ximgproc::DisparityWLSFilter> filter = ximgproc::createDisparityWLSFilter(ssgbm);
    //
    //
    //
    //        filter->filter(dispLMat, imgL, filteredDispMat, dispRMat);
    //        double min, max;
    //        cv::minMaxLoc(dispLMat, &min, &max);
    //        cv::convertScaleAbs(dispLMat, dispLMat, 255 / (max - min), 255 / min);
    //        cv::minMaxLoc(dispRMat, &min, &max);
    //        cv::convertScaleAbs(dispRMat, dispRMat, 255 / (max - min), 255 / min);
    //        cv::minMaxLoc(filteredDispMat, &min, &max);
    //        cv::convertScaleAbs(filteredDispMat, filteredDispMat, 255 / (max - min), 255 / min);
    //        /*
    //        cv::minMaxLoc(filteredDispMat, &min, &max);
    //        cv::convertScaleAbs(filteredDispMat, filteredDispMat, 255 / (max - min), 255 / min);
    //*/
    //
    //
    //        cv::imshow("Right Image", imgR);
    //        cv::imshow("Left Image", imgL);
    //        cv::imshow("DisparityR", dispRMat);
    //        cv::imshow("DisparityL", dispLMat);
    //        cv::imshow("FilteredDispMat", filteredDispMat);
    //
    //    } // End ステレオ法
    //
    //
    //    {
    //
    //        Canny(filteredDispMat, edgesMat, 100, 200);
    //
    //
    //        imshow("EdgesMat", edgesMat);
    //
    //
    //        //Mat srcMat;
    //        //Mat dstMat;
    //        //IplImage srcImage;
    //        //IplImage *dstImage;
    //
    //        //srcImage = dispMat;
    //        //dstImage = cvCloneImage(&srcImage);
    //        //cvSmooth(&srcImage, dstImage, CV_MEDIAN, 5, 0, 0, 0);
    //        //
    //        //srcMat = cvarrToMat(dstImage);
    //        //Sobel(srcMat, dstMat, CV_32F, 1, 1);
    //
    //        //namedWindow("Edge", CV_WINDOW_AUTOSIZE);
    //        ////cvShowImage("Edge", dstImage);
    //        //imshow("Edge", dstMat);
    //        ////// ハフ変換のための前処理
    //        ////cvCanny(src_img_gray, src_img_gray, 50, 200, 3);
    //        ////storage = cvCreateMemStorage(0);
    //
    //
    //    }
    //
    //
    //    {
    //        cvtColor(edgesMat, detectedLinesMat, CV_GRAY2BGR);
    //        vector<Vec4i> lines;
    //        HoughLinesP(edgesMat, lines, 1,  CV_PI / 180, 50, 50, 10);
    //        for (size_t i = 0; i < lines.size(); i++)
    //        {
    //            Vec4i l = lines[i];
    //            line(detectedLinesMat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
    //        }
    //
    //        imshow("DetectedLines", detectedLinesMat);
    //    }
    //



    //cvWaitKey(0);
    //cvDestroyAllWindows();

    return 0;
}