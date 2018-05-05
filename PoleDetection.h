#ifndef INCLUDE_POLE_DETECTION
#define INCLUDE_POLE_DETECTION


#include <iostream>
#include <string>
#include <cmath>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2\ximgproc\disparity_filter.hpp>
using namespace cv;


class PoleDetection {
private:

    int rows, cols;


    void ResizeMatrix(int newRows, int newCols) {
        if (newRows == rows && newCols == cols) {
            return;
        }

        dispLMat = Mat(newRows, newCols, CV_8UC1);
        dispRMat = Mat(newRows, newCols, CV_8UC1);
        filteredDispMat = Mat(newRows, newCols, CV_8UC1);
        edgesMat = Mat(newRows, newCols * focusRange, CV_8UC1);

        rows = newRows;
        cols = newCols;
    }


public:

    Ptr<cv::StereoSGBM> ssgbm;
    Ptr<StereoMatcher> rightMatcher;
    Ptr<ximgproc::DisparityWLSFilter> filter;
    Mat dispLMat;
    Mat dispRMat;
    Mat filteredDispMat;
    Mat edgesMat;
    Mat detectedLinesMat;

    double focusRange = 0.8;


    PoleDetection() {
        int disp = 32;
        int minDisparity = 0;
        int numDisparities = 32;
        int SADWindowSize = 3;
        int P1 = 0;
        int P2 = 0;
        int disp12MaxDiff = 0;
        int preFilterCap = 0;
        int uniquenessRatio = 0;
        int speckleWindowSize = 200;
        int speckleRange = 1;
        /*int speckleWindowSize = 0;
        int speckleRange = 0;*/
        bool fullDp = false;

        ssgbm = StereoSGBM::create(
            minDisparity,
            numDisparities,
            SADWindowSize,
            P1,
            P2, disp12MaxDiff,
            preFilterCap,
            uniquenessRatio,
            speckleWindowSize, speckleRange, StereoSGBM::MODE_SGBM);


        rightMatcher = ximgproc::createRightMatcher(ssgbm);

        filter = ximgproc::createDisparityWLSFilter(ssgbm);
    }



    //
    // ポールとのずれを計算します.
    // 画像中央からのずれです.
    //
    // @param left:
    //  左カメラ画像
    //
    // @param right:
    //  右カメラ画像
    //
    // @param deltaX:
    //  ずれ計算結果.
    // 
    // @return:
    //  trueのときは正常に計算できた.
    //  falseのときは, 何らかの理由で計算できなかった.
    //  何らかの理由とは, ポールが検知できないときなどです.
    //
    bool CalculateDeltaX(InputArray left, InputArray right, double &deltaX) {

        // 画像サイズ変更による行列メモリの再確保
        ResizeMatrix(left.getMat().rows, left.getMat().cols);

        // StereoSGBM法による視差マップ計算
        ssgbm->compute(left, right, dispLMat);

        // 右視点からの視差計算
        rightMatcher->compute(right, left, dispRMat);

        //cout << dispLMat;

        // 視差マップのフィルタ処理(DisparityWLSFilter)
        filter->filter(dispLMat, left, filteredDispMat, dispRMat);

        // グレースケール化
        double min, max;
        cv::minMaxLoc(dispLMat, &min, &max);
        cv::convertScaleAbs(dispLMat, dispLMat, 255 / (max - min), 255 / min);

        cv::minMaxLoc(dispRMat, &min, &max);
        cv::convertScaleAbs(dispRMat, dispRMat, 255 / (max - min), 255 / min);

        cv::minMaxLoc(filteredDispMat, &min, &max);
        cv::convertScaleAbs(filteredDispMat, filteredDispMat, 255 / (max - min), 255 / min);

        // フォーカス部切り取り
        Mat focusedFilterDispMat(filteredDispMat, Rect((filteredDispMat.cols - edgesMat.cols) / 2, 0, edgesMat.cols, edgesMat.rows));

        // Cannyのエッジ抽出
        Canny(focusedFilterDispMat, edgesMat, 100, 200);

        // 確率ハフ変換による直線検出
        cvtColor(edgesMat, detectedLinesMat, CV_GRAY2BGR);
        vector<Vec4i> lines;
        HoughLinesP(edgesMat, lines, 1, CV_PI / 180, 50, 50, 10);

        

        double polePosX = 0.0;
        double totalLength = 0.0;
        int detectedLineCount = 0;

        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];

            // 垂直線のみ選択
            if (std::abs(l[0] - l[2]) < 5) {
                double posX = (l[0] + l[2]) / 2.0;
                double length = std::sqrt((l[0] - l[2]) * (l[0] - l[2]) + (l[1] - l[3]) * (l[1] - l[3]));

                polePosX += posX * length;
                totalLength += length;

                detectedLineCount++;

                line(detectedLinesMat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
            }
        }

        // 垂直線が存在しないとき
        if (detectedLineCount <= 0) {
            return false;
        }


        polePosX /= totalLength;

        line(detectedLinesMat, Point(polePosX, 0), Point(polePosX, detectedLinesMat.rows), Scalar(255, 0, 0), 3, CV_AA);

        deltaX = polePosX - detectedLinesMat.cols / 2.0;

        return true;
    }


};

#endif