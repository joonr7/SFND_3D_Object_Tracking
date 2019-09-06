
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        double fontsize = 0.5; // default: 2
        double text_org_x = 50; // default: 250
        double text_org_y = 30; // default: 50
        double text_org_y_gap = 30; // default: 75
        putText(topviewImg, str1, cv::Point2f(left-text_org_x, bottom+text_org_y), cv::FONT_ITALIC, fontsize, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-text_org_x, bottom+text_org_y+text_org_y_gap), cv::FONT_ITALIC, fontsize, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    for(auto it=kptMatches.begin(); it!=kptMatches.end(); it++)
    {
        // if boundingBox.roi.contains(current Kpts)?
        if(boundingBox.roi.contains(kptsCurr[it->trainIdx].pt))
        {
            // yes: push_back.
            boundingBox.keypoints.push_back(kptsCurr[it->trainIdx]);
            boundingBox.kptMatches.push_back(*it);
        }
    }

    // cout << "after lidar pt size: " << boundingBox.lidarPoints.size() << endl;
    // cout << "after kpts size: " << boundingBox.keypoints.size() << endl; 
    // cout << "after kpt matches size: " << boundingBox.kptMatches.size() << endl;
    
    // cf>
    // cv::DMatch.queryIdx: index of prev kpts
    // cv::DMatch.trainIdx: index of curr kpts.
    // keypoint.pt = cv::Point2f((*it).x, (*it).y);

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    float laneWidth = 4.0;
    float xminPrev=1e8, xminCurr=1e8;
    float xsumPrev = 0, xsumCurr = 0;
    int counterPrev = 0, counterCurr =0 ;
    for(auto it1 = lidarPointsPrev.begin(); it1 != lidarPointsPrev.end(); ++it1)
    {
        // ego line? 
        if (abs(it1->y) <= laneWidth / 2.0)
        {
            xminPrev = xminPrev < it1->x ? xminPrev : it1->x;
            xsumPrev += it1->x;
            counterPrev ++;
        }     
    }    
    for(auto it2 = lidarPointsCurr.begin(); it2 != lidarPointsCurr.end(); ++it2)
    {
        // ego line? 
        if (abs(it2->y) <= laneWidth / 2.0)
        {
            xminCurr = xminCurr < it2->x ? xminCurr : it2->x;
            xsumCurr += it2->x;
            counterCurr ++;
        }
    }

    // float xavePrev = xsumPrev / lidarPointsPrev.size();
    // float xaveCurr = xsumCurr / lidarPointsCurr.size();
    float xavePrev = xsumPrev / counterPrev;
    float xaveCurr = xsumCurr / counterCurr;

    cout << "xminPrev: " << xminPrev << ", xavePrev: " << xavePrev << endl;
    cout << "xminCurr: " << xminCurr << ", xaveCurr: " << xaveCurr << endl; 

    float xdistPrev = xavePrev;
    float xdistCurr = xaveCurr;

    cout << "Pointcloud size: prev: " << lidarPointsPrev.size() << ", " << counterPrev << ", curr: " << lidarPointsCurr.size() << ", " << counterCurr << endl;
    cout <<  "xdistPrev: " << xdistPrev << ", xdistCurr: " << xdistCurr << ", deltaD: " << xdistPrev - xdistCurr << endl;
    float velocityCurr = (xdistPrev - xdistCurr) * frameRate;
    TTC = xminCurr / velocityCurr;
    cout << "xdistCurr: " << xdistCurr << ", velocityCurr: " << velocityCurr << ", TTC" << TTC << endl;

    // vector<float> distRatios;
    // double meanDistRatio = std::accumulate()
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // std::cout << "Matches size: " << matches.size() << std::endl;

    // // Certify which index are of which a point.
    // for(auto it = matches.begin();it != matches.end(); ++it)
    // {
    //     cout << "prevFrame keypoint[" << it->queryIdx << "] :" << prevFrame.keypoints[it->queryIdx].pt.x << ", " << prevFrame.keypoints[it->queryIdx].pt.y << endl;
    //     cout << "currFrame keypoint[" << it->trainIdx << "] :"  << currFrame.keypoints[it->trainIdx].pt.x << ", " << currFrame.keypoints[it->trainIdx].pt.y << endl;
    //     cout << endl;
    // }

    // std::cout << "# of previous frame bounding boxes: " << prevFrame.boundingBoxes.size() << std::endl;
    // std::cout << "# of current frame bounding boxes: " << currFrame.boundingBoxes.size() << std::endl;
    
    // Make a roiMatchingPoint vector(matrix)
    std::vector < std::vector <int> > roiMatchingPoint;
    for(int i = 0; i < prevFrame.boundingBoxes.size(); i++)
    {
        std::vector<int> v;
        for(int j=0; j < currFrame.boundingBoxes.size(); j++)
        {
            v.push_back(0);
        }
        roiMatchingPoint.push_back(v);
    }

    // // Now, roiMatchingPoint size: (Num of BB in Curr, Num of BB in Prev)
    // std::cout << "ROI matching point vector size: " << roiMatchingPoint.size() << ", " << roiMatchingPoint[0].size() << std::endl;
     
    // matches->queryIdx: index of prev kpts, matches->trainIdx: index of curr kpts.
    for(auto it = matches.begin();it != matches.end(); ++it)
    {
        for(int i = 0; i < roiMatchingPoint.size(); i++)
        {
            // cout << "prevframe BB ROI[" << i << "] " << endl;
            // cout << "x,y: " << prevFrame.boundingBoxes[i].roi.x << ", " << prevFrame.boundingBoxes[i].roi.y << endl;
            // cout << "h,w: " << prevFrame.boundingBoxes[i].roi.height << ", " << prevFrame.boundingBoxes[i].roi.width << endl;
            if(prevFrame.boundingBoxes[i].roi.contains(prevFrame.keypoints[it->queryIdx].pt))
            {
                // if( prevFrame.keypoints[it->queryIdx].pt.x > 549 && prevFrame.keypoints[it->queryIdx].pt.x < 700 && prevFrame.keypoints[it->queryIdx].pt.y > 190 && prevFrame.keypoints[it->queryIdx].pt.y < 340){
                //     cout << i << "th BB: " << prevFrame.boundingBoxes[i].roi.x<< ", " << prevFrame.boundingBoxes[i].roi.y << ", " << prevFrame.boundingBoxes[i].roi.height << ", " << prevFrame.boundingBoxes[i].roi.width << endl;
                //     std::cout << "pt: " << prevFrame.keypoints[it->queryIdx].pt.x << ", " << prevFrame.keypoints[it->queryIdx].pt.y << endl; 
                // }
                for(int j = 0; j < roiMatchingPoint[0].size(); j++)
                {
                    if(currFrame.boundingBoxes[j].roi.contains(currFrame.keypoints[it->trainIdx].pt))
                    {
                        roiMatchingPoint[i][j] += 1;
                        // break; // consider that a keypoint can be included in multiple ROIs. 
                    }
                }
                // break; // consider that a keypoint can be included in multiple ROIs. 
            }
        }
    }

    // // Print out the roiMatchingPoint table.
    // int sum = 0;
    // for(int i = 0; i < roiMatchingPoint.size(); i++)
    // {
    //     for(int j = 0; j < roiMatchingPoint[0].size(); j++)
    //     {
    //         sum += roiMatchingPoint[i][j];
    //         std::cout << roiMatchingPoint[i][j] <<"\t";
    //     }
    //     std::cout << "\tMax: " << std::distance(roiMatchingPoint[i].begin(), std::max_element(roiMatchingPoint[i].begin(), roiMatchingPoint[i].end())) << std::endl;
    // }

    for(int i = 0; i < roiMatchingPoint.size(); i++)
    {
        int maxIdx = std::distance(roiMatchingPoint[i].begin(), std::max_element(roiMatchingPoint[i].begin(), roiMatchingPoint[i].end()));        
        if(roiMatchingPoint[i][maxIdx] > 25)
        {
            // std::cout << "[add] " << i << ", " << maxIdx << ", " << roiMatchingPoint[i][maxIdx] << std::endl;
            bbBestMatches.insert(pair<int, int>(i, maxIdx)); //pair(prev, curr)
        }
    }

    std::cout << "bbBestMatches size: " << bbBestMatches.size() << std::endl;

    // // Print matched ROIs
    // for(auto itBB = bbBestMatches.begin(); itBB != bbBestMatches.end(); ++itBB)
    // {
    //     cout << "(prev, curr) = (" << (*itBB).first << "," << (*itBB).second << ")" << " " << roiMatchingPoint[(*itBB).first][(*itBB).second] << endl;
    //     std::cout << "- prev frame BB roi[" << (*itBB).first << "] x,y,h,w: " << prevFrame.boundingBoxes[(*itBB).first].roi.x << ", " << prevFrame.boundingBoxes[(*itBB).first].roi.y;
    //     std::cout << ", " <<prevFrame.boundingBoxes[(*itBB).first].roi.height << ", " << prevFrame.boundingBoxes[(*itBB).first].roi.width << endl;
    //     std::cout << "- curr frame BB roi[" << (*itBB).second << "] x,y,h,w: " << currFrame.boundingBoxes[(*itBB).second].roi.x << ", " << currFrame.boundingBoxes[(*itBB).second].roi.y;
    //     std::cout << ", " << currFrame.boundingBoxes[(*itBB).second].roi.height << ", " << currFrame.boundingBoxes[(*itBB).second].roi.width << endl;
    // }    
}
