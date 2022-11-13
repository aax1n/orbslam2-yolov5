/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    // 初始化图像显示画布
    // 包括：图像、特征点连线形成的轨迹（初始化时）、框（跟踪时的MapPoint）、圈（跟踪时的特征点）
    // ！！！固定画布大小为640*480
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}
/// 准备需要显示的信息，包括图像、特征点、地图、跟踪状态
cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    // step 2：绘制初始化轨迹连线，绘制特征点边框（特征点用小框圈住）
    // step 2.1：初始化时，当前帧的特征坐标与初始帧的特征点坐标连成线，形成轨迹
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        //绘制当前帧特征点到下一帧特征点的连线,其实就是匹配关系
        //NOTICE 就是当初看到的初始化过程中图像中显示的绿线
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            //如果这个点在视觉里程计中有(应该是追踪成功了的意思吧),在局部地图中也有
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
  /*              {
                    std::unique_lock<std::mutex> lock(mMutexPCFinsh);
                    int x = 0; int y = 0; int z = 0;
                    for (auto vit = cp2_mmDetectMap.begin(); vit != cp2_mmDetectMap.end(); vit++)
                    {
                        cout << "我是2"  <<endl;
                        if (vit->second.size() != 0)
                        {
                            //https://blog.csdn.net/qq_51985653/article/details/113392665?
                            if (vit->first == "person"){x = 160; y =32; z = 240;}  //紫色
                            if (vit->first == "chair"){x = 94; y =38; z = 18;}     // 乌贼墨棕色
                            if (vit->first == "book"){x = 218; y =112; z = 214;}   //淡紫色
                            if (vit->first == "car"){x = 0; y =255; z = 255;}      //青色
                            if (vit->first == "keyboard"){x = 128; y =42; z = 42;}  // 棕色
                            if (vit->first == "cup"){x = 3; y =168; z = 158;}
                            if (vit->first == "laptop"){x = 220; y =220; z = 220;}  //金属色
                            if (vit->first == "tvmonitor"){x = 128; y =138; z = 125;}  //冷灰
                            if (vit->first == "knife"){x = 51; y =161; z = 201;}
                            if (vit->first == "sofa"){x = 227; y =168; z = 105;}
                            if (vit->first == "bed"){x = 188; y =143; z = 143;}
                            if (vit->first == "bicycle"){x = 25; y =25; z = 112;}
                            if (vit->first == "bear"){x = 115; y =74; z = 18;}
                            if (vit->first == "motorbike"){x = 218; y =112; z = 214;}  // 淡紫色
                            if (vit->first == "bus"){x = 176; y =224; z = 230;}  //浅灰蓝色
                            if (vit->first == "mouse"){x = 0; y =255; z = 127;}
                            if (vit->first == "clock"){x = 255; y =127; z = 80;}
                            if (vit->first == "refrigerator"){x = 189; y =252; z = 201;}
                            if (vit->first == "teddy bear"){x = 210; y =7180; z = 240;}
                            if (vit->first == "handbag"){x = 128; y =128; z = 105;}
                            if (vit->first == "backpack"){x = 255; y =192; z = 203;} //很正的粉色
                            if (vit->first == "bottle"){x = 135; y =38; z = 37;}  //草莓色
                            if (vit->first == "wine glass"){x = 176; y =23; z = 31;}
                            if (vit->first == "truck"){x = 163; y =148; z = 128;}
                            if (vit->first == "train"){x = 189; y =252; z = 201;}
                            if (vit->first == "vase"){x = 127; y =255; z = 212;}

                            // 未给特殊颜色的
                            if (vit->first == "aeroplane"){x = 160; y =32; z = 240;}  //紫色
                            if (vit->first == "boat"){x = 94; y =38; z = 18;}     // 乌贼墨棕色
                            if (vit->first == "traffic light"){x = 218; y =112; z = 214;}   //淡紫色
                            if (vit->first == "fire hydrant"){x = 0; y =255; z = 255;}      //青色
                            if (vit->first == "stop sign"){x = 128; y =42; z = 42;}  // 棕色
                            if (vit->first == "parking meter"){x = 3; y =168; z = 158;}
                            if (vit->first == "bench"){x = 220; y =220; z = 220;}  //金属色
                            if (vit->first == "bird"){x = 128; y =138; z = 125;}  //冷灰
                            if (vit->first == "cat"){x = 51; y =161; z = 201;}
                            if (vit->first == "dog"){x = 227; y =168; z = 105;}
                            if (vit->first == "horse"){x = 188; y =143; z = 143;}
                            if (vit->first == "sheep"){x = 25; y =25; z = 112;}
                            if (vit->first == "cow"){x = 115; y =74; z = 18;}
                            if (vit->first == "elephant"){x = 218; y =112; z = 214;}  // 淡紫色
                            if (vit->first == "zebra"){x = 176; y =224; z = 230;}  //浅灰蓝色
                            if (vit->first == "mouse"){x = 0; y =255; z = 127;}
                            if (vit->first == "giraffe"){x = 255; y =127; z = 80;}
                            if (vit->first == "backpack"){x = 189; y =252; z = 201;}
                            if (vit->first == "umbrella"){x = 210; y =7180; z = 240;}
                            if (vit->first == "tie"){x = 128; y =128; z = 105;}
                            if (vit->first == "suitcase"){x = 255; y =192; z = 203;} //很正的粉色
                            if (vit->first == "frisbee"){x = 135; y =38; z = 37;}  //草莓色
                            if (vit->first == "skis"){x = 176; y =23; z = 31;}
                            if (vit->first == "snowboard"){x = 163; y =148; z = 128;}
                            if (vit->first == "sports ball"){x = 189; y =252; z = 201;}
                            if (vit->first == "kite"){x = 127; y =255; z = 212;}
                            if (vit->first == "baseball bat"){x = 160; y =32; z = 240;}  //紫色
                            if (vit->first == "baseball glove"){x = 94; y =38; z = 18;}     // 乌贼墨棕色
                            if (vit->first == "skateboard"){x = 218; y =112; z = 214;}   //淡紫色
                            if (vit->first == "surfboard"){x = 0; y =255; z = 255;}      //青色
                            if (vit->first == "tennis racket"){x = 128; y =42; z = 42;}  // 棕色
                            if (vit->first == "fork"){x = 3; y =168; z = 158;}
                            if (vit->first == "spoon"){x = 220; y =220; z = 220;}  //金属色
                            if (vit->first == "bowl"){x = 128; y =138; z = 125;}  //冷灰
                            if (vit->first == "banana"){x = 51; y =161; z = 201;}
                            if (vit->first == "apple"){x = 227; y =168; z = 105;}
                            if (vit->first == "sandwich"){x = 188; y =143; z = 143;}
                            if (vit->first == "orange"){x = 25; y =25; z = 112;}
                            if (vit->first == "broccoli"){x = 115; y =74; z = 18;}
                            if (vit->first == "carrot"){x = 218; y =112; z = 214;}  // 淡紫色
                            if (vit->first == "hot dog"){x = 176; y =224; z = 230;}  //浅灰蓝色
                            if (vit->first == "pizza"){x = 0; y =255; z = 127;}
                            if (vit->first == "donut"){x = 255; y =127; z = 80;}
                            if (vit->first == "cake"){x = 189; y =252; z = 201;}
                            if (vit->first == "pottedplant"){x = 210; y =7180; z = 240;}
                            if (vit->first == "diningtable"){x = 128; y =128; z = 105;}
                            if (vit->first == "toilet"){x = 255; y =192; z = 203;} //很正的粉色
                            if (vit->first == "remote"){x = 135; y =38; z = 37;}  //草莓色
                            if (vit->first == "cell phone"){x = 176; y =23; z = 31;}
                            if (vit->first == "microwave"){x = 163; y =148; z = 128;}
                            if (vit->first == "oven"){x = 189; y =252; z = 201;}
                            if (vit->first == "toaster"){x = 127; y =255; z = 212;}
                            if (vit->first == "sink"){x = 25; y =25; z = 112;}
                            if (vit->first == "scissors"){x = 115; y =74; z = 18;}
                            if (vit->first == "hair drier"){x = 218; y =112; z = 214;}  // 淡紫色
                            if (vit->first == "toothbrush"){x = 176; y =224; z = 230;}  //浅灰蓝色

                            for (auto area : vit->second)
                            {
                                //cv::rectangle(im, area, cv::Scalar(0, 0, 255), 2);
                                cv::rectangle(im, area, cv::Scalar(x, y, z), 2);
                                //函数 cv::putText()：在图像中绘制制定文字
                                cv::putText(im,
                                            vit->first,
                                            cv::Point(area.x, area.y),
                                        //cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 127, 255), 2);
                                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(x, y, z), 2);
                            }

                        }

                    }
                }*/
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
