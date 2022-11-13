// PointcloudMapping.cc
#include "PointcloudMapping.h"

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
namespace ORB_SLAM2 {

    PointCloudMapping::PointCloudMapping(double resolution) {
        mResolution = resolution;
        mCx = 0;
        mCy = 0;
        mFx = 0;
        mFy = 0;
        mbShutdown = false;
        mbFinish = false;

        voxel.setLeafSize(resolution, resolution, resolution);
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(
                1.0); // The distance threshold will be equal to: mean + stddev_mult * stddev

        mPointCloud = boost::make_shared<PointCloud>();  // 用boost::make_shared<>

        viewerThread = std::make_shared<std::thread>(&PointCloudMapping::showPointCloud, this);  // make_unique是c++14的
    }

    PointCloudMapping::~PointCloudMapping() {
        viewerThread->join();
    }

    void PointCloudMapping::requestFinish() {
        {
            unique_lock<mutex> locker(mKeyFrameMtx);
            mbShutdown = true;
        }
        mKeyFrameUpdatedCond.notify_one();
    }

    bool PointCloudMapping::isFinished() {
        return mbFinish;
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth) {
        unique_lock<mutex> locker(mKeyFrameMtx);
        mvKeyFrames.push(kf);
        mvColorImgs.push(color.clone());  // clone()函数进行Mat类型的深拷贝，为什幺深拷贝？？
        mvDepthImgs.push(depth.clone());

        mKeyFrameUpdatedCond.notify_one();
        cout <<BLUE << "receive a keyframe, id = " << kf->mnId << endl;
    }

    /// 画位姿轨迹


/// 显示点云
    void PointCloudMapping::showPointCloud() {
        pcl::visualization::CloudViewer viewer("Dense pointcloud viewer");

        while (true) {
            KeyFrame *kf;
            cv::Mat colorImg, depthImg;

            {
                std::unique_lock<std::mutex> locker(mKeyFrameMtx);
                while (mvKeyFrames.empty() && !mbShutdown) {  // !mbShutdown为了防止所有关键帧映射点云完成后进入无限等待
                    mKeyFrameUpdatedCond.wait(locker);
                }

                if (!(mvDepthImgs.size() == mvColorImgs.size() && mvKeyFrames.size() == mvColorImgs.size())) {
                    std::cout <<RED << "这是不应该出现的情况！" << std::endl;
                    continue;
                }

                if (mbShutdown && mvColorImgs.empty() && mvDepthImgs.empty() && mvKeyFrames.empty()) {
                    break;
                }

                kf = mvKeyFrames.front();
                colorImg = mvColorImgs.front();
                depthImg = mvDepthImgs.front();
                mvKeyFrames.pop();
                mvColorImgs.pop();
                mvDepthImgs.pop();
            }

            if (mCx == 0 || mCy == 0 || mFx == 0 || mFy == 0) {
                mCx = kf->cx;
                mCy = kf->cy;
                mFx = kf->fx;
                mFy = kf->fy;
            }


            {
                std::unique_lock<std::mutex> locker(mPointCloudMtx);
                /// 调用产生点云函数
                generatePointCloud(kf, colorImg, depthImg, kf->mnId);
                viewer.showCloud(mPointCloud);
            }

            std::cout << BLUE << "show point cloud, size=" << mPointCloud->points.size() << std::endl;
        }

        // 存储点云
        string save_path = "./resultPointCloudFile.pcd";
        pcl::io::savePCDFile(save_path, *mPointCloud);
        cout << BLUE << "save pcd files to :  " << save_path << endl;
        mbFinish = true;
    }

    void PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imD, int nId) {
        std::cout << "Converting image: " << nId;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        PointCloud::Ptr current(new PointCloud);

        for (size_t v = 0; v < imD.rows; v += 3) {
            for (size_t u = 0; u < imD.cols; u += 3) {

                cv::Point2i pt(u, v);
                float d = imD.ptr<float>(v)[u];
                if (d < 0.01 || d > 10) { // 深度值为0 表示测量失败
                    continue;
                }
                bool IsDynamic = false;
                for (auto area: kf->mvDynamicArea)
                    if (area.contains(pt)) IsDynamic = true;

                if (!IsDynamic) {
                    /// 生成点云
                    PointT p;
                    p.z = d;
                    p.x = (u - kf->cx) * p.z / kf->fx;
                    p.y = (v - kf->cy) * p.z / kf->fy;

                    p.b = imRGB.ptr<uchar>(v)[u * 3];
                    p.g = imRGB.ptr<uchar>(v)[u * 3 + 1];
                    p.r = imRGB.ptr<uchar>(v)[u * 3 + 2];
                    current->points.push_back(p);
                }
                else
                {
                    PointT p_mvDynamic;
                    p_mvDynamic.z = d;
                    p_mvDynamic.x = (u - kf->cx) * p_mvDynamic.z / kf->fx;
                    p_mvDynamic.y = (v - kf->cy) * p_mvDynamic.z / kf->fy;
                    /// 动态点的点云用黑色显示
                    p_mvDynamic.b = 0;
                    p_mvDynamic.g = 0;
                    p_mvDynamic.r = 0;
                    current->points.push_back(p_mvDynamic);
                }

            }
        }



        Eigen::Isometry3d T = Converter::toSE3Quat( kf->GetPose() );
        PointCloud::Ptr tmp(new PointCloud);
        // tmp为转换到世界坐标系下的点云
        pcl::transformPointCloud(*current, *tmp, T.inverse().matrix());

        // depth filter and statistical removal，离群点剔除
        statistical_filter.setInputCloud(tmp);
        statistical_filter.filter(*current);
        (*mPointCloud) += *current;

        pcl::transformPointCloud(*mPointCloud, *tmp, T.inverse().matrix());
        // 加入新的点云后，对整个点云进行体素滤波
        voxel.setInputCloud(mPointCloud);
        voxel.filter(*tmp);
        mPointCloud->swap(*tmp);
        mPointCloud->is_dense = true;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << RED << ", Cost = " << t << std::endl;
    }


    void PointCloudMapping::getGlobalCloudMap(PointCloud::Ptr &outputMap)
    {
        std::unique_lock<std::mutex> locker(mPointCloudMtx);
        outputMap = mPointCloud;
    }

}