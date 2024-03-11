//
// Created by xiang on 2022/3/15.
//

#include "ch6/lidar_2d_utils.h"
#include <opencv2/imgproc.hpp>

namespace sad {
/***
 *
 * @param scan 激光雷达扫描数据
 * @param pose 激光在世界坐标下位姿，T_w_c
 * @param image  待生成的图片
 * @param color
 * @param image_size 图片尺寸大小
 * @param resolution 占据栅格地图分辨率，即每米内有多少个栅格
 * @param pose_submap 子地图在世界坐标下位姿，T_w_s
 */
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap) {
    if (image.data == nullptr) {
        ///CV_<bit_depth>(S|U|F)C<number_of_channels>，所以 CV_8UC3表示8位无符号3通道
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));   ///T_w_s-1*T_w_c*P
        ///子地图坐标系上点到成像平面上点的转化
        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);

        ///只考虑下x,y在20m以内的激光数据画图，因为800>x*20+400>0,20>x>-20,
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }

    // 同时画出pose自身所在位置
    Vec2d pose_in_image =   ///将激光传感器在子地图坐标系上的坐标(x,y),转化到图像坐标系下
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    ///以圆心为pose_in_image,半径为5(单位？)画圆，表示传感器所在位置
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}

}  // namespace sad