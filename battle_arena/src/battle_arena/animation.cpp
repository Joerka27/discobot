#include <battle_arena/animation.h>
#include <boost/filesystem.hpp>
#include <math.h>

#include <ros/ros.h>

using std::vector;
using std::string;

BattleAnimation::BattleAnimation(const std::string &folder, int type, float ticks_per_image):
    animation_type_(type),
    ticks_per_image_(ticks_per_image)
{
    read_images(folder, "*.png");
    read_images(folder, "*.jpg");
    ROS_INFO("Loaded %zu images from %s", sprites_.size(), folder.c_str());

}

bool BattleAnimation::copyTo(const cv::Mat& sprite, cv::Mat& img, cv::Point p)
{
    int w = sprite.cols;
    int h = sprite.rows;

    int min_x = p.x-w/2;
    int max_x = p.x+w/2;
    int min_y = p.y-h/2;
    int max_y = p.y+h/2;

    if (min_x < 0 || min_y < 0 || max_x >= img.cols || max_y >= img.rows)
    {
        ROS_INFO_THROTTLE(1, "Animation to close to border, not yet implemented");
        return false;
    }
    sprite.copyTo(img.colRange(min_x, max_x).rowRange(min_y, max_y), sprite);
    return true;
}


void BattleAnimation::create_masks(cv::Vec3b back_ground_color)
{
    masks_.clear();
//    cv::Mat m = cv::Mat::zeros(sprites_[0].size(), CV_8UC1);
//    m.setTo(0, sprites_[0] == back_ground_color);

//    for (const cv::Mat& s: sprites_)
//    {
//        s.convertTo();
//    }
}


size_t BattleAnimation::read_images(const std::string& folder, std::string suffix)
{
    vector<string> file_names;
    std::string regex = folder+suffix; // "*.png";
    cv::glob(regex, file_names);
    for (size_t i=0; i<file_names.size(); ++i)
    {
       cv::Mat asd = cv::imread(file_names[i], cv::IMREAD_COLOR);
       sprites_.push_back(asd);
    }
//    ROS_INFO("Loaded %zu images from %s", file_names.size(), regex.c_str());
    return file_names.size();
}


const cv::Mat& BattleAnimation::get_sprite(int index)
{
    assert (sprites_.size() > 0);
    if (index < 0 || index >= sprites_.size())
    {
        ROS_WARN("INVALID index: %i (max at %zu)", index, sprites_.size());
        index = std::max(0, index);
        index = std::min(index, int(sprites_.size()-1));
    }
//    assert(index >= 0 && index<sprites_.size());
    return sprites_[index];
}

int BattleAnimation::get_image_index(float t)
{
    if (t>duration())
    {
        return -1;
    }
    return floor(t/ticks_per_image_);
}

RunningAnimation::RunningAnimation(){}

