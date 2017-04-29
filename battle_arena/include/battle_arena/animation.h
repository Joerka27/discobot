#ifndef BATTLE_ARENA_ANIMATION
#define BATTLE_ARENA_ANIMATION

//#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>




class BattleAnimation
{
public:
    BattleAnimation(){}

    BattleAnimation(const std::string& folder, int type, float ticks_per_image);
    int animation_type_;
    float duration(){ return sprites_.size()*ticks_per_image_; }

    int get_image_index(float t);

    const cv::Mat& get_sprite(int index);
    size_t sprite_count(){ return sprites_.size();}

    static bool copyTo(const cv::Mat& sprite, cv::Mat& img, cv::Point center);

private:

    size_t read_images(const std::string& folder, std::string suffix="*.png");
    void create_masks(cv::Vec3b back_ground_color = cv::Vec3b(0, 0, 0));

    std::vector<cv::Mat> sprites_;
    std::vector<cv::Mat> masks_;

    float ticks_per_image_;
};



class RunningAnimation
{
public:
    RunningAnimation();

    float progress(float current_time){ return current_time-start_time_;}

    // float animation_progress;
    BattleAnimation* animation;
    float start_time_;
};








#endif
