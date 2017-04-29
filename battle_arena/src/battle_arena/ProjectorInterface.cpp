#include <battle_arena/ProjectorInterface.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer_client.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace cv;
using namespace battle_arena_msgs;

BattleProjectorInterface::BattleProjectorInterface() :
    ProjectorCalibrator(),
    border_px_(500),
    use_projector_publisher_(true),  /// param?
    background_color_(125,0,0)
{
    if (!use_projector_publisher_)
    {
        image_screen_ = new ImageScreen();
    }

//    std::map<int, BattleAnimation> animations_templates_; /// Animation_type to Animation
//    std::map<int, RunningAnimation> current_animations_; /// Animation_type to Animation

    /// Load images
    load_type_images_();

    /// Load animations
    int exp_type = battle_arena_msgs::ArenaObjectState::ANIMATION_EXPLOSION;
    BattleAnimation exp_animation("/home/nikolas/Documents/explosion_images/", exp_type, 0.02);
    animations_templates_[exp_type] = exp_animation;

    ROS_INFO("Images for explosion: %zu", animations_templates_[exp_type].sprite_count());

    ROS_INFO("Subscribing to object states and creating visualization");
    string ns = "/arena_manager/";
    sub_object_states_ = nh_private_.subscribe(ns+"object_states", 10, &BattleProjectorInterface::object_states_cb, this);

//    img_ = Mat(projector_resolution_.height+2*border_px_, projector_resolution_.width+2*border_px_ , CV_8UC3);
    img_ = Mat(projector_resolution_, CV_8UC3);
    img_.setTo(background_color_);

    sensor_msgs::CameraInfo projector_intrinsics;

    ProjectorCalibrator::readCalibration(data_folder_path_+"/projector_calib.bag", projector_intrinsics, projector2camera_);

    /// TODO: move to function
    ros::Publisher pub_tf_static = nh_private_.advertise<tf2_msgs::TFMessage>("/tf_static", 1);
    pub_proj_intrinsics = nh_private_.advertise<sensor_msgs::CameraInfo>("/projector_intrinsics", 1, true);

    ros::Duration(0.5).sleep();

    tf2_msgs::TFMessage tfm;
    tfm.transforms.push_back(projector2camera_);
    pub_tf_static.publish(tfm);

    pub_proj_intrinsics.publish(projector_intrinsics);

    pinhole_.fromCameraInfo(projector_intrinsics);

    tf2_ros::BufferClient tf2_client("/tf2_buffer_server", 200.0);
    if (!tf2_client.waitForServer(ros::Duration(0.5)))
    {
        ROS_ERROR("No tf2 server at '/tf2_buffer_server'");
        return;
    }

    arena_frame_ = nh_private_.param<std::string>("/arena_frame", "marker");
    projector_frame_ = projector_intrinsics.header.frame_id;

    try {
        arena2projector_ = tf2_client.lookupTransform(projector_frame_, arena_frame_,
                                                      ros::Time::now(), ros::Duration(0.5));
    } catch (...) {
        ROS_ERROR("TF2 exception %s to %s", arena_frame_.c_str(), projector_frame_.c_str());
        return;
    }

    depth_frame_ = "ensenso_base";
    try {
        arena2depth_ = tf2_client.lookupTransform(depth_frame_, arena_frame_,
                                                      ros::Time::now(), ros::Duration(0.5));
    } catch (...) {
        ROS_ERROR("TF2 exception %s to %s", arena_frame_.c_str(), depth_frame_.c_str());
        return;
    }

    pub_objects_projector_ = nh_private_.advertise<pcl_cloud>("objects_projector_frame", 1);
    redraw_timer_ = nh_private_.createTimer(ros::Duration(0.02), &BattleProjectorInterface::redraw_trigger, this);
}



BattleProjectorInterface::~BattleProjectorInterface()
{
    if (image_screen_)
    {
        image_screen_->close_image_window();
        delete image_screen_;
    }
}


bool BattleProjectorInterface::is_animation(const battle_arena_msgs::ArenaObjectState& object)
{
    int t = object.type;
    return (t == ArenaObjectState::ANIMATION_EXPLOSION ||
            t == ArenaObjectState::ANIMATION_SHIELD_UPGRADE ||
            t == ArenaObjectState::ANIMATION_BOX_COLLECTED ||
            t == ArenaObjectState::ANIMATION_PLAYER_DIED ||
            t == ArenaObjectState::ANIMATION_BANANA ||
            t == ArenaObjectState::ANIMATION_EXPLOSION);
}

void BattleProjectorInterface::object_states_cb(const battle_arena_msgs::ArenaObjectStateListConstPtr& object_list)
{
    if (object_list->battle_time < battle_time_)
    {
        ROS_WARN("New round started");
        current_animations_.clear();
        object_states.clear();
    }

    battle_time_ = object_list->battle_time;

    ROS_INFO_ONCE("Received first object states");
    object_states.clear();
    for (const auto& o: object_list->states)
    {
        object_states[o.object_id] = o;
    }

    /// check for animations
    for (const auto& o: object_list->states)
    {
        if (is_animation(o))
        {
          if (current_animations_.find(o.object_id) == current_animations_.end())
            {
                ROS_INFO("Got new explosion animation with is %i", o.object_id);
                RunningAnimation anim;
                anim.animation = &animations_templates_[o.type]; /// todo: make sure it exists
                anim.start_time_ = object_list->battle_time;
                current_animations_[o.object_id] = anim;
            }
        }
    }

//    for (auto& anim: current_animations_)
//    {
//        anim->second.update_progress(object_list->);
//    }
}

void BattleProjectorInterface::redraw_trigger(const ros::TimerEvent& e)
{
//     ROS_INFO("Drawing, last call took %.1f ms", e.profile.last_duration.toSec()*100.0);
    draw_visualization_simple();
}


void BattleProjectorInterface::load_type_images_()
{
    cv::Mat banana = cv::imread("/home/nikolas/Documents/hackathon/type_images/banana.jpg");
    cv::resize(banana, banana, cv::Size(200, 200));
    ROS_INFO("%i bana", banana.cols);
    type_images_[battle_arena_msgs::ArenaObjectState::BANANA] = banana;



}

bool BattleProjectorInterface::draw_type_image(int type, cv::Point p, cv::Mat& img, float angle_deg)
{
    if (type_images_.find(type) == type_images_.end())
    {
        return false;
    }

    const cv::Mat& sprite = type_images_[type];

    if (fabs(angle_deg) > 0)
    {
        ROS_INFO_ONCE("ROTATED IMAGES ARE NOT SUPPORTED YET");
    }

    if (!BattleAnimation::copyTo(sprite, img, p))
    {
        /// close to border, at least show that there is someting
        cv::circle(img, p, 20, CV_RGB(255, 0, 0), 1);
    }

    return true;
}



/**
 * @brief BattleProjectorInterface::draw_visualization_simple draws a simple visualization for testing
 */
void BattleProjectorInterface::draw_visualization_simple()
{
    img_.setTo(background_color_);

//    ROS_INFO("Object count: %zu", object_states.size());
    pcl_cloud arena_positions;

    if (object_states.size() == 0)
    {
        if (use_projector_publisher_)
        {
            send_image_to_projector(img_);
        }else
        {
            image_screen_->show_image(img_);
        }
        return;
    }

    for (const auto& o: object_states)
    {
        pcl::PointXYZ p(o.second.pose.x_pos, o.second.pose.y_pos, 0);
        arena_positions.push_back(p);
    }

    /// transform points into depth camera frame
    pcl_cloud depth_positions;
    pcl::transformPointCloud(arena_positions, depth_positions, tf2::transformToEigen(arena2depth_));
    depth_positions.header.frame_id = depth_frame_;
    pub_objects_projector_.publish(depth_positions);

    /// projecting points into projector
    vector<cv::Point3f> points_3d;
    for (const auto&p : depth_positions)
    {
        points_3d.push_back(Point3f(p.x, p.y, p.z));
    }

//	/// no further transformation (we could fill these with the arena2projector_ transformation)
    Mat rvec(1,3,CV_64FC1); rvec.setTo(0);
    Mat tvec(1,3,CV_64FC1); tvec.setTo(0);

    geometry_msgs::TransformStamped depth2projector = ProjectorCalibrator::invert(projector2camera_);
    geometryTransform2openCV(depth2projector.transform, rvec, tvec);


    vector<Point2f> projected;
//	cout << rvec << tvec << endl;
    cv::projectPoints(points_3d, rvec, tvec, pinhole_.fullIntrinsicMatrix(), pinhole_.distortionCoeffs(), projected);

//	cout << "Projector intrinsics " <<  pinhole_.fullIntrinsicMatrix() << endl;

    /// simple visualization
//	for (const auto&p : projected)
//	{
//		ROS_INFO("%f %f ", p.x, p.y);
//		cv::circle(img_, p, 100, CV_RGB(0, 0, 255), -1);
//	}


//	for (size_t i=0; i<object_states.size(); ++i)
    int i=-1;
    for (const auto& s: object_states)
    {
        i+=1;
        const Point& p = Point(projected[i].x, projected[i].y);
        battle_arena_msgs::ArenaObjectState state = s.second;
        // ROS_INFO("Visualizaing type %i", state.type);


        if (is_animation(state))
        {
//            ROS_INFO("Updating animation");
            RunningAnimation& run = current_animations_[state.object_id];
            if (run.progress(battle_time_) > run.animation->duration())
            {
//                current_animations_.erase(state.object_id);
                continue;
            }

            int index = run.animation->get_image_index(run.progress(battle_time_));
//            ROS_INFO("Battle time: %f, start: %f, index: %i", battle_time_, run.start_time_, index);
            const cv::Mat& sprite = run.animation->get_sprite(index);

            /// does not draw animation close to border
            BattleAnimation::copyTo(sprite, img_, p);
        }

        if (state.type == state.BANANA)
        {
            draw_type_image(state.type, p, img_); //, state.pose.orientation);
        }


        if (state.type == state.ROCKET)
        {
            cv::circle(img_, p, 20, CV_RGB(255, 0, 0), -1);
        }

        if (state.type == state.SENTRY)
        {
            cv::circle(img_, p, 100, CV_RGB(100, 100, 0), -1);
        }

        if (state.type == state.PLAYER)
        {
//             ROS_INFO("player: %i %i", p.x, p.y);

            float hp = state.player_hp;
            float shield = state.player_shield;

            if (hp <=0)
            {
                cv::circle(img_, p, 70, CV_RGB(255, 0, 0), -1);
            }else
            {
                if (shield > 0)
                {
//                    ROS_INFO("Projected: %i %i", p.x, p.y);
//					cv::ellipse(img_, p, Size(120,120), 0, 0, 360, CV_RGB(0, 255, 255), 2);
                    cv::ellipse(img_, p, Size(100,100), 0, 0, shield/40.0*360, CV_RGB(255, 255, 255), 10);
                }
                cv::ellipse(img_, p, Size(90,90),0,0, hp/100.0*360, CV_RGB(0, 255, 0), 10);
            }
        }

    }

    if (use_projector_publisher_)
    {
        send_image_to_projector(img_);
        //.colRange(border_px_, border_px_+projector_resolution_.width).
        // rowRange(border_px_, border_px_+projector_resolution_.height));
    }else
    {
        image_screen_->show_image(img_);
    }
}
