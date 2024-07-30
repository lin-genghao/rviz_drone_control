#ifndef BACKGROUND_THREAD_H
#define BACKGROUND_THREAD_H

#include <QThread>
#include <iostream>
#include "httplib.h"

namespace rviz_drone_control{
class BackgroundThread : public QThread{
public:
    struct Requst{
        std::string host;
        std::string uri;
    };

    BackgroundThread();
    void run();

    void set_url_param(std::string& url);
    void set_obj_param(const std::string& url, int id);
    void set_rect_param(const std::string& url, int x, int y, int width, int height);
    void set_clean_param(const std::string& url);
    void set_cloud_pitch_param(const std::string& url, int pitch);
    void set_track_drve_param(const std::string& url, int speed);
    void set_follow_param(const std::string& url, float keep_deg, float speed, float keep_z);
    void set_yaw_param(const std::string& url, int yaw);
    void set_mission_param(const std::string& url, int uav_id, float lat, float lon);
    void set_obj_en();
    void set_rect_en();
    void set_clean_en();
    void set_pitch_en();
    void set_track_drve_en();
    void set_follow_en();
    void set_yaw_en();
    void set_mission_en();
    Requst ParseUrl(const std::string& url);
    int HttpPost(const std::string& url, const std::string& body, std::string& result);
    std::string send_http_post(const std::string& url, const std::string& body);
    int set_rect(const std::string& url, int x, int y, int width, int height); 
    int set_obj(const std::string& url, int obj);
    int clean_obj(const std::string& url);
    std::string get_track_rect(const std::string& url);
    int set_cloud_pitch(const std::string& url, int pitch);
    int track_dive(const std::string& url, int speed);
    int follow(const std::string& url, float keep_deg, float speed, float keep_z);
    int set_yaw(const std::string& url, int yaw);
    int set_mission(const std::string& url, int uav_id, float lat, float lon);
    // int set_mission(const std::string& url, int uav_id);

private:
    std::string url_;
    int id_;
    int x_, y_, width_, height_;
    float keep_deg_, keep_z_;
    bool set_obj_en_;
    bool set_rect_en_;
    bool set_clean_en_;
    bool set_pitch_en_;
    bool set_yaw_en_;
    bool set_track_drve_en_;
    bool set_follow_en_;
    bool set_mission_en_;

    int strike_flag_;

    int yaw_;
    int pitch_;
    int speed_;

    float lat_;
    float lon_;

    int uav_id_;
};
} // namespace rviz_drone_control

#endif //BACKGROUND_THREAD_H