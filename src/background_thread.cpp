#include "rviz_drone_control/background_thread.h"

namespace rviz_drone_control{
BackgroundThread::BackgroundThread(){
    set_obj_en_ = false;
    set_rect_en_ = false;
    set_clean_en_ = false;
    set_pitch_en_ = false;
    set_track_drve_en_ = false;
    strike_flag_ = 0;
    set_yaw_en_ = false;
    set_follow_en_ = false;
    set_mission_en_ = false;
    yaw_ = 0;
    pitch_ = 0;
}

void BackgroundThread::run(){
    while (true){
        // ROS_INFO("BackgroundThread run!");
        sleep(2);
        if(set_obj_en_) {
            set_obj(url_, id_);
            set_obj_en_=false;
            std::cout << "set_obj clean!" << std::endl;
        }
        else if(set_rect_en_) {
            set_rect(url_, x_, y_, width_, height_);
            set_rect_en_=false;
            std::cout << "set_rect clean!" << std::endl;
        } 
        else if(set_clean_en_) {
            clean_obj(url_);
            set_clean_en_=false;
            std::cout << "set_clean clean!" << std::endl;
        }
        else if(set_pitch_en_) {
            set_cloud_pitch(url_, pitch_);
            set_pitch_en_=false;
            std::cout << "set_cloud_pitch clean!" << std::endl;
        }
        else if(set_track_drve_en_) {
            track_dive(url_, speed_);
            set_track_drve_en_=false;
            std::cout << "set_track_drve clean!" << std::endl;
        }
        else if(set_yaw_en_) {
            set_yaw(url_, yaw_);
            set_yaw_en_=false;
            std::cout << "set_cloud_yaw clean!" << std::endl;
        }
        else if(set_follow_en_) {
            follow(url_, keep_deg_, speed_, keep_z_);
            set_follow_en_=false;
            std::cout << "set_follow clean!" << std::endl;
        }
        else if(set_mission_en_) {
            set_mission(url_, uav_id_, lat_, lon_);
            set_mission_en_=false;
            std::cout << "set_follow clean!" << std::endl;
        }
    }
}

BackgroundThread::Requst BackgroundThread::ParseUrl(const std::string& url){
    BackgroundThread::Requst req;
    int offset = 0;
    if(url.substr(0,7) == "http://"){
        offset = 7;
    }else if(url.substr(0,8) == "https://"){
        offset = 8;
    }
    req.host = url.substr(0, url.find_first_of("/", offset));
    req.uri = url.substr(url.find_first_of("/", offset));
    return req;
}

int BackgroundThread::HttpPost(const std::string& url, const std::string& body, std::string& result) {
    BackgroundThread::Requst req = ParseUrl(url);
    httplib::Client cli(req.host);

    if (auto res = cli.Post(req.uri.c_str(), body, "application/json")) {
        if (res->status == 200) {
            result = res->body;
        } else {
            std::cout << "error status: " << res->status << std::endl;
            if (res->body.length() > 0) {
                std::cout << "error body: " << res->body << std::endl;
            }
            return -1;
        }
    } else {
        auto err = res.error();
        // std::cout << "http error " << httplib::to_std::string(err).c_str() << std::endl;
        return -1;
    }
    return 0;
}

std::string BackgroundThread::send_http_post(const std::string& url, const std::string& body) {
    std::string result;
    if (HttpPost(url, body, result) != 0) {
        return "";
    }
    return result;
}

// {"x": 100, "y": 100, "width": 200, "height": 200}
int BackgroundThread::set_rect(const std::string& url, int x, int y, int width, int height) {
    std::string body = "{\"x\": " + std::to_string(x) + ", \"y\": " + std::to_string(y) + ", \"width\": " + std::to_string(width) + ", \"height\": " + std::to_string(height) + "}";
    std::string result = send_http_post(url, body);
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

// {"obj": 1}
int BackgroundThread::set_obj(const std::string& url, int obj) {
    std::string body = "{\"id\": " + std::to_string(obj) + "}";
    std::string result = send_http_post(url, body);
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

int BackgroundThread::clean_obj(const std::string& url) {
    std::string body = "";
    std::string result = send_http_post(url, body);
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}
// {"x": 100, "y": 100, "width": 200, "height": 200}
std::string BackgroundThread::get_track_rect(const std::string& url) {
    std::string body = "";
    std::string result = send_http_post(url, body);
    return result;
}

int BackgroundThread::set_cloud_pitch(const std::string& url, int pitch) {
    // std::string url = "http://192.168.144.18:7081/api/set_cloud_pitch?degree=" + std::to_string(pitch);
    std::cout << "url: " << url << std::endl;
    std::string result = send_http_post(url_, "");
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

int BackgroundThread::set_yaw(const std::string& url, int yaw) {
    // std::string url = "http://192.168.144.18:7081/api/set_cloud_pitch?degree=" + std::to_string(pitch);
    std::cout << "url: " << url << std::endl;
    std::string result = send_http_post(url_, "");
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

int BackgroundThread::track_dive(const std::string& url, int speed) {
    // std::string url = "http://192.168.144.18:7081/api/track_dive?speed=" + std::to_string(speed);
    std::string result = send_http_post(url_, "");
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

int BackgroundThread::follow(const std::string& url, float keep_deg, float speed, float keep_z){
            // std::string url = "http://192.168.144.18:7081/api/follow?keep_deg=" + std::to_string(keep_deg) + "&speed=" + to_string(speed) + "&keep_z" + to_string(keep_z);
    std::cout << "url_: " << url_ << std::endl;
    std::string result = send_http_post(url_, "");
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

int BackgroundThread::set_mission(const std::string& url, int uav_id, float lat, float lon) {
    std::cout << "url_: " << url_ << std::endl;
    std::string result = send_http_post(url_, "");
    std::cout << "result: " << result << std::endl;
    if (result.empty())
    {
        return -1;
    }
    return 0;
}

void BackgroundThread::set_url_param(std::string& url) {
    url_ = url;
}

void BackgroundThread::set_obj_param(const std::string& url, int id) {
    url_ = url;
    id_= id;
}

void BackgroundThread::set_clean_param(const std::string& url) {
    url_ = url;
}

void BackgroundThread::set_rect_param(const std::string& url, int x, int y, int width, int height) {
    url_ = url;
    x_ = x;
    y_ = y;
    width_ = width;
    height_ = height;
}    

void BackgroundThread::set_cloud_pitch_param(const std::string& url, int pitch) {
    url_ = url + std::to_string(pitch);
    std::cout << url_ << std::endl;
    pitch_ = pitch;
}

void BackgroundThread::set_track_drve_param(const std::string& url, int speed) {
    url_ = url + std::to_string(speed);
    speed_ = speed;
    std::cout << url_ << std::endl;
}

void BackgroundThread::set_follow_param(const std::string& url, float keep_deg, float speed, float keep_z) {
    // std::string url = "http://192.168.144.18:7081/api/follow?keep_deg=" + std::to_string(keep_deg) + "&speed=" + to_string(speed) + "&keep_z" + to_string(keep_z);
    url_ = url + std::to_string(keep_deg) + "&speed=" + std::to_string(speed) + "&keep_z=" + std::to_string(keep_z);
    std::cout << url_ << std::endl;
    keep_deg_ = keep_deg;
    speed_ = speed;
    keep_z_ = keep_z;
}

void BackgroundThread::set_yaw_param(const std::string& url, int yaw) {
    url_ = url + std::to_string(yaw);
    std::cout << url_ << std::endl;
    yaw_ = yaw;
}

void BackgroundThread::set_mission_param(const std::string& url, int uav_id, float lat, float lon) {
    // std::string url = "http://192.168.144.18:7081/api/follow?keep_deg=" + std::to_string(keep_deg) + "&speed=" + to_string(speed) + "&keep_z" + to_string(keep_z);
    url_ = url + std::to_string(uav_id) + "&lat=" + std::to_string(lat) + "&lon=" + std::to_string(lon);
    std::cout << url_ << std::endl;
    uav_id_ = uav_id;
    lat_ = lat;
    lon_ = lon;
}

void BackgroundThread::set_obj_en() {
    set_obj_en_ = true;
    std::cout << "set_obj enable!" << std::endl;
}

void BackgroundThread::set_rect_en() {
    set_rect_en_ = true;
    std::cout << "set_rect enable!" << std::endl;
}

void BackgroundThread::set_pitch_en() {
    set_pitch_en_ = true;
    std::cout << "set_cloud_pitch enable!" << std::endl;
}

void BackgroundThread::set_clean_en() {
    set_clean_en_ = true;
    std::cout << "set_clean enable!" << std::endl;
}    

void BackgroundThread::set_track_drve_en() {
    set_track_drve_en_ = true;
    std::cout << "set_track_drve enable!" << std::endl;
}    

void BackgroundThread::set_follow_en() {
    set_follow_en_ = true;
    std::cout << "set_follow enable!" << std::endl;
}    

void BackgroundThread::set_yaw_en() {
    set_yaw_en_ = true;
    std::cout << "set_cloud_yaw enable!" << std::endl;
}

void BackgroundThread::set_mission_en() {
    set_mission_en_ = true;
    std::cout << "set_mission enable!" << std::endl;
}
} // namespace rviz_drone_control
