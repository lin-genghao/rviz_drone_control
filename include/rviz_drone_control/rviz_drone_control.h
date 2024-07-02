#ifndef RVIZ_DRONE_CONTROL_H
#define RVIZ_DRONE_CONTROL_H
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QString>
#include <QComboBox>
#include <QObject>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/State.h> // 假设消息类型为mavros_msgs/State
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamPush.h>
#include <mavros_msgs/MessageInterval.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "tf/transform_datatypes.h"
#include "rviz_drone_control/httplib.h"
#include <iostream>
#include <string>
#include <vector>
#include <QProcess>
#include <QThread>
#include <cstdlib> // 对于 std::stoi

namespace rviz_drone_control{
    class BackgroundWorker : public QThread{
    public:
        struct Requst{
            std::string host;
            std::string uri;
        };

        BackgroundWorker();
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

    class UavButton :public QWidget{
        Q_OBJECT
    public:
        std::string url;
        std::string url_0;
        std::string url_1;
        UavButton(ros::NodeHandle* nh, QWidget *parent = nullptr,  const QString& id = QString("uav0")); // 构造函数
        QHBoxLayout* get_layout();
        QWidget* get_button_widget();
        void set_Visible(bool visible);

        void send_reposition_command(double x, double y, double z, double yaw);
        double paramGet(std::string param_str);
        double paramSet(std::string param_str, double vaule);
        BackgroundWorker* threads;

        void set_launch_en();
        double mission_alt_;
        int launch_delay_;
        ros::Time time_start;
        ros::Time time_now;

    protected Q_SLOTS:
        void takeoff_callback();
        void launch_callback();
        void strike_callback();
        void strike_clean_callback();
        void track_callback();
        void follow_callback();
        void return_home_callback();
        void land_callback();
        void gimbal_up_callback();
        void gimbal_down_callback();

        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg);
        void mavrosGPSCallback(const mavros_msgs::GPSRAW::ConstPtr &msg);
        void mavrosCompassCallback(const std_msgs::Float64::ConstPtr &msg);
        void LocalPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void djiPositionCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void boxSelectCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
        
    private:
        QPushButton *takeoff_button_;
        QLineEdit *mission_alt_edit_;
        QPushButton *launch_button_;
        QLineEdit *strike_id_edit_;
        QPushButton *strike_button_;
        QPushButton *strike_clean_button_;
        QPushButton *track_button_;
        QPushButton *follow_button_;
        QLineEdit *return_home_edit_;
        QPushButton *return_home_button_;
        QPushButton *land_button_;
        QHBoxLayout *layout_; // 布局对象作为成员变量
        QGroupBox* group_box_; // 存储QGroupBox的指针 

        ros::NodeHandle nh_;
        ros::Publisher takeoff_pub_;
        ros::Publisher launch_pub_;
        ros::Publisher return_home_pub_;
        ros::Publisher land_pub_;

        bool takeoff_en_ = false;
        bool launch_en_ = false;
        bool strike_en_ = false;
        bool track_en_ = false;
        bool follow_en_ = false;
        bool return_home_en_ = false;
        bool land_en_ = false;
        
        int waypoints_flag;

        double return_home_alt_;
        int strike_id_;

        std::string uav_id_;
        int uav_id_num_;

        std::string url_prefix;
        std::string url_suffix;
        int last_part_of_ip;
        int port_0;
        int port_1;
        int port_2;
        double gimbal_pitch_;

        ros::Subscriber mavros_state_sub_;
        ros::Subscriber mavros_home_sub_;
        ros::Subscriber mavros_gps_sub_;
        ros::Subscriber mavros_compass_sub_;
        ros::Subscriber dji_position_sub_;

        double current_lat;
        double current_long;
        double current_alt;
        double current_yaw;

        mavros_msgs::State current_state;
        mavros_msgs::HomePosition home_position;
        geometry_msgs::TwistStamped current_velocity;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient param_get_client_;
        ros::ServiceClient param_set_client_;
        ros::ServiceClient param_pull_client_;
        ros::ServiceClient param_push_client_;
        ros::ServiceClient command_client_;
        mavros_msgs::CommandLong command_;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::SetMode offb_set_mode;
        std::string id_string;
        sensor_msgs::NavSatFix dji_position;

        ros::Subscriber box_select_sub_;

    };

    class RVIZ_EXPORT RvizDroneControl :public rviz::Panel  {
    Q_OBJECT
    public:
        RvizDroneControl(QWidget *parent = 0);
        virtual void load(const rviz::Config &config);
        virtual void save(rviz::Config config) const;
        std::string uav_select_;
        
    protected Q_SLOTS:
        void test_callback();
        void test2_callback();
        void start_callback();
        void up_callback();
        void turn_left_callback();
        void turn_right_callback();
        void down_callback();
        void front_callback();
        void left_callback();
        void right_callback();
        void back_callback();
        void initManualControlPublisher();
        void uavSelected_callback(int index); // 无人机选择框的槽函数
        void manual_control_callback(const mavros_msgs::ManualControl::ConstPtr &msg); // 无人机选择框的槽函数
        // void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    private:
        ros::NodeHandle nh_;
        QPushButton *test_button_;
        QPushButton *test2_button_;
        QPushButton *start_button_;

        ros::Publisher test_pub_, test2_pub_;
        ros::Publisher mavros_manual_control_pub_;
        ros::Subscriber mavros_state_sub_;
        ros::Subscriber manual_control_sub_;
        ros::Subscriber local_pos_sub;
        ros::Subscriber body_velocity_sub;
        ros::Publisher vec_pub;

        bool uav1_connected = false;
        bool uav2_connected = false;

        UavButton *uav0_buttons_;
        UavButton *uav1_buttons_;
        UavButton *uav2_buttons_;
        QComboBox *uav_combo_box_; // 新增的无人机选择框

        int pitch_;
        int yaw_;
    };
}

#endif //RVIZ_DRONE_CONTROL_H
