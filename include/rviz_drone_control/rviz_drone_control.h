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
#include <mavros_msgs/State.h> // 假设消息类型为mavros_msgs/State
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamPush.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/NavSatFix.h>
#include "tf/transform_datatypes.h"
#include "rviz_drone_control/httplib.h"
#include <iostream>
#include <string>
#include <vector>


namespace rviz_drone_control{
    class UavButton :public QWidget{
        Q_OBJECT
    public:
        struct Requst{
            std::string host;
            std::string uri;
        };

        UavButton(ros::NodeHandle* nh, QWidget *parent = nullptr,  const QString& id = QString("uav0")); // 构造函数
        QHBoxLayout* get_layout();
        QWidget* get_button_widget();
        void set_Visible(bool visible);

        Requst ParseUrl(const std::string& url);
        int HttpPost(const std::string& url, const std::string& body, std::string& result);
        std::string send_http_post(const std::string& url, const std::string& body);
        int set_rect(const std::string& url, int x, int y, int width, int height); 
        int set_obj(const std::string& url, int obj);
        int clean_obj(const std::string& url);
        std::string get_track_rect(const std::string& url);

        double paramGet(std::string param_str);
        double paramSet(std::string param_str, double vaule);

    protected Q_SLOTS:
        void takeoff_callback();
        void launch_callback();
        void strike_callback();
        void return_home_callback();
        void land_callback();
        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg);
        void djiPositionCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void boxSelectCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

    private:
        QPushButton *takeoff_button_;
        QLineEdit *mission_alt_edit_;
        double mission_alt_;
        QPushButton *launch_button_;
        QLineEdit *strike_id_edit_;
        QPushButton *strike_button_;
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
        bool return_home_en_ = false;
        bool land_en_ = false;

        double return_home_alt_;

        ros::Subscriber mavros_state_sub_;
        ros::Subscriber mavros_home_sub_;
        ros::Subscriber dji_position_sub_;
        std::string uav_id_;
        mavros_msgs::State current_state;
        mavros_msgs::HomePosition home_position;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient param_get_client_;
        ros::ServiceClient param_set_client_;
        ros::ServiceClient param_pull_client_;
        ros::ServiceClient param_push_client_;
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
    protected Q_SLOTS:
        void test_callback();
        void test2_callback();
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
        ros::Publisher test_pub_, test2_pub_;
        ros::Publisher mavros_manual_control_pub_;
        ros::Subscriber mavros_state_sub_;
        ros::Subscriber manual_control_sub_;
        bool uav1_connected = false;
        bool uav2_connected = false;

        UavButton *uav0_buttons_;
        UavButton *uav1_buttons_;
        UavButton *uav2_buttons_;
        QComboBox *uav_combo_box_; // 新增的无人机选择框

        std::string uav_select_;
    };
}

#endif //RVIZ_DRONE_CONTROL_H
