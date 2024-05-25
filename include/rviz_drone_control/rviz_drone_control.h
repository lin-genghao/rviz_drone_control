#ifndef RVIZ_DRONE_CONTROL_H
#define RVIZ_DRONE_CONTROL_H
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QString>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h> // 假设消息类型为mavros_msgs/State
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"


namespace rviz_drone_control{
    class UavButton :public QWidget{
        Q_OBJECT
    public:
        UavButton(ros::NodeHandle* nh, QWidget *parent = nullptr,  const QString& id = QString("uav0")); // 构造函数
        QHBoxLayout* get_layout();
        QWidget* get_button_widget();
        void set_Visible(bool visible);

    protected Q_SLOTS:
        void takeoff_callback();
        void launch_callback();
        void return_home_callback();
        void land_callback();
        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg);

    private:
        QPushButton *takeoff_button_;
        QPushButton *launch_button_;
        QPushButton *return_home_button_;
        QPushButton *land_button_;
        QHBoxLayout *layout_; // 布局对象作为成员变量
        QGroupBox* group_box_; // 存储QGroupBox的指针 

        ros::NodeHandle nh_;
        ros::Publisher takeoff_pub_;
        ros::Publisher launch_pub_;
        ros::Publisher return_home_pub_;
        ros::Publisher land_pub_;

        ros::Subscriber mavros_state_sub_;
        ros::Subscriber mavros_home_sub_;
        std::string uav_id_;
        mavros_msgs::State current_state;
        mavros_msgs::HomePosition home_position;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::SetMode offb_set_mode;
        std::string id_string;
    };

    class RvizDroneControl :public rviz::Panel{
    Q_OBJECT
    public:
        RvizDroneControl(QWidget *parent = 0);
        virtual void load(const rviz::Config &config);
        virtual void save(rviz::Config config) const;
    protected Q_SLOTS:
        void test_callback();
        void test2_callback();
        // void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    private:
        ros::NodeHandle nh_;
        QPushButton *test_button_;
        QPushButton *test2_button_;
        ros::Publisher test_pub_, test2_pub_;
        ros::Subscriber mavros_state_sub_;
        bool uav1_connected = false;
        bool uav2_connected = false;

        UavButton *uav0_buttons_;
        UavButton *uav1_buttons_;
        UavButton *uav2_buttons_;

    };
}

#endif //RVIZ_DRONE_CONTROL_H
