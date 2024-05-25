#ifndef UAV_BUTTON_H
#define UAV_BUTTON_H

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QString>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

namespace uav_button{
    class UavButton :public QWidget{
        Q_OBJECT
    public:
        UavButton(ros::NodeHandle& nh, QWidget *parent = nullptr); // 构造函数

    protected Q_SLOTS:
        void takeoff_callback();
        void return_home_callback();
        void land_callback();
        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);

    private:
        QPushButton *takeoff_button_;
        QPushButton *return_home_button_;
        QPushButton *land_button_;
        QHBoxLayout *layout_; // 布局对象作为成员变量
        ros::NodeHandle& nh_;
        ros::Publisher takeoff_pub_;
        ros::Publisher return_home_pub_;
        ros::Publisher land_pub_;
        std::string uav_id_;
        mavros_msgs::State current_state;
        ros::Subscriber mavros_state_sub_;
    };
}

#endif // UAV_BUTTON_H