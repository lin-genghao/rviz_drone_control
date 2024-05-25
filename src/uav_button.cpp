#include "rviz_drone_control/uav_button.h"

namespace uav_button{
    UavButton::UavButton(ros::NodeHandle& nh, QWidget *parent) : nh_() {
        // uav_state = UavState("uav0");
        // 创建起飞、返航、降落按钮
        takeoff_button_ = new QPushButton(tr("起飞"), parent);
        return_home_button_ = new QPushButton(tr("返航"), parent);
        land_button_ = new QPushButton(tr("降落"), parent);

        // // 为按钮创建水平布局
        layout_ = new QHBoxLayout;
        layout_->addWidget(takeoff_button_);
        layout_->addWidget(return_home_button_);
        layout_->addWidget(land_button_);

        setLayout(layout_);
        // 连接按钮信号到槽函数
        connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(takeoff_callback()));
        connect(return_home_button_, SIGNAL(clicked()), this, SLOT(return_home_callback()));
        connect(land_button_, SIGNAL(clicked()), this, SLOT(land_callback()));

        // 为起飞、返航、降落创建ROS发布者
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>("takeoff_topic", 1);
        return_home_pub_ = nh_.advertise<std_msgs::Empty>("return_home_topic", 1);
        land_pub_ = nh_.advertise<std_msgs::Empty>("land_topic", 1);

        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, &UavButton::mavrosStateCallback, this);
    }

    void UavButton::takeoff_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Takeoff button clicked.");
        takeoff_pub_.publish(empty_msg);
    }

    void UavButton::return_home_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Return Home button clicked.");
        return_home_pub_.publish(empty_msg);
    }

    void UavButton::land_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Land button clicked.");
        land_pub_.publish(empty_msg);
    }


    void UavButton::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
        ROS_INFO("uav_state: ", uav_id_);
    }


}
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(uav_button::UavButton, QWidget)