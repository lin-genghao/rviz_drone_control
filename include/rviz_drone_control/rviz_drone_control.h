#ifndef PROJECT_TEST_RVIZ_H
#define PROJECT_TEST_RVIZ_H
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QHBoxLayout>
#include <QPushButton>
#include <QString>
#include <mavros_msgs/State.h> // 假设消息类型为mavros_msgs/State

namespace rviz_drone_control{
    class RvizDroneControl :public rviz::Panel{
    Q_OBJECT
    public:
        RvizDroneControl(QWidget *parent = 0);
        virtual void load(const rviz::Config &config);
        virtual void save(rviz::Config config) const;
    protected Q_SLOTS:
        void test_callback();
        void test2_callback();
        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    private:
        ros::NodeHandle nh_;
        QPushButton *test_button_;
        QPushButton *test2_button_;
        ros::Publisher test_pub_, test2_pub_;
        ros::Subscriber mavros_state_sub_;
    };
}

#endif //PROJECT_TEST_RVIZ_H
