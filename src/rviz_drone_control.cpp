#include <rviz_drone_control/rviz_drone_control.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h> // 假设消息类型为mavros_msgs/State


namespace rviz_drone_control{
    RvizDroneControl::RvizDroneControl(QWidget *parent):Panel(parent)
    {
    	//创建两个按钮---测试按钮和测试按钮2
        auto *button_layout = new QHBoxLayout;
        test_button_ = new QPushButton(tr("测试按钮"),this);
        button_layout->addWidget(test_button_);
        test2_button_ = new QPushButton(tr("测试按钮2"),this);
        button_layout->addWidget(test2_button_);
        // 初始化按钮状态为不可见
        test_button_->setVisible(false);
        test2_button_->setVisible(false);
        setLayout(button_layout);
        
        //信号连接---点击信号发生，连接到槽函数test_callback()
        connect(test_button_,SIGNAL(clicked()),this,SLOT(test_callback()));
        connect(test2_button_,SIGNAL(clicked()),this,SLOT(test2_callback()));
		
        test_pub_ = nh_.advertise<std_msgs::String>("test_topic",10);
        test2_pub_ = nh_.advertise<std_msgs::String>("test2_topic",10);
        // 创建订阅者
        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &RvizDroneControl::mavrosStateCallback, this);
    }
    //加载配置数据---必须要有的
    void RvizDroneControl::load(const rviz::Config &config){
        Panel::load(config);
    }
    //将所有配置数据保存到给定的Config对象中。在这里，重要的是要对父类调用save（），以便保存类id和面板名称。---必须要有的
    void RvizDroneControl::save(rviz::Config config) const{
        Panel::save(config);
    }

    void RvizDroneControl::test_callback()
    {
        std::string str;
        std_msgs::String msg;
        msg.data = "test";
        ROS_INFO("测试消息发布！");
        test_pub_.publish(msg);
    }
    void RvizDroneControl::test2_callback()
    {
        std::string str;
        std_msgs::String msg;
        msg.data = "test2";
        ROS_INFO("测试消息2发布！");
        test_pub_.publish(msg);
    }
    // 回调函数
    void RvizDroneControl::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // 处理接收到的mavros_msgs::State消息
        uint8_t connected = 0;
        connected = msg->connected;
        ROS_INFO("connected:%d", msg->connected);
        if (connected) {
            // 如果无人机连接成功，则启用按钮
            // 如果无人机连接成功，则显示按钮
            test_button_->setVisible(true);
            test2_button_->setVisible(true);
            ROS_INFO("Drone connected. Buttons visible.");
        } else {
            // 如果无人机未连接，则隐藏按钮
            test_button_->setVisible(false);
            test2_button_->setVisible(false);
            ROS_INFO("Drone not connected. Buttons hidden.");
        }   

        // 这里添加处理消息的代码
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_drone_control::RvizDroneControl,rviz::Panel)
