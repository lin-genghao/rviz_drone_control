#include <rviz_drone_control/rviz_drone_control.h>


namespace rviz_drone_control{
    RvizDroneControl::RvizDroneControl(QWidget *parent):Panel(parent)
    {
    	//创建两个按钮---测试按钮和测试按钮2
        auto *button_layout = new QVBoxLayout;

        test_button_ = new QPushButton(tr("测试按钮"),this);
        test2_button_ = new QPushButton(tr("测试按钮2"),this);

        // button_layout->addWidget(test_button_);
        // button_layout->addWidget(test2_button_);

        // 初始化按钮状态为不可见
        test_button_->setVisible(false);
        test2_button_->setVisible(false);
        setLayout(button_layout);
        
        //信号连接---点击信号发生，连接到槽函数test_callback()
        connect(test_button_,SIGNAL(clicked()),this,SLOT(test_callback()));
        connect(test2_button_,SIGNAL(clicked()),this,SLOT(test2_callback()));
		
        test_pub_ = nh_.advertise<std_msgs::String>("test_topic",10);
        test2_pub_ = nh_.advertise<std_msgs::String>("test2_topic",10);

        // 创建UavButton实例
        uav0_buttons_ = new UavButton(&nh_, this, "uav0");
        uav1_buttons_ = new UavButton(&nh_, this, "uav1");
        uav2_buttons_ = new UavButton(&nh_, this, "uav2");

        uav0_buttons_->set_Visible(true);
        uav1_buttons_->set_Visible(true);
        uav2_buttons_->set_Visible(true);

        QWidget* uav0_widget = uav0_buttons_->get_button_widget();
        QWidget* uav1_widget = uav1_buttons_->get_button_widget();
        QWidget* uav2_widget = uav2_buttons_->get_button_widget();

        // 为每个UavButton创建垂直布局
        QVBoxLayout *uav0_layout = new QVBoxLayout;
        QVBoxLayout *uav1_layout = new QVBoxLayout;
        QVBoxLayout *uav2_layout = new QVBoxLayout;

        // 将UavButton的按钮添加到相应的布局中
        uav0_layout->addWidget(uav0_widget);
        uav1_layout->addWidget(uav1_widget);
        uav2_layout->addWidget(uav2_widget);

        // 将垂直布局添加到主布局中
        button_layout->addLayout(uav0_layout);
        button_layout->addLayout(uav1_layout);
        button_layout->addLayout(uav2_layout);
        
    }
    //加载配置数据---必须要有的
    void RvizDroneControl::load(const rviz::Config &config){
        Panel::load(config);
        ROS_INFO("load...");

        uav1_connected = false;
    }
    //将所有配置数据保存到给定的Config对象中。在这里，重要的是要对父类调用save（），以便保存类id和面板名称。---必须要有的
    void RvizDroneControl::save(rviz::Config config) const{
        Panel::save(config);
        ROS_INFO("save...");

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

    UavButton::UavButton(ros::NodeHandle* nh, QWidget *parent, const QString& id) : nh_() {
        // 创建一个QGroupBox
        nh_ = *nh;
        group_box_ = new QGroupBox(id, parent);

        // 创建起飞、返航、降落按钮
        takeoff_button_ = new QPushButton(tr("起飞"), parent);
        launch_button_ = new QPushButton(tr("发射"), parent);
        return_home_button_ = new QPushButton(tr("返航"), parent);
        land_button_ = new QPushButton(tr("降落"), parent);

        // // 为按钮创建水平布局
        layout_ = new QHBoxLayout;
        layout_->addWidget(takeoff_button_);
        layout_->addWidget(launch_button_);
        layout_->addWidget(return_home_button_);
        layout_->addWidget(land_button_);

        group_box_->setLayout(layout_);

        group_box_->setVisible(false);

        // 连接按钮信号到槽函数
        connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(takeoff_callback()));
        connect(launch_button_, SIGNAL(clicked()), this, SLOT(launch_callback()));
        connect(return_home_button_, SIGNAL(clicked()), this, SLOT(return_home_callback()));
        connect(land_button_, SIGNAL(clicked()), this, SLOT(land_callback()));

        id_string = id.toStdString();
        // 为起飞、发射、返航、降落创建ROS发布者
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/takeoff_topic", 1);
        launch_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/launch_topic", 1);
        return_home_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/return_home_topic", 1);
        land_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/land_topic", 1);

        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(id_string + "/mavros/state", 10, &UavButton::mavrosStateCallback, this);
        mavros_home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>(id_string + "/mavros/home_position/home", 10, &UavButton::mavrosHomeCallback, this);
    
        dji_position_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/dji/gps", 10, &UavButton::djiPositionCallBack, this);
        // 服务的客户端（设定无人机的模式、状态）
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
                (id_string + "/mavros/cmd/arming");
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
                (id_string + "/mavros/set_mode");

        offb_set_mode.request.custom_mode = "POSCTL";

        arm_cmd.request.value = true;
    }

    // 返回包含按钮的QWidget对象
    QWidget* UavButton::get_button_widget() {
        return group_box_;
    }

    QHBoxLayout* UavButton::get_layout() {
        return layout_;
    }

    void UavButton::set_Visible(bool visible) {
        group_box_->setVisible(visible);
    }

    void UavButton::takeoff_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Takeoff button clicked.");
        takeoff_pub_.publish(empty_msg);

        // 3. 设置飞行模式为定点模式
        if (current_state.mode != "POSCTL") {
            // 假设"AUTO.TAKEOFF"是您的起飞模式
            offb_set_mode.request.custom_mode = "POSCTL";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("POSCTL mode enabled");
            } else {
                ROS_ERROR("Failed to set POSCTL mode");
                return; // 如果无法设置模式，则退出函数
            }
        }

        // 1. 确保无人机连接并处于正确的模式
        if (!current_state.armed) {
            // 2. 发布ARM命令
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            } else {
                ROS_ERROR("Failed to arm vehicle");
                return; // 如果无法武装无人机，则退出函数
            }
        }

        // 3. 设置飞行模式为起飞模式
        if (current_state.mode != "AUTO.TAKEOFF") {
            offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("TAKEOFF mode enabled");
            } else {
                ROS_ERROR("Failed to set TAKEOFF mode");
                return; // 如果无法设置模式，则退出函数
            }
        }

    }

    void UavButton::launch_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Launch button clicked.");
        // 定义服务响应类型
        // 创建航点清除服务的响应对象
        mavros_msgs::WaypointClear srv;

        // 调用清除航点服务
        if (ros::service::call(id_string + "/mavros/mission/clear", srv)) {
            ROS_INFO("Waypoints cleared successfully before launch.");
        } else {
            ROS_ERROR("Failed to call clear waypoints service before launch.");
            return; // 如果无法清除航点，则退出函数
        }

        launch_pub_.publish(empty_msg);

        // 创建航点列表
        std::vector<mavros_msgs::Waypoint> waypoints;

        // 创建一个航点并设置其参数
        mavros_msgs::Waypoint waypoint;
        waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT; // 例如，使用全局相对高度坐标系
        waypoint.command = mavros_msgs::CommandCode::NAV_WAYPOINT; // 航点命令为导航到指定位置
        waypoint.is_current = false;
        waypoint.autocontinue = true; // 自动继续执行下一个航点
        waypoint.param1 = 0.0;
        waypoint.param2 = 0.0;
        waypoint.param3 = 0.0;
        waypoint.param4 = NAN;
        waypoint.z_alt = 15; // 航点的相对高度

        // waypoint.x_lat = 23.1771954;
        // waypoint.y_long = 112.5778758;
        // waypoints.push_back(waypoint);

        // waypoint.x_lat = 23.1770976;
        // waypoint.y_long = 112.5782201;
        // waypoints.push_back(waypoint);

        // waypoint.x_lat = 23.1777657;
        // waypoint.y_long = 112.5783523;
        // waypoints.push_back(waypoint);

        // waypoint.x_lat = 23.1778295;
        // waypoint.y_long = 112.5782005;
        // waypoints.push_back(waypoint);

        // waypoint.x_lat = 23.1774136;
        // waypoint.y_long = 112.5781111;
        // waypoints.push_back(waypoint);

        waypoint.x_lat = 23.1967469;
        waypoint.y_long = 112.5821592;
        waypoints.push_back(waypoint);

        waypoint.x_lat = 23.1963283;
        waypoint.y_long = 112.5829652;
        waypoints.push_back(waypoint);

        waypoint.x_lat = 23.1970022;
        waypoint.y_long = 112.5834441;
        waypoints.push_back(waypoint);

        waypoint.x_lat = 23.1974289;
        waypoint.y_long = 112.5827578;
        waypoints.push_back(waypoint);

        waypoint.x_lat = 23.1968588;
        waypoint.y_long = 112.5825387;
        waypoints.push_back(waypoint);

        // waypoint.x_lat = dji_position.latitude;
        // waypoint.y_long = dji_position.longitude;
        // waypoint.x_lat = 23.1772709;
        // waypoint.y_long = 112.5781292;

        // 将航点添加到列表中
        // waypoints.push_back(waypoint);

        // 调用服务以推送航点
        mavros_msgs::WaypointPush wp_push;
        wp_push.request.waypoints = waypoints;

        if (ros::service::call(id_string + "/mavros/mission/push", wp_push)) {
            ROS_INFO("Waypoints pushed successfully.");
        } else {
            ROS_ERROR("Failed to push waypoints.");
        }

            // 3. 设置飞行模式为任务模式
        if (current_state.mode != "AUTO.MISSION") {
            offb_set_mode.request.custom_mode = "AUTO.MISSION";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("MISSION mode enabled");
            } else {
                ROS_ERROR("Failed to set MISSION mode");
                return; // 如果无法设置模式，则退出函数
            }
        }
    }

    void UavButton::return_home_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Return Home button clicked.");
        return_home_pub_.publish(empty_msg);
        // 3. 设置飞行模式为返航模式
        if (current_state.mode != "AUTO.RTL") {
            offb_set_mode.request.custom_mode = "AUTO.RTL";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("RTL mode enabled");
            } else {
                ROS_ERROR("Failed to set RTL mode");
                return; // 如果无法设置模式，则退出函数
            }
        }

    }

    void UavButton::land_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Land button clicked.");
        land_pub_.publish(empty_msg);
        // 设置飞行模式为降落模式
        if (current_state.mode != "AUTO.LAND") {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("LAND mode enabled");
            } else {
                ROS_ERROR("Failed to set LAND mode");
                return; // 如果无法设置模式，则退出函数
            }
        }
    }


    void UavButton::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
        // ROS_INFO("uav_state: %s", uav_id_.c_str());
    }

    void UavButton::mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg){
        home_position.geo = msg->geo;
        home_position.position = msg->position;
        home_position.orientation = msg->orientation;
        // ROS_INFO("%s home lat: %f\t lon: %f\t alt: %f",id_string.c_str(), home_position.geo.latitude, home_position.geo.longitude, home_position.position.z);
    }

    void UavButton::djiPositionCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg){
        dji_position = *msg;
        ROS_INFO("lat: %f\t lon: %f\t alt: %f", dji_position.latitude, dji_position.longitude, dji_position.altitude);
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_drone_control::RvizDroneControl,rviz::Panel)
