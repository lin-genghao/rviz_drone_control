#include <rviz_drone_control/rviz_drone_control.h>


namespace rviz_drone_control{
    RvizDroneControl::RvizDroneControl(QWidget *parent):Panel(parent)
    {
        manual_control_sub_ = nh_.subscribe<mavros_msgs::ManualControl>("/manual_control", 1, &RvizDroneControl::manual_control_callback, this);
        
        // body_velocity_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &RvizDroneControl::LocalPoseCallBack, this);

        // vec_pub = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

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

        // 创建无人机选择框并初始化
        uav_combo_box_ = new QComboBox(this);
        uav_combo_box_->addItem("None");
        uav_combo_box_->addItem("UAV 1");
        uav_combo_box_->addItem("UAV 2");
        uav_combo_box_->addItem("UAV 3");

        // 将无人机选择框添加到主布局
        button_layout->addWidget(uav_combo_box_);

        uav_select_ = "/uav";
        initManualControlPublisher();

        // 连接选择框的信号到槽函数
        connect(uav_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(uavSelected_callback(int)));

        auto *all_function_layout = new QHBoxLayout;

        start_button_ = new QPushButton(tr("start"), this);
        connect(start_button_,SIGNAL(clicked()),this,SLOT(start_callback()));
        all_function_layout->addWidget(start_button_);

        button_layout->addLayout(all_function_layout);

        // 将垂直布局添加到主布局中
        // button_layout->addLayout(uav0_layout);
        button_layout->addLayout(uav1_layout);
        button_layout->addLayout(uav2_layout);

        auto* button_move = new QHBoxLayout;

        // 创建用于放置菱形按钮的布局
        auto *diamond_layout = new QVBoxLayout;

        // 菱形的上、下按钮
        auto *top_layout = new QHBoxLayout;
        QPushButton *up_button = new QPushButton(tr("Up"), this);
        top_layout->addStretch(1); // 左侧添加伸展空间
        top_layout->addWidget(up_button);
        top_layout->addStretch(1); // 左侧添加伸展空间
        diamond_layout->addLayout(top_layout);

        // 菱形的左侧、右侧按钮
        auto *turn_left_right_layout = new QHBoxLayout;
        QPushButton *turn_left_button = new QPushButton(tr("Turn Left"), this);
        QPushButton *turn_right_button = new QPushButton(tr("Turn Right"), this);
        turn_left_right_layout->addStretch(1); // 右侧添加伸展空间
        turn_left_right_layout->addWidget(turn_left_button);
        turn_left_right_layout->addStretch(1); // 右侧添加伸展空间
        turn_left_right_layout->addWidget(turn_right_button);
        turn_left_right_layout->addStretch(1); // 右侧添加伸展空间
        diamond_layout->addLayout(turn_left_right_layout);

        auto *bottom_layout = new QHBoxLayout;
        QPushButton *down_button = new QPushButton(tr("Down"), this);
        bottom_layout->addStretch(1); // 左侧添加伸展空间
        bottom_layout->addWidget(down_button);
        bottom_layout->addStretch(1); // 左侧添加伸展空间
        diamond_layout->addLayout(bottom_layout);

        // 创建用于放置菱形按钮的布局
        auto *diamond_layout_2 = new QVBoxLayout;

        // 菱形的上、下按钮
        auto *front_layout = new QHBoxLayout;
        QPushButton *front_button = new QPushButton(tr("Front"), this);
        front_layout->addStretch(1); // 左侧添加伸展空间
        front_layout->addWidget(front_button);
        front_layout->addStretch(1); // 左侧添加伸展空间
        diamond_layout_2->addLayout(front_layout);

        // 菱形的左侧、右侧按钮
        auto *left_right_layout = new QHBoxLayout;
        QPushButton *left_button = new QPushButton(tr("Left"), this);
        QPushButton *right_button = new QPushButton(tr("Right"), this);
        left_right_layout->addStretch(1); // 右侧添加伸展空间
        left_right_layout->addWidget(left_button);
        left_right_layout->addStretch(1); // 右侧添加伸展空间
        left_right_layout->addWidget(right_button);
        left_right_layout->addStretch(1); // 右侧添加伸展空间
        diamond_layout_2->addLayout(left_right_layout);

        auto *back_layout = new QHBoxLayout;
        QPushButton *back_button = new QPushButton(tr("Back"), this);
        back_layout->addStretch(1); // 左侧添加伸展空间
        back_layout->addWidget(back_button);
        back_layout->addStretch(1); // 左侧添加伸展空间
        diamond_layout_2->addLayout(back_layout);

        // 将菱形布局添加到主布局中
        button_move->addLayout(diamond_layout);
        button_move->addLayout(diamond_layout_2);
        up_button->setVisible(true);
        turn_left_button->setVisible(true);
        turn_right_button->setVisible(true);
        down_button->setVisible(true);
        front_button->setVisible(false);
        left_button->setVisible(false);
        right_button->setVisible(false);
        back_button->setVisible(false);

        button_layout->addLayout(button_move);

        // button_move->setVisible(false);
        
        connect(up_button,SIGNAL(clicked()),this,SLOT(up_callback()));
        connect(turn_left_button,SIGNAL(clicked()),this,SLOT(turn_left_callback()));
        connect(turn_right_button,SIGNAL(clicked()),this,SLOT(turn_right_callback()));
        connect(down_button,SIGNAL(clicked()),this,SLOT(down_callback()));
        connect(front_button,SIGNAL(clicked()),this,SLOT(front_callback()));
        connect(left_button,SIGNAL(clicked()),this,SLOT(left_callback()));
        connect(right_button,SIGNAL(clicked()),this,SLOT(right_callback()));

        pitch_ = 0;
    }
    void RvizDroneControl::start_callback() {
        ROS_INFO("start");
	
        uav2_buttons_->set_launch_en();
        // uav0_buttons_->set_launch_en();
        uav1_buttons_->set_launch_en();
        ROS_INFO("uav1 started");
        sleep(5);
        ROS_INFO("uav2 started");
    }  
    void RvizDroneControl::up_callback() {
        ROS_INFO("up, %s", uav_select_.c_str());
        std::string url;
        if(uav_select_ == "/uav1") {
            ROS_INFO("uav1 up");
            url = uav1_buttons_->url_1 + "/api/set_cloud_pitch?degree=";
            pitch_=-5;
            uav1_buttons_->threads->set_cloud_pitch_param(url, pitch_);
            uav1_buttons_->threads->set_pitch_en();
        }
        if(uav_select_ == "/uav2") {
            url = uav2_buttons_->url_1 + "/api/set_cloud_pitch?degree=";
            pitch_=-5;
            uav2_buttons_->threads->set_cloud_pitch_param(url, pitch_);
            uav2_buttons_->threads->set_pitch_en();
        }

    }
    void RvizDroneControl::turn_left_callback() {
        ROS_INFO("left, %s", uav_select_.c_str());
        std::string url;
        if(uav_select_ == "/uav1") {
            ROS_INFO("uav1 left");
            url = uav1_buttons_->url_1 + "/api/rotation?degree=";
            yaw_=10;
            uav1_buttons_->threads->set_yaw_param(url, yaw_);
            uav1_buttons_->threads->set_yaw_en();
        }
        if(uav_select_ == "/uav2") {
            url = uav2_buttons_->url_1 + "/api/rotation?degree=";
            yaw_=10;
            uav2_buttons_->threads->set_yaw_param(url, yaw_);
            uav2_buttons_->threads->set_yaw_en();
        }
    }
    void RvizDroneControl::turn_right_callback() {
        ROS_INFO("right, %s", uav_select_.c_str());
        std::string url;
        if(uav_select_ == "/uav1") {
            ROS_INFO("uav1 right");
            url = uav1_buttons_->url_1 + "/api/rotation?degree=";
            yaw_=-10;
            uav1_buttons_->threads->set_yaw_param(url, yaw_);
            uav1_buttons_->threads->set_yaw_en();
        }
        if(uav_select_ == "/uav2") {
            url = uav2_buttons_->url_1 + "/api/rotation?degree=";
            yaw_=-10;
            uav2_buttons_->threads->set_yaw_param(url, yaw_);
            uav2_buttons_->threads->set_yaw_en();
        }
    }
    void RvizDroneControl::down_callback() {
        ROS_INFO("down");
        std::string url;

        if(uav_select_ == "/uav1") {
            url = uav1_buttons_->url_1 + "/api/set_cloud_pitch?degree=";
            pitch_=5;
            uav1_buttons_->threads->set_cloud_pitch_param(url, pitch_);
            uav1_buttons_->threads->set_pitch_en();
        }
        if(uav_select_ == "/uav2") {
            url = uav2_buttons_->url_1 + "/api/set_cloud_pitch?degree=";
            pitch_=5;
            uav2_buttons_->threads->set_cloud_pitch_param(url, pitch_);
            uav2_buttons_->threads->set_pitch_en();
        }
    }
    void RvizDroneControl::front_callback() {

    }
    void RvizDroneControl::left_callback() {

    }
    void RvizDroneControl::right_callback() {

    }
    void RvizDroneControl::back_callback() {

    }
    void RvizDroneControl::initManualControlPublisher() {
        // 根据当前的uav_select_创建或重新初始化发布者
        std::string topic_name = uav_select_ + "/mavros/manual_control/send";
        mavros_manual_control_pub_ = nh_.advertise<mavros_msgs::ManualControl>(topic_name, 10);
    }
    void RvizDroneControl::uavSelected_callback(int index) {
        // 根据选择框的索引选择相应的UavButton
        switch(index) {
            case 0:
                ROS_INFO("None");
                uav_select_ = "/uav";
                break;
            case 1:
                ROS_INFO("uav 1");
                uav_select_ = "/uav1";
                break;
            case 2:
                ROS_INFO("uav 2");
                uav_select_ = "/uav2";
                break;
            case 3:
                ROS_INFO("uav 3");
                uav_select_ = "/uav3";
                break;
        }
        initManualControlPublisher();
    }
    
    void RvizDroneControl::manual_control_callback(const mavros_msgs::ManualControl::ConstPtr &msg){
        mavros_msgs::ManualControl manual_control_msg_pub = *msg;
        // mavros_manual_control_pub_.publish(manual_control_msg_pub);
        // ROS_INFO("--------manual_control %s", uav_select_.c_str());
        
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

        mission_alt_edit_ = new QLineEdit(parent);
        mission_alt_edit_->setText("30");
        mission_alt_ = 30.0;
        launch_button_ = new QPushButton(tr("start"), parent);

        strike_id_edit_ = new QLineEdit(parent);
        strike_id_edit_->setText("1");
        strike_button_ = new QPushButton(tr("lock"), parent);

        strike_clean_button_ = new QPushButton(tr("stop"), parent);

        track_button_ = new QPushButton(tr("attack"), parent);

        follow_button_ = new QPushButton(tr("follow"), parent);

        return_home_edit_ = new QLineEdit(parent);
        return_home_edit_->setText("None");
        return_home_alt_ = 0;

        return_home_button_ = new QPushButton(tr("返航"), parent);
        land_button_ = new QPushButton(tr("降落"), parent);

        mission_alt_edit_->setVisible(false);
        launch_button_->setVisible(false);
        takeoff_button_->setVisible(false);
        return_home_edit_->setVisible(false);
        return_home_button_->setVisible(true);

        // // 为按钮创建水平布局
        layout_ = new QHBoxLayout;
        // layout_->addWidget(takeoff_button_);
        layout_->addWidget(mission_alt_edit_);
        layout_->addWidget(launch_button_);
        layout_->addWidget(strike_id_edit_);
        layout_->addWidget(strike_button_);
        layout_->addWidget(track_button_);
        layout_->addWidget(follow_button_);
        layout_->addWidget(strike_clean_button_);
        layout_->addWidget(return_home_edit_);
        layout_->addWidget(return_home_button_);
        // layout_->addWidget(land_button_);

        group_box_->setLayout(layout_);

        group_box_->setVisible(false);

        // 连接按钮信号到槽函数
        connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(takeoff_callback()));
        connect(launch_button_, SIGNAL(clicked()), this, SLOT(launch_callback()));
        connect(strike_button_, SIGNAL(clicked()), this, SLOT(strike_callback()));
        connect(strike_clean_button_, SIGNAL(clicked()), this, SLOT(strike_clean_callback()));
        connect(track_button_, SIGNAL(clicked()), this, SLOT(track_callback()));
        connect(follow_button_, SIGNAL(clicked()), this, SLOT(follow_callback()));
        connect(return_home_button_, SIGNAL(clicked()), this, SLOT(return_home_callback()));
        connect(land_button_, SIGNAL(clicked()), this, SLOT(land_callback()));

        id_string = id.toStdString();
        if (!id_string.empty() && std::isdigit(id_string.back())) {
            uav_id_num_ = std::stoi(std::string(1, id_string.back()));
        } else {
            // 处理错误情况，例如抛出异常或设置一个默认值
            // 这里只是一个示例，实际的错误处理可能需要根据你的应用来定
            uav_id_num_ = -1; // 或者其他表示错误的值
        }
        url_prefix = "http://";
        port_0 = 7080;
        port_1 = 7081;
        url_suffix = "/api/set_cloud_pitch?degree=";
        last_part_of_ip = 30 + uav_id_num_;
        url_0 = url_prefix + "192.168.144." + std::to_string(last_part_of_ip) + 
                ":" + std::to_string(port_0);
        url_1 = url_prefix + "192.168.144." + std::to_string(last_part_of_ip) + 
                        ":" + std::to_string(port_1);

        std::cout << url_0 << std::endl;
        std::cout << url_1 << std::endl;

        // 为起飞、发射、返航、降落创建ROS发布者
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/takeoff_topic", 1);
        launch_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/launch_topic", 1);
        return_home_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/return_home_topic", 1);
        land_pub_ = nh_.advertise<std_msgs::Empty>(id_string + "/land_topic", 1);

        takeoff_en_ = false;
        launch_en_ = false;
        strike_en_ = false;
        track_en_ = false;
        follow_en_ = false;
        return_home_en_ = false;
        land_en_ = false;
        
        waypoints_flag = 0;

        // 服务的客户端（设定无人机的模式、状态）
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>(id_string + "/mavros/cmd/arming");
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(id_string + "/mavros/set_mode");
        param_get_client_ = nh_.serviceClient<mavros_msgs::ParamGet>(id_string + "/mavros/param/get");
        param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>(id_string + "/mavros/param/set");
        param_pull_client_ = nh_.serviceClient<mavros_msgs::ParamPull>(id_string + "/mavros/param/pull");
        param_push_client_ = nh_.serviceClient<mavros_msgs::ParamPush>(id_string + "/mavros/param/push");

        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(id_string + "/mavros/state", 10, &UavButton::mavrosStateCallback, this);
        mavros_home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>(id_string + "/mavros/home_position/home", 10, &UavButton::mavrosHomeCallback, this);

        dji_position_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/dji/gps", 10, &UavButton::djiPositionCallBack, this);

        box_select_sub_ = nh_.subscribe<geometry_msgs::PoseArray>(id_string + "/box_select", 10, &UavButton::boxSelectCallback, this);

        // local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &UavButton::LocalPoseCallBack, this);

        offb_set_mode.request.custom_mode = "POSCTL";

        arm_cmd.request.value = true;

        threads = new BackgroundWorker();
        threads->start();
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

    void UavButton::set_launch_en() {
        launch_en_ = true;
        mission_alt_ = 20 + uav_id_num_ * 10;

        float x_lat = 23.19659346083342;
        float y_long = 112.58319910766468;

        std::cout << "" << std::endl;
        url = url_1 + "/api/fly_mission?id=";
        threads->set_mission_param(url, uav_id_num_, x_lat, y_long);
        threads->set_mission_en();
        launch_en_ = false;
    }

    void UavButton::takeoff_callback() {
        std_msgs::Empty empty_msg;
        takeoff_en_ = true;
        ROS_INFO("Takeoff button clicked.");
        takeoff_pub_.publish(empty_msg);
    }

    void UavButton::launch_callback() {
        std_msgs::Empty empty_msg;
        ROS_INFO("Launch button clicked.");
        launch_pub_.publish(empty_msg);

        // 获取文本框的输入内容
        QString inputText = mission_alt_edit_->text();

        // 尝试将输入内容转换为整数
        bool ok;
        mission_alt_ = inputText.toDouble(&ok);

        if (ok) {
            // 输入有效，打印高度值
            std::cout << "mission alt: " << mission_alt_ << std::endl;
            launch_en_ = true;
            // 这里可以添加其他处理用户输入的代码...
        } else {
            // 输入无效，弹出提示信息
            QMessageBox::warning(this, "Input Error", "Please enter a valid double.");
        }
    }

    void UavButton::strike_callback() {
        ROS_INFO("Strike button clicked.");
        // 获取文本框的输入内容
        QString inputText = strike_id_edit_->text();

        // 尝试将输入内容转换为整数
        bool ok;
        strike_id_ = inputText.toInt(&ok);

        if (ok) {
            // 输入有效，打印整数值
            std::cout << "User input value: " << strike_id_ << std::endl;
            strike_en_ = true;
            // 这里可以添加其他处理用户输入的代码...
        } else {
            // 输入无效，弹出提示信息
            QMessageBox::warning(this, "Input Error", "Please enter a valid integer.");
        }
        url = url_0 + "/api/set_obj";
        threads->set_obj_param(url, strike_id_);
        threads->set_obj_en();
        // set_rect(url, 100, 100, 200, 200);
    }

    void UavButton::strike_clean_callback() {
        ROS_INFO("Strike clean button clicked.");
        url = url_0 + "/api/clean_obj";
        // clean_obj(url);
        threads->set_clean_param(url);
        threads->set_clean_en();
    }

    void UavButton::track_callback() {
        ROS_INFO("Track button clicked.");
        url = url_1 + "/api/track_dive?speed=";
        FILE* fp = fopen("/home/lgh/catkin_ws/src/ground_station/launch/dive_speed.txt","r");
        char buff[1024] = {0};
        fread(buff, 1, 1024, fp);
        fclose(fp);
        float speed = atof(buff);
        std::cout << url << std::endl;
        std::cout << speed << std::endl;
        threads->set_track_drve_param(url, speed);
        threads->set_track_drve_en();   
    }

    void UavButton::follow_callback() {
        ROS_INFO("Follow button clicked.");
        // "http://192.168.144.18:7081/api/follow?keep_deg="
        url = url_1 + "/api/follow?keep_deg=";
        FILE* fp = fopen("/home/lgh/catkin_ws/src/ground_station/launch/follow_param.txt","r");
        // char buff[1024] = {0};
        // fread(buff, 1, 1024, fp);
        // fclose(fp);

        if (fp == NULL) {
            ROS_ERROR("Failed to open file.");
            return;
        }
        
        char buff[1024] = {0};
        if (fgets(buff, sizeof(buff), fp) == NULL) {
            ROS_ERROR("Failed to read from file.");
            fclose(fp);
            return;
        }
        fclose(fp);
        // 使用stringstream来分割字符串
        std::stringstream ss(buff);
        float keep_deg, speed, keep_z;

        // 从stringstream中读取三个浮点数
        if (!(ss >> keep_deg >> speed >> keep_z)) {
            ROS_ERROR("Failed to parse parameters from file.");
            return;
        }
        std::cout << url << std::endl;
        std::cout << keep_deg << std::endl;
        std::cout << speed << std::endl;
        std::cout << keep_z << std::endl;
        threads->set_follow_param(url, keep_deg, speed, keep_z);
        threads->set_follow_en();   
    }

    void UavButton::return_home_callback() {
        std_msgs::Empty empty_msg;

        // 获取文本框的输入内容
        QString inputText = return_home_edit_->text();

        // 尝试将输入内容转换为浮点数
        bool ok;
        double response;

        return_home_alt_ = inputText.toDouble(&ok);

        if (ok) {
            // 输入有效，打印高度值
            std::cout << "return home alt: " << return_home_alt_ << std::endl;
            response = paramSet("RTL_RETURN_ALT", return_home_alt_);

            if(response > 10.0 && response < 70.0)
            {    
                return_home_en_ = true;
                ROS_INFO("Return Home button clicked.");
            }

            // 这里可以添加其他处理用户输入的代码...
        } else {
            // 输入无效，弹出提示信息
            QMessageBox::warning(this, "Input Error", "Please enter a valid double.");
        }

        return_home_pub_.publish(empty_msg);
    }

    void UavButton::land_callback() {
        std_msgs::Empty empty_msg;
        land_en_ = true;
        ROS_INFO("Land button clicked.");
        land_pub_.publish(empty_msg);
    }

    double UavButton::paramGet(std::string param_str){
        mavros_msgs::ParamGet srv;
        srv.request.param_id = param_str;

        bool flag = param_get_client_.call(srv);

        double param_value = NAN; // 使用 NAN 或者一个默认值作为初始值

        try {
            // 调用服务
            if (flag) {
                // 检查服务响应是否成功
                if (srv.response.success) {
                    // 返回参数值
                    return srv.response.value.real;
                } else {
                    ROS_ERROR("Failed to get parameter: %s", param_str.c_str());
                    // 处理错误情况，例如返回一个默认值或抛出异常
                    throw std::runtime_error("Parameter get service call failed.");
                }
            } else {
                ROS_ERROR("Service call failed: /%s/mavros/param/get", id_string.c_str());
                // 处理错误情况，例如返回一个默认值或抛出异常
                throw std::runtime_error("Service call to /mavros/param/get failed.");
            }
        } catch (const std::runtime_error& e) {
            // 捕获异常并记录错误信息
            ROS_ERROR("%s", e.what());
            // 可以选择在这里执行其他错误处理操作，例如重试或返回一个特定的错误代码
            return param_value; // 返回初始设置的默认值
        }
    }

    double UavButton::paramSet(std::string param_str, double vaule){
        mavros_msgs::ParamSet srv;
        srv.request.param_id = param_str;
        srv.request.value.real = vaule;
        
        bool flag = param_set_client_.call(srv);

        double param_value = NAN; // 使用 NAN 或者一个默认值作为初始值

        try{
            // 调用服务
            if (flag) {
                // 检查服务响应是否成功
                if (srv.response.success) {
                    // 返回参数值
                    ROS_INFO("Set %s to %lf", param_str.c_str(), srv.response.value.real);
                    return srv.response.value.real;
                } else {
                    ROS_ERROR("Failed to set parameter: %s", param_str.c_str());
                    // 处理错误情况，例如返回一个默认值或抛出异常
                    // throw std::runtime_error("Parameter set service call failed.");
                }
            } else {
                ROS_ERROR("Service call failed: /%s/mavros/param/get", id_string.c_str());
                // 处理错误情况，例如返回一个默认值或抛出异常
                // throw std::runtime_error("Service call to /mavros/param/get failed.");
            }
        } catch (const std::runtime_error& e) {
            // 捕获异常并记录错误信息
            ROS_ERROR("%s", e.what());
            // 可以选择在这里执行其他错误处理操作，例如重试或返回一个特定的错误代码
            return param_value; // 返回初始设置的默认值
        }
    }

    void UavButton::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
        // std::cout << "RTL_RETURN_ALT: " << return_home_alt_ << std::endl;
        if(return_home_alt_ == 0){
            ros::service::waitForService(id_string + "/mavros/param/get");
            std::cout << id_string << std::endl;
            // return_home_alt_ = paramGet("RTL_RETURN_ALT");
            if(uav_id_num_ != -1) {
                return_home_alt_ = 20 + uav_id_num_ * 10;
            }

            QString return_home_alt_str_ = QString::number(return_home_alt_);
            return_home_edit_->setText(return_home_alt_str_);
        }

        if(takeoff_en_){
            // 1. 设置飞行模式为定点模式
            if (current_state.mode != "POSCTL") {
                offb_set_mode.request.custom_mode = "POSCTL";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("POSCTL mode enabled");
                } else {
                    ROS_ERROR("Failed to set POSCTL mode");
                    return; // 如果无法设置模式，则退出函数
                }
            }

            // 2. 确保无人机连接并处于正确的模式
            if (!current_state.armed) {
                // 3. 发布ARM命令
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
                    takeoff_en_ = false;
                    ROS_INFO("TAKEOFF mode enabled");
                } else {
                    ROS_ERROR("Failed to set TAKEOFF mode");
                    return; // 如果无法设置模式，则退出函数
                }
            }
        }
        else if(launch_en_){
            std::cout << waypoints_flag << std::endl;
            // 定义服务响应类型
            // 创建航点清除服务的响应对象
            mavros_msgs::WaypointClear srv;

            // if(waypoints_flag == 0) {
            //     std::cout << "" << std::endl;
            //     url = url_1 + "/api/fly_mission?id=";
            //     threads->set_mission_param(url, uav_id_num_, x_lat, y_long);
            //     threads->set_mission_en();
            //     launch_en_ = false;
            // }

            if(waypoints_flag == 0) {
                // 调用清除航点服务
                if (ros::service::call(id_string + "/mavros/mission/clear", srv)) {
                    waypoints_flag = 1;
                    ROS_INFO("Waypoints cleared successfully before launch.");

                } else {
                    ROS_ERROR("Failed to call clear waypoints service before launch.");
                    return; // 如果无法清除航点，则退出函数
                }
            }

            if(waypoints_flag == 1) {
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

                if(mission_alt_ > 50.0) mission_alt_=50.0;
                if(mission_alt_ < 0) mission_alt_=0;
                waypoint.z_alt = mission_alt_; // 航点的相对高度

                waypoint.x_lat = 23.1968912;
                waypoint.y_long = 112.5846263;
                waypoints.push_back(waypoint);

                waypoint.x_lat = 23.196118;
                waypoint.y_long = 112.585907;
                waypoints.push_back(waypoint);

                waypoint.x_lat = 23.194433513578012;
                waypoint.y_long = 112.58484907323964;
                waypoints.push_back(waypoint);

                waypoint.x_lat = 23.195378852757297;
                waypoint.y_long = 112.58375691005567;
                waypoints.push_back(waypoint);

                waypoint.x_lat = 23.19632118533539;
                waypoint.y_long = 112.5836816404716;
                waypoints.push_back(waypoint);

                waypoint.x_lat = 23.19659346083342;
                waypoint.y_long = 112.58319910766468;
                waypoints.push_back(waypoint);

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

                // waypoint.x_lat = 23.1967469;
                // waypoint.y_long = 112.5821592;
                // waypoints.push_back(waypoint);

                // waypoint.x_lat = 23.1963283;
                // waypoint.y_long = 112.5829652;
                // waypoints.push_back(waypoint);

                // waypoint.x_lat = 23.1970022;
                // waypoint.y_long = 112.5834441;
                // waypoints.push_back(waypoint);

                // waypoint.x_lat = 23.1974289;
                // waypoint.y_long = 112.5827578;
                // waypoints.push_back(waypoint);

                // waypoint.x_lat = 23.1968588;
                // waypoint.y_long = 112.5825387;
                // waypoints.push_back(waypoint);

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
                    waypoints_flag = 2;
                    ROS_INFO("Waypoints pushed successfully.");
                } else {
                    ROS_ERROR("Failed to push waypoints.");
                }
            }

            if(waypoints_flag == 2) {
                waypoints_flag = 3;
                // 1. 设置飞行模式为定点模式
                if (current_state.mode != "POSCTL") {
                    offb_set_mode.request.custom_mode = "POSCTL";
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        waypoints_flag = 3;
                        ROS_INFO("POSCTL mode enabled");

                    } else {
                        ROS_ERROR("Failed to set POSCTL mode");
                        return; // 如果无法设置模式，则退出函数
                    }
                }
            }

            if(waypoints_flag == 3) {
                // 2. 确保无人机连接并处于正确的模式
                if (!current_state.armed) {
                    // 3. 发布ARM命令
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        waypoints_flag = 4;
                        ROS_INFO("Vehicle armed");
                    } else {
                        ROS_ERROR("Failed to arm vehicle");
                        return; // 如果无法武装无人机，则退出函数
                    }
                }
            }

            if(waypoints_flag == 4) {
                // 3. 设置飞行模式为起飞模式
                if (current_state.mode != "AUTO.TAKEOFF") {
                    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        waypoints_flag = 5;
                        ROS_INFO("TAKEOFF mode enabled");
                    } else {
                        ROS_ERROR("Failed to set TAKEOFF mode");
                        return; // 如果无法设置模式，则退出函数
                    }
                }
            }

            if(waypoints_flag == 5) {
                // 3. 设置飞行模式为任务模式
                if (current_state.mode != "AUTO.MISSION") {
                    offb_set_mode.request.custom_mode = "AUTO.MISSION";
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        launch_en_ = false;
                        waypoints_flag = 0;
                        ROS_INFO("MISSION mode enabled");
                    } else {
                        ROS_ERROR("Failed to set MISSION mode");
                        return; // 如果无法设置模式，则退出函数
                    }
                }
            }
        }
        else if(strike_en_){

        }
        else if(return_home_en_){
            // 1. 设置飞行模式为返航模式
            if (current_state.mode != "AUTO.RTL") {
                offb_set_mode.request.custom_mode = "AUTO.RTL";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    return_home_en_ = false;
                    ROS_INFO("RTL mode enabled");
                } else {
                    ROS_ERROR("Failed to set RTL mode");
                    return; // 如果无法设置模式，则退出函数
                }
            }
        }
        else if(land_en_){
            // 设置飞行模式为降落模式
            if (current_state.mode != "AUTO.LAND") {
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    land_en_ = false;
                    ROS_INFO("LAND mode enabled");
                } else {
                    ROS_ERROR("Failed to set LAND mode");
                    return; // 如果无法设置模式，则退出函数
                }
            }
        }
    }

    void UavButton::mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg){
        home_position.geo = msg->geo;
        home_position.position = msg->position;
        home_position.orientation = msg->orientation;
        // ROS_INFO("%s home lat: %f\t lon: %f\t alt: %f",id_string.c_str(), home_position.geo.latitude, home_position.geo.longitude, home_position.position.z);
    }

    // 位置回调函数
    // void UavButton::LocalPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        
    // }

    void UavButton::djiPositionCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg){
        dji_position = *msg;
        // ROS_INFO("lat: %f\t lon: %f\t alt: %f", dji_position.latitude, dji_position.longitude, dji_position.altitude);
    }

    void UavButton::boxSelectCallback(const geometry_msgs::PoseArray::ConstPtr &msg){
        if (msg->poses.size() >= 2) {
            // 假设第一个和最后一个Pose定义了选择区域的对角线
            geometry_msgs::Pose pose1 = msg->poses[0];
            geometry_msgs::Pose pose2 = msg->poses.back();

            std::cout << "pose1 x: "<< pose1.position.x << "  y: " << pose1.position.y << "  pose2 x: "<< pose2.position.x << "  y: " << pose2.position.y << std::endl;
            // std::cout << "pose2 x: "<< pose2.position.x << "  y: " << pose2.position.y << std::endl;
            // 提取位置信息
            double x1 = pose1.position.x;
            double y1 = pose1.position.y;
            double x2 = pose2.position.x;
            double y2 = pose2.position.y;

            // // 计算选择区域的宽度和高度
            int width = std::abs(x2 - x1);
            int height = std::abs(y2 - y1);
            std::cout << "width : "<< width << "  height: " << height << std::endl;

            // // 调用set_rect函数，这里假设UavButton对象的指针或引用可用
            // threads->set_rect(url, x1, y1, width, height);
            url = url_0 + "/api/set_rect";
            threads->set_rect_param(url, x1, y1, width, height);
            threads->set_rect_en();
        }
    }

    BackgroundWorker::BackgroundWorker(){
        set_obj_en_ = false;
        set_rect_en_ = false;
        set_clean_en_ = false;
        set_pitch_en_ = false;
        set_track_drve_en_ = false;
        strike_flag_ = 0;
        set_yaw_en_ = false;
        set_follow_en_ = false;
        set_mission_en_ = false;
    }

    void BackgroundWorker::run(){
        while (true){
            // ROS_INFO("BackgroundWorker run!");
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

    BackgroundWorker::Requst BackgroundWorker::ParseUrl(const std::string& url){
        BackgroundWorker::Requst req;
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

    int BackgroundWorker::HttpPost(const std::string& url, const std::string& body, std::string& result) {
        BackgroundWorker::Requst req = ParseUrl(url);
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

    std::string BackgroundWorker::send_http_post(const std::string& url, const std::string& body) {
        std::string result;
        if (HttpPost(url, body, result) != 0) {
            return "";
        }
        return result;
    }

    // {"x": 100, "y": 100, "width": 200, "height": 200}
    int BackgroundWorker::set_rect(const std::string& url, int x, int y, int width, int height) {
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
    int BackgroundWorker::set_obj(const std::string& url, int obj) {
        std::string body = "{\"id\": " + std::to_string(obj) + "}";
        std::string result = send_http_post(url, body);
        std::cout << "result: " << result << std::endl;
        if (result.empty())
        {
            return -1;
        }
        return 0;
    }

    int BackgroundWorker::clean_obj(const std::string& url) {
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
    std::string BackgroundWorker::get_track_rect(const std::string& url) {
        std::string body = "";
        std::string result = send_http_post(url, body);
        return result;
    }

    int BackgroundWorker::set_cloud_pitch(const std::string& url, int pitch) {
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

    int BackgroundWorker::set_yaw(const std::string& url, int yaw) {
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

    int BackgroundWorker::track_dive(const std::string& url, int speed) {
        // std::string url = "http://192.168.144.18:7081/api/track_dive?speed=" + std::to_string(speed);
        std::string result = send_http_post(url_, "");
        std::cout << "result: " << result << std::endl;
        if (result.empty())
        {
            return -1;
        }
        return 0;
    }

    int BackgroundWorker::follow(const std::string& url, float keep_deg, float speed, float keep_z){
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

    int BackgroundWorker::set_mission(const std::string& url, int uav_id, float lat, float lon) {
        std::cout << "url_: " << url_ << std::endl;
        std::string result = send_http_post(url_, "");
        std::cout << "result: " << result << std::endl;
        if (result.empty())
        {
            return -1;
        }
        return 0;
    }

    void BackgroundWorker::set_url_param(std::string& url) {
        url_ = url;
    }

    void BackgroundWorker::set_obj_param(const std::string& url, int id) {
        url_ = url;
        id_= id;
    }

    void BackgroundWorker::set_clean_param(const std::string& url) {
        url_ = url;
    }

    void BackgroundWorker::set_rect_param(const std::string& url, int x, int y, int width, int height) {
        url_ = url;
        x_ = x;
        y_ = y;
        width_ = width;
        height_ = height;
    }    

    void BackgroundWorker::set_cloud_pitch_param(const std::string& url, int pitch) {
        url_ = url + std::to_string(pitch);
        std::cout << url_ << std::endl;
        pitch_ = pitch;
    }

    void BackgroundWorker::set_track_drve_param(const std::string& url, int speed) {
        url_ = url + std::to_string(speed);
        speed_ = speed;
        std::cout << url_ << std::endl;
    }

    void BackgroundWorker::set_follow_param(const std::string& url, float keep_deg, float speed, float keep_z) {
        // std::string url = "http://192.168.144.18:7081/api/follow?keep_deg=" + std::to_string(keep_deg) + "&speed=" + to_string(speed) + "&keep_z" + to_string(keep_z);
        url_ = url + std::to_string(keep_deg) + "&speed=" + std::to_string(speed) + "&keep_z=" + std::to_string(keep_z);
        std::cout << url_ << std::endl;
        keep_deg_ = keep_deg;
        speed_ = speed;
        keep_z_ = keep_z;

    }

    void BackgroundWorker::set_yaw_param(const std::string& url, int yaw) {
        url_ = url + std::to_string(yaw_);
        std::cout << url_ << std::endl;
        yaw_ = yaw;
    }

    void BackgroundWorker::set_mission_param(const std::string& url, int uav_id, float lat, float lon) {
        // std::string url = "http://192.168.144.18:7081/api/follow?keep_deg=" + std::to_string(keep_deg) + "&speed=" + to_string(speed) + "&keep_z" + to_string(keep_z);
        url_ = url + std::to_string(uav_id) + "&lat=" + std::to_string(lat) + "&lon=" + std::to_string(lon);
        std::cout << url_ << std::endl;
        uav_id_ = uav_id;
        lat_ = lat;
        lon_ = lon;
    }

    void BackgroundWorker::set_obj_en() {
        set_obj_en_ = true;
        std::cout << "set_obj enable!" << std::endl;
    }

    void BackgroundWorker::set_rect_en() {
        set_rect_en_ = true;
        std::cout << "set_rect enable!" << std::endl;
    }

    void BackgroundWorker::set_pitch_en() {
        set_pitch_en_ = true;
        std::cout << "set_cloud_pitch enable!" << std::endl;
    }

    void BackgroundWorker::set_clean_en() {
        set_clean_en_ = true;
        std::cout << "set_clean enable!" << std::endl;
    }    

    void BackgroundWorker::set_track_drve_en() {
        set_track_drve_en_ = true;
        std::cout << "set_track_drve enable!" << std::endl;
    }    

    void BackgroundWorker::set_follow_en() {
        set_follow_en_ = true;
        std::cout << "set_follow enable!" << std::endl;
    }    

    void BackgroundWorker::set_yaw_en() {
        set_yaw_en_ = true;
        std::cout << "set_cloud_yaw enable!" << std::endl;
    }

    void BackgroundWorker::set_mission_en() {
        set_mission_en_ = true;
        std::cout << "set_mission enable!" << std::endl;
    }

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_drone_control::RvizDroneControl,rviz::Panel)
