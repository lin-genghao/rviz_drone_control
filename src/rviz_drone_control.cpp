#include <rviz_drone_control/rviz_drone_control.h>

namespace rviz_drone_control
{
droneDisplay::droneDisplay() : ImageDisplayBase(), texture_()
{
  RosTopicProperty* test2_topic_property_ = new RosTopicProperty(
      "Image Topic", "/test2", QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "sensor_msgs::Image topic to subscribe to.", this, &droneDisplay::test2_updateTopic);

  normalize_property_ = new BoolProperty(
      "Normalize Range", true,
      "If set to true, will try to estimate the range of possible values from the received images.",
      this, &droneDisplay::updateNormalizeOptions);

  min_property_ = new FloatProperty("Min Value", 0.0, "Value which will be displayed as black.", this,
                                    &droneDisplay::updateNormalizeOptions);

  max_property_ = new FloatProperty("Max Value", 1.0, "Value which will be displayed as white.", this,
                                    &droneDisplay::updateNormalizeOptions);

  median_buffer_size_property_ =
      new IntProperty("Median window", 5, "Window size for median filter used for computin min/max.",
                      this, &droneDisplay::updateNormalizeOptions);

  got_float_image_ = false;
}

void droneDisplay::onInitialize()
{
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "droneDisplay" << count++;
#if (OGRE_VERSION < OGRE_VERSION_CHECK(13, 0, 0))
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
#else
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(
        Ogre::DefaultSceneManagerFactory::FACTORY_TYPE_NAME, ss.str());
#endif
  }

  img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "ImageDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    setMaterial(*screen_rect_, material_);
    img_scene_node_->attachObject(screen_rect_);
  }

  main_window = new QWidget();
  QHBoxLayout* main_layout = new QHBoxLayout(main_window);
  // 创建一个垂直的 QSplitter
  QSplitter* splitter = new QSplitter(Qt::Horizontal, main_window);

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(img_scene_manager_, context_);
  // main_layout->addWidget(render_panel_);

  QWidget* control_panel = new QWidget;
  single_uav_ctrl_panel_.setupUi(control_panel);
  // main_layout->addWidget(control_panel);

  // 将 render_panel_ 和 control_panel 添加到 splitter 中
  splitter->addWidget(render_panel_);
  splitter->addWidget(control_panel);

  // 将 splitter 添加到主布局
  main_layout->addWidget(splitter);

  setAssociatedWidget(main_window);
  if (auto* dock = getAssociatedWidgetPanel())
    dock->addMaximizeButton();

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  updateNormalizeOptions();

  mouse_click_ = new MouseClick(render_panel_, update_nh_);

  connect(single_uav_ctrl_panel_.current_uav_connect_button, SIGNAL(clicked()), this, SLOT(uav_connect_callback()));
}

droneDisplay::~droneDisplay()
{
  if (initialized())
  {
    delete render_panel_;
    delete screen_rect_;
    removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(), img_scene_node_);
  }
}

void droneDisplay::onEnable()
{
  ImageDisplayBase::subscribe();
  mouse_click_->enable();

  render_panel_->getRenderWindow()->setActive(true);
}

void droneDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  mouse_click_->disable();

  reset();
}

void droneDisplay::test2_updateTopic()
{
  std::cout << "test2 update topic" << std::endl;
  // unsubscribe();
  // reset();
  // subscribe();
  // context_->queueRender();
}

void droneDisplay::updateNormalizeOptions()
{
  if (got_float_image_)
  {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_.setMedianFrames(median_buffer_size_property_->getInt());
  }
  else
  {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void droneDisplay::update(float wall_dt, float ros_dt)
{
  Q_UNUSED(wall_dt)
  Q_UNUSED(ros_dt)
  try
  {
    texture_.update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect)
      {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                                 -1.0f * win_aspect / img_aspect, false);
      }
      else
      {
        screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect,
                                 -1.0f, false);
      }
    }

    render_panel_->getRenderWindow()->update();

    mouse_click_->setDimensions(img_width, img_height, win_width, win_height);
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(StatusProperty::Error, "Image", e.what());
  }
}

void droneDisplay::reset()
{
  ImageDisplayBase::reset();
  texture_.clear();
  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

/* This is called by incomingMessage(). */
void droneDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  bool got_float_image = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
                         msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_)
  {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }
  texture_.addMessage(msg);
}

void droneDisplay::updateTopic()
{
  ImageDisplayBase::updateTopic();
  mouse_click_->setImageTopic(topic_property_->getTopic());
}


void droneDisplay::uav_connect_callback(){
    ROS_INFO("------------->uav_connect_callback");
    QString uav_ip = single_uav_ctrl_panel_.current_uav_text->toPlainText();
    auto part_vec = uav_ip.split(".");
    if(part_vec.count() != 4){
        std::cout << "input ip error " << uav_ip.toStdString();
        QMessageBox::warning(main_window, "IP Input Error", "Please enter a valid IP.");
        _uav_connected = false;
        return;
    }
    auto last_num = part_vec.rbegin()->toStdString();
    auto last_bit = last_num.at(last_num.length() -1);
    // std::cout << "last_num " << last_num << "last bit " << last_bit << std::endl;
    uav_select_ = "/uav";
    uav_select_ +=  last_bit;    // 不能直接等于"/uav" + last_bit https://blog.csdn.net/weixin_43336281/article/details/100588048
    
    std::cout << "uav_select_ " << uav_select_ << std::endl;
    _uav_connected = true;
    if(_uav_connected){
        connect(single_uav_ctrl_panel_.target_lock_button,SIGNAL(clicked()),this,SLOT(target_lock_callback()));
        connect(single_uav_ctrl_panel_.drone_attack_button,SIGNAL(clicked()),this,SLOT(drone_attack_callback()));
        connect(single_uav_ctrl_panel_.target_follow_button,SIGNAL(clicked()),this,SLOT(target_follow_callback()));
        connect(single_uav_ctrl_panel_.target_stop_button,SIGNAL(clicked()),this,SLOT(target_stop_callback()));
        connect(single_uav_ctrl_panel_.gimbal_up_button,SIGNAL(clicked()),this,SLOT(gimbal_up_callback()));
        connect(single_uav_ctrl_panel_.gimbal_down_button,SIGNAL(clicked()),this,SLOT(gimbal_down_callback()));
        connect(single_uav_ctrl_panel_.drone_yaw_left_button,SIGNAL(clicked()),this,SLOT(turn_left_callback()));
        connect(single_uav_ctrl_panel_.drone_yaw_right_button,SIGNAL(clicked()),this,SLOT(turn_right_callback()));
    } else {
        disconnect(single_uav_ctrl_panel_.target_lock_button,SIGNAL(clicked()),this,SLOT(target_lock_callback()));
        disconnect(single_uav_ctrl_panel_.drone_attack_button,SIGNAL(clicked()),this,SLOT(drone_attack_callback()));
        disconnect(single_uav_ctrl_panel_.target_follow_button,SIGNAL(clicked()),this,SLOT(target_follow_callback()));
        disconnect(single_uav_ctrl_panel_.target_stop_button,SIGNAL(clicked()),this,SLOT(target_stop_callback()));
        disconnect(single_uav_ctrl_panel_.gimbal_up_button,SIGNAL(clicked()),this,SLOT(gimbal_up_callback()));
        disconnect(single_uav_ctrl_panel_.gimbal_down_button,SIGNAL(clicked()),this,SLOT(gimbal_down_callback()));
        disconnect(single_uav_ctrl_panel_.drone_yaw_left_button,SIGNAL(clicked()),this,SLOT(turn_left_callback()));
        disconnect(single_uav_ctrl_panel_.drone_yaw_right_button,SIGNAL(clicked()),this,SLOT(turn_right_callback()));
    }
    

    
}

void droneDisplay::turn_left_callback() {
    ROS_INFO("done yaw trun left, %s", uav_select_.c_str());
    std::string url;
    url = ai_gimbal_ctrl_url + "/api/rotation?degree=";
    yaw_=10;
    send_reposition_command(0, 0, 0, -5.0);

}

void droneDisplay::turn_right_callback() {
    ROS_INFO("done yaw trun right, %s", uav_select_.c_str());
    std::string url;
    url = ai_gimbal_ctrl_url + "/api/rotation?degree=";
    yaw_=-10;
    send_reposition_command(0, 0, 0, 5.0);
}

void droneDisplay::target_lock_callback() {
    ROS_INFO("Strike button clicked.");
    // 获取文本框的输入内容
    QString inputText = single_uav_ctrl_panel_.track_obj_text->toPlainText();

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
        QMessageBox::warning(main_window, "Input Error", "Please enter a valid integer.");
    }
    std::string url = ai_ctrl_url + "/api/set_obj";
    background_thread_->set_obj_param(url, strike_id_);
    background_thread_->set_obj_en();
    // set_rect(url, 100, 100, 200, 200);
}

void droneDisplay::target_stop_callback() {
    ROS_INFO("Strike clean button clicked.");
    std::string url = ai_ctrl_url + "/api/clean_obj";
    // clean_obj(url);
    background_thread_->set_clean_param(url);
    background_thread_->set_clean_en();
    send_reposition_command(0, 0, 0, 0);
}

void droneDisplay::drone_attack_callback() {
    ROS_INFO("Track button clicked.");
    std::string url = ai_gimbal_ctrl_url + "/api/track_dive?speed=";
    background_thread_->set_track_drve_param(url, attack_speed_);
    background_thread_->set_track_drve_en();   
}

void droneDisplay::target_follow_callback() {
    ROS_INFO("Follow button clicked.");
    // "http://192.168.144.18:7081/api/follow?keep_deg="
    std::string url = ai_gimbal_ctrl_url + "/api/follow?keep_deg=";
    std::cout << "set_follow_param" << std::endl;
    background_thread_->set_follow_param(url, follow_pitch_, follow_speed_, follow_height_);
    background_thread_->set_follow_en();   
}

void droneDisplay::return_home_callback() {
    std_msgs::Empty empty_msg;

    // 获取文本框的输入内容
    QString inputText = "10";//return_home_edit_->text();

    // 尝试将输入内容转换为浮点数
    bool ok;
    double response;

    // return_home_alt_ = inputText.toDouble(&ok);

    // if (ok) {
    //     // 输入有效，打印高度值
    //     std::cout << "return home alt: " << return_home_alt_ << std::endl;
    //     response = paramSet("RTL_RETURN_ALT", return_home_alt_);

        if(return_home_alt_ > 10.0 && return_home_alt_ < 70.0)
        {    
            return_home_en_ = true;
            ROS_INFO("Return Home button clicked.");
        }

        // 这里可以添加其他处理用户输入的代码...
    // } else {
    //     // 输入无效，弹出提示信息
    //     QMessageBox::warning(this, "Input Error", "Please enter a valid double.");
    // }

    return_home_pub_.publish(empty_msg);
}

void droneDisplay::gimbal_up_callback() {
    std::string url;
    url = ai_gimbal_ctrl_url + "/api/set_cloud_pitch?degree=";
    gimbal_pitch_ = -5;
    background_thread_->set_cloud_pitch_param(url, gimbal_pitch_);
    background_thread_->set_pitch_en();
}

void droneDisplay::gimbal_down_callback() {
    std::string url;
    url = ai_gimbal_ctrl_url + "/api/set_cloud_pitch?degree=";
    gimbal_pitch_ = 5;
    background_thread_->set_cloud_pitch_param(url, gimbal_pitch_);
    background_thread_->set_pitch_en();
}

void droneDisplay::send_reposition_command(double x, double y, double z, double yaw) {
    using mavlink::common::MAV_CMD;
    bool flag = false;

    // 3. 设置飞行模式为等待模式
    if (current_state.mode != "AUTO.LOITER") {
        offb_set_mode.request.custom_mode = "AUTO.LOITER";
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("LOITER mode enabled");
        } else {
            ROS_ERROR("Failed to set LOITER mode");
            return; // 如果无法设置模式，则退出函数
        }
    }

    double current_yaw_rad = current_yaw * (M_PI / 180.0);
    ROS_INFO("current_yaw_rad : %lf", current_yaw_rad);

    // 假设 x 和 y 是相对于当前偏航角的方向和侧向移动的距离，单位为米
    double distance_x = x * 0.00001 * cos(current_yaw_rad) + y * 0.00001 * sin(current_yaw_rad);
    double distance_y = x * 0.00001 * sin(current_yaw_rad) - y * 0.00001 * cos(current_yaw_rad);
    ROS_INFO("distance_x : %lf", distance_x);
    ROS_INFO("distance_y : %lf", distance_y);

    // 将本地坐标系中的偏移量转换为经纬度坐标系中的偏移量
    double delta_lat = distance_x;
    double delta_long = distance_y;

    // 更新目标经纬度
    double target_lat = current_lat + delta_lat;
    double target_long = current_long + delta_long;

    // 确保经纬度在合法范围内
    target_lat = fmax(fmin(target_lat, 90.0), -90.0);
    target_long = fmax(fmin(target_long, 180.0), -180.0);

    command_.request.broadcast = false;
    command_.request.command = mavros::utils::enum_value(MAV_CMD::DO_REPOSITION);
    command_.request.confirmation = 0;
    command_.request.param1 = 0; //speed
    command_.request.param2 = 0; 
    command_.request.param3 = 0; 
    command_.request.param4 = (current_yaw + yaw) * M_PI / 180; 
    command_.request.param5 = target_lat; // 转换回度
    command_.request.param6 = target_long; // 转换回度
    command_.request.param7 = NAN; 
    // command_.request.param7 = current_alt + z; 
    ROS_INFO("%s current lat: %f\t lon: %f\t alt: %f",id_string.c_str(), current_lat, current_long, current_alt);
    ROS_INFO("%s target lat: %f\t lon: %f\t alt: %f",id_string.c_str(), command_.request.param5, command_.request.param6, command_.request.param7);


    try {

        flag = command_client_.call(command_);

        if(flag) {
            ROS_INFO("Service call successful, result: %d", command_.response.result);
        } else {
            ROS_ERROR("Service call failed,  result: %d", command_.response.result);
        }
    }
  catch (ros::InvalidNameException &ex) {
    ROS_ERROR_NAMED("command", "command: %s", ex.what());
  }

}

void droneDisplay::CommunicationInit() {
    id_string = "test61";// id.toStdString();
    if (!id_string.empty() && std::isdigit(id_string.back())) {
        uav_id_num_ = std::stoi(std::string(1, id_string.back()));
    } else {
        // 处理错误情况，例如抛出异常或设置一个默认值
        // 这里只是一个示例，实际的错误处理可能需要根据你的应用来定
        uav_id_num_ = -1; // 或者其他表示错误的值
    }

    std::string param_prefix = "/rviz/" + id_string;
    std::string param_name = "";

    mission_alt_ = 0;
    return_home_alt_ = 0;
    attack_speed_ = 0;
    follow_pitch_ = 0;
    follow_speed_ = 0;
    follow_height_ = 0;
    launch_delay_ = 0;
    current_state.connected = false;

    param_name = param_prefix + "/ai_ip";
    nh_.getParam(param_name, last_part_of_ip);
    std::cout<< param_name <<": "<< last_part_of_ip << std::endl;

    param_name = param_prefix + "/launch_delay";
    nh_.getParam(param_name, launch_delay_);
    std::cout<< param_name <<": "<< launch_delay_ << std::endl;

    param_name = param_prefix + "/mission_alt";
    nh_.getParam(param_name, mission_alt_);
    std::cout<< param_name <<": "<< mission_alt_ << std::endl;

    param_name = param_prefix + "/mission_speed";
    nh_.getParam(param_name, mission_speed_);
    std::cout<< param_name <<": "<< mission_speed_ << std::endl;

    param_name = param_prefix + "/return_home_alt";
    nh_.getParam(param_name, return_home_alt_);
    std::cout<< param_name <<": "<< return_home_alt_ << std::endl;

    param_name = param_prefix + "/attack_speed";
    nh_.getParam(param_name, attack_speed_);
    std::cout<< param_name <<": "<< attack_speed_ << std::endl;

    param_name = param_prefix + "/follow_pitch";
    nh_.getParam(param_name, follow_pitch_);
    std::cout<< param_name <<": "<< follow_pitch_ << std::endl;  

    param_name = param_prefix + "/follow_speed";
    nh_.getParam(param_name, follow_speed_);
    std::cout<< param_name <<": "<< follow_speed_ << std::endl;  

    param_name = param_prefix + "/follow_height";
    nh_.getParam(param_name, follow_height_);
    std::cout<< param_name <<": "<< follow_height_ << std::endl;       

    url_prefix = "http://";
    ai_board_ctrl_port = 7080;
    ai_gimbal_ctrl_port = 7081;
    url_suffix = "/api/set_cloud_pitch/*  */?degree=";
    // last_part_of_ip = 30 + uav_id_num_;
    // if(last_part_of_ip == 31) {
    //     last_part_of_ip = 33;
    // }
    ai_ctrl_url = url_prefix + "192.168.144." + std::to_string(last_part_of_ip) + 
            ":" + std::to_string(ai_board_ctrl_port);
    ai_gimbal_ctrl_url = url_prefix + "192.168.144." + std::to_string(last_part_of_ip) + 
                    ":" + std::to_string(ai_gimbal_ctrl_port);

    std::cout << ai_ctrl_url << std::endl;
    std::cout << ai_gimbal_ctrl_url << std::endl;

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
    turn_left_en_ = false;
    turn_right_en_ = false;
    param_init_en_ = false;
    waypoints_flag = 0;
    error_count = 0;

    // 服务的客户端（设定无人机的模式、状态）
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>(id_string + "/mavros/cmd/arming");
    command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>(id_string + "/mavros/cmd/command");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(id_string + "/mavros/set_mode");
    param_get_client_ = nh_.serviceClient<mavros_msgs::ParamGet>(id_string + "/mavros/param/get");
    param_set_client_ = nh_.serviceClient<mavros_msgs::ParamSet>(id_string + "/mavros/param/set");
    param_pull_client_ = nh_.serviceClient<mavros_msgs::ParamPull>(id_string + "/mavros/param/pull");
    param_push_client_ = nh_.serviceClient<mavros_msgs::ParamPush>(id_string + "/mavros/param/push");

    mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(id_string + "/mavros/state", 10, &droneDisplay::mavrosStateCallback, this);
    mavros_gps_sub_ = nh_.subscribe<mavros_msgs::GPSRAW>(id_string + "/mavros/gpsstatus/gps1/raw", 10, &droneDisplay::mavrosGPSCallback, this);
    mavros_compass_sub_ = nh_.subscribe<std_msgs::Float64>(id_string + "/mavros/global_position/compass_hdg", 10, &droneDisplay::mavrosCompassCallback, this);
    mavros_home_sub_ = nh_.subscribe<mavros_msgs::HomePosition>(id_string + "/mavros/home_position/home", 10, &droneDisplay::mavrosHomeCallback, this);

    dji_position_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/dji/gps", 10, &droneDisplay::djiPositionCallBack, this);

    box_select_sub_ = nh_.subscribe<geometry_msgs::PoseArray>(id_string + "/box_select", 10, &droneDisplay::boxSelectCallback, this);

    // local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &droneDisplay::LocalPoseCallBack, this);


    offb_set_mode.request.custom_mode = "POSCTL";

    arm_cmd.request.value = true;

    background_thread_ = new BackgroundThread();
    background_thread_->start();
}

double droneDisplay::paramGet(std::string param_str){
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

double droneDisplay::paramSet(std::string param_str, double vaule){
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
                return NAN;
            }
        } else {
            ROS_ERROR("Service call failed: /%s/mavros/param/get", id_string.c_str());
            // 处理错误情况，例如返回一个默认值或抛出异常
            // throw std::runtime_error("Service call to /mavros/param/get failed.");
            return NAN;
        }
    } catch (const std::runtime_error& e) {
        // 捕获异常并记录错误信息
        ROS_ERROR("%s", e.what());
        // 可以选择在这里执行其他错误处理操作，例如重试或返回一个特定的错误代码
        return param_value; // 返回初始设置的默认值
    }
}

void droneDisplay::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

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
        //     url = ai_gimbal_ctrl_url + "/api/fly_mission?id=";
        //     background_thread_->set_mission_param(url, uav_id_num_, x_lat, y_long);
        //     background_thread_->set_mission_en();
        //     launch_en_ = false;
        // }
        if(error_count > 10 || time_now.toSec() - time_start.toSec() > launch_delay_ + 10) {
            launch_en_ = false;
            ROS_INFO("Exit Mission.");
        }

        if(waypoints_flag == 0) {
            time_now = ros::Time::now();
            ROS_INFO("now: %f", time_now.toSec());
            if(time_now.toSec() - time_start.toSec() > launch_delay_) {
                waypoints_flag = 1;
            }
            ROS_INFO("Start Mission.");
        }

        if(waypoints_flag == 1) {
            // 调用清除航点服务
            if (ros::service::call(id_string + "/mavros/mission/clear", srv)) {
                waypoints_flag = 2;
                ROS_INFO("Waypoints cleared successfully before launch.");

            } else {
                error_count = error_count + 1;
                ROS_ERROR("Failed to call clear waypoints service before launch. error count: %d", error_count);
                return; // 如果无法清除航点，则退出函数
            }
        }

        if(waypoints_flag == 2) {
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
                waypoints_flag = 3;
                ROS_INFO("Waypoints pushed successfully.");
            } else {
                error_count = error_count + 1;
                ROS_ERROR("Failed to push waypoints. error count: %d", error_count);
            }
        }

        if(waypoints_flag == 3) {
            waypoints_flag = 4;
            // 1. 设置飞行模式为定点模式
            if (current_state.mode != "POSCTL") {
                offb_set_mode.request.custom_mode = "POSCTL";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    waypoints_flag = 4;
                    ROS_INFO("POSCTL mode enabled");
                } else {
                    error_count = error_count + 1;
                    ROS_ERROR("Failed to set POSCTL mode. error count: %d", error_count);
                    return; // 如果无法设置模式，则退出函数
                }
            }
            else {
                waypoints_flag = 4;
                ROS_INFO("POSCTL mode enabled");     
            }
        }

        if(waypoints_flag == 4) {
            // 2. 确保无人机连接并处于正确的模式
            if (!current_state.armed) {
                // 3. 发布ARM命令
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    waypoints_flag = 5;
                    ROS_INFO("Vehicle armed");
                } else {
                    error_count = error_count + 1;
                    ROS_ERROR("Failed to arm vehicle. error count: %d", error_count);
                    return; // 如果无法武装无人机，则退出函数
                }
            }
            else {
                waypoints_flag = 5;
                ROS_INFO("Vehicle armed");
            }
        }

        if(waypoints_flag == 5) {
            // 3. 设置飞行模式为起飞模式
            if (current_state.mode != "AUTO.TAKEOFF") {
                offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    waypoints_flag = 6;
                    ROS_INFO("TAKEOFF mode enabled");
                } else {
                    error_count = error_count + 1;
                    ROS_ERROR("Failed to set TAKEOFF mode. error count: %d", error_count);
                    return; // 如果无法设置模式，则退出函数
                }
            }
        }

        if(waypoints_flag == 6) {
            // 3. 设置飞行模式为任务模式
            if (current_state.mode != "AUTO.MISSION") {
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    launch_en_ = false;
                    waypoints_flag = 0;
                    ROS_INFO("MISSION mode enabled");
                } else {
                    error_count = error_count + 1;
                    ROS_ERROR("Failed to set MISSION mode. error count: %d", error_count);
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
    else if(turn_left_en_) {
        send_reposition_command(0, 0, 0, -5.0);
        turn_left_en_ = false;
        ROS_INFO("drone turn left");
    }
    else if(turn_right_en_) {
        send_reposition_command(0, 0, 0, 5.0);
        turn_right_en_ = false;
        ROS_INFO("drone turn right");
    }
}

void droneDisplay::mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg){
    home_position.geo = msg->geo;
    home_position.position = msg->position;
    home_position.orientation = msg->orientation;
    // ROS_INFO("%s home lat: %f\t lon: %f\t alt: %f",id_string.c_str(), home_position.geo.latitude, home_position.geo.longitude, home_position.position.z);
}

void droneDisplay::mavrosGPSCallback(const mavros_msgs::GPSRAW::ConstPtr &msg) {
    current_lat = msg->lat / 1e7;
    current_long = msg->lon / 1e7;
    current_alt = msg->alt / 1e3;
    // ROS_INFO("%s current lat: %f\t lon: %f\t alt: %f",id_string.c_str(), current_lat, current_long, current_alt);
}

void droneDisplay::mavrosCompassCallback(const std_msgs::Float64::ConstPtr &msg) {
    current_yaw = msg->data;
    // ROS_INFO("%s current compass: %f\t",id_string.c_str(), current_yaw);

}
// 位置回调函数
// void droneDisplay::LocalPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    
// }

void droneDisplay::djiPositionCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg){
    dji_position = *msg;
    // ROS_INFO("lat: %f\t lon: %f\t alt: %f", dji_position.latitude, dji_position.longitude, dji_position.altitude);
}

void droneDisplay::boxSelectCallback(const geometry_msgs::PoseArray::ConstPtr &msg){
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
        // background_thread_->set_rect(url, x1, y1, width, height);
        std::string url = ai_ctrl_url + "/api/set_rect";
        background_thread_->set_rect_param(url, x1, y1, width, height);
        background_thread_->set_rect_en();
    }
}
} // namespace rviz_drone_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_drone_control::droneDisplay, rviz::Display)