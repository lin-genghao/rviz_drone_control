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

  QWidget* main_window = new QWidget();
  QVBoxLayout* main_layout = new QVBoxLayout(main_window);
  // 创建一个垂直的 QSplitter
  QSplitter* splitter = new QSplitter(Qt::Vertical, main_window);

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


} // namespace rviz_drone_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_drone_control::droneDisplay, rviz::Display)