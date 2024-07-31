#ifndef RVIZ_DRONE_CONTROL_H
#define RVIZ_DRONE_CONTROL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/image/mouse_click.h"
#include "rviz/render_panel.h"

#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#endif

#include <boost/bind/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>

#include <sensor_msgs/image_encodings.h>

#include <rviz/default_plugin/image_display.h>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QGroupBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QPushButton>
#include <QMessageBox>
#include <QString>
#include <QComboBox>
#include <QObject>
#include <QFont>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/State.h> // 假设消息类型为mavros_msgs/State
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamPush.h>
#include <mavros_msgs/MessageInterval.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "tf/transform_datatypes.h"
#include "rviz_drone_control/httplib.h"
#include <iostream>
#include <chrono>
#include <string>
#include <iomanip>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <QProcess>
#include <QThread>
#include <cstdlib> // 对于 std::stoi
#include <fstream>

#include "single_uav_ctrl_panel.h"

#include "rviz_drone_control/background_thread.h"
#include "rviz_drone_control/sub_process.h"

namespace Ogre
{
class SceneNode;
class Rectangle2D;
} // namespace Ogre

namespace rviz_drone_control
{
/**
 * \class droneDisplay
 *
 */
using namespace rviz;
class droneDisplay : public ImageDisplayBase
{
  Q_OBJECT
public:
  droneDisplay();
  ~droneDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  virtual void updateNormalizeOptions();

  void uav_connect_callback();
  void target_lock_callback();
  void target_stop_callback();
  void drone_attack_callback();
  void target_follow_callback();
  void return_home_callback();
  void gimbal_up_callback();
  void gimbal_down_callback();
  void turn_left_callback();
  void turn_right_callback();
  
protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;
  void updateTopic() override;
  void test2_updateTopic();

  QWidget* main_window;

  Ogre::SceneManager* img_scene_manager_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;

  Ui::single_uav_ctrl_panel single_uav_ctrl_panel_;

  BackgroundThread* background_thread_;
  
  SubProcess* mavros_process_;

  std::ofstream logfile;

  std::string log_path;
  std::string log_file;
private:
  void CommunicationInit();
  void send_reposition_command(double x, double y, double z, double yaw);
  double paramGet(std::string param_str);
  double paramSet(std::string param_str, double vaule);
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
  void mavrosHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg);
  void mavrosGPSCallback(const mavros_msgs::GPSRAW::ConstPtr &msg);
  void mavrosCompassCallback(const std_msgs::Float64::ConstPtr &msg);
  void LocalPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void djiPositionCallBack(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void boxSelectCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  BoolProperty* normalize_property_;
  FloatProperty* min_property_;
  FloatProperty* max_property_;
  IntProperty* median_buffer_size_property_;
  bool got_float_image_;

  MouseClick* mouse_click_;

  ros::NodeHandle nh_;

  ros::Time time_start;
  ros::Time time_now;

  ros::Publisher mavros_manual_control_pub_;
  ros::Subscriber mavros_state_sub_;
  ros::Subscriber local_pos_sub;
  ros::Subscriber body_velocity_sub;
  ros::Publisher vec_pub;

  mavros_msgs::State current_state;
  std::string ai_ctrl_url;
  std::string ai_gimbal_ctrl_url;

  bool _uav_connected = false;
  std::string uav_select_;
  std::string uav_ip_;
  std::string uav_port_;

  ros::Publisher takeoff_pub_;
  ros::Publisher launch_pub_;
  ros::Publisher return_home_pub_;
  ros::Publisher land_pub_;

  bool takeoff_en_ = false;
  bool launch_en_ = false;
  bool strike_en_ = false;
  bool track_en_ = false;
  bool follow_en_ = false;
  bool return_home_en_ = false;
  bool land_en_ = false;
  bool param_init_en_ = false;
  bool turn_left_en_ = false;
  bool turn_right_en_ = false;
  
  int waypoints_flag;

  int error_count;

  double return_home_alt_;
  double mission_alt_;
  double mission_speed_;
  int launch_delay_;
  int strike_id_;
  double attack_speed_;
  double follow_pitch_;
  double follow_speed_;
  double follow_height_;

  std::string uav_id_;
  int uav_id_num_;

  std::string url_prefix;
  std::string url_suffix;
  int last_part_of_ip;
  int ai_board_ctrl_port;
  int ai_gimbal_ctrl_port;
  int port_2;
  double gimbal_pitch_;

  ros::Subscriber mavros_home_sub_;
  ros::Subscriber mavros_gps_sub_;
  ros::Subscriber mavros_compass_sub_;
  ros::Subscriber dji_position_sub_;

  double current_lat;
  double current_long;
  double current_alt;
  double current_yaw;

  mavros_msgs::HomePosition home_position;
  geometry_msgs::TwistStamped current_velocity;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::ServiceClient param_get_client_;
  ros::ServiceClient param_set_client_;
  ros::ServiceClient param_pull_client_;
  ros::ServiceClient param_push_client_;
  ros::ServiceClient command_client_;
  mavros_msgs::CommandLong command_;
  mavros_msgs::CommandBool arm_cmd;
  mavros_msgs::SetMode offb_set_mode;
  std::string id_string;  // 直接通过ip 来指定
  sensor_msgs::NavSatFix dji_position;

  ros::Subscriber box_select_sub_;


  int pitch_;
  int yaw_;
};

} // namespace rviz_drone_control

#endif //RVIZ_DRONE_CONTROL_H
