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
#include <string>
#include <vector>
#include <QProcess>
#include <QThread>
#include <cstdlib> // 对于 std::stoi

#include "single_uav_ctrl_panel.h"

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

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;
  void updateTopic() override;
  void test2_updateTopic();

  Ogre::SceneManager* img_scene_manager_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;

  Ui::single_uav_ctrl_panel single_uav_ctrl_panel_;

private:
  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  BoolProperty* normalize_property_;
  FloatProperty* min_property_;
  FloatProperty* max_property_;
  IntProperty* median_buffer_size_property_;
  bool got_float_image_;

  MouseClick* mouse_click_;
};

} // namespace rviz_drone_control

#endif //RVIZ_DRONE_CONTROL_H
