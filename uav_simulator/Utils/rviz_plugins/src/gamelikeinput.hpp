#ifndef GAMELIKEINPUT_HPP
#define GAMELIKEINPUT_HPP

#ifndef Q_MOC_RUN
#include <QObject>

#include "rviz_common/default_plugin/tools/selection_tool.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/selection/forwards.hpp"
#include "rviz_common/selection/selection_manager.hpp"
#include "rviz_common/tool.hpp"

#include "rviz_common/default_plugin/tools/interaction_tool.hpp"

#include "rclcpp/rclcpp.hpp"

#endif

namespace rviz
{
class Arrow;
class StringProperty;
} // namespace rviz

class GameLikeInput : public rviz::SelectionTool
{
  Q_OBJECT

protected Q_SLOTS:
  void updateTopic();

public:
  GameLikeInput();
  virtual ~GameLikeInput();

  virtual void onInitialize();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

  void sendMessage();

protected:
  virtual void onPoseSet(double x, double y, double z_vector, double theta);

private:
  rviz::InteractionTool* move_tool_;

  bool selecting_;
  int  sel_start_x_;
  int  sel_start_y_;

  rviz::M_Picked selection_;

  bool moving_;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr  pub_pointlist;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr  pub_selection;
  rclcpp::Publisher<quadrotor_msgs::msg::SwarmCommand>::SharedPtr  pub_swarm;

  double z_max;
  double z_min;
  double x_max;
  double x_min;
  double y_max;
  double y_min;

  rviz::FloatProperty* property_z_max;
  rviz::FloatProperty* property_z_min;
  rviz::FloatProperty* property_x_max;
  rviz::FloatProperty* property_x_min;
  rviz::FloatProperty* property_y_max;
  rviz::FloatProperty* property_y_min;

  rviz::StringProperty* topic_property_wp_;
  rviz::StringProperty* topic_property_drone_;
  rviz::StringProperty* topic_property_swarm_;

private:
  rviz::Arrow*              arrow_;
  std::vector<rviz::Arrow*> arrow_array;
  std::vector<double>       z_vector;

  enum State
  {
    None,
    Position,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;

  //  boost::recursive_mutex global_mutex_;
};

#endif // GAMELIKEINPUT_HPP
