#ifndef MOVE_COMMANDER_H
#define MOVE_COMMANDER_H

#include "ui_move_commander.h"
#include <QMainWindow>

#include <QPixmap>
#include <QTimer>

#include <hammerhead/hammerhead.h>
#include <hammerhead_control/MoveCmd.h>
#include <hammerhead_control/MoveCmds.h>
#include <queue>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

#include <string>

namespace Ui {
class MoveCommander;
}

class MoveCommander : public QMainWindow {
  Q_OBJECT

public:
  explicit MoveCommander(ros::NodeHandle _nh, QWidget *parent = 0);
  ~MoveCommander();
  QPixmap logo;

private:
  Ui::MoveCommander *ui;

  ros::NodeHandle nh;

  ros::Publisher in_sub;
  ros::Publisher mode_pub;
  ros::Publisher move_cmd_pub;

  int end_mode_cmd, endmode, is_absolute;

  std_msgs::UInt8 set_mode;
  hammerhead_control::MoveCmd move_cmd;
  std::string LOGO_PATH;
private slots:
  void publish_move_command();
  void set_surface_command();
  void set_hover_command();
  void set_movement_command();
  void set_endmode();
  void reset_parameters();
};

#endif // MOVE_COMMANDER_H
