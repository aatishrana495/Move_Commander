#include "move_commander/move_commander.h"
#include <QApplication>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "move_commander");
  ros::NodeHandle nh;
  QApplication a(argc, argv);
  MoveCommander w(nh);
  w.show();
  ros::Rate rate(30);
  while (ros::ok() && w.isVisible()) {
    ros::spinOnce();
    rate.sleep();
    a.processEvents();
  }

  return 0;
}
