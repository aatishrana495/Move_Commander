#include "move_commander/move_commander.h"

MoveCommander::MoveCommander(ros::NodeHandle _nh, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MoveCommander), nh(_nh) {
  ui->setupUi(this);
  LOGO_PATH = ros::package::getPath("move_commander") + "/utils/logo.jpeg";
  logo.load(LOGO_PATH.c_str());
  int h = ui->logo_label->height(), w = ui->logo_label->width();
  ui->logo_label->setPixmap(logo.scaled(w, h, Qt::KeepAspectRatio));
  this->reset_parameters();
  ui->surface_mode_rb->setCheckable(true);
  ui->hover_mode_rb->setCheckable(true);
  ui->movement_mode_rb->setCheckable(true);

  ui->surface_mode_rb->setChecked(false);
  ui->hover_mode_rb->setChecked(false);
  ui->movement_mode_rb->setChecked(false);

  move_cmd_pub = nh.advertise<hammerhead_control::MoveCmd>("/move_cmd", 100);
  mode_pub = nh.advertise<std_msgs::UInt8>("/set_mode", 1);

  connect(ui->addCmd_button, SIGNAL(pressed()), this,
          SLOT(publish_move_command()));
  connect(ui->surface_mode_rb, SIGNAL(clicked()), this,
          SLOT(set_surface_command()));
  connect(ui->hover_mode_rb, SIGNAL(clicked()), this,
          SLOT(set_hover_command()));
  connect(ui->movement_mode_rb, SIGNAL(clicked()), this,
          SLOT(set_movement_command()));
  connect(ui->surface_endmode_rb, SIGNAL(clicked()), this, SLOT(set_endmode()));
  connect(ui->hover_endmode_rb, SIGNAL(clicked()), this, SLOT(set_endmode()));
  connect(ui->movement_endmode_rb, SIGNAL(clicked()), this,
          SLOT(set_endmode()));
  connect(ui->reset_bt, SIGNAL(pressed()), this, SLOT(reset_parameters()));
}

MoveCommander::~MoveCommander() { delete ui; }

void MoveCommander::publish_move_command() {

  mode_sub.publish(set_mode);
  move_cmd.surge =
      std::atoi(((ui->surge_position->toPlainText()).toStdString()).c_str());
  move_cmd.surge_speed =
      std::atoi(((ui->surge_speed->toPlainText()).toStdString()).c_str());
  move_cmd.surge_time =
      std::atoi(((ui->surge_time->toPlainText()).toStdString()).c_str());

  move_cmd.sway =
      std::atoi(((ui->sway_position->toPlainText()).toStdString()).c_str());
  move_cmd.sway_speed =
      std::atoi(((ui->sway_speed->toPlainText()).toStdString()).c_str());
  move_cmd.sway_time =
      std::atoi(((ui->sway_time->toPlainText()).toStdString()).c_str());

  move_cmd.depth =
      std::atoi(((ui->heave_position->toPlainText()).toStdString()).c_str());
  move_cmd.depth_speed =
      std::atoi(((ui->heave_speed->toPlainText()).toStdString()).c_str());
  move_cmd.depth_time =
      std::atoi(((ui->heave_time->toPlainText()).toStdString()).c_str());
  move_cmd.yaw =
      std::atoi(((ui->yaw_position->toPlainText()).toStdString()).c_str());
  move_cmd.yaw_speed =
      std::atoi(((ui->yaw_speed->toPlainText()).toStdString()).c_str());
  move_cmd.yaw_time =
      std::atoi(((ui->yaw_time->toPlainText()).toStdString()).c_str());

  if (ui->isAbsolute->isChecked()) {
    move_cmd.isAbsolute = 1;
  } else {
    move_cmd.isAbsolute = 0;
  }
  if (ui->wait_max_timeout->isChecked()) {
    move_cmd.wait_for_max_timer_to_timeout = 1;
  } else {
    move_cmd.wait_for_max_timer_to_timeout = 0;
  }

  move_cmd.mode_after_last_cmd = endmode;
  move_cmd_pub.publish(move_cmd);
}

void MoveCommander::set_surface_command() {
  set_mode.data = 0;
  ui->hover_mode_rb->setChecked(false);
  ui->movement_mode_rb->setChecked(false);
  mode_pub.publish(set_mode);
}
void MoveCommander::set_hover_command() {
  set_mode.data = 1;
  ui->surface_mode_rb->setChecked(false);
  ui->movement_mode_rb->setChecked(false);
  mode_pub.publish(set_mode);
}
void MoveCommander::set_movement_command() {
  set_mode.data = 2;
  ui->hover_mode_rb->setChecked(false);
  ui->surface_mode_rb->setChecked(false);
  mode_pub.publish(set_mode);
  this->publish_move_command();
}

void MoveCommander::set_endmode() {
  if (ui->surface_endmode_rb->isChecked()) {
    endmode = 0;
    ui->hover_endmode_rb->setChecked(false);
    ui->movement_endmode_rb->setChecked(false);
  }
  if (ui->hover_endmode_rb->isChecked()) {
    endmode = 1;
    ui->surface_endmode_rb->setChecked(false);
    ui->movement_endmode_rb->setChecked(false);
  }
  if (ui->movement_endmode_rb->isChecked()) {
    endmode = 2;
    ui->hover_endmode_rb->setChecked(false);
    ui->surface_endmode_rb->setChecked(false);
  }
}

void MoveCommander::reset_parameters() {
  ui->surge_position->setText("0");
  ui->sway_position->setText("0");
  ui->heave_position->setText("0");
  ui->yaw_position->setText("0");
  ui->roll_position->setText("0");
  ui->pitch_position->setText("0");

  ui->surge_speed->setText("0");
  ui->sway_speed->setText("0");
  ui->heave_speed->setText("0");
  ui->yaw_speed->setText("0");
  ui->roll_speed->setText("0");
  ui->pitch_speed->setText("0");

  ui->surge_time->setText("0");
  ui->sway_time->setText("0");
  ui->heave_time->setText("0");
  ui->yaw_time->setText("0");
  ui->roll_time->setText("0");
  ui->pitch_time->setText("0");
  ui->surface_endmode_rb->setCheckable(true);
  ui->hover_endmode_rb->setCheckable(true);
  ui->movement_endmode_rb->setCheckable(true);

  ui->surface_endmode_rb->setChecked(false);
  ui->hover_endmode_rb->setChecked(false);
  ui->movement_endmode_rb->setChecked(false);
}
