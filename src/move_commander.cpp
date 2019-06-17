#include "move_commander/move_commander.h"

MoveCommander::MoveCommander(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MoveCommander) {
  ui->setupUi(this);
}

MoveCommander::~MoveCommander() { delete ui; }
