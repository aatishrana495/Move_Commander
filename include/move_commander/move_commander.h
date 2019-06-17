#ifndef MOVE_COMMANDER_H
#define MOVE_COMMANDER_H

#include "ui_move_commander.h"
#include <QMainWindow>

namespace Ui {
class MoveCommander;
}

class MoveCommander : public QMainWindow {
  Q_OBJECT

public:
  explicit MoveCommander(QWidget *parent = 0);
  ~MoveCommander();

private:
  Ui::MoveCommander *ui;
};

#endif // MOVE_COMMANDER_H
