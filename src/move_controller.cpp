#include "move_commander/move_commander.h"
#include <QApplication>

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  MoveCommander w;
  w.show();

  return a.exec();
}
