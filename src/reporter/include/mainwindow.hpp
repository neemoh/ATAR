#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QProgressBar>
#include <rosclass.hpp>
#include "cvimagewidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
  void StartRosLoop() { ros_obj.start(); }
  void showImage();

private slots:

  void on_button_state_0_clicked();
  void on_state_1_clicked();
  void on_state_2_clicked();
  void on_state_3_clicked();
  void on_state_4_clicked();
  void on_state_5_clicked();

  void on_stop_released();

  void on_record_clicked();
  void on_button_get_point_clicked();

  void on_buttin_reset_points_clicked();


private:
  Ui::MainWindow *ui;
  QLabel *stat_label;
  QProgressBar *stat_progress;
  RosObj ros_obj;
  CVImageWidget *imageWidget;

};

#endif // MAINWINDOW_H
