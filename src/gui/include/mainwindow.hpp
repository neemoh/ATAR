#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QPushButton>
#include <QPageLayout>

#include "rosclass.hpp"
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

    void on_task_1_clicked();
    void on_task_2_clicked();
    void on_task_3_clicked();
    void on_task_4_clicked();
    void on_task_5_clicked();
    void on_task_6_clicked();
    void on_task_7_clicked();
    void on_task_8_clicked();

    void on_home_masters_clicked();
    void on_pub_imgs_state_changed(bool state);

    void on_calib_arm1_clicked();
    void on_calib_arm2_clicked();

    void on_exit_clicked();
    void on_kill_core_clicked();

    void on_stop_released();

    void on_record_clicked();
    void on_pause_clicked();
    void on_button_repeat_clicked();

    void on_button_reset_clicked();

    void onTimeout();

    void StopRecording();


private:
    Ui::MainWindow *ui;
    QLabel *stat_label;
    QProgressBar *stat_progress;
    RosBridge ros_obj;
    CVImageWidget *imageWidget;
    QTimer *timer;

};

#endif // MAINWINDOW_H
