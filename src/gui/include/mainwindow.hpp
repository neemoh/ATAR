#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QPushButton>
#include <QPageLayout>
#include "ui_mainwindow.h"

#include "RosBridge.hpp"
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
    // removed on_ from function names that confused qt autoconnector
    void task_1_clicked();
    void task_2_clicked();
    void task_3_clicked();
    void task_4_clicked();
    void task_5_clicked();
    void task_6_clicked();
    void task_7_clicked();
    void task_8_clicked();

    void home_masters_clicked();
    void pub_imgs_state_changed(bool state);

    void button_haptics_disable_checked();
    void button_haptics_skill_checked();
    void button_haptics_manual_checked();

    void calib_arm1_clicked();
    void calib_arm2_clicked();

    void exit_clicked();
    void kill_core_clicked();
    void on_stop_released();

    void on_record_clicked();
    void pause_clicked();
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
