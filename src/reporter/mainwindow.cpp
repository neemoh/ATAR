#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <fstream>
#include <std_msgs/Int8.h>
#include "../ac_overlay_vtk/ControlEvents.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    ros_obj(this, "ros_reporter")
{
    ui->setupUi(this);

    // Set up ROS.
    // Create the image widget
    imageWidget = new CVImageWidget();
    //  ui->imageWidgetUI
    //ui->imageLayout->addWidget(imageWidget);
    connect(ui->button_task_1, SIGNAL(released()), this, SLOT(on_task_1_clicked()) );
    connect(ui->button_task_2, SIGNAL(released()), this, SLOT(on_task_2_clicked()) );
    connect(ui->button_task_3, SIGNAL(released()), this, SLOT(on_task_3_clicked()) );
    connect(ui->button_task_4, SIGNAL(released()), this, SLOT(on_task_4_clicked()) );
    connect(ui->button_task_5, SIGNAL(released()), this, SLOT(on_task_5_clicked()) );
    connect(ui->button_task_6, SIGNAL(released()), this, SLOT(on_task_6_clicked()) );

    connect(ui->button_fullScreen, SIGNAL(released()), this, SLOT(on_full_screen_clicked()) );

    connect(ui->checkBox_pub_imgs, SIGNAL(stateChanged(int)), this, SLOT(on_pub_imgs_cheked()) );

    connect(ui->button_calib_arm1, SIGNAL(released()), this, SLOT(on_calib_arm1_clicked()) );
    connect(ui->button_calib_arm2, SIGNAL(released()), this, SLOT(on_calib_arm2_clicked()) );

    connect(ui->button_exit, SIGNAL(released()), this, SLOT(on_exit_clicked()) );

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));

    timer->start(100);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onTimeout()
{

    ui->line_edit_num_repetitions->setText(QString::number(ros_obj.task_state.number_of_repetition));

    ui->line_edit_duration->setText(QString::number(ros_obj.task_state.time_stamp, 'f', 1));
}

void MainWindow::showImage(){

    // Load an image
    //  cv::Mat image = cv::imread("/home/nearlab/Downloads/VRFT_R2014a/1dof.jpg", true);
    //  imageWidget->showImage(image);

}

void MainWindow::on_task_1_clicked(){
    ros_obj.SetStateLabel(1);
    qDebug() <<"clicked task 1";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK1;
    ros_obj.publisher_recording_events.publish(msg);
}

void MainWindow::on_task_2_clicked(){
    ros_obj.SetStateLabel(2);
    qDebug() <<"clicked task 2";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK2;
    ros_obj.publisher_recording_events.publish(msg);
}

void MainWindow::on_task_3_clicked(){
    ros_obj.SetStateLabel(3);
    qDebug() << "clicked task 3";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK3;
    ros_obj.publisher_recording_events.publish(msg);
}


void MainWindow::on_task_4_clicked(){
    ros_obj.SetStateLabel(4);
    qDebug() << "clicked task 4";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK4;
    ros_obj.publisher_recording_events.publish(msg);
}

void MainWindow::on_task_5_clicked()
{
    ros_obj.SetStateLabel(5);
    qDebug() << "clicked task 5";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK5;
    ros_obj.publisher_recording_events.publish(msg);

}


void MainWindow::on_task_6_clicked()
{
    ros_obj.SetStateLabel(6);
    qDebug() << "clicked task 6";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK6;
    ros_obj.publisher_recording_events.publish(msg);

}


void MainWindow::on_task_7_clicked()
{
    ros_obj.SetStateLabel(7);
    qDebug() << "clicked task 7";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK7;
    ros_obj.publisher_recording_events.publish(msg);

}


void MainWindow::on_task_8_clicked()
{
    ros_obj.SetStateLabel(8);
    qDebug() << "clicked task 8";
    std_msgs::Int8 msg;
    msg.data =  CE_START_TASK8;
    ros_obj.publisher_recording_events.publish(msg);

}


void MainWindow::on_full_screen_clicked(){

    std_msgs::Int8 msg;
    msg.data =  CE_TOGGLE_FULLSCREEN;
    ros_obj.publisher_recording_events.publish(msg);

}

void MainWindow::on_pub_imgs_stateChanged(){

    std_msgs::Int8 msg;
    msg.data =  CE_PUBLISh_IMGS;
    ros_obj.publisher_recording_events.publish(msg);
}

void MainWindow::on_calib_arm1_clicked(){
    std_msgs::Int8 msg;
    msg.data =  CE_CALIB_ARM1;
    ros_obj.publisher_recording_events.publish(msg);

}

void MainWindow::on_calib_arm2_clicked(){
    std_msgs::Int8 msg;
    msg.data =  CE_CALIB_ARM2;
    ros_obj.publisher_recording_events.publish(msg);

}

void MainWindow::on_exit_clicked(){
    std_msgs::Int8 msg;
    msg.data =  CE_EXIT;
    ros_obj.publisher_recording_events.publish(msg);
    ros_obj.quit();
}


void MainWindow::on_record_clicked()
{
    std::string filename = ui->file_name->text().toStdString();
    if(filename.empty()){
        ui->record->setChecked(false);
    }
    else{
        //  qDebug() << ui->file_name->text();
        ros_obj.OpenRecordingFile(filename);
        ui->file_name->setDisabled(true);
        ros_obj.StartRecording();
        qDebug() << "Started recording." ;

    }


}


void MainWindow::on_stop_released()
{
    qDebug() << "Stopping the recording." ;
    ros_obj.StopRecording();
    ros_obj.CloseRecordingFile();
    ui->record->setChecked(false);
    ui->file_name->setDisabled(false);

    //  QPalette pal = ui->record->palette();
    //  pal.setColor(QPalette::Window, QColor(Qt::blue));
    //  ui->record->setAutoFillBackground(true);
    //  ui->record->setPalette(pal);
    //  ui->record->update();
}

void MainWindow::on_button_repeat_clicked()
{
    // TODO IS THIS NEEDED?
    std_msgs::Int8 msg;
    msg.data =  CE_RESET_TASK;
    ros_obj.publisher_recording_events.publish(msg);
}

void MainWindow::on_button_reset_clicked()
{
    std_msgs::Int8 msg;
    msg.data =  CE_RESET_TASK;
    ros_obj.publisher_recording_events.publish(msg);

}
