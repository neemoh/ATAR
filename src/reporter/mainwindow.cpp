#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <fstream>

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
  ui->imageLayout->addWidget(imageWidget);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::showImage(){

  // Load an image
  cv::Mat image = cv::imread("/home/nearlab/Downloads/VRFT_R2014a/1dof.jpg", true);
  imageWidget->showImage(image);

}

void MainWindow::on_button_state_0_clicked()
{
  ros_obj.SetStateLabel(0);
}

void MainWindow::on_state_1_clicked()
{
  ros_obj.SetStateLabel(1);
}

void MainWindow::on_state_2_clicked()
{
  ros_obj.SetStateLabel(2);
}

void MainWindow::on_state_3_clicked()
{
  ros_obj.SetStateLabel(3);
}
void MainWindow::on_state_4_clicked()
{
  ros_obj.SetStateLabel(4);
}
void MainWindow::on_state_5_clicked()
{
  ros_obj.SetStateLabel(5);
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

}


void MainWindow::on_button_get_point_clicked()
{
  int num_points = ros_obj.AddRegistrationPoint();
  ui->line_edit_num_reg_points->setText(QString::number(num_points));
}

void MainWindow::on_buttin_reset_points_clicked()
{
  ros_obj.ResetRegistrationPoints();
  ui->line_edit_num_reg_points->setText("");

}
