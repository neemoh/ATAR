#include <mainwindow.hpp>
#include <QApplication>
#include <ros/ros.h>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_node");

  QApplication a(argc, argv);
  MainWindow w;
  w.StartRosLoop();
  w.showImage();

  w.show();
  a.exec();

  return 1;

}



