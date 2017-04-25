/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionDo_Something;
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QPushButton *state_4;
    QPushButton *state_5;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *record;
    QPushButton *stop;
    QWidget *widget;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *lineEdit;
    QLineEdit *file_name;
    QGroupBox *groupBox_3;
    QPushButton *button_repeat;
    QPushButton *button_reset;
    QLineEdit *line_edit_num_repetitions;
    QLineEdit *line_edit_duration;
    QStatusBar *statusBar;
    QButtonGroup *buttonGroup;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(245, 453);
        actionDo_Something = new QAction(MainWindow);
        actionDo_Something->setObjectName(QStringLiteral("actionDo_Something"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 270, 221, 161));
        groupBox->setCheckable(false);
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        state_4 = new QPushButton(groupBox);
        buttonGroup = new QButtonGroup(MainWindow);
        buttonGroup->setObjectName(QStringLiteral("buttonGroup"));
        buttonGroup->addButton(state_4);
        state_4->setObjectName(QStringLiteral("state_4"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(state_4->sizePolicy().hasHeightForWidth());
        state_4->setSizePolicy(sizePolicy);
        state_4->setCheckable(true);

        verticalLayout->addWidget(state_4);

        state_5 = new QPushButton(groupBox);
        buttonGroup->addButton(state_5);
        state_5->setObjectName(QStringLiteral("state_5"));
        sizePolicy.setHeightForWidth(state_5->sizePolicy().hasHeightForWidth());
        state_5->setSizePolicy(sizePolicy);
        state_5->setCheckable(true);

        verticalLayout->addWidget(state_5);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 130, 221, 131));
        layoutWidget = new QWidget(groupBox_2);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 90, 201, 31));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        record = new QPushButton(layoutWidget);
        record->setObjectName(QStringLiteral("record"));
        record->setStyleSheet(QLatin1String("#record:checked{\n"
"color:rgb(255, 0, 4);\n"
"background-color: red;\n"
"}\n"
"	\n"
""));
        record->setCheckable(true);

        horizontalLayout_3->addWidget(record);

        stop = new QPushButton(layoutWidget);
        stop->setObjectName(QStringLiteral("stop"));

        horizontalLayout_3->addWidget(stop);

        widget = new QWidget(groupBox_2);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(11, 31, 201, 52));
        verticalLayout_2 = new QVBoxLayout(widget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        lineEdit = new QLineEdit(widget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setStyleSheet(QLatin1String("#file_name{\n"
"background-color: red\n"
"}"));
        lineEdit->setReadOnly(true);

        verticalLayout_2->addWidget(lineEdit);

        file_name = new QLineEdit(widget);
        file_name->setObjectName(QStringLiteral("file_name"));
        file_name->setReadOnly(false);

        verticalLayout_2->addWidget(file_name);

        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 30, 221, 91));
        button_repeat = new QPushButton(groupBox_3);
        button_repeat->setObjectName(QStringLiteral("button_repeat"));
        button_repeat->setGeometry(QRect(90, 30, 121, 23));
        button_reset = new QPushButton(groupBox_3);
        button_reset->setObjectName(QStringLiteral("button_reset"));
        button_reset->setGeometry(QRect(91, 60, 121, 23));
        line_edit_num_repetitions = new QLineEdit(groupBox_3);
        line_edit_num_repetitions->setObjectName(QStringLiteral("line_edit_num_repetitions"));
        line_edit_num_repetitions->setGeometry(QRect(10, 30, 51, 23));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(line_edit_num_repetitions->sizePolicy().hasHeightForWidth());
        line_edit_num_repetitions->setSizePolicy(sizePolicy1);
        line_edit_duration = new QLineEdit(groupBox_3);
        line_edit_duration->setObjectName(QStringLiteral("line_edit_duration"));
        line_edit_duration->setGeometry(QRect(10, 60, 51, 23));
        sizePolicy1.setHeightForWidth(line_edit_duration->sizePolicy().hasHeightForWidth());
        line_edit_duration->setSizePolicy(sizePolicy1);
        MainWindow->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        actionDo_Something->setText(QApplication::translate("MainWindow", "Do Something", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Tasks", 0));
        state_4->setText(QApplication::translate("MainWindow", "Task 1", 0));
        state_5->setText(QApplication::translate("MainWindow", "Task 2", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Recording", 0));
        record->setText(QApplication::translate("MainWindow", "Record", 0));
        stop->setText(QApplication::translate("MainWindow", "Stop", 0));
        lineEdit->setText(QApplication::translate("MainWindow", "File Name:", 0));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Acquisition Control", 0));
        button_repeat->setText(QApplication::translate("MainWindow", "Repeat Last Acquisition", 0));
        button_reset->setText(QApplication::translate("MainWindow", "Reset Task", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
