/********************************************************************************
** Form generated from reading UI file 'single_uav_ctrl_panel.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef SINGLE_UAV_CTRL_PANEL_H
#define SINGLE_UAV_CTRL_PANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_single_uav_ctrl_panel
{
public:
    QGridLayout *gridLayout_4;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QGridLayout *gridLayout_5;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QTextEdit *current_uav_text;
    QPushButton *current_uav_connect_button;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *all_uavs_start_button;
    QPushButton *current_uav_start_button;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QTextEdit *track_obj_text;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *target_lock_button;
    QPushButton *target_follow_button;
    QPushButton *target_stop_button;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *drone_attack_button;
    QPushButton *drone_return_button;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QVBoxLayout *verticalLayout_3;
    QPushButton *gimbal_up_button;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *drone_yaw_left_button;
    QPushButton *drone_yaw_right_button;
    QPushButton *gimbal_down_button;

    void setupUi(QWidget *single_uav_ctrl_panel)
    {
        if (single_uav_ctrl_panel->objectName().isEmpty())
            single_uav_ctrl_panel->setObjectName(QString::fromUtf8("single_uav_ctrl_panel"));
        single_uav_ctrl_panel->resize(388, 240);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(single_uav_ctrl_panel->sizePolicy().hasHeightForWidth());
        single_uav_ctrl_panel->setSizePolicy(sizePolicy);
        single_uav_ctrl_panel->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_4 = new QGridLayout(single_uav_ctrl_panel);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        scrollArea = new QScrollArea(single_uav_ctrl_panel);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 354, 431));
        gridLayout_5 = new QGridLayout(scrollAreaWidgetContents);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        groupBox_4 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setMaximumSize(QSize(16777215, 16777215));
        gridLayout = new QGridLayout(groupBox_4);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_2 = new QLabel(groupBox_4);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setMinimumSize(QSize(100, 10));
        label_2->setMaximumSize(QSize(0, 0));
        label_2->setMidLineWidth(0);
        label_2->setTextFormat(Qt::AutoText);
        label_2->setScaledContents(true);
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout->addWidget(label_2);

        current_uav_text = new QTextEdit(groupBox_4);
        current_uav_text->setObjectName(QString::fromUtf8("current_uav_text"));
        sizePolicy.setHeightForWidth(current_uav_text->sizePolicy().hasHeightForWidth());
        current_uav_text->setSizePolicy(sizePolicy);
        current_uav_text->setMinimumSize(QSize(200, 30));
        current_uav_text->setMaximumSize(QSize(400, 60));
        current_uav_text->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout->addWidget(current_uav_text);


        verticalLayout->addLayout(horizontalLayout);

        current_uav_connect_button = new QPushButton(groupBox_4);
        current_uav_connect_button->setObjectName(QString::fromUtf8("current_uav_connect_button"));
        sizePolicy.setHeightForWidth(current_uav_connect_button->sizePolicy().hasHeightForWidth());
        current_uav_connect_button->setSizePolicy(sizePolicy);
        current_uav_connect_button->setMaximumSize(QSize(16777215, 16777215));

        verticalLayout->addWidget(current_uav_connect_button);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        all_uavs_start_button = new QPushButton(groupBox_4);
        all_uavs_start_button->setObjectName(QString::fromUtf8("all_uavs_start_button"));
        sizePolicy.setHeightForWidth(all_uavs_start_button->sizePolicy().hasHeightForWidth());
        all_uavs_start_button->setSizePolicy(sizePolicy);
        all_uavs_start_button->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_2->addWidget(all_uavs_start_button);

        current_uav_start_button = new QPushButton(groupBox_4);
        current_uav_start_button->setObjectName(QString::fromUtf8("current_uav_start_button"));
        sizePolicy.setHeightForWidth(current_uav_start_button->sizePolicy().hasHeightForWidth());
        current_uav_start_button->setSizePolicy(sizePolicy);
        current_uav_start_button->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_2->addWidget(current_uav_start_button);


        verticalLayout->addLayout(horizontalLayout_2);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);


        gridLayout_5->addWidget(groupBox_4, 0, 0, 1, 1);

        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(0, 0));
        groupBox->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMinimumSize(QSize(100, 10));
        label->setMaximumSize(QSize(0, 0));
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_3->addWidget(label);

        track_obj_text = new QTextEdit(groupBox);
        track_obj_text->setObjectName(QString::fromUtf8("track_obj_text"));
        sizePolicy.setHeightForWidth(track_obj_text->sizePolicy().hasHeightForWidth());
        track_obj_text->setSizePolicy(sizePolicy);
        track_obj_text->setMinimumSize(QSize(200, 30));
        track_obj_text->setMaximumSize(QSize(400, 60));

        horizontalLayout_3->addWidget(track_obj_text);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        target_lock_button = new QPushButton(groupBox);
        target_lock_button->setObjectName(QString::fromUtf8("target_lock_button"));
        sizePolicy.setHeightForWidth(target_lock_button->sizePolicy().hasHeightForWidth());
        target_lock_button->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(target_lock_button);

        target_follow_button = new QPushButton(groupBox);
        target_follow_button->setObjectName(QString::fromUtf8("target_follow_button"));
        sizePolicy.setHeightForWidth(target_follow_button->sizePolicy().hasHeightForWidth());
        target_follow_button->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(target_follow_button);

        target_stop_button = new QPushButton(groupBox);
        target_stop_button->setObjectName(QString::fromUtf8("target_stop_button"));
        sizePolicy.setHeightForWidth(target_stop_button->sizePolicy().hasHeightForWidth());
        target_stop_button->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(target_stop_button);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        drone_attack_button = new QPushButton(groupBox);
        drone_attack_button->setObjectName(QString::fromUtf8("drone_attack_button"));
        sizePolicy.setHeightForWidth(drone_attack_button->sizePolicy().hasHeightForWidth());
        drone_attack_button->setSizePolicy(sizePolicy);
        drone_attack_button->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_5->addWidget(drone_attack_button);

        drone_return_button = new QPushButton(groupBox);
        drone_return_button->setObjectName(QString::fromUtf8("drone_return_button"));
        sizePolicy.setHeightForWidth(drone_return_button->sizePolicy().hasHeightForWidth());
        drone_return_button->setSizePolicy(sizePolicy);
        drone_return_button->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_5->addWidget(drone_return_button);


        verticalLayout_2->addLayout(horizontalLayout_5);


        gridLayout_2->addLayout(verticalLayout_2, 0, 0, 1, 1);


        gridLayout_5->addWidget(groupBox, 1, 0, 1, 1);

        groupBox_2 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        gimbal_up_button = new QPushButton(groupBox_2);
        gimbal_up_button->setObjectName(QString::fromUtf8("gimbal_up_button"));
        sizePolicy.setHeightForWidth(gimbal_up_button->sizePolicy().hasHeightForWidth());
        gimbal_up_button->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(gimbal_up_button);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        drone_yaw_left_button = new QPushButton(groupBox_2);
        drone_yaw_left_button->setObjectName(QString::fromUtf8("drone_yaw_left_button"));
        sizePolicy.setHeightForWidth(drone_yaw_left_button->sizePolicy().hasHeightForWidth());
        drone_yaw_left_button->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(drone_yaw_left_button);

        drone_yaw_right_button = new QPushButton(groupBox_2);
        drone_yaw_right_button->setObjectName(QString::fromUtf8("drone_yaw_right_button"));
        sizePolicy.setHeightForWidth(drone_yaw_right_button->sizePolicy().hasHeightForWidth());
        drone_yaw_right_button->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(drone_yaw_right_button);


        verticalLayout_3->addLayout(horizontalLayout_6);

        gimbal_down_button = new QPushButton(groupBox_2);
        gimbal_down_button->setObjectName(QString::fromUtf8("gimbal_down_button"));
        sizePolicy.setHeightForWidth(gimbal_down_button->sizePolicy().hasHeightForWidth());
        gimbal_down_button->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(gimbal_down_button);


        gridLayout_3->addLayout(verticalLayout_3, 0, 0, 1, 1);


        gridLayout_5->addWidget(groupBox_2, 2, 0, 1, 1);

        scrollArea->setWidget(scrollAreaWidgetContents);

        gridLayout_4->addWidget(scrollArea, 0, 0, 1, 1);


        retranslateUi(single_uav_ctrl_panel);

        QMetaObject::connectSlotsByName(single_uav_ctrl_panel);
    } // setupUi

    void retranslateUi(QWidget *single_uav_ctrl_panel)
    {
        single_uav_ctrl_panel->setWindowTitle(QApplication::translate("single_uav_ctrl_panel", "single_uav_ctrl_panel", nullptr));
        groupBox_4->setTitle(QApplication::translate("single_uav_ctrl_panel", "Link Manager", nullptr));
        label_2->setText(QApplication::translate("single_uav_ctrl_panel", "UAV IP", nullptr));
        current_uav_text->setHtml(QApplication::translate("single_uav_ctrl_panel", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">192.168.144.61</span></p></body></html>", nullptr));
        current_uav_connect_button->setText(QApplication::translate("single_uav_ctrl_panel", "Connect", nullptr));
        all_uavs_start_button->setText(QApplication::translate("single_uav_ctrl_panel", "All UAVs Start", nullptr));
        current_uav_start_button->setText(QApplication::translate("single_uav_ctrl_panel", "Current Uav Start", nullptr));
        groupBox->setTitle(QApplication::translate("single_uav_ctrl_panel", "Tracking Operation", nullptr));
        label->setText(QApplication::translate("single_uav_ctrl_panel", "TargetIdInput", nullptr));
        target_lock_button->setText(QApplication::translate("single_uav_ctrl_panel", "Lock", nullptr));
        target_follow_button->setText(QApplication::translate("single_uav_ctrl_panel", "Follow", nullptr));
        target_stop_button->setText(QApplication::translate("single_uav_ctrl_panel", "Stop", nullptr));
        drone_attack_button->setText(QApplication::translate("single_uav_ctrl_panel", "Attack", nullptr));
        drone_return_button->setText(QApplication::translate("single_uav_ctrl_panel", "UAVReturn", nullptr));
        groupBox_2->setTitle(QApplication::translate("single_uav_ctrl_panel", "Drone or Gimbal Operation", nullptr));
        gimbal_up_button->setText(QApplication::translate("single_uav_ctrl_panel", "Gimbal Up", nullptr));
        drone_yaw_left_button->setText(QApplication::translate("single_uav_ctrl_panel", "Yaw Left", nullptr));
        drone_yaw_right_button->setText(QApplication::translate("single_uav_ctrl_panel", "Yaw Right", nullptr));
        gimbal_down_button->setText(QApplication::translate("single_uav_ctrl_panel", "Gimbal Down", nullptr));
    } // retranslateUi

};

namespace Ui {
    class single_uav_ctrl_panel: public Ui_single_uav_ctrl_panel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SINGLE_UAV_CTRL_PANEL_H
