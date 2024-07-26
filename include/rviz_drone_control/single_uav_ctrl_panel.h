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
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_single_uav_ctrl_panel
{
public:
    QGridLayout *gridLayout_4;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout;
    QGridLayout *gridLayout_3;
    QGridLayout *gridLayout_9;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_5;
    QVBoxLayout *verticalLayout_4;
    QPushButton *gimbal_up_button;
    QGridLayout *gridLayout_6;
    QPushButton *drone_yaw_left_button;
    QPushButton *drone_yaw_right_button;
    QPushButton *gimbal_down_button;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_10;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QTextEdit *track_obj_text;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *target_lock_button;
    QPushButton *target_follow_button;
    QPushButton *target_stop_button;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *drone_attack_button;
    QPushButton *drone_return_button;
    QLabel *uav_info_label;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_8;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout_7;
    QLabel *label_2;
    QTextEdit *current_uav_text;
    QPushButton *current_uav_connect_button;
    QVBoxLayout *verticalLayout;
    QPushButton *current_uav_start_button;
    QPushButton *all_uavs_start_button;
    QLabel *video_label;

    void setupUi(QWidget *single_uav_ctrl_panel)
    {
        if (single_uav_ctrl_panel->objectName().isEmpty())
            single_uav_ctrl_panel->setObjectName(QString::fromUtf8("single_uav_ctrl_panel"));
        single_uav_ctrl_panel->resize(676, 624);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(single_uav_ctrl_panel->sizePolicy().hasHeightForWidth());
        single_uav_ctrl_panel->setSizePolicy(sizePolicy);
        single_uav_ctrl_panel->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_4 = new QGridLayout(single_uav_ctrl_panel);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        groupBox_3 = new QGroupBox(single_uav_ctrl_panel);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setMaximumSize(QSize(16777215, 200));
        gridLayout = new QGridLayout(groupBox_3);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_9 = new QGridLayout();
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        groupBox_2 = new QGroupBox(groupBox_3);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_5 = new QGridLayout(groupBox_2);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        gimbal_up_button = new QPushButton(groupBox_2);
        gimbal_up_button->setObjectName(QString::fromUtf8("gimbal_up_button"));
        sizePolicy.setHeightForWidth(gimbal_up_button->sizePolicy().hasHeightForWidth());
        gimbal_up_button->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(gimbal_up_button);

        gridLayout_6 = new QGridLayout();
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        drone_yaw_left_button = new QPushButton(groupBox_2);
        drone_yaw_left_button->setObjectName(QString::fromUtf8("drone_yaw_left_button"));
        sizePolicy.setHeightForWidth(drone_yaw_left_button->sizePolicy().hasHeightForWidth());
        drone_yaw_left_button->setSizePolicy(sizePolicy);

        gridLayout_6->addWidget(drone_yaw_left_button, 0, 0, 1, 1);

        drone_yaw_right_button = new QPushButton(groupBox_2);
        drone_yaw_right_button->setObjectName(QString::fromUtf8("drone_yaw_right_button"));
        sizePolicy.setHeightForWidth(drone_yaw_right_button->sizePolicy().hasHeightForWidth());
        drone_yaw_right_button->setSizePolicy(sizePolicy);

        gridLayout_6->addWidget(drone_yaw_right_button, 0, 1, 1, 1);


        verticalLayout_4->addLayout(gridLayout_6);

        gimbal_down_button = new QPushButton(groupBox_2);
        gimbal_down_button->setObjectName(QString::fromUtf8("gimbal_down_button"));
        sizePolicy.setHeightForWidth(gimbal_down_button->sizePolicy().hasHeightForWidth());
        gimbal_down_button->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(gimbal_down_button);


        gridLayout_5->addLayout(verticalLayout_4, 0, 0, 1, 1);


        gridLayout_9->addWidget(groupBox_2, 0, 1, 1, 1);

        groupBox = new QGroupBox(groupBox_3);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(0, 0));
        groupBox->setMaximumSize(QSize(300, 300));
        gridLayout_10 = new QGridLayout(groupBox);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout->addWidget(label);

        track_obj_text = new QTextEdit(groupBox);
        track_obj_text->setObjectName(QString::fromUtf8("track_obj_text"));
        sizePolicy.setHeightForWidth(track_obj_text->sizePolicy().hasHeightForWidth());
        track_obj_text->setSizePolicy(sizePolicy);
        track_obj_text->setMaximumSize(QSize(16777215, 30));

        horizontalLayout->addWidget(track_obj_text);


        verticalLayout_3->addLayout(horizontalLayout);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        target_lock_button = new QPushButton(groupBox);
        target_lock_button->setObjectName(QString::fromUtf8("target_lock_button"));
        sizePolicy.setHeightForWidth(target_lock_button->sizePolicy().hasHeightForWidth());
        target_lock_button->setSizePolicy(sizePolicy);

        horizontalLayout_5->addWidget(target_lock_button);

        target_follow_button = new QPushButton(groupBox);
        target_follow_button->setObjectName(QString::fromUtf8("target_follow_button"));
        sizePolicy.setHeightForWidth(target_follow_button->sizePolicy().hasHeightForWidth());
        target_follow_button->setSizePolicy(sizePolicy);

        horizontalLayout_5->addWidget(target_follow_button);

        target_stop_button = new QPushButton(groupBox);
        target_stop_button->setObjectName(QString::fromUtf8("target_stop_button"));
        sizePolicy.setHeightForWidth(target_stop_button->sizePolicy().hasHeightForWidth());
        target_stop_button->setSizePolicy(sizePolicy);

        horizontalLayout_5->addWidget(target_stop_button);


        verticalLayout_3->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        drone_attack_button = new QPushButton(groupBox);
        drone_attack_button->setObjectName(QString::fromUtf8("drone_attack_button"));
        sizePolicy.setHeightForWidth(drone_attack_button->sizePolicy().hasHeightForWidth());
        drone_attack_button->setSizePolicy(sizePolicy);
        drone_attack_button->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_6->addWidget(drone_attack_button);

        drone_return_button = new QPushButton(groupBox);
        drone_return_button->setObjectName(QString::fromUtf8("drone_return_button"));
        sizePolicy.setHeightForWidth(drone_return_button->sizePolicy().hasHeightForWidth());
        drone_return_button->setSizePolicy(sizePolicy);
        drone_return_button->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_6->addWidget(drone_return_button);


        verticalLayout_3->addLayout(horizontalLayout_6);


        gridLayout_10->addLayout(verticalLayout_3, 0, 0, 1, 1);


        gridLayout_9->addWidget(groupBox, 0, 0, 1, 1);


        gridLayout_3->addLayout(gridLayout_9, 1, 0, 1, 1);

        uav_info_label = new QLabel(groupBox_3);
        uav_info_label->setObjectName(QString::fromUtf8("uav_info_label"));
        uav_info_label->setMaximumSize(QSize(200, 30));
        uav_info_label->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(uav_info_label, 0, 0, 1, 1);


        gridLayout->addLayout(gridLayout_3, 0, 1, 1, 1);

        groupBox_4 = new QGroupBox(groupBox_3);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setMaximumSize(QSize(200, 16777215));
        gridLayout_8 = new QGridLayout(groupBox_4);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_7 = new QGridLayout();
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        label_2 = new QLabel(groupBox_4);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMaximumSize(QSize(16777215, 30));

        gridLayout_7->addWidget(label_2, 0, 0, 1, 1);

        current_uav_text = new QTextEdit(groupBox_4);
        current_uav_text->setObjectName(QString::fromUtf8("current_uav_text"));
        sizePolicy.setHeightForWidth(current_uav_text->sizePolicy().hasHeightForWidth());
        current_uav_text->setSizePolicy(sizePolicy);
        current_uav_text->setMinimumSize(QSize(0, 0));
        current_uav_text->setMaximumSize(QSize(16777215, 30));
        current_uav_text->setLayoutDirection(Qt::LeftToRight);

        gridLayout_7->addWidget(current_uav_text, 0, 1, 1, 1);


        gridLayout_2->addLayout(gridLayout_7, 0, 0, 1, 1);

        current_uav_connect_button = new QPushButton(groupBox_4);
        current_uav_connect_button->setObjectName(QString::fromUtf8("current_uav_connect_button"));

        gridLayout_2->addWidget(current_uav_connect_button, 1, 0, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        current_uav_start_button = new QPushButton(groupBox_4);
        current_uav_start_button->setObjectName(QString::fromUtf8("current_uav_start_button"));

        verticalLayout->addWidget(current_uav_start_button);

        all_uavs_start_button = new QPushButton(groupBox_4);
        all_uavs_start_button->setObjectName(QString::fromUtf8("all_uavs_start_button"));
        sizePolicy.setHeightForWidth(all_uavs_start_button->sizePolicy().hasHeightForWidth());
        all_uavs_start_button->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(all_uavs_start_button);


        gridLayout_2->addLayout(verticalLayout, 2, 0, 1, 1);


        gridLayout_8->addLayout(gridLayout_2, 0, 0, 1, 1);


        gridLayout->addWidget(groupBox_4, 0, 0, 1, 1);


        gridLayout_4->addWidget(groupBox_3, 0, 0, 1, 1);

        video_label = new QLabel(single_uav_ctrl_panel);
        video_label->setObjectName(QString::fromUtf8("video_label"));
        video_label->setMaximumSize(QSize(640, 400));
        video_label->setInputMethodHints(Qt::ImhNone);
        video_label->setPixmap(QPixmap(QString::fromUtf8("../mulit_uav_gcs/src/rviz_drone_control/include/rviz_drone_control/wild_1.jpg")));
        video_label->setScaledContents(true);
        video_label->setWordWrap(false);

        gridLayout_4->addWidget(video_label, 1, 0, 1, 1);


        retranslateUi(single_uav_ctrl_panel);

        QMetaObject::connectSlotsByName(single_uav_ctrl_panel);
    } // setupUi

    void retranslateUi(QWidget *single_uav_ctrl_panel)
    {
        single_uav_ctrl_panel->setWindowTitle(QApplication::translate("single_uav_ctrl_panel", "single_uav_ctrl_panel", nullptr));
        groupBox_3->setTitle(QApplication::translate("single_uav_ctrl_panel", "UAV_X Control Panel", nullptr));
        groupBox_2->setTitle(QApplication::translate("single_uav_ctrl_panel", "Drone or Gimbal Operation", nullptr));
        gimbal_up_button->setText(QApplication::translate("single_uav_ctrl_panel", "Gimbal Up", nullptr));
        drone_yaw_left_button->setText(QApplication::translate("single_uav_ctrl_panel", "Yaw Left", nullptr));
        drone_yaw_right_button->setText(QApplication::translate("single_uav_ctrl_panel", "Yaw Right", nullptr));
        gimbal_down_button->setText(QApplication::translate("single_uav_ctrl_panel", "Gimbal Down", nullptr));
        groupBox->setTitle(QApplication::translate("single_uav_ctrl_panel", "Tracking Operation", nullptr));
        label->setText(QApplication::translate("single_uav_ctrl_panel", "TargetIdInput", nullptr));
        target_lock_button->setText(QApplication::translate("single_uav_ctrl_panel", "Lock", nullptr));
        target_follow_button->setText(QApplication::translate("single_uav_ctrl_panel", "Follow", nullptr));
        target_stop_button->setText(QApplication::translate("single_uav_ctrl_panel", "Stop", nullptr));
        drone_attack_button->setText(QApplication::translate("single_uav_ctrl_panel", "Attack", nullptr));
        drone_return_button->setText(QApplication::translate("single_uav_ctrl_panel", "UAVReturn", nullptr));
        uav_info_label->setText(QApplication::translate("single_uav_ctrl_panel", "UAV INFO : IP: XX.XX.XX.XX", nullptr));
        groupBox_4->setTitle(QApplication::translate("single_uav_ctrl_panel", "Link Manager", nullptr));
        label_2->setText(QApplication::translate("single_uav_ctrl_panel", "UAV IP", nullptr));
        current_uav_text->setHtml(QApplication::translate("single_uav_ctrl_panel", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">192.168.144.61</span></p></body></html>", nullptr));
        current_uav_connect_button->setText(QApplication::translate("single_uav_ctrl_panel", "Connect", nullptr));
        current_uav_start_button->setText(QApplication::translate("single_uav_ctrl_panel", "Current Uav Start", nullptr));
        all_uavs_start_button->setText(QApplication::translate("single_uav_ctrl_panel", "All UAVs Start", nullptr));
        video_label->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class single_uav_ctrl_panel: public Ui_single_uav_ctrl_panel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SINGLE_UAV_CTRL_PANEL_H
