/********************************************************************************
** Form generated from reading UI file 'single_uav_ctrl_panel.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
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
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGroupBox *groupBox_3;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *uav_info_label;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QTextEdit *track_obj_text;
    QPushButton *target_stop_button;
    QPushButton *drone_return_button;
    QPushButton *drone_attack_button;
    QPushButton *target_follow_button;
    QPushButton *target_lock_button;
    QGroupBox *groupBox_2;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QPushButton *gimbal_up_button;
    QSpacerItem *horizontalSpacer_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *drone_yaw_left_button;
    QPushButton *drone_yaw_right_button;
    QPushButton *gimbal_down_button;
    QWidget *widget;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QTextEdit *current_uav_text;
    QPushButton *current_uav_connect_button;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *current_uav_start_button;
    QPushButton *all_uavs_start_button;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QStringLiteral("Form"));
        Form->resize(774, 450);
        groupBox_3 = new QGroupBox(Form);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 130, 751, 301));
        layoutWidget = new QWidget(groupBox_3);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(0, 20, 731, 261));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        uav_info_label = new QLabel(layoutWidget);
        uav_info_label->setObjectName(QStringLiteral("uav_info_label"));
        uav_info_label->setMaximumSize(QSize(16777215, 20));
        uav_info_label->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(uav_info_label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        groupBox = new QGroupBox(layoutWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setMaximumSize(QSize(16777215, 200));
        gridLayoutWidget = new QWidget(groupBox);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 50, 340, 96));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setVerticalSpacing(0);
        gridLayout->setContentsMargins(0, 40, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setMaximumSize(QSize(90, 16777215));

        horizontalLayout->addWidget(label);

        track_obj_text = new QTextEdit(gridLayoutWidget);
        track_obj_text->setObjectName(QStringLiteral("track_obj_text"));
        sizePolicy.setHeightForWidth(track_obj_text->sizePolicy().hasHeightForWidth());
        track_obj_text->setSizePolicy(sizePolicy);
        track_obj_text->setMaximumSize(QSize(90, 30));

        horizontalLayout->addWidget(track_obj_text);


        gridLayout->addLayout(horizontalLayout, 1, 0, 1, 1);

        target_stop_button = new QPushButton(gridLayoutWidget);
        target_stop_button->setObjectName(QStringLiteral("target_stop_button"));

        gridLayout->addWidget(target_stop_button, 2, 1, 1, 1);

        drone_return_button = new QPushButton(gridLayoutWidget);
        drone_return_button->setObjectName(QStringLiteral("drone_return_button"));

        gridLayout->addWidget(drone_return_button, 2, 2, 1, 1);

        drone_attack_button = new QPushButton(gridLayoutWidget);
        drone_attack_button->setObjectName(QStringLiteral("drone_attack_button"));

        gridLayout->addWidget(drone_attack_button, 1, 2, 1, 1);

        target_follow_button = new QPushButton(gridLayoutWidget);
        target_follow_button->setObjectName(QStringLiteral("target_follow_button"));

        gridLayout->addWidget(target_follow_button, 2, 0, 1, 1);

        target_lock_button = new QPushButton(gridLayoutWidget);
        target_lock_button->setObjectName(QStringLiteral("target_lock_button"));

        gridLayout->addWidget(target_lock_button, 1, 1, 1, 1);


        horizontalLayout_2->addWidget(groupBox);

        groupBox_2 = new QGroupBox(layoutWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setMaximumSize(QSize(16777215, 200));
        gridLayoutWidget_2 = new QWidget(groupBox_2);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(60, 50, 291, 140));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setHorizontalSpacing(12);
        gridLayout_2->setVerticalSpacing(20);
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        gimbal_up_button = new QPushButton(gridLayoutWidget_2);
        gimbal_up_button->setObjectName(QStringLiteral("gimbal_up_button"));

        gridLayout_2->addWidget(gimbal_up_button, 0, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_2, 0, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 0, 0, 1, 1);

        drone_yaw_left_button = new QPushButton(gridLayoutWidget_2);
        drone_yaw_left_button->setObjectName(QStringLiteral("drone_yaw_left_button"));

        gridLayout_2->addWidget(drone_yaw_left_button, 1, 0, 1, 1);

        drone_yaw_right_button = new QPushButton(gridLayoutWidget_2);
        drone_yaw_right_button->setObjectName(QStringLiteral("drone_yaw_right_button"));

        gridLayout_2->addWidget(drone_yaw_right_button, 1, 2, 1, 1);

        gimbal_down_button = new QPushButton(gridLayoutWidget_2);
        gimbal_down_button->setObjectName(QStringLiteral("gimbal_down_button"));

        gridLayout_2->addWidget(gimbal_down_button, 2, 1, 1, 1);


        horizontalLayout_2->addWidget(groupBox_2);


        verticalLayout->addLayout(horizontalLayout_2);

        widget = new QWidget(Form);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(180, 10, 389, 106));
        verticalLayout_2 = new QVBoxLayout(widget);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_2 = new QLabel(widget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_3->addWidget(label_2);

        current_uav_text = new QTextEdit(widget);
        current_uav_text->setObjectName(QStringLiteral("current_uav_text"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(current_uav_text->sizePolicy().hasHeightForWidth());
        current_uav_text->setSizePolicy(sizePolicy1);
        current_uav_text->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout_3->addWidget(current_uav_text);

        current_uav_connect_button = new QPushButton(widget);
        current_uav_connect_button->setObjectName(QStringLiteral("current_uav_connect_button"));

        horizontalLayout_3->addWidget(current_uav_connect_button);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        current_uav_start_button = new QPushButton(widget);
        current_uav_start_button->setObjectName(QStringLiteral("current_uav_start_button"));

        horizontalLayout_4->addWidget(current_uav_start_button);

        all_uavs_start_button = new QPushButton(widget);
        all_uavs_start_button->setObjectName(QStringLiteral("all_uavs_start_button"));

        horizontalLayout_4->addWidget(all_uavs_start_button);


        verticalLayout_2->addLayout(horizontalLayout_4);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", nullptr));
        groupBox_3->setTitle(QApplication::translate("Form", "UAV_X Control Panel", nullptr));
        uav_info_label->setText(QApplication::translate("Form", "UAV INFO : IP: XX.XX.XX.XX", nullptr));
        groupBox->setTitle(QApplication::translate("Form", "Tracking Operation", nullptr));
        label->setText(QApplication::translate("Form", "TargetIdInput", nullptr));
        target_stop_button->setText(QApplication::translate("Form", "Stop", nullptr));
        drone_return_button->setText(QApplication::translate("Form", "UAVReturn", nullptr));
        drone_attack_button->setText(QApplication::translate("Form", "UAVAttack", nullptr));
        target_follow_button->setText(QApplication::translate("Form", "TargetFollow", nullptr));
        target_lock_button->setText(QApplication::translate("Form", "TargetLock", nullptr));
        groupBox_2->setTitle(QApplication::translate("Form", "Drone or Gimbal Operation", nullptr));
        gimbal_up_button->setText(QApplication::translate("Form", "Gimbal Up", nullptr));
        drone_yaw_left_button->setText(QApplication::translate("Form", "Drone Yaw Left", nullptr));
        drone_yaw_right_button->setText(QApplication::translate("Form", "Drone Yaw Right", nullptr));
        gimbal_down_button->setText(QApplication::translate("Form", "Gimbal Down", nullptr));
        label_2->setText(QApplication::translate("Form", "UAV IP", nullptr));
        current_uav_text->setHtml(QApplication::translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">192.168.144.61</p></body></html>", nullptr));
        current_uav_connect_button->setText(QApplication::translate("Form", "Connect", nullptr));
        current_uav_start_button->setText(QApplication::translate("Form", "Current Uav Start", nullptr));
        all_uavs_start_button->setText(QApplication::translate("Form", "All UAVs Start", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SINGLE_UAV_CTRL_PANEL_H
