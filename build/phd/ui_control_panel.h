/********************************************************************************
** Form generated from reading UI file 'control_panel.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROL_PANEL_H
#define UI_CONTROL_PANEL_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QTabWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Control_Form
{
public:
    QTabWidget *tabWidget;
    QWidget *tab;
    QPushButton *fscan_button;
    QLineEdit *pos_box;
    QPushButton *nav_mode_button;
    QPushButton *scan_button;
    QPushButton *localization_button;
    QPushButton *localization_button_2;
    QPushButton *cluster1;
    QPushButton *gen_trajectory;
    QLabel *marker_info;
    QPushButton *start_point;
    QPushButton *exe_trajectory;
    QPushButton *soft_stop;
    QPushButton *plot_path;
    QPushButton *exe_path;
    QPushButton *thickness;
    QLabel *marker_info_2;
    QPushButton *step_button;
    QPushButton *est_pos;
    QPushButton *show_nav_button;
    QLineEdit *filename_box;
    QCheckBox *auto_localize;
    QCheckBox *set_home;
    QLineEdit *marker_name_box;
    QWidget *tab_2;
    QPushButton *armDown;
    QPushButton *roll_pos;
    QPushButton *armUp;
    QLabel *label_4;
    QFrame *frame_2;
    QPushButton *yaw_neg;
    QLabel *label;
    QLabel *label_5;
    QPushButton *yaw_pos;
    QPushButton *armLeft;
    QPushButton *turnLeft;
    QPushButton *roll_neg;
    QLabel *label_2;
    QPushButton *armRev;
    QPushButton *revButton;
    QPushButton *pitch_pos;
    QLabel *label_3;
    QPushButton *armRight;
    QPushButton *pitch_neg;
    QFrame *frame;
    QPushButton *armFwd;
    QPushButton *fwdButton;
    QPushButton *turnRight;

    void setupUi(QWidget *Control_Form)
    {
        if (Control_Form->objectName().isEmpty())
            Control_Form->setObjectName(QString::fromUtf8("Control_Form"));
        Control_Form->resize(276, 656);
        tabWidget = new QTabWidget(Control_Form);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 291, 661));
        tabWidget->setMovable(false);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        fscan_button = new QPushButton(tab);
        fscan_button->setObjectName(QString::fromUtf8("fscan_button"));
        fscan_button->setGeometry(QRect(10, 385, 81, 31));
        pos_box = new QLineEdit(tab);
        pos_box->setObjectName(QString::fromUtf8("pos_box"));
        pos_box->setGeometry(QRect(140, 304, 121, 31));
        nav_mode_button = new QPushButton(tab);
        nav_mode_button->setObjectName(QString::fromUtf8("nav_mode_button"));
        nav_mode_button->setGeometry(QRect(10, 305, 121, 31));
        scan_button = new QPushButton(tab);
        scan_button->setObjectName(QString::fromUtf8("scan_button"));
        scan_button->setGeometry(QRect(10, 0, 121, 31));
        localization_button = new QPushButton(tab);
        localization_button->setObjectName(QString::fromUtf8("localization_button"));
        localization_button->setGeometry(QRect(10, 465, 121, 31));
        localization_button_2 = new QPushButton(tab);
        localization_button_2->setObjectName(QString::fromUtf8("localization_button_2"));
        localization_button_2->setGeometry(QRect(10, 345, 121, 31));
        cluster1 = new QPushButton(tab);
        cluster1->setObjectName(QString::fromUtf8("cluster1"));
        cluster1->setGeometry(QRect(10, 70, 101, 27));
        gen_trajectory = new QPushButton(tab);
        gen_trajectory->setObjectName(QString::fromUtf8("gen_trajectory"));
        gen_trajectory->setEnabled(true);
        gen_trajectory->setGeometry(QRect(120, 110, 141, 27));
        marker_info = new QLabel(tab);
        marker_info->setObjectName(QString::fromUtf8("marker_info"));
        marker_info->setGeometry(QRect(120, 75, 151, 17));
        start_point = new QPushButton(tab);
        start_point->setObjectName(QString::fromUtf8("start_point"));
        start_point->setGeometry(QRect(10, 110, 101, 27));
        exe_trajectory = new QPushButton(tab);
        exe_trajectory->setObjectName(QString::fromUtf8("exe_trajectory"));
        exe_trajectory->setEnabled(true);
        exe_trajectory->setGeometry(QRect(120, 150, 141, 27));
        soft_stop = new QPushButton(tab);
        soft_stop->setObjectName(QString::fromUtf8("soft_stop"));
        soft_stop->setEnabled(true);
        soft_stop->setGeometry(QRect(10, 150, 101, 27));
        soft_stop->setStyleSheet(QString::fromUtf8("background-color: rgb(170, 0, 0);\n"
"color: rgb(255, 255, 255);"));
        plot_path = new QPushButton(tab);
        plot_path->setObjectName(QString::fromUtf8("plot_path"));
        plot_path->setEnabled(true);
        plot_path->setGeometry(QRect(10, 190, 111, 27));
        exe_path = new QPushButton(tab);
        exe_path->setObjectName(QString::fromUtf8("exe_path"));
        exe_path->setEnabled(true);
        exe_path->setGeometry(QRect(130, 190, 131, 27));
        thickness = new QPushButton(tab);
        thickness->setObjectName(QString::fromUtf8("thickness"));
        thickness->setEnabled(true);
        thickness->setGeometry(QRect(130, 225, 131, 31));
        marker_info_2 = new QLabel(tab);
        marker_info_2->setObjectName(QString::fromUtf8("marker_info_2"));
        marker_info_2->setGeometry(QRect(20, 275, 221, 17));
        step_button = new QPushButton(tab);
        step_button->setObjectName(QString::fromUtf8("step_button"));
        step_button->setGeometry(QRect(10, 425, 121, 31));
        est_pos = new QPushButton(tab);
        est_pos->setObjectName(QString::fromUtf8("est_pos"));
        est_pos->setGeometry(QRect(140, 425, 121, 31));
        show_nav_button = new QPushButton(tab);
        show_nav_button->setObjectName(QString::fromUtf8("show_nav_button"));
        show_nav_button->setGeometry(QRect(10, 225, 111, 31));
        filename_box = new QLineEdit(tab);
        filename_box->setObjectName(QString::fromUtf8("filename_box"));
        filename_box->setGeometry(QRect(100, 385, 161, 31));
        auto_localize = new QCheckBox(tab);
        auto_localize->setObjectName(QString::fromUtf8("auto_localize"));
        auto_localize->setGeometry(QRect(140, 346, 121, 30));
        set_home = new QCheckBox(tab);
        set_home->setObjectName(QString::fromUtf8("set_home"));
        set_home->setGeometry(QRect(140, 2, 121, 30));
        set_home->setChecked(true);
        marker_name_box = new QLineEdit(tab);
        marker_name_box->setObjectName(QString::fromUtf8("marker_name_box"));
        marker_name_box->setGeometry(QRect(10, 34, 251, 30));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        armDown = new QPushButton(tab_2);
        armDown->setObjectName(QString::fromUtf8("armDown"));
        armDown->setGeometry(QRect(100, 330, 71, 71));
        armDown->setMouseTracking(false);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/newPrefix/aiga-down-arrow-bg.svg"), QSize(), QIcon::Normal, QIcon::Off);
        armDown->setIcon(icon);
        armDown->setIconSize(QSize(60, 60));
        roll_pos = new QPushButton(tab_2);
        roll_pos->setObjectName(QString::fromUtf8("roll_pos"));
        roll_pos->setGeometry(QRect(180, 420, 71, 31));
        QFont font;
        font.setPointSize(24);
        roll_pos->setFont(font);
        armUp = new QPushButton(tab_2);
        armUp->setObjectName(QString::fromUtf8("armUp"));
        armUp->setGeometry(QRect(100, 250, 71, 71));
        armUp->setMouseTracking(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/newPrefix/aiga-up-arrow-bg.svg"), QSize(), QIcon::Normal, QIcon::Off);
        armUp->setIcon(icon1);
        armUp->setIconSize(QSize(60, 60));
        label_4 = new QLabel(tab_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(120, 454, 41, 21));
        label_4->setLayoutDirection(Qt::LeftToRight);
        frame_2 = new QFrame(tab_2);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(10, 240, 251, 171));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        yaw_neg = new QPushButton(tab_2);
        yaw_neg->setObjectName(QString::fromUtf8("yaw_neg"));
        yaw_neg->setGeometry(QRect(100, 476, 71, 31));
        yaw_neg->setFont(font);
        label = new QLabel(tab_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 20, 181, 17));
        label_5 = new QLabel(tab_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(200, 454, 41, 21));
        label_5->setLayoutDirection(Qt::LeftToRight);
        yaw_pos = new QPushButton(tab_2);
        yaw_pos->setObjectName(QString::fromUtf8("yaw_pos"));
        yaw_pos->setGeometry(QRect(100, 420, 71, 31));
        yaw_pos->setFont(font);
        armLeft = new QPushButton(tab_2);
        armLeft->setObjectName(QString::fromUtf8("armLeft"));
        armLeft->setGeometry(QRect(20, 250, 71, 71));
        armLeft->setMouseTracking(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/newPrefix/aiga-left-arrow-bg.svg"), QSize(), QIcon::Normal, QIcon::Off);
        armLeft->setIcon(icon2);
        armLeft->setIconSize(QSize(60, 60));
        turnLeft = new QPushButton(tab_2);
        turnLeft->setObjectName(QString::fromUtf8("turnLeft"));
        turnLeft->setGeometry(QRect(20, 90, 71, 71));
        turnLeft->setMouseTracking(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/newPrefix/Rotate-Arrow-Bottom-to-Left.svg"), QSize(), QIcon::Normal, QIcon::Off);
        turnLeft->setIcon(icon3);
        turnLeft->setIconSize(QSize(60, 60));
        roll_neg = new QPushButton(tab_2);
        roll_neg->setObjectName(QString::fromUtf8("roll_neg"));
        roll_neg->setGeometry(QRect(180, 476, 71, 31));
        roll_neg->setFont(font);
        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 216, 181, 21));
        armRev = new QPushButton(tab_2);
        armRev->setObjectName(QString::fromUtf8("armRev"));
        armRev->setGeometry(QRect(20, 330, 71, 71));
        armRev->setMouseTracking(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/newPrefix/aiga-left-and-down-arrow.svg"), QSize(), QIcon::Normal, QIcon::Off);
        armRev->setIcon(icon4);
        armRev->setIconSize(QSize(60, 60));
        revButton = new QPushButton(tab_2);
        revButton->setObjectName(QString::fromUtf8("revButton"));
        revButton->setGeometry(QRect(100, 130, 71, 71));
        revButton->setMouseTracking(false);
        revButton->setIcon(icon);
        revButton->setIconSize(QSize(60, 60));
        pitch_pos = new QPushButton(tab_2);
        pitch_pos->setObjectName(QString::fromUtf8("pitch_pos"));
        pitch_pos->setGeometry(QRect(20, 420, 71, 31));
        pitch_pos->setFont(font);
        label_3 = new QLabel(tab_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(40, 454, 41, 21));
        label_3->setLayoutDirection(Qt::LeftToRight);
        armRight = new QPushButton(tab_2);
        armRight->setObjectName(QString::fromUtf8("armRight"));
        armRight->setGeometry(QRect(180, 330, 71, 71));
        armRight->setMouseTracking(false);
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/newPrefix/aiga-right-arrow-bg.svg"), QSize(), QIcon::Normal, QIcon::Off);
        armRight->setIcon(icon5);
        armRight->setIconSize(QSize(60, 60));
        pitch_neg = new QPushButton(tab_2);
        pitch_neg->setObjectName(QString::fromUtf8("pitch_neg"));
        pitch_neg->setGeometry(QRect(20, 476, 71, 31));
        pitch_neg->setFont(font);
        frame = new QFrame(tab_2);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(10, 40, 251, 171));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        armFwd = new QPushButton(tab_2);
        armFwd->setObjectName(QString::fromUtf8("armFwd"));
        armFwd->setGeometry(QRect(180, 250, 71, 71));
        armFwd->setMouseTracking(false);
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/newPrefix/aiga-forward-and-right-arow-bg.svg"), QSize(), QIcon::Normal, QIcon::Off);
        armFwd->setIcon(icon6);
        armFwd->setIconSize(QSize(60, 60));
        fwdButton = new QPushButton(tab_2);
        fwdButton->setObjectName(QString::fromUtf8("fwdButton"));
        fwdButton->setGeometry(QRect(100, 50, 71, 71));
        fwdButton->setMouseTracking(false);
        fwdButton->setLayoutDirection(Qt::LeftToRight);
        fwdButton->setIcon(icon1);
        fwdButton->setIconSize(QSize(60, 60));
        turnRight = new QPushButton(tab_2);
        turnRight->setObjectName(QString::fromUtf8("turnRight"));
        turnRight->setGeometry(QRect(180, 90, 71, 71));
        turnRight->setMouseTracking(false);
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/newPrefix/Rotate-Arrow-Bottom-to-Right.svg"), QSize(), QIcon::Normal, QIcon::Off);
        turnRight->setIcon(icon7);
        turnRight->setIconSize(QSize(60, 60));
        tabWidget->addTab(tab_2, QString());

        retranslateUi(Control_Form);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Control_Form);
    } // setupUi

    void retranslateUi(QWidget *Control_Form)
    {
        Control_Form->setWindowTitle(QApplication::translate("Control_Form", "Form", 0, QApplication::UnicodeUTF8));
        fscan_button->setText(QApplication::translate("Control_Form", "Fake Scan", 0, QApplication::UnicodeUTF8));
        pos_box->setText(QApplication::translate("Control_Form", "-1.57", 0, QApplication::UnicodeUTF8));
        nav_mode_button->setText(QApplication::translate("Control_Form", "Nav Mode", 0, QApplication::UnicodeUTF8));
        scan_button->setText(QApplication::translate("Control_Form", "Scan", 0, QApplication::UnicodeUTF8));
        localization_button->setText(QApplication::translate("Control_Form", "Set Home", 0, QApplication::UnicodeUTF8));
        localization_button_2->setText(QApplication::translate("Control_Form", "Test Localization", 0, QApplication::UnicodeUTF8));
        cluster1->setText(QApplication::translate("Control_Form", "Cluster Pt.1", 0, QApplication::UnicodeUTF8));
        gen_trajectory->setText(QApplication::translate("Control_Form", "Generate Trajectory", 0, QApplication::UnicodeUTF8));
        marker_info->setText(QApplication::translate("Control_Form", "Ready to cluster", 0, QApplication::UnicodeUTF8));
        start_point->setText(QApplication::translate("Control_Form", "Start Point", 0, QApplication::UnicodeUTF8));
        exe_trajectory->setText(QApplication::translate("Control_Form", "Execute Trajectory", 0, QApplication::UnicodeUTF8));
        soft_stop->setText(QApplication::translate("Control_Form", "STOP", 0, QApplication::UnicodeUTF8));
        plot_path->setText(QApplication::translate("Control_Form", "Plot Path", 0, QApplication::UnicodeUTF8));
        exe_path->setText(QApplication::translate("Control_Form", "Execute Path", 0, QApplication::UnicodeUTF8));
        thickness->setText(QApplication::translate("Control_Form", "Estimate Thickness", 0, QApplication::UnicodeUTF8));
        marker_info_2->setText(QApplication::translate("Control_Form", "Testing/Research Functions", 0, QApplication::UnicodeUTF8));
        step_button->setText(QApplication::translate("Control_Form", "Step", 0, QApplication::UnicodeUTF8));
        est_pos->setText(QApplication::translate("Control_Form", "Estimate Position", 0, QApplication::UnicodeUTF8));
        show_nav_button->setText(QApplication::translate("Control_Form", "Show Nav Goal", 0, QApplication::UnicodeUTF8));
        filename_box->setText(QApplication::translate("Control_Form", "/home/mike/testing/file.pcd", 0, QApplication::UnicodeUTF8));
        auto_localize->setText(QApplication::translate("Control_Form", "Auto Localize", 0, QApplication::UnicodeUTF8));
        set_home->setText(QApplication::translate("Control_Form", "Set Home", 0, QApplication::UnicodeUTF8));
        marker_name_box->setText(QApplication::translate("Control_Form", "/home/mike/testing/", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("Control_Form", "Operation", 0, QApplication::UnicodeUTF8));
        armDown->setText(QString());
        roll_pos->setText(QApplication::translate("Control_Form", "+", 0, QApplication::UnicodeUTF8));
        armUp->setText(QString());
        label_4->setText(QApplication::translate("Control_Form", "Yaw", 0, QApplication::UnicodeUTF8));
        yaw_neg->setText(QApplication::translate("Control_Form", "-", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Control_Form", "Manual Base Control", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("Control_Form", "Roll", 0, QApplication::UnicodeUTF8));
        yaw_pos->setText(QApplication::translate("Control_Form", "+", 0, QApplication::UnicodeUTF8));
        armLeft->setText(QString());
        turnLeft->setText(QString());
        roll_neg->setText(QApplication::translate("Control_Form", "-", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Control_Form", "Manual Arm Control", 0, QApplication::UnicodeUTF8));
        armRev->setText(QString());
        revButton->setText(QString());
        pitch_pos->setText(QApplication::translate("Control_Form", "+", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Control_Form", "Pitch", 0, QApplication::UnicodeUTF8));
        armRight->setText(QString());
        pitch_neg->setText(QApplication::translate("Control_Form", "-", 0, QApplication::UnicodeUTF8));
        armFwd->setText(QString());
        fwdButton->setText(QString());
        turnRight->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("Control_Form", "Navigation", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Control_Form: public Ui_Control_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROL_PANEL_H
