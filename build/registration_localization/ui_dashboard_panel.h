/********************************************************************************
** Form generated from reading UI file 'dashboard_panel.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DASHBOARD_PANEL_H
#define UI_DASHBOARD_PANEL_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QPushButton *launch_node_button;
    QLineEdit *location_box;
    QLineEdit *filename_box;
    QLabel *label;
    QLabel *label_2;
    QPushButton *load_button;
    QPushButton *align_button;
    QLabel *label_3;
    QLineEdit *topic_box;
    QLabel *label_4;
    QLabel *label_5;
    QPushButton *show_marker;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(400, 307);
        launch_node_button = new QPushButton(Form);
        launch_node_button->setObjectName(QString::fromUtf8("launch_node_button"));
        launch_node_button->setGeometry(QRect(20, 20, 171, 31));
        location_box = new QLineEdit(Form);
        location_box->setObjectName(QString::fromUtf8("location_box"));
        location_box->setGeometry(QRect(20, 120, 171, 27));
        filename_box = new QLineEdit(Form);
        filename_box->setObjectName(QString::fromUtf8("filename_box"));
        filename_box->setGeometry(QRect(20, 170, 131, 27));
        filename_box->setLayoutDirection(Qt::LeftToRight);
        filename_box->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label = new QLabel(Form);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 100, 66, 17));
        label_2 = new QLabel(Form);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 150, 66, 17));
        load_button = new QPushButton(Form);
        load_button->setObjectName(QString::fromUtf8("load_button"));
        load_button->setGeometry(QRect(20, 60, 171, 31));
        align_button = new QPushButton(Form);
        align_button->setObjectName(QString::fromUtf8("align_button"));
        align_button->setGeometry(QRect(20, 210, 171, 27));
        label_3 = new QLabel(Form);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 240, 141, 17));
        topic_box = new QLineEdit(Form);
        topic_box->setObjectName(QString::fromUtf8("topic_box"));
        topic_box->setGeometry(QRect(30, 260, 161, 27));
        label_4 = new QLabel(Form);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 256, 20, 31));
        label_5 = new QLabel(Form);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(150, 170, 66, 31));
        show_marker = new QPushButton(Form);
        show_marker->setObjectName(QString::fromUtf8("show_marker"));
        show_marker->setGeometry(QRect(210, 90, 98, 27));

        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", 0, QApplication::UnicodeUTF8));
        launch_node_button->setText(QApplication::translate("Form", "Save Marker", 0, QApplication::UnicodeUTF8));
        location_box->setText(QApplication::translate("Form", "/home/mike/marker/", 0, QApplication::UnicodeUTF8));
        filename_box->setText(QApplication::translate("Form", "marker", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Form", "Location", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Form", "Filename", 0, QApplication::UnicodeUTF8));
        load_button->setText(QApplication::translate("Form", "Load Marker", 0, QApplication::UnicodeUTF8));
        align_button->setText(QApplication::translate("Form", "Align", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Form", "Input Cloud Topic", 0, QApplication::UnicodeUTF8));
        topic_box->setText(QApplication::translate("Form", "assembled_cloud", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("Form", "/", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("Form", ".pcd", 0, QApplication::UnicodeUTF8));
        show_marker->setText(QApplication::translate("Form", "ShowMarker", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DASHBOARD_PANEL_H
