/********************************************************************************
** Form generated from reading UI file 'wheelchair_medical.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WHEELCHAIR_MEDICAL_H
#define UI_WHEELCHAIR_MEDICAL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDateTimeEdit>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_wheelchair_medical
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *gbCallWheelchair;
    QFormLayout *formCall;
    QLabel *lbPatientName;
    QLineEdit *lePatientName;
    QLabel *lbRoomClinic;
    QHBoxLayout *hlRoomClinic;
    QLineEdit *leRoom;
    QLineEdit *leClinic;
    QLabel *lbWheelchairEmergency;
    QHBoxLayout *hlWheelchairEmergency;
    QComboBox *cbWheelchair;
    QCheckBox *cbEmergency;
    QLabel *lbReserveTime;
    QHBoxLayout *hlReserveTime;
    QDateTimeEdit *dtReserveTime;
    QPushButton *pbSendWheelchair;
    QGroupBox *gbReservation;
    QVBoxLayout *vlReservation;
    QTableWidget *twReservation;
    QHBoxLayout *hlBottom;
    QGroupBox *gbStatusDetail;
    QVBoxLayout *vlStatus;
    QHBoxLayout *hlStatusLine;
    QLabel *lbStatusTitle;
    QLabel *lbStatusValue;
    QHBoxLayout *hlStatusIndicator;
    QLabel *lbStatusRequest;
    QLabel *lbStatusMoving;
    QLabel *lbStatusArrived;
    QGroupBox *gbBoarding;
    QHBoxLayout *hlBoarding;
    QLabel *lbBoardingTitle;
    QCheckBox *cbBoarded;

    void setupUi(QWidget *wheelchair_medical)
    {
        if (wheelchair_medical->objectName().isEmpty())
            wheelchair_medical->setObjectName(QString::fromUtf8("wheelchair_medical"));
        wheelchair_medical->resize(900, 600);
        verticalLayout = new QVBoxLayout(wheelchair_medical);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gbCallWheelchair = new QGroupBox(wheelchair_medical);
        gbCallWheelchair->setObjectName(QString::fromUtf8("gbCallWheelchair"));
        formCall = new QFormLayout(gbCallWheelchair);
        formCall->setObjectName(QString::fromUtf8("formCall"));
        lbPatientName = new QLabel(gbCallWheelchair);
        lbPatientName->setObjectName(QString::fromUtf8("lbPatientName"));

        formCall->setWidget(0, QFormLayout::LabelRole, lbPatientName);

        lePatientName = new QLineEdit(gbCallWheelchair);
        lePatientName->setObjectName(QString::fromUtf8("lePatientName"));

        formCall->setWidget(0, QFormLayout::FieldRole, lePatientName);

        lbRoomClinic = new QLabel(gbCallWheelchair);
        lbRoomClinic->setObjectName(QString::fromUtf8("lbRoomClinic"));

        formCall->setWidget(1, QFormLayout::LabelRole, lbRoomClinic);

        hlRoomClinic = new QHBoxLayout();
        hlRoomClinic->setObjectName(QString::fromUtf8("hlRoomClinic"));
        leRoom = new QLineEdit(gbCallWheelchair);
        leRoom->setObjectName(QString::fromUtf8("leRoom"));

        hlRoomClinic->addWidget(leRoom);

        leClinic = new QLineEdit(gbCallWheelchair);
        leClinic->setObjectName(QString::fromUtf8("leClinic"));

        hlRoomClinic->addWidget(leClinic);


        formCall->setLayout(1, QFormLayout::FieldRole, hlRoomClinic);

        lbWheelchairEmergency = new QLabel(gbCallWheelchair);
        lbWheelchairEmergency->setObjectName(QString::fromUtf8("lbWheelchairEmergency"));

        formCall->setWidget(2, QFormLayout::LabelRole, lbWheelchairEmergency);

        hlWheelchairEmergency = new QHBoxLayout();
        hlWheelchairEmergency->setObjectName(QString::fromUtf8("hlWheelchairEmergency"));
        cbWheelchair = new QComboBox(gbCallWheelchair);
        cbWheelchair->addItem(QString());
        cbWheelchair->addItem(QString());
        cbWheelchair->addItem(QString());
        cbWheelchair->setObjectName(QString::fromUtf8("cbWheelchair"));

        hlWheelchairEmergency->addWidget(cbWheelchair);

        cbEmergency = new QCheckBox(gbCallWheelchair);
        cbEmergency->setObjectName(QString::fromUtf8("cbEmergency"));

        hlWheelchairEmergency->addWidget(cbEmergency);


        formCall->setLayout(2, QFormLayout::FieldRole, hlWheelchairEmergency);

        lbReserveTime = new QLabel(gbCallWheelchair);
        lbReserveTime->setObjectName(QString::fromUtf8("lbReserveTime"));

        formCall->setWidget(3, QFormLayout::LabelRole, lbReserveTime);

        hlReserveTime = new QHBoxLayout();
        hlReserveTime->setObjectName(QString::fromUtf8("hlReserveTime"));
        dtReserveTime = new QDateTimeEdit(gbCallWheelchair);
        dtReserveTime->setObjectName(QString::fromUtf8("dtReserveTime"));
        dtReserveTime->setCalendarPopup(true);

        hlReserveTime->addWidget(dtReserveTime);

        pbSendWheelchair = new QPushButton(gbCallWheelchair);
        pbSendWheelchair->setObjectName(QString::fromUtf8("pbSendWheelchair"));

        hlReserveTime->addWidget(pbSendWheelchair);


        formCall->setLayout(3, QFormLayout::FieldRole, hlReserveTime);


        verticalLayout->addWidget(gbCallWheelchair);

        gbReservation = new QGroupBox(wheelchair_medical);
        gbReservation->setObjectName(QString::fromUtf8("gbReservation"));
        vlReservation = new QVBoxLayout(gbReservation);
        vlReservation->setObjectName(QString::fromUtf8("vlReservation"));
        twReservation = new QTableWidget(gbReservation);
        if (twReservation->columnCount() < 7)
            twReservation->setColumnCount(7);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        twReservation->setHorizontalHeaderItem(6, __qtablewidgetitem6);
        twReservation->setObjectName(QString::fromUtf8("twReservation"));
        twReservation->setColumnCount(7);
        twReservation->setRowCount(0);

        vlReservation->addWidget(twReservation);


        verticalLayout->addWidget(gbReservation);

        hlBottom = new QHBoxLayout();
        hlBottom->setObjectName(QString::fromUtf8("hlBottom"));
        gbStatusDetail = new QGroupBox(wheelchair_medical);
        gbStatusDetail->setObjectName(QString::fromUtf8("gbStatusDetail"));
        vlStatus = new QVBoxLayout(gbStatusDetail);
        vlStatus->setObjectName(QString::fromUtf8("vlStatus"));
        hlStatusLine = new QHBoxLayout();
        hlStatusLine->setObjectName(QString::fromUtf8("hlStatusLine"));
        lbStatusTitle = new QLabel(gbStatusDetail);
        lbStatusTitle->setObjectName(QString::fromUtf8("lbStatusTitle"));

        hlStatusLine->addWidget(lbStatusTitle);

        lbStatusValue = new QLabel(gbStatusDetail);
        lbStatusValue->setObjectName(QString::fromUtf8("lbStatusValue"));

        hlStatusLine->addWidget(lbStatusValue);


        vlStatus->addLayout(hlStatusLine);

        hlStatusIndicator = new QHBoxLayout();
        hlStatusIndicator->setObjectName(QString::fromUtf8("hlStatusIndicator"));
        lbStatusRequest = new QLabel(gbStatusDetail);
        lbStatusRequest->setObjectName(QString::fromUtf8("lbStatusRequest"));

        hlStatusIndicator->addWidget(lbStatusRequest);

        lbStatusMoving = new QLabel(gbStatusDetail);
        lbStatusMoving->setObjectName(QString::fromUtf8("lbStatusMoving"));

        hlStatusIndicator->addWidget(lbStatusMoving);

        lbStatusArrived = new QLabel(gbStatusDetail);
        lbStatusArrived->setObjectName(QString::fromUtf8("lbStatusArrived"));

        hlStatusIndicator->addWidget(lbStatusArrived);


        vlStatus->addLayout(hlStatusIndicator);


        hlBottom->addWidget(gbStatusDetail);

        gbBoarding = new QGroupBox(wheelchair_medical);
        gbBoarding->setObjectName(QString::fromUtf8("gbBoarding"));
        hlBoarding = new QHBoxLayout(gbBoarding);
        hlBoarding->setObjectName(QString::fromUtf8("hlBoarding"));
        lbBoardingTitle = new QLabel(gbBoarding);
        lbBoardingTitle->setObjectName(QString::fromUtf8("lbBoardingTitle"));

        hlBoarding->addWidget(lbBoardingTitle);

        cbBoarded = new QCheckBox(gbBoarding);
        cbBoarded->setObjectName(QString::fromUtf8("cbBoarded"));

        hlBoarding->addWidget(cbBoarded);


        hlBottom->addWidget(gbBoarding);


        verticalLayout->addLayout(hlBottom);


        retranslateUi(wheelchair_medical);

        QMetaObject::connectSlotsByName(wheelchair_medical);
    } // setupUi

    void retranslateUi(QWidget *wheelchair_medical)
    {
        wheelchair_medical->setWindowTitle(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 \352\264\200\353\246\254", nullptr));
        gbCallWheelchair->setTitle(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 \355\230\270\354\266\234 \354\204\244\354\240\225", nullptr));
        lbPatientName->setText(QCoreApplication::translate("wheelchair_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204", nullptr));
        lePatientName->setPlaceholderText(QCoreApplication::translate("wheelchair_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204 \354\236\205\353\240\245", nullptr));
        lbRoomClinic->setText(QCoreApplication::translate("wheelchair_medical", "\353\263\221\354\213\244 / \354\247\204\353\243\214\354\213\244", nullptr));
        leRoom->setPlaceholderText(QCoreApplication::translate("wheelchair_medical", "\354\230\210) 3\354\270\265 305\355\230\270", nullptr));
        leClinic->setPlaceholderText(QCoreApplication::translate("wheelchair_medical", "\354\230\210) \353\202\264\352\263\274 1\354\247\204\353\243\214\354\213\244", nullptr));
        lbWheelchairEmergency->setText(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 / \354\235\221\352\270\211", nullptr));
        cbWheelchair->setItemText(0, QCoreApplication::translate("wheelchair_medical", "WC-1", nullptr));
        cbWheelchair->setItemText(1, QCoreApplication::translate("wheelchair_medical", "WC-2", nullptr));
        cbWheelchair->setItemText(2, QCoreApplication::translate("wheelchair_medical", "WC-3", nullptr));

        cbEmergency->setText(QCoreApplication::translate("wheelchair_medical", "\354\235\221\352\270\211", nullptr));
        lbReserveTime->setText(QCoreApplication::translate("wheelchair_medical", "\354\230\210\354\225\275 \354\213\234\352\260\204", nullptr));
        dtReserveTime->setDisplayFormat(QCoreApplication::translate("wheelchair_medical", "yyyy-MM-dd HH:mm", nullptr));
        pbSendWheelchair->setText(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 \355\230\270\354\266\234", nullptr));
        gbReservation->setTitle(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 / \354\230\210\354\225\275 \355\230\204\355\231\251", nullptr));
        QTableWidgetItem *___qtablewidgetitem = twReservation->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("wheelchair_medical", "\354\204\240\355\203\235", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = twReservation->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = twReservation->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("wheelchair_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = twReservation->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("wheelchair_medical", "\353\263\221\354\213\244", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = twReservation->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("wheelchair_medical", "\354\247\204\353\243\214\354\213\244", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = twReservation->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("wheelchair_medical", "\354\230\210\354\225\275 \354\213\234\352\260\204", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = twReservation->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("wheelchair_medical", "\354\203\201\355\203\234", nullptr));
        gbStatusDetail->setTitle(QCoreApplication::translate("wheelchair_medical", "\355\234\240\354\262\264\354\226\264 \354\203\201\355\203\234", nullptr));
        lbStatusTitle->setText(QCoreApplication::translate("wheelchair_medical", "\355\230\204\354\236\254 \354\203\201\355\203\234:", nullptr));
        lbStatusValue->setText(QCoreApplication::translate("wheelchair_medical", "\353\214\200\352\270\260\354\244\221", nullptr));
        lbStatusRequest->setText(QCoreApplication::translate("wheelchair_medical", "\354\232\224\354\262\255\354\244\221", nullptr));
        lbStatusMoving->setText(QCoreApplication::translate("wheelchair_medical", "\354\235\264\353\217\231\354\244\221", nullptr));
        lbStatusArrived->setText(QCoreApplication::translate("wheelchair_medical", "\353\217\204\354\260\251", nullptr));
        gbBoarding->setTitle(QCoreApplication::translate("wheelchair_medical", "\355\203\221\354\212\271 \354\240\225\353\263\264", nullptr));
        lbBoardingTitle->setText(QCoreApplication::translate("wheelchair_medical", "\355\203\221\354\212\271 \354\227\254\353\266\200:", nullptr));
        cbBoarded->setText(QCoreApplication::translate("wheelchair_medical", "\355\203\221\354\212\271 \354\231\204\353\243\214", nullptr));
    } // retranslateUi

};

namespace Ui {
    class wheelchair_medical: public Ui_wheelchair_medical {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WHEELCHAIR_MEDICAL_H
