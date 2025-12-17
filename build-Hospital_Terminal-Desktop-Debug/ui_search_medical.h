/********************************************************************************
** Form generated from reading UI file 'search_medical.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SEARCH_MEDICAL_H
#define UI_SEARCH_MEDICAL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
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

class Ui_search_medical
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *hlSearchPatient;
    QLabel *lbPatientName;
    QLineEdit *leSearchName;
    QPushButton *pbSearchPatient;
    QTableWidget *twPatientList;
    QGroupBox *gbPatientDetail;
    QFormLayout *formLayout;
    QLabel *lbEditName;
    QLineEdit *leEditName;
    QLabel *lbEditDisease;
    QLineEdit *leEditDisease;
    QLabel *lbEditRoom;
    QLineEdit *leEditRoom;
    QLabel *lbEditClinic;
    QLineEdit *leEditClinic;
    QLabel *lbEditType;
    QComboBox *cbEditType;
    QLabel *lbEditButtons;
    QHBoxLayout *hlEditButtons;
    QPushButton *pbAddPatient;
    QPushButton *pbUpdatePatient;
    QPushButton *pbDeletePatient;

    void setupUi(QWidget *search_medical)
    {
        if (search_medical->objectName().isEmpty())
            search_medical->setObjectName(QString::fromUtf8("search_medical"));
        search_medical->resize(900, 600);
        verticalLayout = new QVBoxLayout(search_medical);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        hlSearchPatient = new QHBoxLayout();
        hlSearchPatient->setObjectName(QString::fromUtf8("hlSearchPatient"));
        lbPatientName = new QLabel(search_medical);
        lbPatientName->setObjectName(QString::fromUtf8("lbPatientName"));

        hlSearchPatient->addWidget(lbPatientName);

        leSearchName = new QLineEdit(search_medical);
        leSearchName->setObjectName(QString::fromUtf8("leSearchName"));

        hlSearchPatient->addWidget(leSearchName);

        pbSearchPatient = new QPushButton(search_medical);
        pbSearchPatient->setObjectName(QString::fromUtf8("pbSearchPatient"));

        hlSearchPatient->addWidget(pbSearchPatient);


        verticalLayout->addLayout(hlSearchPatient);

        twPatientList = new QTableWidget(search_medical);
        if (twPatientList->columnCount() < 6)
            twPatientList->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        twPatientList->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        twPatientList->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        twPatientList->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        twPatientList->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        twPatientList->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        twPatientList->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        twPatientList->setObjectName(QString::fromUtf8("twPatientList"));
        twPatientList->setColumnCount(6);
        twPatientList->setRowCount(0);

        verticalLayout->addWidget(twPatientList);

        gbPatientDetail = new QGroupBox(search_medical);
        gbPatientDetail->setObjectName(QString::fromUtf8("gbPatientDetail"));
        formLayout = new QFormLayout(gbPatientDetail);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        lbEditName = new QLabel(gbPatientDetail);
        lbEditName->setObjectName(QString::fromUtf8("lbEditName"));

        formLayout->setWidget(0, QFormLayout::LabelRole, lbEditName);

        leEditName = new QLineEdit(gbPatientDetail);
        leEditName->setObjectName(QString::fromUtf8("leEditName"));

        formLayout->setWidget(0, QFormLayout::FieldRole, leEditName);

        lbEditDisease = new QLabel(gbPatientDetail);
        lbEditDisease->setObjectName(QString::fromUtf8("lbEditDisease"));

        formLayout->setWidget(1, QFormLayout::LabelRole, lbEditDisease);

        leEditDisease = new QLineEdit(gbPatientDetail);
        leEditDisease->setObjectName(QString::fromUtf8("leEditDisease"));

        formLayout->setWidget(1, QFormLayout::FieldRole, leEditDisease);

        lbEditRoom = new QLabel(gbPatientDetail);
        lbEditRoom->setObjectName(QString::fromUtf8("lbEditRoom"));

        formLayout->setWidget(2, QFormLayout::LabelRole, lbEditRoom);

        leEditRoom = new QLineEdit(gbPatientDetail);
        leEditRoom->setObjectName(QString::fromUtf8("leEditRoom"));

        formLayout->setWidget(2, QFormLayout::FieldRole, leEditRoom);

        lbEditClinic = new QLabel(gbPatientDetail);
        lbEditClinic->setObjectName(QString::fromUtf8("lbEditClinic"));

        formLayout->setWidget(3, QFormLayout::LabelRole, lbEditClinic);

        leEditClinic = new QLineEdit(gbPatientDetail);
        leEditClinic->setObjectName(QString::fromUtf8("leEditClinic"));

        formLayout->setWidget(3, QFormLayout::FieldRole, leEditClinic);

        lbEditType = new QLabel(gbPatientDetail);
        lbEditType->setObjectName(QString::fromUtf8("lbEditType"));

        formLayout->setWidget(4, QFormLayout::LabelRole, lbEditType);

        cbEditType = new QComboBox(gbPatientDetail);
        cbEditType->addItem(QString());
        cbEditType->addItem(QString());
        cbEditType->setObjectName(QString::fromUtf8("cbEditType"));

        formLayout->setWidget(4, QFormLayout::FieldRole, cbEditType);

        lbEditButtons = new QLabel(gbPatientDetail);
        lbEditButtons->setObjectName(QString::fromUtf8("lbEditButtons"));

        formLayout->setWidget(5, QFormLayout::LabelRole, lbEditButtons);

        hlEditButtons = new QHBoxLayout();
        hlEditButtons->setObjectName(QString::fromUtf8("hlEditButtons"));
        pbAddPatient = new QPushButton(gbPatientDetail);
        pbAddPatient->setObjectName(QString::fromUtf8("pbAddPatient"));

        hlEditButtons->addWidget(pbAddPatient);

        pbUpdatePatient = new QPushButton(gbPatientDetail);
        pbUpdatePatient->setObjectName(QString::fromUtf8("pbUpdatePatient"));

        hlEditButtons->addWidget(pbUpdatePatient);

        pbDeletePatient = new QPushButton(gbPatientDetail);
        pbDeletePatient->setObjectName(QString::fromUtf8("pbDeletePatient"));

        hlEditButtons->addWidget(pbDeletePatient);


        formLayout->setLayout(5, QFormLayout::FieldRole, hlEditButtons);


        verticalLayout->addWidget(gbPatientDetail);


        retranslateUi(search_medical);

        QMetaObject::connectSlotsByName(search_medical);
    } // setupUi

    void retranslateUi(QWidget *search_medical)
    {
        search_medical->setWindowTitle(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \354\241\260\355\232\214", nullptr));
        lbPatientName->setText(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204", nullptr));
        leSearchName->setPlaceholderText(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204 \354\236\205\353\240\245", nullptr));
        pbSearchPatient->setText(QCoreApplication::translate("search_medical", "\354\241\260\355\232\214", nullptr));
        QTableWidgetItem *___qtablewidgetitem = twPatientList->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("search_medical", "\354\204\240\355\203\235", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = twPatientList->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = twPatientList->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("search_medical", "\353\263\221\353\252\205", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = twPatientList->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("search_medical", "\353\263\221\354\213\244", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = twPatientList->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("search_medical", "\354\247\204\353\243\214\354\213\244", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = twPatientList->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \355\203\200\354\236\205", nullptr));
        gbPatientDetail->setTitle(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \354\240\225\353\263\264", nullptr));
        lbEditName->setText(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \354\235\264\353\246\204", nullptr));
        lbEditDisease->setText(QCoreApplication::translate("search_medical", "\353\263\221\353\252\205", nullptr));
        lbEditRoom->setText(QCoreApplication::translate("search_medical", "\353\263\221\354\213\244", nullptr));
        lbEditClinic->setText(QCoreApplication::translate("search_medical", "\354\247\204\353\243\214\354\213\244", nullptr));
        lbEditType->setText(QCoreApplication::translate("search_medical", "\355\231\230\354\236\220 \355\203\200\354\236\205", nullptr));
        cbEditType->setItemText(0, QCoreApplication::translate("search_medical", "\354\236\205\354\233\220", nullptr));
        cbEditType->setItemText(1, QCoreApplication::translate("search_medical", "\354\231\270\353\236\230", nullptr));

        lbEditButtons->setText(QCoreApplication::translate("search_medical", "\354\236\221\354\227\205", nullptr));
        pbAddPatient->setText(QCoreApplication::translate("search_medical", "\354\266\224\352\260\200", nullptr));
        pbUpdatePatient->setText(QCoreApplication::translate("search_medical", "\354\210\230\354\240\225", nullptr));
        pbDeletePatient->setText(QCoreApplication::translate("search_medical", "\354\202\255\354\240\234", nullptr));
    } // retranslateUi

};

namespace Ui {
    class search_medical: public Ui_search_medical {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SEARCH_MEDICAL_H
