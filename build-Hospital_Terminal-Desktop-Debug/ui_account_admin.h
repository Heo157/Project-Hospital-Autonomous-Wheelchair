/********************************************************************************
** Form generated from reading UI file 'account_admin.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ACCOUNT_ADMIN_H
#define UI_ACCOUNT_ADMIN_H

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

class Ui_account_admin
{
public:
    QVBoxLayout *verticalLayout;
    QGroupBox *gbUserEdit;
    QFormLayout *formLayoutUser;
    QLabel *lbUserId;
    QLineEdit *leUserId;
    QLabel *lbUserName;
    QLineEdit *leUserName;
    QLabel *lbRole;
    QComboBox *cbRole;
    QLabel *lbNote;
    QLineEdit *leNote;
    QLabel *lbButtons;
    QHBoxLayout *hlButtons;
    QPushButton *pbAddUser;
    QPushButton *pbUpdateUser;
    QPushButton *pbDeleteUser;
    QPushButton *pbClearForm;
    QGroupBox *gbUserList;
    QVBoxLayout *vlUserList;
    QTableWidget *twUserList;

    void setupUi(QWidget *account_admin)
    {
        if (account_admin->objectName().isEmpty())
            account_admin->setObjectName(QString::fromUtf8("account_admin"));
        account_admin->resize(900, 600);
        verticalLayout = new QVBoxLayout(account_admin);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gbUserEdit = new QGroupBox(account_admin);
        gbUserEdit->setObjectName(QString::fromUtf8("gbUserEdit"));
        formLayoutUser = new QFormLayout(gbUserEdit);
        formLayoutUser->setObjectName(QString::fromUtf8("formLayoutUser"));
        lbUserId = new QLabel(gbUserEdit);
        lbUserId->setObjectName(QString::fromUtf8("lbUserId"));

        formLayoutUser->setWidget(0, QFormLayout::LabelRole, lbUserId);

        leUserId = new QLineEdit(gbUserEdit);
        leUserId->setObjectName(QString::fromUtf8("leUserId"));

        formLayoutUser->setWidget(0, QFormLayout::FieldRole, leUserId);

        lbUserName = new QLabel(gbUserEdit);
        lbUserName->setObjectName(QString::fromUtf8("lbUserName"));

        formLayoutUser->setWidget(1, QFormLayout::LabelRole, lbUserName);

        leUserName = new QLineEdit(gbUserEdit);
        leUserName->setObjectName(QString::fromUtf8("leUserName"));

        formLayoutUser->setWidget(1, QFormLayout::FieldRole, leUserName);

        lbRole = new QLabel(gbUserEdit);
        lbRole->setObjectName(QString::fromUtf8("lbRole"));

        formLayoutUser->setWidget(2, QFormLayout::LabelRole, lbRole);

        cbRole = new QComboBox(gbUserEdit);
        cbRole->addItem(QString());
        cbRole->addItem(QString());
        cbRole->setObjectName(QString::fromUtf8("cbRole"));

        formLayoutUser->setWidget(2, QFormLayout::FieldRole, cbRole);

        lbNote = new QLabel(gbUserEdit);
        lbNote->setObjectName(QString::fromUtf8("lbNote"));

        formLayoutUser->setWidget(3, QFormLayout::LabelRole, lbNote);

        leNote = new QLineEdit(gbUserEdit);
        leNote->setObjectName(QString::fromUtf8("leNote"));

        formLayoutUser->setWidget(3, QFormLayout::FieldRole, leNote);

        lbButtons = new QLabel(gbUserEdit);
        lbButtons->setObjectName(QString::fromUtf8("lbButtons"));

        formLayoutUser->setWidget(4, QFormLayout::LabelRole, lbButtons);

        hlButtons = new QHBoxLayout();
        hlButtons->setObjectName(QString::fromUtf8("hlButtons"));
        pbAddUser = new QPushButton(gbUserEdit);
        pbAddUser->setObjectName(QString::fromUtf8("pbAddUser"));

        hlButtons->addWidget(pbAddUser);

        pbUpdateUser = new QPushButton(gbUserEdit);
        pbUpdateUser->setObjectName(QString::fromUtf8("pbUpdateUser"));

        hlButtons->addWidget(pbUpdateUser);

        pbDeleteUser = new QPushButton(gbUserEdit);
        pbDeleteUser->setObjectName(QString::fromUtf8("pbDeleteUser"));

        hlButtons->addWidget(pbDeleteUser);

        pbClearForm = new QPushButton(gbUserEdit);
        pbClearForm->setObjectName(QString::fromUtf8("pbClearForm"));

        hlButtons->addWidget(pbClearForm);


        formLayoutUser->setLayout(4, QFormLayout::FieldRole, hlButtons);


        verticalLayout->addWidget(gbUserEdit);

        gbUserList = new QGroupBox(account_admin);
        gbUserList->setObjectName(QString::fromUtf8("gbUserList"));
        vlUserList = new QVBoxLayout(gbUserList);
        vlUserList->setObjectName(QString::fromUtf8("vlUserList"));
        twUserList = new QTableWidget(gbUserList);
        if (twUserList->columnCount() < 5)
            twUserList->setColumnCount(5);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        twUserList->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        twUserList->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        twUserList->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        twUserList->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        twUserList->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        twUserList->setObjectName(QString::fromUtf8("twUserList"));
        twUserList->setColumnCount(5);
        twUserList->setRowCount(0);

        vlUserList->addWidget(twUserList);


        verticalLayout->addWidget(gbUserList);


        retranslateUi(account_admin);

        QMetaObject::connectSlotsByName(account_admin);
    } // setupUi

    void retranslateUi(QWidget *account_admin)
    {
        account_admin->setWindowTitle(QCoreApplication::translate("account_admin", "\352\263\204\354\240\225 \352\264\200\353\246\254", nullptr));
        gbUserEdit->setTitle(QCoreApplication::translate("account_admin", "\352\263\204\354\240\225 \354\266\224\352\260\200 / \354\210\230\354\240\225", nullptr));
        lbUserId->setText(QCoreApplication::translate("account_admin", "\354\202\254\354\232\251\354\236\220 ID", nullptr));
        leUserId->setPlaceholderText(QCoreApplication::translate("account_admin", "\353\241\234\352\267\270\354\235\270 ID \354\236\205\353\240\245", nullptr));
        lbUserName->setText(QCoreApplication::translate("account_admin", "\354\235\264\353\246\204", nullptr));
        leUserName->setPlaceholderText(QCoreApplication::translate("account_admin", "\354\235\264\353\246\204 \354\236\205\353\240\245", nullptr));
        lbRole->setText(QCoreApplication::translate("account_admin", "\352\266\214\355\225\234", nullptr));
        cbRole->setItemText(0, QCoreApplication::translate("account_admin", "\352\264\200\353\246\254\354\236\220", nullptr));
        cbRole->setItemText(1, QCoreApplication::translate("account_admin", "\354\235\230\353\243\214\354\247\204", nullptr));

        lbNote->setText(QCoreApplication::translate("account_admin", "\353\271\204\352\263\240", nullptr));
        lbButtons->setText(QCoreApplication::translate("account_admin", "\354\236\221\354\227\205", nullptr));
        pbAddUser->setText(QCoreApplication::translate("account_admin", "\354\266\224\352\260\200", nullptr));
        pbUpdateUser->setText(QCoreApplication::translate("account_admin", "\354\210\230\354\240\225", nullptr));
        pbDeleteUser->setText(QCoreApplication::translate("account_admin", "\354\202\255\354\240\234", nullptr));
        pbClearForm->setText(QCoreApplication::translate("account_admin", "\354\236\205\353\240\245 \354\264\210\352\270\260\355\231\224", nullptr));
        gbUserList->setTitle(QCoreApplication::translate("account_admin", "\352\263\204\354\240\225 \353\252\251\353\241\235", nullptr));
        QTableWidgetItem *___qtablewidgetitem = twUserList->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("account_admin", "\354\204\240\355\203\235", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = twUserList->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("account_admin", "\354\202\254\354\232\251\354\236\220 ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = twUserList->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("account_admin", "\354\235\264\353\246\204", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = twUserList->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("account_admin", "\352\266\214\355\225\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = twUserList->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("account_admin", "\353\271\204\352\263\240", nullptr));
    } // retranslateUi

};

namespace Ui {
    class account_admin: public Ui_account_admin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ACCOUNT_ADMIN_H
