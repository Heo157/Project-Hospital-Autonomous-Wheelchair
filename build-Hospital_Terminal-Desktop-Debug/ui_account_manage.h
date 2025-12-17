/********************************************************************************
** Form generated from reading UI file 'account_manage.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ACCOUNT_MANAGE_H
#define UI_ACCOUNT_MANAGE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableView>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_account_manage
{
public:
    QVBoxLayout *verticalLayout;
    QTextEdit *textEdit;
    QPushButton *pushButton;
    QTableView *tableView;

    void setupUi(QWidget *account_manage)
    {
        if (account_manage->objectName().isEmpty())
            account_manage->setObjectName(QString::fromUtf8("account_manage"));
        account_manage->resize(400, 300);
        verticalLayout = new QVBoxLayout(account_manage);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        textEdit = new QTextEdit(account_manage);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));

        verticalLayout->addWidget(textEdit);

        pushButton = new QPushButton(account_manage);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);

        tableView = new QTableView(account_manage);
        tableView->setObjectName(QString::fromUtf8("tableView"));

        verticalLayout->addWidget(tableView);


        retranslateUi(account_manage);

        QMetaObject::connectSlotsByName(account_manage);
    } // setupUi

    void retranslateUi(QWidget *account_manage)
    {
        account_manage->setWindowTitle(QCoreApplication::translate("account_manage", "Form", nullptr));
        pushButton->setText(QCoreApplication::translate("account_manage", "PushButton", nullptr));
    } // retranslateUi

};

namespace Ui {
    class account_manage: public Ui_account_manage {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ACCOUNT_MANAGE_H
