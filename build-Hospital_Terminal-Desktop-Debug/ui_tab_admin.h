/********************************************************************************
** Form generated from reading UI file 'tab_admin.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TAB_ADMIN_H
#define UI_TAB_ADMIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_tab_admin
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *tab_2;
    QWidget *tab_3;

    void setupUi(QWidget *tab_admin)
    {
        if (tab_admin->objectName().isEmpty())
            tab_admin->setObjectName(QString::fromUtf8("tab_admin"));
        tab_admin->resize(400, 300);
        verticalLayout = new QVBoxLayout(tab_admin);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(tab_admin);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setTabPosition(QTabWidget::TabPosition::South);
        tabWidget->setTabShape(QTabWidget::TabShape::Triangular);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        tabWidget->addTab(tab_3, QString());

        verticalLayout->addWidget(tabWidget);


        retranslateUi(tab_admin);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(tab_admin);
    } // setupUi

    void retranslateUi(QWidget *tab_admin)
    {
        tab_admin->setWindowTitle(QCoreApplication::translate("tab_admin", "\352\264\200\353\246\254\354\236\220 \355\203\255", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("tab_admin", "\352\263\204\354\240\225 \352\264\200\353\246\254", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("tab_admin", "\355\234\240\354\262\264\354\226\264 \353\252\250\353\213\210\355\204\260\353\247\201", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QCoreApplication::translate("tab_admin", "\355\204\260\353\257\270\353\204\220", nullptr));
    } // retranslateUi

};

namespace Ui {
    class tab_admin: public Ui_tab_admin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TAB_ADMIN_H
