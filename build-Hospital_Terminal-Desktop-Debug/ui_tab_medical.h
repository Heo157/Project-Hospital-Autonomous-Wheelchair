/********************************************************************************
** Form generated from reading UI file 'tab_medical.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TAB_MEDICAL_H
#define UI_TAB_MEDICAL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_tab_medical
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *tab_2;

    void setupUi(QWidget *tab_medical)
    {
        if (tab_medical->objectName().isEmpty())
            tab_medical->setObjectName(QString::fromUtf8("tab_medical"));
        tab_medical->resize(400, 300);
        verticalLayout = new QVBoxLayout(tab_medical);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(tab_medical);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setTabPosition(QTabWidget::TabPosition::South);
        tabWidget->setTabShape(QTabWidget::TabShape::Triangular);
        tabWidget->setMovable(false);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());

        verticalLayout->addWidget(tabWidget);


        retranslateUi(tab_medical);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(tab_medical);
    } // setupUi

    void retranslateUi(QWidget *tab_medical)
    {
        tab_medical->setWindowTitle(QCoreApplication::translate("tab_medical", "Form", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("tab_medical", "\355\231\230\354\236\220 \354\240\225\353\263\264", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QCoreApplication::translate("tab_medical", "\355\234\240\354\262\264\354\226\264 \354\240\225\353\263\264", nullptr));
    } // retranslateUi

};

namespace Ui {
    class tab_medical: public Ui_tab_medical {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TAB_MEDICAL_H
