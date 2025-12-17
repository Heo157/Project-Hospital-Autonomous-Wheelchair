/********************************************************************************
** Form generated from reading UI file 'wheelchair_map.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WHEELCHAIR_MAP_H
#define UI_WHEELCHAIR_MAP_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_wheelchair_map
{
public:
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QTextBrowser *pbTerm;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_5;
    QLineEdit *pbLeInput;
    QPushButton *pbEnter;

    void setupUi(QWidget *wheelchair_map)
    {
        if (wheelchair_map->objectName().isEmpty())
            wheelchair_map->setObjectName(QString::fromUtf8("wheelchair_map"));
        wheelchair_map->resize(400, 300);
        verticalLayout_3 = new QVBoxLayout(wheelchair_map);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        pbTerm = new QTextBrowser(wheelchair_map);
        pbTerm->setObjectName(QString::fromUtf8("pbTerm"));

        verticalLayout_2->addWidget(pbTerm);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        verticalLayout_2->addItem(horizontalSpacer);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        pbLeInput = new QLineEdit(wheelchair_map);
        pbLeInput->setObjectName(QString::fromUtf8("pbLeInput"));

        horizontalLayout_5->addWidget(pbLeInput);

        pbEnter = new QPushButton(wheelchair_map);
        pbEnter->setObjectName(QString::fromUtf8("pbEnter"));

        horizontalLayout_5->addWidget(pbEnter);


        verticalLayout_2->addLayout(horizontalLayout_5);


        verticalLayout_3->addLayout(verticalLayout_2);


        retranslateUi(wheelchair_map);

        QMetaObject::connectSlotsByName(wheelchair_map);
    } // setupUi

    void retranslateUi(QWidget *wheelchair_map)
    {
        wheelchair_map->setWindowTitle(QCoreApplication::translate("wheelchair_map", "Form", nullptr));
        pbEnter->setText(QCoreApplication::translate("wheelchair_map", "ENTER", nullptr));
    } // retranslateUi

};

namespace Ui {
    class wheelchair_map: public Ui_wheelchair_map {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WHEELCHAIR_MAP_H
