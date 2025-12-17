/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QFrame *topPanel;
    QHBoxLayout *topPanelLayout;
    QLabel *label_status;
    QSpacerItem *horizontalSpacer;
    QLabel *label_time;
    QLabel *label_wifi;
    QPushButton *btn_logout;
    QStackedWidget *stackedWidget;
    QWidget *page_0;
    QWidget *page_dashboard;
    QVBoxLayout *verticalLayout_3;
    QTabWidget *mainTabWidget;
    QWidget *tab;
    QWidget *tab_2;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout_2 = new QVBoxLayout(centralwidget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        topPanel = new QFrame(centralwidget);
        topPanel->setObjectName(QString::fromUtf8("topPanel"));
        topPanel->setMaximumSize(QSize(16777215, 60));
        topPanel->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 192, 203);"));
        topPanel->setFrameShape(QFrame::Shape::StyledPanel);
        topPanel->setFrameShadow(QFrame::Shadow::Raised);
        topPanelLayout = new QHBoxLayout(topPanel);
        topPanelLayout->setObjectName(QString::fromUtf8("topPanelLayout"));
        topPanelLayout->setContentsMargins(15, 0, 15, 0);
        label_status = new QLabel(topPanel);
        label_status->setObjectName(QString::fromUtf8("label_status"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_status->sizePolicy().hasHeightForWidth());
        label_status->setSizePolicy(sizePolicy);
        label_status->setStyleSheet(QString::fromUtf8("color: rgb(255, 255, 255);\n"
"            "));
        label_status->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignVCenter);

        topPanelLayout->addWidget(label_status);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        topPanelLayout->addItem(horizontalSpacer);

        label_time = new QLabel(topPanel);
        label_time->setObjectName(QString::fromUtf8("label_time"));
        label_time->setStyleSheet(QString::fromUtf8("color: white;"));

        topPanelLayout->addWidget(label_time);

        label_wifi = new QLabel(topPanel);
        label_wifi->setObjectName(QString::fromUtf8("label_wifi"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_wifi->sizePolicy().hasHeightForWidth());
        label_wifi->setSizePolicy(sizePolicy1);
        label_wifi->setMinimumSize(QSize(15, 15));
        label_wifi->setMaximumSize(QSize(15, 15));
        label_wifi->setScaledContents(true);

        topPanelLayout->addWidget(label_wifi);

        btn_logout = new QPushButton(topPanel);
        btn_logout->setObjectName(QString::fromUtf8("btn_logout"));

        topPanelLayout->addWidget(btn_logout);

        label_status->raise();
        label_time->raise();
        btn_logout->raise();
        label_wifi->raise();

        verticalLayout->addWidget(topPanel);

        stackedWidget = new QStackedWidget(centralwidget);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        page_0 = new QWidget();
        page_0->setObjectName(QString::fromUtf8("page_0"));
        stackedWidget->addWidget(page_0);
        page_dashboard = new QWidget();
        page_dashboard->setObjectName(QString::fromUtf8("page_dashboard"));
        verticalLayout_3 = new QVBoxLayout(page_dashboard);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        mainTabWidget = new QTabWidget(page_dashboard);
        mainTabWidget->setObjectName(QString::fromUtf8("mainTabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        mainTabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        mainTabWidget->addTab(tab_2, QString());

        verticalLayout_3->addWidget(mainTabWidget);

        stackedWidget->addWidget(page_dashboard);

        verticalLayout->addWidget(stackedWidget);


        verticalLayout_2->addLayout(verticalLayout);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label_status->setText(QCoreApplication::translate("MainWindow", "Text", nullptr));
        label_time->setText(QCoreApplication::translate("MainWindow", "0000-00-00 00:00:00", nullptr));
        btn_logout->setText(QCoreApplication::translate("MainWindow", "LOG OUT", nullptr));
        mainTabWidget->setTabText(mainTabWidget->indexOf(tab), QCoreApplication::translate("MainWindow", "Tab 1", nullptr));
        mainTabWidget->setTabText(mainTabWidget->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Tab 2", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
