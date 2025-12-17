/********************************************************************************
** Form generated from reading UI file 'wheelchair_admin.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WHEELCHAIR_ADMIN_H
#define UI_WHEELCHAIR_ADMIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_wheelchair_admin
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *hlTop;
    QGroupBox *gbRobotStatus;
    QVBoxLayout *vlRobotStatus;
    QTableWidget *twRobotStatus;
    QGroupBox *gbCallLog;
    QVBoxLayout *vlCallLog;
    QTableWidget *twCallLog;
    QGroupBox *gbMap;
    QVBoxLayout *vlMap;
    QLabel *lbMapPlaceholder;
    QHBoxLayout *hlSelectedRobot;
    QLabel *lbSelectedRobot;
    QLineEdit *leSelectedRobot;
    QGridLayout *glButtons;
    QPushButton *pbStop;
    QPushButton *pbGoWait;
    QPushButton *pbGoCharge;
    QPushButton *pbDirectCall;
    QPushButton *pbResume;
    QPushButton *pbAddWheel;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *wheelchair_admin)
    {
        if (wheelchair_admin->objectName().isEmpty())
            wheelchair_admin->setObjectName(QString::fromUtf8("wheelchair_admin"));
        wheelchair_admin->resize(900, 600);
        verticalLayout = new QVBoxLayout(wheelchair_admin);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        hlTop = new QHBoxLayout();
        hlTop->setObjectName(QString::fromUtf8("hlTop"));
        gbRobotStatus = new QGroupBox(wheelchair_admin);
        gbRobotStatus->setObjectName(QString::fromUtf8("gbRobotStatus"));
        vlRobotStatus = new QVBoxLayout(gbRobotStatus);
        vlRobotStatus->setObjectName(QString::fromUtf8("vlRobotStatus"));
        twRobotStatus = new QTableWidget(gbRobotStatus);
        if (twRobotStatus->columnCount() < 7)
            twRobotStatus->setColumnCount(7);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        twRobotStatus->setHorizontalHeaderItem(6, __qtablewidgetitem6);
        twRobotStatus->setObjectName(QString::fromUtf8("twRobotStatus"));
        twRobotStatus->setRowCount(0);
        twRobotStatus->setColumnCount(7);

        vlRobotStatus->addWidget(twRobotStatus);

        gbCallLog = new QGroupBox(gbRobotStatus);
        gbCallLog->setObjectName(QString::fromUtf8("gbCallLog"));
        vlCallLog = new QVBoxLayout(gbCallLog);
        vlCallLog->setObjectName(QString::fromUtf8("vlCallLog"));
        twCallLog = new QTableWidget(gbCallLog);
        if (twCallLog->columnCount() < 5)
            twCallLog->setColumnCount(5);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        twCallLog->setHorizontalHeaderItem(0, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        twCallLog->setHorizontalHeaderItem(1, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        twCallLog->setHorizontalHeaderItem(2, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        twCallLog->setHorizontalHeaderItem(3, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        twCallLog->setHorizontalHeaderItem(4, __qtablewidgetitem11);
        twCallLog->setObjectName(QString::fromUtf8("twCallLog"));
        twCallLog->setRowCount(0);
        twCallLog->setColumnCount(5);

        vlCallLog->addWidget(twCallLog);


        vlRobotStatus->addWidget(gbCallLog);


        hlTop->addWidget(gbRobotStatus);

        gbMap = new QGroupBox(wheelchair_admin);
        gbMap->setObjectName(QString::fromUtf8("gbMap"));
        QSizePolicy sizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(gbMap->sizePolicy().hasHeightForWidth());
        gbMap->setSizePolicy(sizePolicy);
        vlMap = new QVBoxLayout(gbMap);
        vlMap->setObjectName(QString::fromUtf8("vlMap"));
        lbMapPlaceholder = new QLabel(gbMap);
        lbMapPlaceholder->setObjectName(QString::fromUtf8("lbMapPlaceholder"));
        sizePolicy.setHeightForWidth(lbMapPlaceholder->sizePolicy().hasHeightForWidth());
        lbMapPlaceholder->setSizePolicy(sizePolicy);
        lbMapPlaceholder->setPixmap(QPixmap(QString::fromUtf8("../SLAM_MAP/ImageToStl.com_map_Hospital.jpg")));
        lbMapPlaceholder->setScaledContents(true);
        lbMapPlaceholder->setAlignment(Qt::AlignCenter);
        lbMapPlaceholder->setWordWrap(true);

        vlMap->addWidget(lbMapPlaceholder);

        hlSelectedRobot = new QHBoxLayout();
        hlSelectedRobot->setObjectName(QString::fromUtf8("hlSelectedRobot"));
        lbSelectedRobot = new QLabel(gbMap);
        lbSelectedRobot->setObjectName(QString::fromUtf8("lbSelectedRobot"));

        hlSelectedRobot->addWidget(lbSelectedRobot);

        leSelectedRobot = new QLineEdit(gbMap);
        leSelectedRobot->setObjectName(QString::fromUtf8("leSelectedRobot"));
        leSelectedRobot->setReadOnly(true);

        hlSelectedRobot->addWidget(leSelectedRobot);


        vlMap->addLayout(hlSelectedRobot);

        glButtons = new QGridLayout();
        glButtons->setObjectName(QString::fromUtf8("glButtons"));
        pbStop = new QPushButton(gbMap);
        pbStop->setObjectName(QString::fromUtf8("pbStop"));

        glButtons->addWidget(pbStop, 0, 1, 1, 1);

        pbGoWait = new QPushButton(gbMap);
        pbGoWait->setObjectName(QString::fromUtf8("pbGoWait"));

        glButtons->addWidget(pbGoWait, 1, 0, 1, 1);

        pbGoCharge = new QPushButton(gbMap);
        pbGoCharge->setObjectName(QString::fromUtf8("pbGoCharge"));

        glButtons->addWidget(pbGoCharge, 1, 1, 1, 1);

        pbDirectCall = new QPushButton(gbMap);
        pbDirectCall->setObjectName(QString::fromUtf8("pbDirectCall"));

        glButtons->addWidget(pbDirectCall, 0, 0, 1, 1);

        pbResume = new QPushButton(gbMap);
        pbResume->setObjectName(QString::fromUtf8("pbResume"));

        glButtons->addWidget(pbResume, 0, 2, 1, 1);

        pbAddWheel = new QPushButton(gbMap);
        pbAddWheel->setObjectName(QString::fromUtf8("pbAddWheel"));

        glButtons->addWidget(pbAddWheel, 1, 2, 1, 1);


        vlMap->addLayout(glButtons);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        vlMap->addItem(verticalSpacer);

        vlMap->setStretch(0, 5);
        vlMap->setStretch(1, 1);
        vlMap->setStretch(2, 3);
        vlMap->setStretch(3, 3);

        hlTop->addWidget(gbMap);


        verticalLayout->addLayout(hlTop);


        retranslateUi(wheelchair_admin);

        QMetaObject::connectSlotsByName(wheelchair_admin);
    } // setupUi

    void retranslateUi(QWidget *wheelchair_admin)
    {
        wheelchair_admin->setWindowTitle(QCoreApplication::translate("wheelchair_admin", "\355\234\240\354\262\264\354\226\264 \353\252\250\353\213\210\355\204\260\353\247\201 / \354\240\234\354\226\264", nullptr));
        gbRobotStatus->setTitle(QCoreApplication::translate("wheelchair_admin", "\355\234\240\354\262\264\354\226\264 \354\203\201\355\203\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem = twRobotStatus->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("wheelchair_admin", "\355\234\240\354\262\264\354\226\264 ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = twRobotStatus->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("wheelchair_admin", "\353\260\260\355\204\260\353\246\254(%)", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = twRobotStatus->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("wheelchair_admin", "\354\203\201\355\203\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = twRobotStatus->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("wheelchair_admin", "\355\230\204\354\236\254 \354\234\204\354\271\230", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = twRobotStatus->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("wheelchair_admin", "\353\252\251\355\221\234 \354\234\204\354\271\230", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = twRobotStatus->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("wheelchair_admin", "X\354\242\214\355\221\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = twRobotStatus->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("wheelchair_admin", "Y\354\242\214\355\221\234", nullptr));
        gbCallLog->setTitle(QCoreApplication::translate("wheelchair_admin", "\355\230\270\354\266\234 \354\235\264\353\240\245", nullptr));
        QTableWidgetItem *___qtablewidgetitem7 = twCallLog->horizontalHeaderItem(0);
        ___qtablewidgetitem7->setText(QCoreApplication::translate("wheelchair_admin", "\354\213\234\352\260\204", nullptr));
        QTableWidgetItem *___qtablewidgetitem8 = twCallLog->horizontalHeaderItem(1);
        ___qtablewidgetitem8->setText(QCoreApplication::translate("wheelchair_admin", "\355\234\240\354\262\264\354\226\264 ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem9 = twCallLog->horizontalHeaderItem(2);
        ___qtablewidgetitem9->setText(QCoreApplication::translate("wheelchair_admin", "\354\266\234\353\260\234 \354\234\204\354\271\230", nullptr));
        QTableWidgetItem *___qtablewidgetitem10 = twCallLog->horizontalHeaderItem(3);
        ___qtablewidgetitem10->setText(QCoreApplication::translate("wheelchair_admin", "\353\217\204\354\260\251 \354\234\204\354\271\230", nullptr));
        QTableWidgetItem *___qtablewidgetitem11 = twCallLog->horizontalHeaderItem(4);
        ___qtablewidgetitem11->setText(QCoreApplication::translate("wheelchair_admin", "\355\230\270\354\266\234\354\236\220", nullptr));
        gbMap->setTitle(QCoreApplication::translate("wheelchair_admin", "\353\263\221\354\233\220 \353\247\244\355\225\221 / \355\234\240\354\262\264\354\226\264 \354\240\234\354\226\264", nullptr));
        lbMapPlaceholder->setText(QString());
        lbSelectedRobot->setText(QCoreApplication::translate("wheelchair_admin", "\354\204\240\355\203\235 \353\241\234\353\264\207:", nullptr));
        leSelectedRobot->setPlaceholderText(QCoreApplication::translate("wheelchair_admin", "\353\241\234\353\264\207 \354\203\201\355\203\234 \355\205\214\354\235\264\353\270\224\354\227\220\354\204\234 \354\204\240\355\203\235", nullptr));
        pbStop->setText(QCoreApplication::translate("wheelchair_admin", "\354\246\211\354\213\234 \354\240\225\354\247\200", nullptr));
        pbGoWait->setText(QCoreApplication::translate("wheelchair_admin", "\353\214\200\352\270\260 \354\212\244\355\205\214\354\235\264\354\205\230 \352\267\200\355\231\230", nullptr));
        pbGoCharge->setText(QCoreApplication::translate("wheelchair_admin", "\354\266\251\354\240\204 \354\212\244\355\205\214\354\235\264\354\205\230 \354\235\264\353\217\231", nullptr));
        pbDirectCall->setText(QCoreApplication::translate("wheelchair_admin", "\355\234\240\354\262\264\354\226\264 \354\247\201\354\240\221 \355\230\270\354\266\234", nullptr));
        pbResume->setText(QCoreApplication::translate("wheelchair_admin", "\353\217\231\354\236\221 \354\236\254\352\260\234", nullptr));
        pbAddWheel->setText(QCoreApplication::translate("wheelchair_admin", "\355\234\240\354\262\264\354\226\264 \353\223\261\353\241\235", nullptr));
    } // retranslateUi

};

namespace Ui {
    class wheelchair_admin: public Ui_wheelchair_admin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WHEELCHAIR_ADMIN_H
