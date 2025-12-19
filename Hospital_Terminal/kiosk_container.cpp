#include "kiosk_container.h"
#include "ui_kiosk_container.h"

#include "kiosk_main.h"
#include "kiosk_search.h"
#include "kiosk_login.h"
#include "kiosk_wheel.h"
#include "kiosk_back.h"

kiosk_container::kiosk_container(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_container)
{
    ui->setupUi(this);

    // ---------------------------------
    // 1. 페이지 생성
    // ---------------------------------
    kiosk_main   *pageMain   = new kiosk_main(this);
    kiosk_search *pageSearch = new kiosk_search(this);
    kiosk_login  *pageLogin  = new kiosk_login(this);
    kiosk_wheel  *pageWheel  = new kiosk_wheel(this);
    kiosk_back   *pageBack   = new kiosk_back(this);

    // ---------------------------------
    // 2. stackedWidget 교체
    // ---------------------------------
    ui->stackedWidget->removeWidget(ui->stackedWidget->widget(0));
    ui->stackedWidget->insertWidget(0, pageMain);

    ui->stackedWidget->removeWidget(ui->stackedWidget->widget(1));
    ui->stackedWidget->insertWidget(1, pageSearch);

    ui->stackedWidget->removeWidget(ui->stackedWidget->widget(2));
    ui->stackedWidget->insertWidget(2, pageLogin);

    ui->stackedWidget->removeWidget(ui->stackedWidget->widget(3));
    ui->stackedWidget->insertWidget(3, pageWheel);

    ui->stackedWidget->removeWidget(ui->stackedWidget->widget(4));
    ui->stackedWidget->insertWidget(4, pageBack);

    ui->stackedWidget->setCurrentWidget(pageMain);

    // ---------------------------------
    // kiosk_main
    // ---------------------------------
    connect(pageMain, &kiosk_main::goSearch, this, [=]() {
        ui->stackedWidget->setCurrentWidget(pageSearch);
    });

    connect(pageMain, &kiosk_main::goLogin, this, [=]() {
        ui->stackedWidget->setCurrentWidget(pageLogin);
    });

    // ---------------------------------
    // kiosk_search
    // ---------------------------------
    connect(pageSearch, &kiosk_search::searchAccepted, this, [=]() {
        prevPage = pageSearch;
        ui->stackedWidget->setCurrentWidget(pageWheel);
    });

    connect(pageSearch, &kiosk_search::goBack, this, [=]() {
        ui->stackedWidget->setCurrentWidget(pageMain);
    });

    // ---------------------------------
    // kiosk_login
    // ---------------------------------
    connect(pageLogin, &kiosk_login::loginAccepted, this, [=]() {
        prevPage = pageLogin;
        ui->stackedWidget->setCurrentWidget(pageWheel);
    });

    connect(pageLogin, &kiosk_login::goBack, this, [=]() {
        ui->stackedWidget->setCurrentWidget(pageMain);
    });

    // ---------------------------------
    // kiosk_wheel
    // ---------------------------------
    connect(pageWheel, &kiosk_wheel::wheelConfirmed, this, [=]() {
        ui->stackedWidget->setCurrentWidget(pageBack);
    });

    connect(pageWheel, &kiosk_wheel::goBack, this, [=]() {
        if (prevPage)
            ui->stackedWidget->setCurrentWidget(prevPage);
    });

    // ---------------------------------
    // kiosk_back
    // ---------------------------------
    connect(pageBack, &kiosk_back::goMain, this, [=]() {
        ui->stackedWidget->setCurrentWidget(pageMain);
    });
}

kiosk_container::~kiosk_container()
{
    delete ui;
}
