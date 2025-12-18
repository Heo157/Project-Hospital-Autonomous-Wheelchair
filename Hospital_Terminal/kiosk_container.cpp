#include "kiosk_container.h"
#include "ui_kiosk_container.h"

#include <QPushButton>

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
    // 1. 실제 kiosk 페이지 생성
    // ---------------------------------
    kiosk_main   *pageMain   = new kiosk_main(this);
    kiosk_search *pageSearch = new kiosk_search(this);
    kiosk_login  *pageLogin  = new kiosk_login(this);
    kiosk_wheel  *pageWheel  = new kiosk_wheel(this);
    kiosk_back   *pageBack   = new kiosk_back(this);

    // ---------------------------------
    // 2. stackedWidget 교체 삽입
    // (Qt Designer 기본 페이지 제거)
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
    // 3. 버튼 연결 (화면 전환)
    // ---------------------------------

    // ===== kiosk_main =====

    // 메인 → 환자 조회
    connect(pageMain->findChild<QPushButton*>("pbkioskpatient"),
            &QPushButton::clicked,
            this, [=]() {
                prevPage = pageMain;
                ui->stackedWidget->setCurrentWidget(pageSearch);
            });

    // 메인 → 휠체어 호출 (로그인)
    connect(pageMain->findChild<QPushButton*>("pbkioskwheel"),
            &QPushButton::clicked,
            this, [=]() {
                prevPage = pageMain;
                ui->stackedWidget->setCurrentWidget(pageLogin);
            });

    // ===== kiosk_login =====

    // 로그인 → 휠체어 호출
    connect(pageLogin->findChild<QPushButton*>("btn_login"),
            &QPushButton::clicked,
            this, [=]() {
                prevPage = pageLogin;
                ui->stackedWidget->setCurrentWidget(pageWheel);
            });

    // 로그인 → 뒤로
    connect(pageLogin->findChild<QPushButton*>("btn_back"),
            &QPushButton::clicked,
            this, [=]() {
                ui->stackedWidget->setCurrentWidget(pageMain);
            });

    // ===== kiosk_search =====

    // 환자 조회 → 휠체어 호출
    connect(pageSearch->findChild<QPushButton*>("btn_call"),
            &QPushButton::clicked,
            this, [=]() {
                prevPage = pageSearch;
                ui->stackedWidget->setCurrentWidget(pageWheel);
            });

    // 환자 조회 → 뒤로
    connect(pageSearch->findChild<QPushButton*>("btn_back"),
            &QPushButton::clicked,
            this, [=]() {
                ui->stackedWidget->setCurrentWidget(pageMain);
            });

    // ===== kiosk_wheel =====

    // 휠체어 호출 → 뒤로 (이전 페이지)
    connect(pageWheel->findChild<QPushButton*>("btn_back"),
            &QPushButton::clicked,
            this, [=]() {
                if (prevPage)
                    ui->stackedWidget->setCurrentWidget(prevPage);
            });

    // 휠체어 호출 → 완료
    connect(pageWheel->findChild<QPushButton*>("btn_call"),
            &QPushButton::clicked,
            this, [=]() {
                ui->stackedWidget->setCurrentWidget(pageBack);
            });

    // ===== kiosk_back =====

    // 완료 → 메인
    connect(pageBack->findChild<QPushButton*>("btn_main"),
            &QPushButton::clicked,
            this, [=]() {
                ui->stackedWidget->setCurrentWidget(pageMain);
            });
}

kiosk_container::~kiosk_container()
{
    delete ui;
}
