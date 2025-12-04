#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QLineEdit> // QLineEdit 캐스팅을 위해 필요

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ---------------------------------------------------------
    // 1. 가상 키보드 설정 (화면 하단에 붙이기)
    // ---------------------------------------------------------
    virtualKeyboard = new Keyboard(this);
    
    // 메인 레이아웃의 맨 아래에 추가 (이렇게 해야 화면을 밀어 올림)
    // ui->verticalLayout이 mainwindow.ui에 정의되어 있어야 합니다.
    ui->verticalLayout->addWidget(virtualKeyboard);
    
    // 높이 고정 (300px), 너비는 화면에 맞춤
    virtualKeyboard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    virtualKeyboard->setFixedHeight(300);
    virtualKeyboard->hide(); // 기본은 숨김 상태

    // ---------------------------------------------------------
    // 2. 로그인 페이지 조립 (기존 Login 클래스 재활용)
    // ---------------------------------------------------------
    loginPage = new Login(this);
    
    // 스택 위젯의 0번 페이지로 추가
    ui->stackedWidget->addWidget(loginPage);
    ui->stackedWidget->setCurrentWidget(loginPage);

    // [신호 연결] Login 위젯이 "성공 신호"를 보내면 -> 내 함수(onLoginSuccess) 실행
    connect(loginPage, &Login::loginSuccess, this, &MainWindow::onLoginSuccess);

    // 로그인 페이지의 입력창(LineEdit)들도 키보드 이벤트 필터에 등록해야 합니다.
    // Login 객체의 자식 위젯들을 찾아서 필터를 설치합니다.
    QList<QLineEdit *> lineEdits = loginPage->findChildren<QLineEdit *>();
    for (QLineEdit *edit : lineEdits) {
        edit->installEventFilter(this);
    }

    // ---------------------------------------------------------
    // 3. 로그아웃 버튼 연결 (UI에 btn_logout이 있다고 가정)
    // ---------------------------------------------------------
    // page_dashboard (ui->stackedWidget의 다른 페이지) 안에 있는 btn_logout 버튼을 찾아 연결
    // ui파일에서 직접 접근 가능하다면 ui->btn_logout으로 써도 됩니다.
    if (ui->btn_logout) {
        connect(ui->btn_logout, &QPushButton::clicked, this, &MainWindow::onLogoutClicked);
    }
    
    // 초기 상태 메시지
    if (ui->label_status) {
        ui->label_status->setText("시스템 로그인 대기중...");
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

// ---------------------------------------------------------
// 기능 1: 물리 키보드 연결 확인 (리눅스 전용)
// ---------------------------------------------------------
bool MainWindow::isPhysicalKeyboardConnected()
{
    QFile file("/proc/bus/input/devices");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false; // 파일 못 읽으면 없는 것으로 간주 (가상키보드 사용)
    }

    QTextStream in(&file);
    bool isKeyboard = false;
    
    while (!in.atEnd()) {
        QString line = in.readLine();
        // Handlers 항목에 kbd가 있으면 키보드로 인식
        // (가상 장치 필터링이 필요할 수 있으나, 기본적으로 kbd 확인)
        if (line.contains("Handlers=") && line.contains("kbd")) {
            isKeyboard = true;
            break; 
        }
    }
    file.close();
    return isKeyboard;
}

// ---------------------------------------------------------
// 기능 2: 이벤트 필터 (터치 감지 -> 키보드 띄우기)
// ---------------------------------------------------------
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    // 포커스를 얻거나 마우스를 클릭했을 때
    if (event->type() == QEvent::FocusIn || event->type() == QEvent::MouseButtonPress) {
        QLineEdit *lineEdit = qobject_cast<QLineEdit*>(obj);
        if (lineEdit) {
            // 조건 a-1: 물리 키보드가 있으면 -> 가상 키보드 숨김
            if (isPhysicalKeyboardConnected()) {
                virtualKeyboard->hide();
            } 
            // 조건 a-2: 물리 키보드가 없으면 -> 가상 키보드 띄움
            else {
                virtualKeyboard->setLineEdit(lineEdit);
                virtualKeyboard->show();
                // 키보드가 보이면 레이아웃이 다시 계산되어 상단 화면이 밀려 올라감
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

// ---------------------------------------------------------
// 기능 3: 로그인 성공 처리 (화면 전환 & 탭 구성)
// ---------------------------------------------------------
void MainWindow::onLoginSuccess(QString role)
{
    // 1. 대시보드 페이지로 이동
    // mainwindow.ui에서 만든 page_dashboard 위젯으로 전환
    if (ui->page_dashboard) {
        ui->stackedWidget->setCurrentWidget(ui->page_dashboard);
    }

    // 2. 키보드 숨기기 (성공했으니 화면 넓게)
    virtualKeyboard->hide();

    // 3. 역할(Role)에 따라 탭 구성
    if (ui->mainTabWidget) {
        ui->mainTabWidget->clear(); // 기존 탭 제거

        if (role == "admin") {
            if (ui->label_status) ui->label_status->setText("관리자 모드");
            // 관리자용 탭들 추가 (나중에 파일 생성 후 주석 해제)
            // ui->mainTabWidget->addTab(new TabAdmin(this), "시스템 관리");
            // ui->mainTabWidget->addTab(new TabMedical(this), "의료진 보기");
            // ui->mainTabWidget->addTab(new TabPatient(this), "환자 보기");
        } 
        else if (role == "medical") {
            if (ui->label_status) ui->label_status->setText("의료진 모드");
            // 의료진용 탭들 추가 (나중에 파일 생성 후 주석 해제)
            // ui->mainTabWidget->addTab(new TabMedical(this), "내 업무");
            // ui->mainTabWidget->addTab(new TabPatient(this), "환자 모니터링");
        } 
        else if (role == "patient") {
            if (ui->label_status) ui->label_status->setText("환자 모드");
            // 환자용 탭 추가 (나중에 파일 생성 후 주석 해제)
            // ui->mainTabWidget->addTab(new TabPatient(this), "휠체어 호출");
        }
    }
}

// ---------------------------------------------------------
// 기능 4: 로그아웃 처리
// ---------------------------------------------------------
void MainWindow::onLogoutClicked()
{
    // 1. 탭 내용 비우기 (메모리 정리 포함)
    if (ui->mainTabWidget) {
        while (ui->mainTabWidget->count() > 0) {
            QWidget* widget = ui->mainTabWidget->widget(0);
            ui->mainTabWidget->removeTab(0);
            delete widget;
        }
    }

    // 2. 로그인 화면으로 복귀
    ui->stackedWidget->setCurrentWidget(loginPage);
    
    // 3. 상태 메시지 초기화
    if (ui->label_status) ui->label_status->setText("로그아웃 되었습니다.");
    
    // 4. 키보드 숨김
    virtualKeyboard->hide();
    
    // 5. 로그인 입력창 초기화 (선택 사항)
    // loginPage의 자식 위젯을 찾아서 지울 수 있습니다.
    // QLineEdit *idEdit = loginPage->findChild<QLineEdit *>("lineEdit");
    // if(idEdit) idEdit->clear();
}