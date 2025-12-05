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
    , virtualKeyboard(nullptr)
{
    ui->setupUi(this);

    // ---------------------------------------------------------
    // 1. 가상 키보드 설정 (조건부 생성)
    // ---------------------------------------------------------
    
    // 디버깅용: 현재 키보드 상태 출력
    bool hasKeyboard = isPhysicalKeyboardConnected();
    qDebug() << "Initial Keyboard Check:" << hasKeyboard;

    if (!hasKeyboard) {
        virtualKeyboard = new Keyboard(this);
        
        // 메인 레이아웃의 맨 아래에 추가
        ui->verticalLayout->addWidget(virtualKeyboard);
        
        // 높이 고정 및 사이즈 정책 설정
        virtualKeyboard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        virtualKeyboard->setFixedHeight(300);
        
        // 초기 상태는 숨김
        virtualKeyboard->hide();
        virtualKeyboard->setVisible(false);
    }
    // 물리 키보드가 있다면 virtualKeyboard는 nullptr 상태로 유지

    // ---------------------------------------------------------
    // 2. 로그인 페이지 조립
    // ---------------------------------------------------------
    loginPage = new Login(this);
    
    ui->stackedWidget->addWidget(loginPage);
    ui->stackedWidget->setCurrentWidget(loginPage);

    // 신호 연결
    connect(loginPage, &Login::loginSuccess, this, &MainWindow::onLoginSuccess);

    // 로그인 페이지의 입력창(LineEdit)들도 이벤트 필터에 등록
    QList<QLineEdit *> lineEdits = loginPage->findChildren<QLineEdit *>();
    for (QLineEdit *edit : lineEdits) {
        edit->installEventFilter(this);
    }

    // ---------------------------------------------------------
    // 3. 로그아웃 버튼 연결 및 초기 상태
    // ---------------------------------------------------------
    if (ui->btn_logout) {
        connect(ui->btn_logout, &QPushButton::clicked, this, &MainWindow::onLogoutClicked);
    }
    
    if (ui->label_status) {
        ui->label_status->setText("시스템 로그인 대기중...");
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

// ---------------------------------------------------------
// [수정됨] 기능 1: 물리 키보드 연결 확인 (정밀 검사)
// ---------------------------------------------------------
bool MainWindow::isPhysicalKeyboardConnected()
{
    QFile file("/proc/bus/input/devices");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open input devices file";
        return false; 
    }

    // 파일 전체 읽기 (0바이트 문제 해결)
    QByteArray data = file.readAll();
    file.close();

    QString content = QString::fromUtf8(data);
    
    // 각 장치 블록은 빈 줄로 구분됩니다.
    // 하지만 간단하게 줄 단위로 읽으면서 'Context'를 파악하는 방식으로 합니다.
    
    QStringList lines = content.split('\n');
    
    bool hasKbdHandler = false;
    bool isUsb = false;
    bool isKeyName = false;

    // 파일을 순차적으로 읽으면서 하나의 장치 블록을 분석합니다.
    // I: Bus=... 로 시작하는 부분이 새 장치의 시작입니다.
    
    for (const QString &line : lines) {
        if (line.trimmed().isEmpty()) {
            // 빈 줄을 만나면 이전 장치 분석 결과를 확인
            if (hasKbdHandler && (isUsb || isKeyName)) {
                qDebug() << "Real Physical Keyboard Detected!";
                return true;
            }
            
            // 다음 장치 분석을 위해 초기화
            hasKbdHandler = false;
            isUsb = false;
            isKeyName = false;
            continue;
        }

        // 1. 핸들러 확인 (kbd가 있는지)
        if (line.contains("Handlers=") && line.contains("kbd")) {
            hasKbdHandler = true;
        }

        // 2. 물리적 연결 방식 확인 (USB인지)
        if (line.contains("Phys=usb")) {
            isUsb = true;
        }

        // 3. 이름 확인 (Keyboard라는 단어가 들어가는지) - 대소문자 무시
        if (line.contains("Name=") && line.contains("keyboard", Qt::CaseInsensitive)) {
            isKeyName = true;
        }
    }
    
    // 마지막 장치 확인 (파일 끝이라 빈 줄이 없을 수 있음)
    if (hasKbdHandler && (isUsb || isKeyName)) {
        qDebug() << "Real Physical Keyboard Detected (Last device)!";
        return true;
    }

    qDebug() << "No physical keyboard found.";
    return false;
}

// ---------------------------------------------------------
// 기능 2: 이벤트 필터 (터치 감지 -> 가상 키보드 띄우기)
// ---------------------------------------------------------
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::FocusIn || event->type() == QEvent::MouseButtonPress) {
        QLineEdit *lineEdit = qobject_cast<QLineEdit*>(obj);
        if (lineEdit) {
            
            // 가상 키보드가 생성된 경우(nullptr가 아님)에만 동작
            // 즉, 시작할 때 물리 키보드가 없었던 경우에만 띄움
            if (virtualKeyboard != nullptr) {
                virtualKeyboard->setLineEdit(lineEdit);
                virtualKeyboard->show();
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

// ---------------------------------------------------------
// 기능 3: 로그인 성공 처리
// ---------------------------------------------------------
void MainWindow::onLoginSuccess(QString role)
{
    if (ui->page_dashboard) {
        ui->stackedWidget->setCurrentWidget(ui->page_dashboard);
    }

    if (virtualKeyboard != nullptr) {
        virtualKeyboard->hide();
    }

    if (ui->mainTabWidget) {
        ui->mainTabWidget->clear();

        if (role == "admin") {
            if (ui->label_status) ui->label_status->setText("관리자 모드");
            // ui->mainTabWidget->addTab(new TabAdmin(this), "시스템 관리");
        } 
        else if (role == "medical") {
            if (ui->label_status) ui->label_status->setText("의료진 모드");
            // ui->mainTabWidget->addTab(new TabMedical(this), "진료 업무");
        } 
        else if (role == "patient") {
            if (ui->label_status) ui->label_status->setText("환자 모드");
            // ui->mainTabWidget->addTab(new TabPatient(this), "휠체어 호출");
        }
    }
}

// ---------------------------------------------------------
// 기능 4: 로그아웃 처리
// ---------------------------------------------------------
void MainWindow::onLogoutClicked()
{
    if (ui->mainTabWidget) {
        while (ui->mainTabWidget->count() > 0) {
            QWidget* widget = ui->mainTabWidget->widget(0);
            ui->mainTabWidget->removeTab(0);
            delete widget;
        }
    }

    ui->stackedWidget->setCurrentWidget(loginPage);
    
    if (ui->label_status) ui->label_status->setText("로그아웃 되었습니다.");
    
    if (virtualKeyboard != nullptr) {
        virtualKeyboard->hide();
    }
}