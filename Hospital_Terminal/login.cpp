#include "login.h"
#include "ui_login.h"
#include "mainwindow.h"       // 메인 화면 띄우려면 필요
#include "database_manager.h" // DB 매니저
#include <QMessageBox>        // 알림창

Login::Login(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Login)
{
    ui->setupUi(this);
    // (선택사항) 시작하자마자 DB 연결 시도
    DatabaseManager::instance().connectToDb();
}

Login::~Login()
{
    delete ui;
}

void Login::on_pushButton_clicked()
{
    // 1. 화면의 입력창(lineEdit)에서 글자 가져오기 (이 부분이 빠져있었음!)
    QString id = ui->lineEdit->text();
    QString pw = ui->lineEdit_2->text();

    // 2. DB 매니저에게 물어보기
    bool success = DatabaseManager::instance().checkIdPassword(id, pw);

    // 3. 결과에 따라 처리하기
    if (success) {
        // 성공 시 메인 화면으로
        MainWindow *mw = new MainWindow();
        mw->show();
        this->close();
    } else {
        // 실패 시 경고창 띄우기
        QMessageBox::warning(this, "로그인 실패", "아이디 또는 비밀번호가 틀렸습니다.");
    }
}
