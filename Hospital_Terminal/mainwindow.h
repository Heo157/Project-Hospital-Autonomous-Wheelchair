#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>
#include "keyboard.h" 
#include "login.h" // [필수] 로그인 위젯 헤더 포함

// [수정] 아직 만들지 않은 탭 화면 헤더들은 주석 처리 (에러 방지)
// #include "tab_admin.h"
// #include "tab_medical.h"
// #include "tab_patient.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    // 이벤트 필터: 화면 터치나 클릭 이벤트를 가로채서 처리함 (키보드 띄우기용)
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    // 로그인 성공 시 실행될 함수 (role: admin, medical, patient 등)
    void onLoginSuccess(QString role);
    
    // 로그아웃 버튼 클릭 시 실행될 함수
    void onLogoutClicked();

private:
    Ui::MainWindow *ui;
    
    // 가상 키보드 객체
    Keyboard *virtualKeyboard;
    
    // 로그인 페이지 객체 (초기 화면용)
    Login *loginPage; 

    // 물리 키보드 연결 여부 확인 함수
    bool isPhysicalKeyboardConnected();
};
#endif // MAINWINDOW_H