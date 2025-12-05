#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>
#include "keyboard.h" 
#include "login.h" 

// [주석] 탭 화면 헤더 (나중에 파일 만들면 주석 해제)
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
    // 이벤트 필터: 사용자가 화면을 터치할 때마다 감시
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void onLoginSuccess(QString role);
    void onLogoutClicked();

private:
    Ui::MainWindow *ui;
    
    // 가상 키보드
    Keyboard *virtualKeyboard = nullptr;
    
    // 로그인 페이지
    Login *loginPage; 

    // [핵심] 물리 키보드가 꽂혀있는지 확인하는 함수
    bool isPhysicalKeyboardConnected();
};
#endif // MAINWINDOW_H
