/*
 * Virtual Keyboard Implementation
 * - Removed internal input lineEdit
 * - Directly inserts text into target QLineEdit
 * - Supports Shift (Uppercase) and Char (Special characters)
 */

#include "keyboard.h"
#include "ui_keyboard.h"
#include <QDebug>
#include <QMetaObject> // for invoking returnPressed signal

Keyboard::Keyboard(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Keyboard),
    outputLineEdit(nullptr), // 초기 타겟은 없음
    shift(false)
{
    ui->setupUi(this);

    // ---------------------------------------------------------
    // 모든 버튼의 클릭 신호를 keyboardHandler와 연결
    // ---------------------------------------------------------
    // 화면에 있는 모든 QPushButton을 찾습니다.
    QList<QPushButton *> allButtons = this->findChildren<QPushButton *>();
    
    for (QPushButton *btn : allButtons) {
        QString name = btn->objectName();
        
        // 기능키(Shift, Char, Enter)는 제외하고 나머지 문자 키들만 연결
        // (이들은 각각 별도의 슬롯 함수가 연결되어 있음)
        if (name != "shift" && name != "char_2" && name != "enterButton") {
            connect(btn, SIGNAL(clicked()), this, SLOT(keyboardHandler()));
        }
    }
}

Keyboard::~Keyboard()
{
    delete ui;
}

/**
 * @brief 글자가 입력될 타겟 QLineEdit 위젯을 설정합니다.
 * @param line 대상 QLineEdit 포인터
 */
void Keyboard::setLineEdit(QLineEdit *line)
{
    outputLineEdit = line;
}

/**
 * @brief 키 버튼이 눌렸을 때 실행되는 함수
 * - 버튼의 텍스트를 가져와서 타겟 입력창에 바로 삽입합니다.
 */
void Keyboard::keyboardHandler()
{
    // 입력할 대상(outputLineEdit)이 설정되지 않았으면 아무것도 안 함
    if (!outputLineEdit) return;

    QPushButton *button = (QPushButton *)sender();
    QString inputText = button->text();
    QString charToInsert;

    // 특수 키 처리
    if (inputText == "Space") {
        charToInsert = " ";
    } 
    else if (inputText == "&&") {
        charToInsert = "&";
    } 
    else if (inputText == "\\") {
        charToInsert = "\\";
    }
    else {
        // 일반 문자 처리 (Shift 상태에 따라 대소문자 변환)
        if (shift) {
            charToInsert = inputText.toUpper();
            // shift = false; // 한 번만 대문자로 쓰고 싶다면 주석 해제
        } else {
            charToInsert = inputText;
        }
    }
    
    // [핵심] 타겟 입력창의 현재 커서 위치에 문자를 삽입
    outputLineEdit->insert(charToInsert);
}

/**
 * @brief Shift 키 처리 (대소문자 토글)
 */
void Keyboard::on_shift_clicked()
{
    shift = !shift; // 상태 반전
    // (선택 사항) Shift 상태에 따라 버튼 텍스트를 대문자로 바꿀 수도 있음
}

/**
 * @brief Enter 키 처리
 * - 키보드를 숨기고, 타겟 입력창에 엔터 이벤트를 발생시킵니다.
 */
void Keyboard::on_enterButton_clicked()
{
    // 1. 키보드 숨기기
    this->hide();
    
    // 2. 타겟 입력창에 Enter 키가 눌린 것과 같은 효과를 냄
    // (이로 인해 로그인 버튼 클릭 슬롯 등이 실행될 수 있음)
    if (outputLineEdit) {
        QMetaObject::invokeMethod(outputLineEdit, "returnPressed");
    }
}

/**
 * @brief 특수문자 모드 전환 (Char 버튼)
 */
void Keyboard::on_char_2_toggled(bool checked)
{
    if(checked) {
        // 특수문자 모드
        ui->Button0->setText("`"); ui->Button1->setText("~"); ui->Button2->setText("!");
        ui->Button3->setText("@"); ui->Button4->setText("#"); ui->Button5->setText("$");
        ui->Button6->setText("%"); ui->Button7->setText("^"); ui->Button8->setText("&&");
        ui->Button9->setText("*");

        ui->Buttonq->setText("("); ui->Buttonw->setText(")"); ui->Buttone->setText("-");
        ui->Buttonr->setText("_"); ui->Buttont->setText("="); ui->Buttony->setText("+");
        ui->Buttonu->setText("["); ui->Buttoni->setText("]"); ui->Buttono->setText("{");
        ui->Buttonp->setText("}");

        ui->Buttona->setText("\\"); ui->Buttons->setText("|"); ui->Buttond->setText(";");
        ui->Buttonf->setText(":"); ui->Buttong->setText("'"); ui->Buttonh->setText("\"");
        ui->Buttonj->setText("/"); ui->Buttonk->setText("?"); ui->Buttonl->setText("");

        ui->Buttonz->setText("<"); ui->Buttonx->setText(">"); ui->Buttonc->setText(",");
        ui->Buttonv->setText("."); ui->Buttonb->setText(""); ui->Buttonn->setText("");
        ui->Buttonm->setText("");

    } else {
        // 일반 숫자/문자 모드 복귀
        ui->Button0->setText("0"); ui->Button1->setText("1"); ui->Button2->setText("2");
        ui->Button3->setText("3"); ui->Button4->setText("4"); ui->Button5->setText("5");
        ui->Button6->setText("6"); ui->Button7->setText("7"); ui->Button8->setText("8");
        ui->Button9->setText("9");

        ui->Buttonq->setText("q"); ui->Buttonw->setText("w"); ui->Buttone->setText("e");
        ui->Buttonr->setText("r"); ui->Buttont->setText("t"); ui->Buttony->setText("y");
        ui->Buttonu->setText("u"); ui->Buttoni->setText("i"); ui->Buttono->setText("o");
        ui->Buttonp->setText("p");

        ui->Buttona->setText("a"); ui->Buttons->setText("s"); ui->Buttond->setText("d");
        ui->Buttonf->setText("f"); ui->Buttong->setText("g"); ui->Buttonh->setText("h");
        ui->Buttonj->setText("j"); ui->Buttonk->setText("k"); ui->Buttonl->setText("l");

        ui->Buttonz->setText("z"); ui->Buttonx->setText("x"); ui->Buttonc->setText("c");
        ui->Buttonv->setText("v"); ui->Buttonb->setText("b"); ui->Buttonn->setText("n");
        ui->Buttonm->setText("m");
    }
}