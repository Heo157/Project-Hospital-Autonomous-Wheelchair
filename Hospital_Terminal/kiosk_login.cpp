#include "kiosk_login.h"
#include "ui_kiosk_login.h"

#include <QMessageBox>

kiosk_login::kiosk_login(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_login)
{
    ui->setupUi(this);

    // =================================================
    // 1. 마스코트 이미지 설정
    // =================================================
    QPixmap pm(":/icons/HJK_2.png");
    ui->label->setPixmap(pm);
    ui->label->setScaledContents(true);
    ui->label->setAlignment(Qt::AlignCenter);

    connect(ui->btn_back, &QPushButton::clicked,
            this, &kiosk_login::goBack);

    connect(ui->btn_login, &QPushButton::clicked,
            this, [=]() {

                QString name = ui->edit_name->text().trimmed();
                QString id   = ui->edit_id->text().trimmed();

                if (name.isEmpty() || id.isEmpty()) {
                    QMessageBox::information(this, "오류", "이름과 환자번호를 입력해주세요.");
                    return;
                }

                if (name == "김철수" && id == "P1001") {
                    emit loginAccepted();
                } else {
                    QMessageBox::information(this, "오류", "환자 정보가 일치하지 않습니다.");
                }
            });
}

kiosk_login::~kiosk_login()
{
    delete ui;
}
