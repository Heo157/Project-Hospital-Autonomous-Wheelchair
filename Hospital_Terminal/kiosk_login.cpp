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
                name = name + "/외래";

                if (name.isEmpty()) {
                    QMessageBox::information(this, "오류", "이름을 입력해주세요.");
                    return;
                }

                emit loginAccepted(name);

            });
}

kiosk_login::~kiosk_login()
{
    delete ui;
}
