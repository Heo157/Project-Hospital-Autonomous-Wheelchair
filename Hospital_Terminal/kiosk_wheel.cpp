#include "kiosk_wheel.h"
#include "ui_kiosk_wheel.h"

#include <QMessageBox>

kiosk_wheel::kiosk_wheel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_wheel)
{
    ui->setupUi(this);

    // ---------------------------------
    // 선택된 목적지 초기화
    // ---------------------------------
    selectedDestination.clear();

    // ---------------------------------
    // 목적지 선택 공통 처리
    // ---------------------------------
    auto selectDest = [&](const QString &dest) {

        // 부모 없는 QMessageBox (안 꺼지게)
        QMessageBox msg;
        msg.setWindowTitle("목적지 선택");
        msg.setText(
            QString("목적지를 '%1'(으)로 선택하셨습니다.\n이동하시겠습니까?")
                .arg(dest)
            );
        msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msg.setDefaultButton(QMessageBox::No);
        msg.setIcon(QMessageBox::Question);
        msg.setWindowModality(Qt::ApplicationModal);

        int r = msg.exec();

        if (r == QMessageBox::Yes) {
            // YES → 목적지 선택
            selectedDestination = dest;
        } else {
            // NO → 선택 취소
            selectedDestination.clear();
        }
    };

    // ---------------------------------
    // 목적지 버튼 연결
    // ---------------------------------
    connect(ui->btn_dest_food, &QPushButton::clicked,
            this, [=]() { selectDest("식당"); });

    connect(ui->btn_dest_jung, &QPushButton::clicked,
            this, [=]() { selectDest("정형외과"); });

    connect(ui->btn_dest_room, &QPushButton::clicked,
            this, [=]() { selectDest("병실"); });

    connect(ui->btn_dest_jae, &QPushButton::clicked,
            this, [=]() { selectDest("재활의학과"); });

    // ---------------------------------
    // 뒤로가기
    // ---------------------------------
    connect(ui->btn_back, &QPushButton::clicked,
            this, &kiosk_wheel::goBack);

    // ---------------------------------
    // 휠체어 호출
    // ---------------------------------
    connect(ui->btn_call, &QPushButton::clicked,
            this, [=]() {

                if (selectedDestination.isEmpty()) {
                    QMessageBox::information(
                        nullptr,
                        "알림",
                        "목적지를 선택해주세요."
                        );
                    return;
                }

                emit wheelConfirmed();
            });
}

kiosk_wheel::~kiosk_wheel()
{
    delete ui;
}
