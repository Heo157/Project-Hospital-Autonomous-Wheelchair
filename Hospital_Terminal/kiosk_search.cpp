#include "kiosk_search.h"
#include "ui_kiosk_search.h"

#include <QMessageBox>
#include <QTableWidgetItem>

kiosk_search::kiosk_search(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_search)
{
    ui->setupUi(this);

    // =================================================
    // 1. 마스코트 이미지 설정
    // =================================================
    QPixmap pm(":/icons/HJK_2.png");
    ui->label->setPixmap(pm);
    ui->label->setScaledContents(true);
    ui->label->setAlignment(Qt::AlignCenter);

    // ---------------------------------
    // 테이블 초기 상태
    // (검색 전에는 선택 불가 상태)
    // ---------------------------------
    ui->table_basic->clearSelection();
    ui->table_basic->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->table_basic->setSelectionMode(QAbstractItemView::SingleSelection);

    // ---------------------------------
    // 더미 데이터 (현재 단계)
    // 나중에 DB 결과로 교체
    // ---------------------------------
    ui->table_basic->setRowCount(1);
    ui->table_basic->setItem(0, 0, new QTableWidgetItem("김철수"));
    ui->table_basic->setItem(0, 1, new QTableWidgetItem("P1001"));
    ui->table_basic->setItem(0, 2, new QTableWidgetItem("재활의학과"));
    ui->table_basic->setItem(0, 3, new QTableWidgetItem("십자인대 파열"));

    // ---------------------------------
    // 뒤로가기 → 메인
    // ---------------------------------
    connect(ui->btn_back, &QPushButton::clicked,
            this, &kiosk_search::goBack);

    // ---------------------------------
    // 검색 버튼
    // ---------------------------------
    connect(ui->btn_search, &QPushButton::clicked,
            this, [=]() {

                QString key = ui->edit_search->text().trimmed();

                if (key.isEmpty()) {
                    QMessageBox::information(
                        this,
                        "알림",
                        "검색어를 입력해주세요."
                        );
                    return;
                }

                // 더미 기준 검색 성공 / 실패
                if (key == "김철수" || key == "P1001") {
                    QMessageBox::information(
                        this,
                        "알림",
                        "환자 조회 성공"
                        );
                } else {
                    QMessageBox::information(
                        this,
                        "알림",
                        "조회된 환자가 없습니다."
                        );
                    ui->table_basic->clearSelection();
                }
            });

    // ---------------------------------
    // 선택 환자 → 휠체어 호출
    // ---------------------------------
    connect(ui->btn_call, &QPushButton::clicked,
            this, [=]() {

                // 테이블에서 선택된 행이 있는지 확인
                if (!ui->table_basic->selectionModel()->hasSelection()) {
                    QMessageBox::information(
                        this,
                        "알림",
                        "환자를 선택해주세요."
                        );
                    return;
                }

                // 선택된 환자 있음 → 다음 화면
                emit searchAccepted();
            });
}

kiosk_search::~kiosk_search()
{
    delete ui;
}
