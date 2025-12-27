#include "kiosk_search.h"
#include "ui_kiosk_search.h"
#include "database_manager.h" // [필수] DB 매니저 헤더 포함
#include <QShowEvent>
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
    // 테이블 초기 상태 설정
    // ---------------------------------
    ui->table_basic->clearSelection();
    ui->table_basic->setSelectionBehavior(QAbstractItemView::SelectRows);   // 행 단위 선택
    ui->table_basic->setSelectionMode(QAbstractItemView::SingleSelection);  // 하나만 선택 가능
    ui->table_basic->setEditTriggers(QAbstractItemView::NoEditTriggers);    // 수정 불가능하게

    // [변경] 더미 데이터 삭제 -> 초기엔 빈 테이블로 시작
    ui->table_basic->setRowCount(0);

    // 헤더 설정 (이름, ID, 병동, 병명)
    QStringList headers;
    headers << "이름" << "환자번호" << "병동/위치" << "병명";
    ui->table_basic->setColumnCount(4);
    ui->table_basic->setHorizontalHeaderLabels(headers);
    // 컬럼 너비 조정 (보기 좋게)
    ui->table_basic->horizontalHeader()->setStretchLastSection(true);


    // ---------------------------------
    // 뒤로가기 → 메인
    // ---------------------------------
    connect(ui->btn_back, &QPushButton::clicked,
            this, &kiosk_search::goBack);

    // ---------------------------------
    // [핵심] 검색 버튼 클릭 시 DB 조회
    // ---------------------------------
    connect(ui->btn_search, &QPushButton::clicked, this, [=]() {
        QString key = ui->edit_search->text().trimmed();

        if (key.isEmpty()) {
            QMessageBox::warning(this, "알림", "검색어(이름)를 입력해주세요.");
            return;
        }

        // 1. DB에서 환자 검색 (DatabaseManager 사용)
        // searchPatients 함수는 이름에 포함된 문자열(LIKE)로 검색합니다.
        QList<PatientFullInfo> results = DatabaseManager::instance().searchPatients(key);

        // 2. 결과 처리
        if (results.isEmpty()) {
            QMessageBox::information(this, "알림", "조회된 환자가 없습니다.");
            ui->table_basic->setRowCount(0); // 테이블 비우기
            return;
        }

        // 3. 테이블에 데이터 채워넣기
        ui->table_basic->setRowCount(results.size()); // 결과 개수만큼 행 생성

        for (int i = 0; i < results.size(); ++i) {
            const PatientFullInfo &info = results.at(i);

            // (0: 이름, 1: ID, 2: 병동, 3: 병명)
            ui->table_basic->setItem(i, 0, new QTableWidgetItem(info.name));
            ui->table_basic->setItem(i, 1, new QTableWidgetItem(info.id));

            // 병동 정보가 없으면(외래) "외래"라고 표시하거나 type 사용
            QString location = (info.type == "IN") ? info.ward : "외래";
            ui->table_basic->setItem(i, 2, new QTableWidgetItem(location));

            ui->table_basic->setItem(i, 3, new QTableWidgetItem(info.diseaseNameKR));

            // 텍스트 중앙 정렬 (보기 좋게)
            for(int c=0; c<4; c++) {
                if(ui->table_basic->item(i, c))
                    ui->table_basic->item(i, c)->setTextAlignment(Qt::AlignCenter);
            }
        }

        QMessageBox::information(this, "알림", QString("총 %1명의 환자가 검색되었습니다.").arg(results.size()));
    });

    // ---------------------------------
    // [핵심] 선택 환자 → 휠체어 호출 (다음 화면으로 데이터 전달)
    // ---------------------------------
    connect(ui->btn_call, &QPushButton::clicked, this, [=]() {
        // 1. 선택 확인
        if (!ui->table_basic->selectionModel()->hasSelection()) {
            QMessageBox::warning(this, "알림", "호출할 환자를 목록에서 선택해주세요.");
            return;
        }

        // 2. 선택된 행(Row) 정보 가져오기
        int currentRow = ui->table_basic->currentRow();

        // 이름 (0번 컬럼), ID (1번 컬럼) 가져오기
        QString name = ui->table_basic->item(currentRow, 0)->text();
        QString id   = ui->table_basic->item(currentRow, 1)->text();

        // 3. 다음 화면으로 신호 전송 (인자 포함)
        emit searchAccepted(name, id);
    });
}

void kiosk_search::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);

    ui->edit_search->clear();

    ui->table_basic->setRowCount(0);
}

kiosk_search::~kiosk_search()
{
    delete ui;
}
