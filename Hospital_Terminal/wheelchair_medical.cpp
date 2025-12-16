#include "wheelchair_medical.h"
#include "ui_wheelchair_medical.h"

#include <QTableWidgetItem>
#include <QHeaderView>
#include <QDateTime>
#include <QMessageBox>

wheelchair_medical::wheelchair_medical(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::wheelchair_medical)
{
    ui->setupUi(this);

    // 예약 테이블 기본 설정
    initReservationTable();

    // 예약 시간 기본값 = 현재 시간
    ui->dtReserveTime->setDateTime(QDateTime::currentDateTime());

    // 초기 상태
    updateStatus(tr("대기중"));
}

wheelchair_medical::~wheelchair_medical()
{
    delete ui;
}

/* ====== 내부 헬퍼 ====== */

void wheelchair_medical::initReservationTable()
{
    auto *table = ui->twReservation;

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setStretchLastSection(true);
    table->verticalHeader()->setVisible(false);

    // 컬럼 폭 대충 맞추기 (원하는대로 조정 가능)
    table->setColumnWidth(0, 50);   // 선택
    table->setColumnWidth(1, 90);   // 휠체어ID
    table->setColumnWidth(2, 120);  // 환자 이름
    table->setColumnWidth(3, 120);  // 병실
    table->setColumnWidth(4, 140);  // 진료실
    table->setColumnWidth(5, 160);  // 예약 시간
    // 상태 컬럼은 stretchLastSection 로 자동
}

void wheelchair_medical::resetCallForm()
{
    ui->lePatientName->clear();
    ui->leRoom->clear();
    ui->leClinic->clear();
    ui->cbEmergency->setChecked(false);
    ui->cbWheelchair->setCurrentIndex(0);
    ui->dtReserveTime->setDateTime(QDateTime::currentDateTime());
}

void wheelchair_medical::updateStatus(const QString &text)
{
    ui->lbStatusValue->setText(text);
}

void wheelchair_medical::addReservationRow(const QString &wheelId,
                                           const QString &patient,
                                           const QString &room,
                                           const QString &clinic,
                                           const QString &reserveTime,
                                           const QString &statusText)
{
    auto *table = ui->twReservation;
    int row = table->rowCount();
    table->insertRow(row);

    // 0: 선택(체크박스)
    QTableWidgetItem *checkItem = new QTableWidgetItem;
    checkItem->setFlags(checkItem->flags()
                        | Qt::ItemIsUserCheckable
                        | Qt::ItemIsEnabled);
    checkItem->setCheckState(Qt::Unchecked);
    table->setItem(row, 0, checkItem);

    // 1~6: 데이터
    table->setItem(row, 1, new QTableWidgetItem(wheelId));
    table->setItem(row, 2, new QTableWidgetItem(patient));
    table->setItem(row, 3, new QTableWidgetItem(room));
    table->setItem(row, 4, new QTableWidgetItem(clinic));
    table->setItem(row, 5, new QTableWidgetItem(reserveTime));
    table->setItem(row, 6, new QTableWidgetItem(statusText));
}

/* ====== 슬롯 구현 ====== */

// 휠체어 호출 버튼
void wheelchair_medical::on_pbSendWheelchair_clicked()
{
    const QString patient   = ui->lePatientName->text().trimmed();
    const QString room      = ui->leRoom->text().trimmed();
    const QString clinic    = ui->leClinic->text().trimmed();
    const QString wheelId   = ui->cbWheelchair->currentText();
    const bool emergency    = ui->cbEmergency->isChecked();
    const QDateTime dt      = ui->dtReserveTime->dateTime();

    if (patient.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("환자 이름을 입력하세요."));
        return;
    }
    if (room.isEmpty() || clinic.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("병실과 진료실을 모두 입력하세요."));
        return;
    }

    // 예약 시간 문자열
    const QString reserveTimeStr = dt.toString("yyyy-MM-dd HH:mm");

    // TODO:
    //  1) 여기에서 DB에 예약/호출 정보 INSERT
    //  2) ROS2/서버로 호출 메시지 publish
    //     - emergency == true 이면 priority 1로 설정
    //     - wheelId, patient, room, clinic, reserveTimeStr 등 전송

    // 지금은 디자인/테스트용으로 테이블에 한 줄 추가하고
    // 상태 텍스트만 간단히 표현
    QString statusText;
    if (dt <= QDateTime::currentDateTime()) {
        statusText = emergency ? tr("[응급] 요청중") : tr("요청중");
        updateStatus(statusText);
    } else {
        statusText = emergency ? tr("[응급] 예약") : tr("예약");
        updateStatus(tr("대기중"));
    }

    addReservationRow(wheelId, patient, room, clinic, reserveTimeStr, statusText);
    resetCallForm();
}

// 탑승 여부 체크박스
void wheelchair_medical::on_cbBoarded_stateChanged(int state)
{
    Q_UNUSED(state);

    // TODO: 탑승 여부를 DB/서버에 반영할지 결정
    // 여기서는 단순히 라벨만 살짝 바꾸는 정도로 사용해도 됨.
    // ex) 체크 시 "탑승 완료" 문구를 상태에 덧붙이거나 별도 표시 등
}

// 테이블 셀 클릭 시: 해당 예약의 상태를 상단 "현재 상태"에 보여줌
void wheelchair_medical::on_twReservation_cellClicked(int row, int column)
{
    Q_UNUSED(column);
    auto *table = ui->twReservation;
    if (row < 0 || row >= table->rowCount())
        return;

    QTableWidgetItem *statusItem = table->item(row, 6);
    if (!statusItem)
        return;

    updateStatus(statusItem->text());
}
