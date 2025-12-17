#include "wheelchair_admin.h"
#include "ui_wheelchair_admin.h"

#include <QHeaderView>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QDebug>

#include <QDialog>
#include <QFormLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>

wheelchair_admin::wheelchair_admin(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::wheelchair_admin)
{
    ui->setupUi(this);

    initRobotTable();
    initCallLogTable();

    ui->leSelectedRobot->clear();
}

wheelchair_admin::~wheelchair_admin()
{
    delete ui;
}

void wheelchair_admin::initRobotTable()
{
    auto *t = ui->twRobotStatus;

    t->setSelectionBehavior(QAbstractItemView::SelectRows);
    t->setSelectionMode(QAbstractItemView::SingleSelection);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->verticalHeader()->setVisible(false);
    t->horizontalHeader()->setStretchLastSection(true);

    t->setColumnWidth(0, 80);   // 로봇 ID
    t->setColumnWidth(1, 90);   // 배터리
    t->setColumnWidth(2, 100);  // 상태
    t->setColumnWidth(3, 150);  // 현재 위치
    // 목표 위치는 자동

    // TODO: 실제 구현 시 DB/서버에서 로봇 상태를 읽어와서 행 추가
    // --- 예시 더미 데이터 (테스트용) ---
/*
    t->setRowCount(2);
    t->setItem(0, 0, new QTableWidgetItem("WC-1"));
    t->setItem(0, 1, new QTableWidgetItem("85"));
    t->setItem(0, 2, new QTableWidgetItem("대기중"));
    t->setItem(0, 3, new QTableWidgetItem("대기 스테이션"));
    t->setItem(0, 4, new QTableWidgetItem("없음"));

    t->setItem(1, 0, new QTableWidgetItem("WC-2"));
    t->setItem(1, 1, new QTableWidgetItem("30"));
    t->setItem(1, 2, new QTableWidgetItem("이동중"));
    t->setItem(1, 3, new QTableWidgetItem("3층 복도"));
    t->setItem(1, 4, new QTableWidgetItem("내과1진료실"));
*/
}

void wheelchair_admin::initCallLogTable()
{
    auto *t = ui->twCallLog;

    t->setSelectionBehavior(QAbstractItemView::SelectRows);
    t->setSelectionMode(QAbstractItemView::SingleSelection);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->verticalHeader()->setVisible(false);
    t->horizontalHeader()->setStretchLastSection(true);

    t->setColumnWidth(0, 160);  // 시간
    t->setColumnWidth(1, 80);   // 로봇 ID
    t->setColumnWidth(2, 150);  // 출발 위치
    t->setColumnWidth(3, 150);  // 도착 위치
    // 호출자는 자동

    // TODO: 실제 구현 시 DB에서 호출 이력을 읽어서 채우기
}

QString wheelchair_admin::selectedRobotId() const
{
    // 우선 우측 lineEdit 기준으로 사용
    const QString idFromEdit = ui->leSelectedRobot->text().trimmed();
    if (!idFromEdit.isEmpty())
        return idFromEdit;

    // 혹시 lineEdit이 비었으면, 현재 선택된 행에서 직접 가져오기
    auto *t = ui->twRobotStatus;
    int row = t->currentRow();
    if (row < 0)
        return QString();

    QTableWidgetItem *idItem = t->item(row, 0);
    if (!idItem)
        return QString();

    return idItem->text().trimmed();
}

/* ───────── 슬롯 구현 ───────── */

// 로봇 상태 테이블 클릭 → 선택 로봇 표시
void wheelchair_admin::on_twRobotStatus_cellClicked(int row, int column)
{
    Q_UNUSED(column);
    auto *t = ui->twRobotStatus;
    if (row < 0 || row >= t->rowCount())
        return;

    QTableWidgetItem *idItem = t->item(row, 0);  // 0: 로봇 ID
    if (!idItem)
        return;

    ui->leSelectedRobot->setText(idItem->text());
}

/* 제어 버튼들 – 지금은 메시지박스로만 확인, 나중에 ROS/서버 연동 */

void wheelchair_admin::on_pbDirectCall_clicked()
{
    const QString robotId = selectedRobotId();
    if (robotId.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("로봇 상태에서 제어할 로봇을 먼저 선택하세요."));
        return;
    }

    // TODO: 관리자용 로봇 직접 호출 다이얼로그 or 기본 목적지 호출
    QMessageBox::information(this, tr("직접 호출"),
                             tr("로봇 %1 에 직접 호출 명령을 전송했습니다. (TODO)").arg(robotId));
}

void wheelchair_admin::on_pbStop_clicked()
{
    const QString robotId = selectedRobotId();
    if (robotId.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("즉시 정지할 로봇을 선택하세요."));
        return;
    }

    // TODO: 서버/ROS2에 E-stop 또는 goal cancel 명령 전송
    QMessageBox::information(this, tr("즉시 정지"),
                             tr("로봇 %1 에 즉시 정지 명령을 전송했습니다. (TODO)").arg(robotId));
}

void wheelchair_admin::on_pbResume_clicked()
{
    const QString robotId = selectedRobotId();
    if (robotId.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("동작 재개할 로봇을 선택하세요."));
        return;
    }

    // TODO: 일시정지 해제 / 재개 명령 전송
    QMessageBox::information(this, tr("동작 재개"),
                             tr("로봇 %1 에 동작 재개 명령을 전송했습니다. (TODO)").arg(robotId));
}

void wheelchair_admin::on_pbGoWait_clicked()
{
    const QString robotId = selectedRobotId();
    if (robotId.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("대기 스테이션으로 보낼 로봇을 선택하세요."));
        return;
    }

    // TODO: '대기 스테이션' 좌표로 이동 goal 전송
    QMessageBox::information(this, tr("대기 스테이션 귀환"),
                             tr("로봇 %1 을 대기 스테이션으로 이동시키는 명령을 전송했습니다. (TODO)").arg(robotId));
}

void wheelchair_admin::on_pbGoCharge_clicked()
{
    const QString robotId = selectedRobotId();
    if (robotId.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("충전 스테이션으로 보낼 로봇을 선택하세요."));
        return;
    }

    // TODO: '충전 스테이션' 좌표로 이동 goal 전송
    QMessageBox::information(this, tr("충전 스테이션 이동"),
                             tr("로봇 %1 을 충전 스테이션으로 이동시키는 명령을 전송했습니다. (TODO)").arg(robotId));
}

void wheelchair_admin::on_pbAddWheel_clicked()
{
    const QString robotId = selectedRobotId();
    if (robotId.isEmpty()) {
        QMessageBox::warning(this, tr("알림"), tr("추가할 로봇을 입력하세요."));
        return;
    }

    // TODO: '충전 스테이션' 좌표로 이동 goal 전송
    QMessageBox::information(this, tr("로봇 추가중"),
                             tr("로봇 추가를 완료 했습니다. (TODO)").arg(robotId));
}
