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

#include "add_robot_dialog.h" // 위에서 만든 다이얼로그 헤더 포함
#include "database_manager.h" // DB 매니저 포함

wheelchair_admin::wheelchair_admin(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::wheelchair_admin)
{
    ui->setupUi(this);

    initRobotTable();
    initCallLogTable();

    connect(ui->cbSelectedRobot, QOverload<int>::of(&QComboBox::activated), this, [this](int index){
        // 1. 현재 선택된 텍스트(로봇 ID)를 가져옵니다.
        QString text = ui->cbSelectedRobot->itemText(index);

        // 2. 비어있지 않다면 해당 ID로 로봇 선택 함수를 호출합니다.
        if (!text.isEmpty()) {
            selectRobot(text.toInt());
        }
    });


    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &wheelchair_admin::refreshRobotTable);
    updateTimer->start(800); // 1000ms = 1초
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

    t->setContextMenuPolicy(Qt::CustomContextMenu);

    // 기존에 연결된 게 있다면 끊고 다시 연결 (중복 방지)
    disconnect(t, &QTableWidget::customContextMenuRequested, this, &wheelchair_admin::onCustomContextMenuRequested);
    connect(t, &QTableWidget::customContextMenuRequested, this, &wheelchair_admin::onCustomContextMenuRequested);
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
    return ui->cbSelectedRobot->currentText();
}

/* ───────── 슬롯 구현 ───────── */

// 로봇 상태 테이블 클릭 → 선택 로봇 표시
void wheelchair_admin::on_twRobotStatus_cellClicked(int row, int column)
{
    // 클릭된 행의 0번 컬럼(ID) 가져오기
    QTableWidgetItem *item = ui->twRobotStatus->item(row, 0);
    if (!item) return;

    int id = item->text().toInt();

    // 통합 선택 함수 호출
    selectRobot(id);

    qDebug() << "Table clicked, Robot ID selected:" << id;
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
    // 다이얼로그 띄우기
    AddRobotDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted) {
        // 사용자가 OK를 눌렀을 때 DB에 저장
        bool success = DatabaseManager::instance().addRobot(
            dlg.getName(),
            dlg.getIp(),
            dlg.getX(),
            dlg.getY(),
            dlg.getTheta()
        );

        if (success) {
            QMessageBox::information(this, "성공", "휠체어가 등록되었습니다.");
            refreshRobotTable(); // 테이블 즉시 갱신
        } else {
            QMessageBox::critical(this, "오류", "DB 등록 실패. 로그를 확인하세요.");
        }
    }
}

void wheelchair_admin::updateMapMarkers(const QList<RobotInfo> &robotList)
{
    // 맵을 표시하는 라벨
    QLabel *mapLabel = ui->lbMapPlaceholder;

    // [수정 1] 포인터 체크 대신 isNull() 함수 사용 (값/포인터 모두 호환 가능)
    // 맵 이미지가 없으면 함수 종료
    if (mapLabel->pixmap(Qt::ReturnByValue).isNull()) return;

    // ---------------------------------------------------------
    // 1. 화면상에 그려진 실제 맵 이미지 영역 계산
    // ---------------------------------------------------------
    QSize labelSize = mapLabel->size();

    // [수정 2] 화살표(->)를 점(.)으로 변경하거나, 명시적으로 값을 가져옴
    // Qt 버전에 따라 동작이 다를 수 있어 가장 안전한 방법인 Qt::ReturnByValue 사용
    QSize pixmapSize = mapLabel->pixmap(Qt::ReturnByValue).size();

    // 가로/세로 비율 계산
    double scaleW = (double)labelSize.width() / pixmapSize.width();
    double scaleH = (double)labelSize.height() / pixmapSize.height();
    double scale = qMin(scaleW, scaleH);

    // 실제 화면에 그려진 이미지의 크기
    int drawnW = pixmapSize.width() * scale;
    int drawnH = pixmapSize.height() * scale;

    // 중앙 정렬 여백(Offset)
    int offsetX = (labelSize.width() - drawnW) / 2;
    int offsetY = (labelSize.height() - drawnH) / 2;

    // ---------------------------------------------------------
    // 2. 버튼 생성 및 위치 이동
    // ---------------------------------------------------------
    QSet<int> currentIds;

    for (const RobotInfo &info : robotList) {
        currentIds.insert(info.id);
        QPushButton *btn;

        if (m_robotButtons.contains(info.id)) {
            btn = m_robotButtons[info.id];
        } else {
            btn = new QPushButton(mapLabel);
            btn->setText(info.name);
            btn->resize(30, 30);

            // 버튼 스타일 (원형)
            btn->setStyleSheet(
                "QPushButton { "
                "  border-radius: 15px; "
                "  border: 2px solid white; "
                "  font-weight: bold; "
                "}"
            );
            btn->show();

            connect(btn, &QPushButton::clicked, this, [this, info]() {
                selectRobot(info.id);
            });
            m_robotButtons.insert(info.id, btn);
        }

        // 색상 업데이트
        if (info.id == m_selectedRobotId) {
             btn->setStyleSheet("background-color: red; color: white; border-radius: 10px; border: 1px solid white;");
        } else {
             btn->setStyleSheet("background-color: pink; color: black; border-radius: 10px; border: 1px solid gray;");
        }

        QString tooltipInfo = QString("ID: %1\n상태: %2\n배터리: %3%\n위치: (%4, %5)")
                              .arg(info.id)
                              .arg(info.status)
                              .arg(info.battery)
                              .arg(info.current_x, 0, 'f', 1)
                              .arg(info.current_y, 0, 'f', 1);
        btn->setToolTip(tooltipInfo);

        // ---------------------------------------------------------
        // [핵심] 좌표 변환
        // ---------------------------------------------------------
        double ratioX = (info.current_x - MAP_ORIGIN_X) / MAP_REAL_WIDTH;
        double ratioY = (info.current_y - MAP_ORIGIN_Y) / MAP_REAL_HEIGHT;

        int pixelX = offsetX + (int)(ratioX * drawnW);

        // Y축 방향 (이미지 좌표계 기준)
        int pixelY = offsetY + (int)(ratioY * drawnH);

        // 버튼 중심점 보정
        btn->move(pixelX - btn->width()/2, pixelY - btn->height()/2);
    }

    // 사라진 로봇 버튼 삭제
    auto existingIds = m_robotButtons.keys();
    for (int id : existingIds) {
        if (!currentIds.contains(id)) {
            QPushButton *btn = m_robotButtons.take(id);
            btn->deleteLater();
        }
    }
}

void wheelchair_admin::refreshRobotTable()
{
    // ---------------------------------------------------------
    // 1. DB에서 최신 데이터 가져오기
    // ---------------------------------------------------------
    QSqlQuery query = DatabaseManager::instance().getRobotStatusQuery();

    if (!query.isActive()) {
        qDebug() << "Query is not active. DB error?";
        return;
    }

    // 2. 데이터를 구조체 리스트로 임시 저장
    QList<RobotInfo> robotList;
    int debugCount = 0;

    while(query.next()) {
        RobotInfo info;
        info.id = query.value("robot_id").toInt();
        info.name = query.value("name").toString();
        info.ip = query.value("ip_address").toString();
        info.status = query.value("op_status").toString();
        info.battery = query.value("battery_percent").toInt();
        // info.is_charging = query.value("is_charging").toBool();
        info.current_x = query.value("current_x").toDouble();
        info.current_y = query.value("current_y").toDouble();

        robotList.append(info);
        debugCount++;
    }

    if (debugCount == 0) {
        // 데이터가 없는 경우 처리 (필요시 로그 출력)
        // qDebug() << "No rows found in robot_status table.";
    }

    // ---------------------------------------------------------
    // [추가됨] 3. 콤보박스(선택 로봇) 목록 업데이트
    // DB에 있는 로봇 ID들을 콤보박스에 채워 넣습니다.
    // ---------------------------------------------------------
    // (1) 현재 선택되어 있던 값을 기억해둠 (새로고침 시 선택 풀림 방지)
    QString currentComboText = ui->cbSelectedRobot->currentText();

    ui->cbSelectedRobot->blockSignals(true); // 업데이트 중 불필요한 시그널 차단
    ui->cbSelectedRobot->clear();            // 기존 목록 초기화

    // (2) 가져온 로봇 리스트를 콤보박스에 추가
    for(const RobotInfo &info : robotList) {
        ui->cbSelectedRobot->addItem(QString::number(info.id));
    }

    // (3) 아까 기억해둔 값이 목록에 있다면 다시 선택 상태로 복구
    int index = ui->cbSelectedRobot->findText(currentComboText);
    if (index != -1) {
        ui->cbSelectedRobot->setCurrentIndex(index);
    }
    ui->cbSelectedRobot->blockSignals(false); // 차단 해제


    // ---------------------------------------------------------
    // 4. 테이블 위젯(twRobotStatus) 업데이트
    // ---------------------------------------------------------
    auto *t = ui->twRobotStatus;

    // 행 개수 맞추기 (깜빡임 최소화)
    if (t->rowCount() != robotList.size()) {
        t->setRowCount(robotList.size());
    }

    // 테이블 갱신 중 시그널 발생을 막아 불필요한 이벤트 방지
    t->blockSignals(true);

    for(int i = 0; i < robotList.size(); ++i) {
        const RobotInfo &info = robotList[i];

        // 0: 로봇 ID
        if (!t->item(i, 0)) t->setItem(i, 0, new QTableWidgetItem());
        t->item(i, 0)->setText(QString::number(info.id));
        t->item(i, 0)->setTextAlignment(Qt::AlignCenter);

        // 1: 배터리
        if (!t->item(i, 1)) t->setItem(i, 1, new QTableWidgetItem());
        t->item(i, 1)->setText(QString("%1%").arg(info.battery));
        t->item(i, 1)->setTextAlignment(Qt::AlignCenter);

        // 배터리 부족 시 글자색 빨강
        if (info.battery < 20) t->item(i, 1)->setForeground(Qt::red);
        else t->item(i, 1)->setForeground(Qt::black);

        // 2: 상태
        if (!t->item(i, 2)) t->setItem(i, 2, new QTableWidgetItem());
        t->item(i, 2)->setText(info.status);
        t->item(i, 2)->setTextAlignment(Qt::AlignCenter);

        // 3: 현재 위치
        if (!t->item(i, 3)) t->setItem(i, 3, new QTableWidgetItem());
        QString loc = QString("(%1, %2)").arg(info.current_x, 0, 'f', 1).arg(info.current_y, 0, 'f', 1);
        t->item(i, 3)->setText(loc);
        t->item(i, 3)->setTextAlignment(Qt::AlignCenter);
    }

    t->blockSignals(false);

    // ---------------------------------------------------------
    // 5. 맵 위의 버튼(마커) 위치 및 정보 업데이트
    // ---------------------------------------------------------
    updateMapMarkers(robotList);


    // ---------------------------------------------------------
    // 6. 선택 상태 복구 (테이블 하이라이트 & 버튼 색상 & 콤보박스 동기화)
    // 테이블이 새로 그려지면 선택 표시가 사라지므로 다시 칠해줌
    // ---------------------------------------------------------
    if (m_selectedRobotId != -1) {
        selectRobot(m_selectedRobotId);
    }
}

void wheelchair_admin::selectRobot(int robotId)
{
    m_selectedRobotId = robotId; // 현재 선택된 ID 기억

    // =========================================================
    // 1. 테이블 하이라이트 (파란색)
    // =========================================================
    auto *t = ui->twRobotStatus;
    t->blockSignals(true); // 테이블 클릭 시그널이 또 발생하지 않게 차단

    bool found = false;
    for(int i = 0; i < t->rowCount(); ++i) {
        // 0번 컬럼(ID)의 텍스트를 숫자로 바꿔서 비교
        if (t->item(i, 0)->text().toInt() == robotId) {
            t->selectRow(i); // 해당 행 선택
            found = true;
            break;
        }
    }
    if (!found) t->clearSelection(); // 못 찾으면 선택 해제
    t->blockSignals(false); // 차단 해제


    // =========================================================
    // 2. 버튼 색상 변경 (버튼: 핑크 -> 빨강)
    // =========================================================
    for (auto it = m_robotButtons.begin(); it != m_robotButtons.end(); ++it) {
        QPushButton *btn = it.value();
        if (it.key() == robotId) {
            // 선택됨: 빨간색 배경, 흰색 글자, 진한 테두리
            btn->setStyleSheet("background-color: #FF0000; color: white; border: 2px solid black; border-radius: 10px;");
            btn->raise(); // 버튼을 맨 위로
        } else {
            // 평소: 핑크색 배경, 검은 글자
            btn->setStyleSheet("background-color: #FFC0CB; color: black; border: 1px solid gray; border-radius: 10px;");
        }
    }

    // =========================================================
    // [여기가 핵심!] 3. 콤보박스 선택값 동기화
    // =========================================================
    ui->cbSelectedRobot->blockSignals(true); // 콤보박스 변경 시그널이 발생해 무한루프 도는 것 방지

    // 콤보박스 목록에서 robotId와 같은 숫자를 가진 항목을 찾음
    int comboIndex = ui->cbSelectedRobot->findText(QString::number(robotId));

    // 찾았으면 그 항목으로 강제 변경
    if (comboIndex != -1) {
        ui->cbSelectedRobot->setCurrentIndex(comboIndex);
    }

    ui->cbSelectedRobot->blockSignals(false); // 차단 해제
}

void wheelchair_admin::onCustomContextMenuRequested(const QPoint &pos)
{
    // 1. 클릭한 위치에 있는 아이템 가져오기
    QTableWidgetItem *item = ui->twRobotStatus->itemAt(pos);
    if (!item) return; // 빈 공간을 클릭했으면 무시

    // 2. 해당 행의 로봇 ID 가져오기 (0번 컬럼이 ID라고 가정)
    int row = item->row();
    int robotId = ui->twRobotStatus->item(row, 0)->text().toInt();

    // 3. 메뉴 만들기
    QMenu menu(this);
    QAction *deleteAction = menu.addAction("삭제하기");

    // 4. 메뉴 실행 (마우스 위치에 띄움)
    // exec은 사용자가 메뉴를 선택할 때까지 대기하다가, 선택한 Action을 반환함
    QAction *selectedItem = menu.exec(ui->twRobotStatus->mapToGlobal(pos));

    // 5. '삭제하기'를 눌렀다면 처리 함수 호출
    if (selectedItem == deleteAction) {
        deleteRobotProcess(robotId);
    }
}

void wheelchair_admin::deleteRobotProcess(int robotId)
{
    // 1. 정말 삭제할지 물어보기 (안전장치)
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "삭제 확인",
                                  QString("정말 로봇 ID %1 번을 삭제하시겠습니까?\n이 작업은 되돌릴 수 없습니다.").arg(robotId),
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::No) return; // '아니오' 누르면 취소

    // 2. DB에서 삭제 시도
    bool success = DatabaseManager::instance().deleteRobot(robotId);

    if (success) {
        QMessageBox::information(this, "성공", "삭제되었습니다.");

        // 만약 현재 선택된 로봇을 삭제했다면, 선택 상태 초기화
        if (m_selectedRobotId == robotId) {
            m_selectedRobotId = -1;
            ui->cbSelectedRobot->clear(); // 필요시 초기화
        }

        // 3. 화면 갱신 (테이블에서 사라지게)
        refreshRobotTable();
    } else {
        QMessageBox::critical(this, "오류", "삭제에 실패했습니다. DB 로그를 확인하세요.");
    }
}

void wheelchair_admin::resizeEvent(QResizeEvent *event)
{
    // 부모 클래스의 기본 동작 수행
    QWidget::resizeEvent(event);

    // 창 크기가 변했으니 맵 위의 마커 위치도 다시 계산해야 함
    // (현재 로봇 리스트가 저장되어 있지 않다면, 마지막에 저장해둔 멤버변수를 쓰거나 DB를 다시 읽어야 함)
    // 여기서는 간단히 화면 갱신 함수를 호출 (데이터가 최신이라 가정)
    refreshRobotTable();
}
