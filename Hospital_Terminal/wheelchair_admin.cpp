#include "wheelchair_admin.h"
#include "ui_wheelchair_admin.h"

#include <QHeaderView>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QDebug>
#include <QVBoxLayout>
#include <QGridLayout> // 그리드 레이아웃 사용

#include <QDialog>
#include <QFormLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <QFrame>

#include "add_robot_dialog.h"
#include "database_manager.h"

wheelchair_admin::wheelchair_admin(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::wheelchair_admin)
{
    ui->setupUi(this);

    initRobotTable();
    initCallLogTable(); // 호출 이력 테이블 초기화

    // 로봇 선택 콤보박스 이벤트
    connect(ui->cbSelectedRobot, QOverload<int>::of(&QComboBox::activated), this, [this](int index){
        QString text = ui->cbSelectedRobot->itemText(index);
        if (!text.isEmpty()) {
            selectRobot(text.toInt());
        }
    });

    // 타이머 설정 (로봇 상태 + 호출 이력 동시 갱신)
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &wheelchair_admin::refreshAll);
    updateTimer->start(800);
}

wheelchair_admin::~wheelchair_admin()
{
    delete ui;
}

// 통합 갱신 함수
void wheelchair_admin::refreshAll()
{
    refreshRobotTable(); // 로봇 상태 갱신
    refreshCallLog();    // 호출 이력 갱신 (요구사항 1)
}

void wheelchair_admin::initRobotTable()
{
    auto *t = ui->twRobotStatus;
    t->setSelectionBehavior(QAbstractItemView::SelectRows);
    t->setSelectionMode(QAbstractItemView::SingleSelection);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->verticalHeader()->setVisible(false);

    // [수정] 컬럼 수 6개로 증가 (ID, 배터리, 상태, 현재위치, 출발위치, 도착위치)
    t->setColumnCount(6);

    // [수정] 헤더 이름 명시적 설정
    QStringList headers;
    headers << "ID" << "배터리" << "상태" << "현재 위치" << "출발 위치" << "도착 위치";
    t->setHorizontalHeaderLabels(headers);

    t->horizontalHeader()->setStretchLastSection(true);

    // 컬럼 너비 설정
    t->setColumnWidth(0, 80);   // ID
    t->setColumnWidth(1, 90);   // 배터리
    t->setColumnWidth(2, 100);  // 상태
    t->setColumnWidth(3, 150);  // 현재 위치
    // [추가] 새로운 컬럼 너비
    t->setColumnWidth(4, 150);  // 출발 위치
    t->setColumnWidth(5, 150);  // 도착 위치

    t->setContextMenuPolicy(Qt::CustomContextMenu);
    disconnect(t, &QTableWidget::customContextMenuRequested, this, &wheelchair_admin::onCustomContextMenuRequested);
    connect(t, &QTableWidget::customContextMenuRequested, this, &wheelchair_admin::onCustomContextMenuRequested);
}

// [요구사항 1] 휠체어/예약 현황과 동일하게 보이도록 설정
void wheelchair_admin::initCallLogTable()
{
    auto *table = ui->twCallLog; // Qt Designer에서 이름이 twCallLog인지 확인

    table->setColumnCount(6); // 컬럼 6개
    QStringList headers;
    headers << "호출 시간" << "환자 이름" << "호출 위치" << "목적지" << "배차 여부" << "ETA";
    table->setHorizontalHeaderLabels(headers);

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->verticalHeader()->setVisible(false);

    // 컬럼 너비 조정
    table->setColumnWidth(0, 160); // 시간
    table->setColumnWidth(1, 100); // 이름
    table->setColumnWidth(2, 120); // 출발
    table->setColumnWidth(3, 120); // 도착
    table->setColumnWidth(4, 100); // 상태
    table->horizontalHeader()->setStretchLastSection(true); // ETA
}

// [요구사항 1] 호출 이력 데이터 실시간 갱신
void wheelchair_admin::refreshCallLog()
{
    // DB에서 Call Queue 가져오기 (기존 함수 재사용)
    QList<CallQueueItem> list = DatabaseManager::instance().getCallQueue();
    auto *table = ui->twCallLog;

    // 행 개수 맞추기
    if (table->rowCount() != list.size()) {
        table->setRowCount(list.size());
    }

    // 데이터 채우기
    for (int i = 0; i < list.size(); ++i) {
        const auto &item = list[i];

        // 0: 시간
        if(!table->item(i, 0)) table->setItem(i, 0, new QTableWidgetItem());
        table->item(i, 0)->setText(item.call_time);
        table->item(i, 0)->setTextAlignment(Qt::AlignCenter);

        // 1: 환자 이름
        if(!table->item(i, 1)) table->setItem(i, 1, new QTableWidgetItem());
        table->item(i, 1)->setText(item.caller_name);
        table->item(i, 1)->setTextAlignment(Qt::AlignCenter);

        // 2: 출발지
        if(!table->item(i, 2)) table->setItem(i, 2, new QTableWidgetItem());
        table->item(i, 2)->setText(item.start_loc);
        table->item(i, 2)->setTextAlignment(Qt::AlignCenter);

        // 3: 목적지
        if(!table->item(i, 3)) table->setItem(i, 3, new QTableWidgetItem());
        table->item(i, 3)->setText(item.dest_loc);
        table->item(i, 3)->setTextAlignment(Qt::AlignCenter);

        // 4: 배차 여부
        QString status = (item.is_dispatched == 1) ? "배차완료" : "대기중";
        if(!table->item(i, 4)) table->setItem(i, 4, new QTableWidgetItem());
        table->item(i, 4)->setText(status);
        table->item(i, 4)->setTextAlignment(Qt::AlignCenter);

        if (item.is_dispatched == 1) table->item(i, 4)->setForeground(Qt::blue);
        else table->item(i, 4)->setForeground(Qt::red);

        // 5: ETA
        if(!table->item(i, 5)) table->setItem(i, 5, new QTableWidgetItem());
        table->item(i, 5)->setText(item.eta);
        table->item(i, 5)->setTextAlignment(Qt::AlignCenter);
    }
}


void wheelchair_admin::on_pbDirectCall_clicked()
{
    // 1. 로봇 선택 확인
    if (m_selectedRobotId == -1) {
        QMessageBox::warning(this, "알림", "제어할 로봇을 먼저 선택해주세요.");
        return;
    }

    // 2. 다이얼로그 생성
    QDialog dialog(this);
    dialog.setWindowTitle("목적지 선택 및 좌표 입력");
    dialog.resize(400, 350); // 높이를 좀 더 키움

    QVBoxLayout *mainLayout = new QVBoxLayout(&dialog);

    // --- [섹션 1] 기존 장소 버튼 ---
    QLabel *label = new QLabel("저장된 장소로 이동:", &dialog);
    label->setAlignment(Qt::AlignCenter);
    QFont font = label->font();
    font.setBold(true);
    font.setPointSize(12);
    label->setFont(font);
    mainLayout->addWidget(label);

    QGridLayout *gridLayout = new QGridLayout();
    mainLayout->addLayout(gridLayout);

    QStringList locations = {"정형외과", "재활의학과", "식당", "입원실", "키오스크"};
    QString btnStyle = "QPushButton { padding: 15px; font-size: 14px; font-weight: bold; background-color: #E0E0E0; border-radius: 5px; }"
                       "QPushButton:hover { background-color: #D0D0D0; }"
                       "QPushButton:pressed { background-color: #B0B0B0; }";

    bool commandSent = false;

    // 버튼 생성 및 연결
    for(int i=0; i<locations.size(); ++i) {
        QString locName = locations[i];
        QPushButton *btn = new QPushButton(locName, &dialog);
        btn->setStyleSheet(btnStyle);
        gridLayout->addWidget(btn, i / 2, i % 2);

        connect(btn, &QPushButton::clicked, [&, locName]() {
            QPair<double, double> coord = DatabaseManager::instance().getLocation(locName);
            if (coord.first < 0 && coord.second < 0) {
                QMessageBox::critical(&dialog, "오류", QString("'%1' 좌표 없음").arg(locName));
                return;
            }

            QString who = "admin";
            if (DatabaseManager::instance().updateRobotGoal(m_selectedRobotId, coord.first, coord.second, 1, who)) {
                QMessageBox::information(&dialog, "명령 전송", QString("'%1'(%2, %3)로 이동").arg(locName).arg(coord.first).arg(coord.second));
                commandSent = true;
                dialog.accept();
            } else {
                QMessageBox::critical(&dialog, "오류", "DB 업데이트 실패");
            }
        });
    }

    // --- 구분선 추가 ---
    QFrame *line = new QFrame(&dialog);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    mainLayout->addSpacing(10);
    mainLayout->addWidget(line);
    mainLayout->addSpacing(10);

    // --- [섹션 2] 수동 좌표 입력 ---
    QLabel *manualLabel = new QLabel("좌표 직접 입력:", &dialog);
    manualLabel->setFont(font);
    manualLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(manualLabel);

    QHBoxLayout *inputLayout = new QHBoxLayout();

    // X 좌표 입력
    QLabel *lblX = new QLabel("X:", &dialog);
    QDoubleSpinBox *sbX = new QDoubleSpinBox(&dialog);
    sbX->setRange(-100.0, 100.0); // 맵 크기에 맞게 범위 설정
    sbX->setDecimals(2);
    sbX->setSingleStep(0.1);

    // Y 좌표 입력
    QLabel *lblY = new QLabel("Y:", &dialog);
    QDoubleSpinBox *sbY = new QDoubleSpinBox(&dialog);
    sbY->setRange(-100.0, 100.0);
    sbY->setDecimals(2);
    sbY->setSingleStep(0.1);

    // 이동 버튼
    QPushButton *btnManualGo = new QPushButton("좌표 이동", &dialog);
    btnManualGo->setStyleSheet("QPushButton { padding: 5px 15px; background-color: #4CAF50; color: white; font-weight: bold; border-radius: 4px; }");

    inputLayout->addWidget(lblX);
    inputLayout->addWidget(sbX);
    inputLayout->addSpacing(10);
    inputLayout->addWidget(lblY);
    inputLayout->addWidget(sbY);
    inputLayout->addWidget(btnManualGo);

    mainLayout->addLayout(inputLayout);

    // 수동 이동 버튼 이벤트 연결
    connect(btnManualGo, &QPushButton::clicked, [&]() {
        double x = sbX->value();
        double y = sbY->value();
        QString who = "admin";

        if (DatabaseManager::instance().updateRobotGoal(m_selectedRobotId, x, y, 1, who)) {
            QMessageBox::information(&dialog, "명령 전송", QString("좌표 (%1, %2)로 이동합니다.").arg(x).arg(y));
            commandSent = true;
            dialog.accept();
        } else {
            QMessageBox::critical(&dialog, "오류", "DB 업데이트 실패");
        }
    });

    // --- [하단] 취소 버튼 ---
    mainLayout->addSpacing(10);
    QPushButton *btnCancel = new QPushButton("취소", &dialog);
    connect(btnCancel, &QPushButton::clicked, &dialog, &QDialog::reject);
    mainLayout->addWidget(btnCancel);

    dialog.exec();

    if (commandSent) {
        refreshAll();
    }
}

// ... (Stop, Resume, Wait, Charge 등 나머지 버튼 기능은 기존 코드 유지) ...
// 기존 코드에서 selectRobot, refreshRobotTable, updateMapMarkers 등도 그대로 유지
// 단, refreshRobotTable 안에서 refreshCallLog()를 호출하도록 refreshAll()로 구조 변경함에 주의

void wheelchair_admin::on_pbStop_clicked() {
    if (m_selectedRobotId == -1) { QMessageBox::warning(this, "알림", "선택 필요"); return; }
    if (QMessageBox::Yes == QMessageBox::question(this, "정지", "즉시 정지하시겠습니까?")) {
        QString who = "admin";
        DatabaseManager::instance().updateRobotOrder(m_selectedRobotId, 2, who); // 2: STOP
        refreshAll();
    }
}

void wheelchair_admin::on_pbResume_clicked() {
    if (m_selectedRobotId == -1) { QMessageBox::warning(this, "알림", "선택 필요"); return; }
    if (QMessageBox::Yes == QMessageBox::question(this, "재개", "동작을 재개하시겠습니까?")) {
        QString who = "admin";
        DatabaseManager::instance().updateRobotOrder(m_selectedRobotId, 3, who); // 3: RESUME
        refreshAll();
    }
}

void wheelchair_admin::on_pbGoWait_clicked() {
    // 1. 로봇 선택 확인
    if (m_selectedRobotId == -1) {
        QMessageBox::warning(this, "알림", "선택 필요");
        return;
    }

    // 2. 확인 메시지
    if (QMessageBox::Yes == QMessageBox::question(this, "복귀", "대기 장소(키오스크)로 복귀하시겠습니까?")) {

        // 3. DB에서 '키오스크' 좌표 조회
        QString targetName = "키오스크";
        QPair<double, double> coord = DatabaseManager::instance().getLocation(targetName);

        // 4. DB 업데이트 (이동 명령 전송: order=1)
        QString who = "admin";
        bool success = DatabaseManager::instance().updateRobotGoal(
            m_selectedRobotId,
            coord.first,
            coord.second,
            4, // order 1: MOVE/CALL
            who
        );

        if (success) {
            // 성공 시 테이블 갱신
            refreshAll();
            QMessageBox::information(this, "명령 전송", QString("로봇이 '%1'로 이동합니다.").arg(targetName));
        } else {
            QMessageBox::critical(this, "오류", "DB 업데이트 실패");
        }
    }
}

void wheelchair_admin::on_pbGoCharge_clicked() {
    // 1. 로봇 선택 확인
    if (m_selectedRobotId == -1) {
        QMessageBox::warning(this, "알림", "선택 필요");
        return;
    }

    // 2. 확인 메시지
    if (QMessageBox::Yes == QMessageBox::question(this, "충전", "충전소(충전스테이션)로 이동하시겠습니까?")) {

        // 3. DB에서 '충전스테이션' 좌표 조회
        QString targetName = "충전스테이션";
        QPair<double, double> coord = DatabaseManager::instance().getLocation(targetName);


        // 4. DB 업데이트 (이동 명령 전송: order=1)
        QString who = "admin";
        bool success = DatabaseManager::instance().updateRobotGoal(
            m_selectedRobotId,
            coord.first,
            coord.second,
            5, // order 1: MOVE/CALL
            who
        );

        if (success) {
            refreshAll();
            QMessageBox::information(this, "명령 전송", QString("로봇이 '%1'로 이동합니다.").arg(targetName));
        } else {
            QMessageBox::critical(this, "오류", "DB 업데이트 실패");
        }
    }
}

void wheelchair_admin::on_pbAddWheel_clicked() {
    AddRobotDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted) {
        DatabaseManager::instance().addRobot(dlg.getName(), dlg.getIp(), dlg.getX(), dlg.getY(), dlg.getTheta());
        refreshAll();
    }
}

void wheelchair_admin::updateMapMarkers(const QList<RobotInfo> &robotList) {
    // (기존 코드와 동일하게 유지)
    // 맵 라벨에서 사이즈 가져와서 비율 계산 후 버튼 이동
     QLabel *mapLabel = ui->lbMapPlaceholder;
     if (mapLabel->pixmap(Qt::ReturnByValue).isNull()) return;

     QSize labelSize = mapLabel->size();
     QSize pixmapSize = mapLabel->pixmap(Qt::ReturnByValue).size();
     double scaleW = (double)labelSize.width() / pixmapSize.width();
     double scaleH = (double)labelSize.height() / pixmapSize.height();
     double scale = qMin(scaleW, scaleH);
     int drawnW = pixmapSize.width() * scale;
     int drawnH = pixmapSize.height() * scale;
     int offsetX = (labelSize.width() - drawnW) / 2;
     int offsetY = (labelSize.height() - drawnH) / 2;

     QSet<int> currentIds;
     for (const RobotInfo &info : robotList) {
         currentIds.insert(info.id);
         QPushButton *btn;
         if (m_robotButtons.contains(info.id)) btn = m_robotButtons[info.id];
         else {
             btn = new QPushButton(mapLabel);
             btn->setText(info.name);
             btn->resize(30, 30);
             btn->setStyleSheet("QPushButton { border-radius: 15px; border: 2px solid white; font-weight: bold; }");
             btn->show();
             connect(btn, &QPushButton::clicked, this, [this, info]() { selectRobot(info.id); });
             m_robotButtons.insert(info.id, btn);
         }

         if (info.id == m_selectedRobotId) btn->setStyleSheet("background-color: red; color: white; border-radius: 15px; border: 2px solid white;");
         else btn->setStyleSheet("background-color: pink; color: black; border-radius: 15px; border: 1px solid gray;");

         double ratioX = (info.current_x - MAP_ORIGIN_X) / MAP_REAL_WIDTH;
         double ratioY = (info.current_y - MAP_ORIGIN_Y) / MAP_REAL_HEIGHT;
         int pixelX = offsetX + (int)(ratioX * drawnW);
         int pixelY = offsetY + (int)(ratioY * drawnH);
         btn->move(pixelX - btn->width()/2, pixelY - btn->height()/2);
     }
}

void wheelchair_admin::refreshRobotTable() {
    QSqlQuery query = DatabaseManager::instance().getRobotStatusQuery();
    QList<RobotInfo> robotList;

    // [1] DB 데이터 읽기
    while(query.next()) {
        RobotInfo info;
        info.id = query.value("robot_id").toInt();
        info.name = query.value("name").toString();
        info.ip = query.value("ip_address").toString();
        info.status = query.value("op_status").toString();
        info.battery = query.value("battery_percent").toInt();

        // 좌표 읽기 (소수점)
        info.current_x = query.value("current_x").toDouble();
        info.current_y = query.value("current_y").toDouble();

        // [추가] 출발지, 목적지 좌표 읽기
        // (DB 컬럼명이 start_x, start_y, goal_x, goal_y 라고 가정)
        info.start_x = query.value("start_x").toDouble();
        info.start_y = query.value("start_y").toDouble();
        info.goal_x = query.value("goal_x").toDouble();
        info.goal_y = query.value("goal_y").toDouble();

        robotList.append(info);
        m_robotCache.insert(info.id, info);
    }

    // 콤보박스 갱신 로직 (기존 코드 유지)
    QString currentCombo = ui->cbSelectedRobot->currentText();
    ui->cbSelectedRobot->blockSignals(true);
    ui->cbSelectedRobot->clear();
    for(const auto &info : robotList) ui->cbSelectedRobot->addItem(QString::number(info.id));
    int idx = ui->cbSelectedRobot->findText(currentCombo);
    if(idx != -1) ui->cbSelectedRobot->setCurrentIndex(idx);
    ui->cbSelectedRobot->blockSignals(false);

    // [2] 테이블 갱신
    auto *t = ui->twRobotStatus;
    t->setRowCount(robotList.size());

    for(int i=0; i<robotList.size(); ++i) {
        // Col 0: ID
        t->setItem(i, 0, new QTableWidgetItem(QString::number(robotList[i].id)));

        // Col 1: 배터리
        t->setItem(i, 1, new QTableWidgetItem(QString("%1%").arg(robotList[i].battery)));

        // Col 2: 상태
        t->setItem(i, 2, new QTableWidgetItem(robotList[i].status));

        // Col 3: 현재 위치 (0.0, 0.0) 형식
        t->setItem(i, 3, new QTableWidgetItem(
            QString("(%1, %2)")
            .arg(robotList[i].current_x, 0, 'f', 1)
            .arg(robotList[i].current_y, 0, 'f', 1)));

        // [추가] Col 4: 출발 위치
        t->setItem(i, 4, new QTableWidgetItem(
            QString("(%1, %2)")
            .arg(robotList[i].start_x, 0, 'f', 1)
            .arg(robotList[i].start_y, 0, 'f', 1)));

        // [추가] Col 5: 도착 위치
        t->setItem(i, 5, new QTableWidgetItem(
            QString("(%1, %2)")
            .arg(robotList[i].goal_x, 0, 'f', 1)
            .arg(robotList[i].goal_y, 0, 'f', 1)));

        // 가운데 정렬 (옵션)
        for(int col=0; col<6; col++) {
            if(t->item(i, col)) t->item(i, col)->setTextAlignment(Qt::AlignCenter);
        }
    }

    updateMapMarkers(robotList);
    if(m_selectedRobotId != -1) selectRobot(m_selectedRobotId);
}

void wheelchair_admin::selectRobot(int robotId) {
    m_selectedRobotId = robotId;
    // ... 테이블 하이라이트 ...
    // ... 버튼 색상 변경 ...
    // (기존 코드 동일)
}

void wheelchair_admin::on_twRobotStatus_cellClicked(int row, int column) {
    QTableWidgetItem *item = ui->twRobotStatus->item(row, 0);
    if(item) selectRobot(item->text().toInt());
}

void wheelchair_admin::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    refreshRobotTable();
}

void wheelchair_admin::onCustomContextMenuRequested(const QPoint &pos) {
    QTableWidgetItem *item = ui->twRobotStatus->itemAt(pos);
    if (!item) return;

    int row = item->row();
    // 0번 컬럼에 로봇 ID가 있다고 가정
    int robotId = ui->twRobotStatus->item(row, 0)->text().toInt();

    QMenu menu(this);

    // [수정] 변수명을 delAction으로 명확하게 통일했습니다.
    QAction *delAction = menu.addAction("삭제하기");

    // [신규] SSH 연결 액션 추가
    QAction *sshAction = menu.addAction("SSH 연결");

    menu.addSeparator(); // 구분선

    // 메뉴를 실행하고 선택된 액션을 받습니다.
    QAction *selectedItem = menu.exec(ui->twRobotStatus->mapToGlobal(pos));

    // 선택된 액션이 무엇인지 확인하고 처리
    if (selectedItem == delAction) {
        deleteRobotProcess(robotId);
    }
    else if (selectedItem == sshAction) {
        openSshDialog(robotId); // SSH 다이얼로그 호출
    }
}

void wheelchair_admin::deleteRobotProcess(int robotId) {
    if(QMessageBox::Yes == QMessageBox::question(this, "삭제", "삭제하시겠습니까?")) {
        DatabaseManager::instance().deleteRobot(robotId);
        if(m_selectedRobotId == robotId) m_selectedRobotId = -1;
        refreshAll();
    }
}

QString wheelchair_admin::selectedRobotId() const { return ui->cbSelectedRobot->currentText(); }

// [신규 기능] SSH 접속용 팝업창
void wheelchair_admin::openSshDialog(int robotId)
{
    // 1. 로봇 정보 조회 (캐시에서)
    if (!m_robotCache.contains(robotId)) {
        QMessageBox::warning(this, "오류", "로봇 정보를 찾을 수 없습니다.");
        return;
    }
    RobotInfo info = m_robotCache[robotId];

    // 2. 다이얼로그 생성
    QDialog dialog(this);
    dialog.setWindowTitle("SSH 연결");
    dialog.resize(300, 150);

    QVBoxLayout *layout = new QVBoxLayout(&dialog);

    // 안내 라벨
    QLabel *label = new QLabel("접속할 주소 (User@IP):", &dialog);
    layout->addWidget(label);

    // 텍스트 입력창 (자동완성: name@ip)
    QLineEdit *lineEdit = new QLineEdit(&dialog);
    // 예: wc1@10.10.14.31 (만약 IP가 없으면 name만 뜸)
    QString defaultStr = QString("%1@%2").arg(info.name).arg(info.ip);
    lineEdit->setText(defaultStr);
    layout->addWidget(lineEdit);

    // 버튼 박스 (연결 / 취소)
    QDialogButtonBox *btnBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    btnBox->button(QDialogButtonBox::Ok)->setText("연결");
    btnBox->button(QDialogButtonBox::Cancel)->setText("취소");
    layout->addWidget(btnBox);

    connect(btnBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(btnBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    // 3. 다이얼로그 실행 및 결과 처리
    if (dialog.exec() == QDialog::Accepted) {
        QString sshTarget = lineEdit->text().trimmed();
        if (!sshTarget.isEmpty()) {
            launchSshTerminal(sshTarget);
        }
    }
}

// [신규 기능] 새 터미널 창을 열어서 SSH 실행
void wheelchair_admin::launchSshTerminal(const QString &sshTarget)
{
    // Ubuntu 기본 터미널인 gnome-terminal을 사용합니다.
    // 명령 형태: gnome-terminal -- ssh user@ip

    QString program = "gnome-terminal";
    QStringList arguments;

    // "--" 옵션은 이후 나오는 인자들을 터미널 내부에서 실행되는 명령어로 처리하라는 뜻입니다.
    arguments << "--" << "ssh" << sshTarget;

    // startDetached를 사용해야 현재 프로그램(Admin)과 별개로 터미널 창이 독립적으로 뜹니다.
    bool success = QProcess::startDetached(program, arguments);

    if (!success) {
        // gnome-terminal이 없는 경우 xterm 시도 (혹시 모르니)
        arguments.clear();
        arguments << "-e" << "ssh" << sshTarget;
        success = QProcess::startDetached("xterm", arguments);

        if (!success) {
            QMessageBox::critical(this, "실패", "터미널을 실행할 수 없습니다.\n(gnome-terminal 또는 xterm 필요)");
        }
    }
}
