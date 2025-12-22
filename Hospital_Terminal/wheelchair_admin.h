#ifndef WHEELCHAIR_ADMIN_H
#define WHEELCHAIR_ADMIN_H

#include <QWidget>
#include <QObject>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDebug>
#include <QList>
#include <QDialog>
#include <QFormLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <QTimer>
#include <QMenu>
#include <QAction>
#include <QPushButton> // 버튼 포함
#include <QMap>
#include <QSet> // QSet 포함

// 로봇 정보 구조체
struct RobotInfo{
    int id;
    QString name;
    QString ip;
    QString status;
    int battery;
    bool is_charging;
    double current_x;
    double current_y;
    double theta;
    double goal_x;
    double goal_y;
    int sensor;
    bool order;
};

namespace Ui {
class wheelchair_admin;
}

class wheelchair_admin : public QWidget
{
    Q_OBJECT

protected:
    void resizeEvent(QResizeEvent *event) override;

public:
    explicit wheelchair_admin(QWidget *parent = nullptr);
    ~wheelchair_admin();

private slots:
    void on_twRobotStatus_cellClicked(int row, int column);

    // 제어 버튼들
    void on_pbDirectCall_clicked();
    void on_pbStop_clicked();
    void on_pbResume_clicked();
    void on_pbGoWait_clicked();
    void on_pbGoCharge_clicked();
    void on_pbAddWheel_clicked();

    // [통합 갱신] 타이머에 의해 호출됨
    void refreshAll();

    void onCustomContextMenuRequested(const QPoint &pos);

private:
    Ui::wheelchair_admin *ui;

    void initRobotTable();
    void initCallLogTable(); // 테이블 헤더 설정

    // [신규] 호출 이력 데이터 채우기
    void refreshCallLog();
    // [기존] 로봇 상태 데이터 채우기
    void refreshRobotTable();

    QString selectedRobotId() const;

    void updateMapMarkers(const QList<RobotInfo> &robotList);

    int m_selectedRobotId = -1;
    void selectRobot(int robotId);

    QMap<int, QPushButton*> m_robotButtons;
    QTimer *updateTimer;

    void deleteRobotProcess(int robotId);

    // 맵 상수
    const double MAP_REAL_WIDTH = 20.0;
    const double MAP_REAL_HEIGHT = 10.0;
    const double MAP_ORIGIN_X = 0.0;
    const double MAP_ORIGIN_Y = 0.0;
};

#endif // WHEELCHAIR_ADMIN_H
