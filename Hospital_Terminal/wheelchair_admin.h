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
    //창 크기가 변할 때 호출되는 이벤트 함수
    void resizeEvent(QResizeEvent *event) override;

public:
    explicit wheelchair_admin(QWidget *parent = nullptr);
    ~wheelchair_admin();

private slots:
    // 로봇 상태 테이블에서 행 선택 시
    void on_twRobotStatus_cellClicked(int row, int column);

    // 제어 버튼들
    void on_pbDirectCall_clicked();
    void on_pbStop_clicked();
    void on_pbResume_clicked();
    void on_pbGoWait_clicked();
    void on_pbGoCharge_clicked();
    void on_pbAddWheel_clicked();

    void refreshRobotTable();

    //우클릭 메뉴 요청 시 실행될 슬롯
    void onCustomContextMenuRequested(const QPoint &pos);


private:
    Ui::wheelchair_admin *ui;

    void initRobotTable();
    void initCallLogTable();
    QString selectedRobotId() const;  // 현재 선택된 로봇 ID
    bool addNewRobotToDB(QString name, double x, double y, double theta);

    QList<QPushButton*> mapButtons;
    void updateMapMarkers(const QList<RobotInfo> &robotList);

    int m_selectedRobotId = -1;

    void selectRobot(int robotId);

    QMap<int, QPushButton*> m_robotButtons;

    QTimer *updateTimer;

    //db 삭제 로직
    void deleteRobotProcess(int robotId);

    //맵의 실제 크기
    const double MAP_REAL_WIDTH = 20.0;
    const double MAP_REAL_HEIGHT = 10.0;

    // (옵션) 맵 원점 보정 (맵의 (0,0)이 시작하는 오프셋이 있다면 사용)
    const double MAP_ORIGIN_X = 0.0;
    const double MAP_ORIGIN_Y = 0.0;

};

#endif // WHEELCHAIR_ADMIN_H
