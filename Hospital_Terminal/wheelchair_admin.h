#ifndef WHEELCHAIR_ADMIN_H
#define WHEELCHAIR_ADMIN_H

#include <QWidget>

namespace Ui {
class wheelchair_admin;
}

class wheelchair_admin : public QWidget
{
    Q_OBJECT

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

private:
    Ui::wheelchair_admin *ui;

    void initRobotTable();
    void initCallLogTable();

    QString selectedRobotId() const;  // 현재 선택된 로봇 ID
};

#endif // WHEELCHAIR_ADMIN_H
