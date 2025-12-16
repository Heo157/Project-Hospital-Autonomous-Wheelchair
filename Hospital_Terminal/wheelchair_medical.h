#ifndef WHEELCHAIR_MEDICAL_H
#define WHEELCHAIR_MEDICAL_H

#include <QWidget>

namespace Ui {
class wheelchair_medical;
}

class wheelchair_medical : public QWidget
{
    Q_OBJECT

public:
    explicit wheelchair_medical(QWidget *parent = nullptr);
    ~wheelchair_medical();

private slots:
    // 휠체어 호출 버튼
    void on_pbSendWheelchair_clicked();

    // 탑승 여부 체크박스 변경
    void on_cbBoarded_stateChanged(int state);

    // 예약 테이블 셀 클릭 (선택된 예약 기준으로 상태 보여줄 때 사용)
    void on_twReservation_cellClicked(int row, int column);

private:
    Ui::wheelchair_medical *ui;

    // 내부 헬퍼 함수들 (디자인 테스트 + 나중 로직 재사용)
    void initReservationTable();  // 테이블 기본 설정
    void resetCallForm();         // 호출 폼 초기화
    void updateStatus(const QString &text);     // 상태 라벨 갱신
    void addReservationRow(const QString &wheelId,
                           const QString &patient,
                           const QString &room,
                           const QString &clinic,
                           const QString &reserveTime,
                           const QString &statusText);
};

#endif // WHEELCHAIR_MEDICAL_H
