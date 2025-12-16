#ifndef SEARCH_MEDICAL_H
#define SEARCH_MEDICAL_H

#include <QWidget>

namespace Ui {
class search_medical;
}

class search_medical : public QWidget
{
    Q_OBJECT

public:
    explicit search_medical(QWidget *parent = nullptr);
    ~search_medical();

private slots:
    void on_pbSearchPatient_clicked();   // 환자 조회
    void on_pbAddPatient_clicked();      // 환자 추가
    void on_pbUpdatePatient_clicked();   // 선택 환자 수정
    void on_pbDeletePatient_clicked();   // 선택 환자 삭제

    void on_twPatientList_cellClicked(int row, int column); // 행 선택 시 폼 채우기

private:
    Ui::search_medical *ui;

    void clearPatientForm();        // 입력 폼 초기화
    void fillFormFromRow(int row);  // 테이블 row → 폼
    int  firstCheckedRow() const;   // 체크된 첫 행 인덱스 (없으면 -1)
};

#endif // SEARCH_MEDICAL_H
