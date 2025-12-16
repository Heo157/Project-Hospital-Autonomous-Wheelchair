#ifndef WHEELCHAIR_MAP_H
#define WHEELCHAIR_MAP_H

#include <QWidget>

namespace Ui {
class wheelchair_map;
}

class wheelchair_map : public QWidget
{
    Q_OBJECT

public:
    explicit wheelchair_map(QWidget *parent = nullptr);
    ~wheelchair_map();

private slots:
    void on_pbEnter_clicked();              // ENTER 버튼
    void on_pbLeInput_returnPressed();      // 엔터 키 입력

private:
    Ui::wheelchair_map *ui;
};

#endif // WHEELCHAIR_MAP_H
