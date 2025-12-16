#include "wheelchair_map.h"
#include "ui_wheelchair_map.h"

#include <QDateTime>

wheelchair_map::wheelchair_map(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::wheelchair_map)
{
    ui->setupUi(this);

    // 시작 메시지 (선택)
    ui->pbTerm->append("=== Wheelchair Map Console ===");
}

wheelchair_map::~wheelchair_map()
{
    delete ui;
}

// ENTER 버튼 클릭 시
void wheelchair_map::on_pbEnter_clicked()
{
    QString text = ui->pbLeInput->text().trimmed();
    if (text.isEmpty())
        return;

    QString time = QDateTime::currentDateTime().toString("HH:mm:ss");
    ui->pbTerm->append(QString("[%1] %2").arg(time, text));

    ui->pbLeInput->clear();
}

// 입력창에서 엔터 키 눌렀을 때도 동일 동작
void wheelchair_map::on_pbLeInput_returnPressed()
{
    on_pbEnter_clicked();
}
