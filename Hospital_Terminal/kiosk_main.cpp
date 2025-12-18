#include "kiosk_main.h"
#include "ui_kiosk_main.h"

kiosk_main::kiosk_main(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_main)
{
    ui->setupUi(this);
        QPixmap pm(":/icons/mascot.png");
        ui->label->setPixmap(pm);
        ui->label->setScaledContents(true);
        ui->label->setAlignment(Qt::AlignCenter);
}

kiosk_main::~kiosk_main()
{
    delete ui;
}
