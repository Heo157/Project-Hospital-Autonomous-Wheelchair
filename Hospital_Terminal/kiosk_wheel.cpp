#include "kiosk_wheel.h"
#include "ui_kiosk_wheel.h"

kiosk_wheel::kiosk_wheel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_wheel)
{
    ui->setupUi(this);
}

kiosk_wheel::~kiosk_wheel()
{
    delete ui;
}
