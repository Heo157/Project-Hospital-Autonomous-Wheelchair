#include "kiosk_login.h"
#include "ui_kiosk_login.h"

kiosk_login::kiosk_login(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_login)
{
    ui->setupUi(this);
}

kiosk_login::~kiosk_login()
{
    delete ui;
}
