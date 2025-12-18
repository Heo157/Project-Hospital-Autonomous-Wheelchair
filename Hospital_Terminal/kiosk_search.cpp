#include "kiosk_search.h"
#include "ui_kiosk_search.h"

kiosk_search::kiosk_search(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_search)
{
    ui->setupUi(this);
}

kiosk_search::~kiosk_search()
{
    delete ui;
}
