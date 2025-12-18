#include "kiosk_back.h"
#include "ui_kiosk_back.h"

kiosk_back::kiosk_back(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::kiosk_back)
{
    ui->setupUi(this);

    QPixmap pm(":/icons/mascotbye.png");
    ui->label_image->setPixmap(pm);
    ui->label_image->setScaledContents(true);
    ui->label_image->setAlignment(Qt::AlignCenter);
}

kiosk_back::~kiosk_back()
{
    delete ui;
}
