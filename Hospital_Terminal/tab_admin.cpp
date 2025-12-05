#include "tab_admin.h"
#include "ui_tab_admin.h"

tab_admin::tab_admin(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::tab_admin)
{
    ui->setupUi(this);
}

tab_admin::~tab_admin()
{
    delete ui;
}
