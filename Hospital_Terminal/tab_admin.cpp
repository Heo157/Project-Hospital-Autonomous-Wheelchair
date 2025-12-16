#include "tab_admin.h"
#include "ui_tab_admin.h"

#include "account_admin.h"
#include "wheelchair_admin.h"
#include "wheelchair_map.h"

#include <QVBoxLayout>

tab_admin::tab_admin(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::tab_admin)
{
    ui->setupUi(this);

    // 탭1: 계정 관리
    QWidget *tab1 = ui->tabWidget->widget(0);
    auto *layout1 = new QVBoxLayout(tab1);
    layout1->setContentsMargins(0, 0, 0, 0);
    layout1->addWidget(new account_admin(tab1));

    // 탭2: 휠체어 모니터링
    QWidget *tab2 = ui->tabWidget->widget(1);
    auto *layout2 = new QVBoxLayout(tab2);
    layout2->setContentsMargins(0, 0, 0, 0);
    layout2->addWidget(new wheelchair_admin(tab2));

    // 탭3: 터미널
    QWidget *tab3 = ui->tabWidget->widget(2);
    auto *layout3 = new QVBoxLayout(tab3);
    layout3->setContentsMargins(0, 0, 0, 0);
    layout3->addWidget(new wheelchair_map(tab3));
}

tab_admin::~tab_admin()
{
    delete ui;
}
