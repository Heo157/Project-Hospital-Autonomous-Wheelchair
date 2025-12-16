#include "account_admin.h"
#include "ui_account_admin.h"

#include <QTableWidgetItem>
#include <QHeaderView>

account_admin::account_admin(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::account_admin)
{
    ui->setupUi(this);
    initUserTable();
}

account_admin::~account_admin()
{
    delete ui;
}

void account_admin::initUserTable()
{
    auto *table = ui->twUserList;

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->verticalHeader()->setVisible(false);
    table->horizontalHeader()->setStretchLastSection(true);

    table->setColumnWidth(0, 50);   // 선택
    table->setColumnWidth(1, 120);  // 사용자 ID
    table->setColumnWidth(2, 120);  // 이름
    table->setColumnWidth(3, 90);   // 권한
    // 비고는 자동
}

void account_admin::on_pbAddUser_clicked()
{
    // TODO: 나중에 DB INSERT 로직 추가
}

void account_admin::on_pbUpdateUser_clicked()
{
    // TODO: 선택된 사용자 정보 수정
}

void account_admin::on_pbDeleteUser_clicked()
{
    // TODO: 선택된 사용자 삭제
}

void account_admin::on_pbClearForm_clicked()
{
    ui->leUserId->clear();
    ui->leUserName->clear();
    ui->leNote->clear();
    ui->cbRole->setCurrentIndex(0);
}

void account_admin::on_twUserList_cellClicked(int row, int column)
{
    Q_UNUSED(column);
    Q_UNUSED(row);
}
