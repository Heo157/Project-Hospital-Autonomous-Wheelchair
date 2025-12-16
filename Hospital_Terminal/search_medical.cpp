#include "search_medical.h"
#include "ui_search_medical.h"

#include <QTableWidgetItem>
#include <QHeaderView>

search_medical::search_medical(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::search_medical)
{
    ui->setupUi(this);

    ui->twPatientList->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->twPatientList->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->twPatientList->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->twPatientList->horizontalHeader()->setStretchLastSection(true);
    ui->twPatientList->verticalHeader()->setVisible(false);

    clearPatientForm();
}

search_medical::~search_medical()
{
    delete ui;
}


void search_medical::clearPatientForm()
{
    ui->leEditName->clear();
    ui->leEditDisease->clear();
    ui->leEditRoom->clear();
    ui->cbEditType->setCurrentIndex(0);
}

void search_medical::fillFormFromRow(int row)
{
    if (row < 0 || row >= ui->twPatientList->rowCount())
        return;

    // 체크박스 컬럼은 0번, 실제 데이터는 1~4번
    QTableWidgetItem *nameItem    = ui->twPatientList->item(row, 1);
    QTableWidgetItem *diseaseItem = ui->twPatientList->item(row, 2);
    QTableWidgetItem *roomItem    = ui->twPatientList->item(row, 3);
    QTableWidgetItem *typeItem    = ui->twPatientList->item(row, 4);

    if (!nameItem || !diseaseItem || !roomItem || !typeItem)
        return;

    ui->leEditName->setText(nameItem->text());
    ui->leEditDisease->setText(diseaseItem->text());
    ui->leEditRoom->setText(roomItem->text());

    int idx = ui->cbEditType->findText(typeItem->text());
    if (idx >= 0)
        ui->cbEditType->setCurrentIndex(idx);
}

int search_medical::firstCheckedRow() const
{
    for (int r = 0; r < ui->twPatientList->rowCount(); ++r) {
        QTableWidgetItem *checkItem = ui->twPatientList->item(r, 0);
        if (checkItem && checkItem->checkState() == Qt::Checked)
            return r;
    }
    return -1;
}

void search_medical::on_pbSearchPatient_clicked()
{

}

void search_medical::on_pbAddPatient_clicked()
{

}


void search_medical::on_pbUpdatePatient_clicked()
{

}


void search_medical::on_pbDeletePatient_clicked()
{

}


void search_medical::on_twPatientList_cellClicked(int row, int column)
{
    if (row < 0)
        return;

    if (column == 0) {
        QTableWidgetItem *checkItem = ui->twPatientList->item(row, 0);
        if (checkItem) {
            checkItem->setCheckState(
                checkItem->checkState() == Qt::Checked
                    ? Qt::Unchecked
                    : Qt::Checked
            );
        }
    }
    fillFormFromRow(row);
}
