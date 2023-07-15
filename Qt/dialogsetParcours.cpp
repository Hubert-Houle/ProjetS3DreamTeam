#include "dialogsetParcours.h"
#include "ui_dialogsetParcours.h"

SetParcour::SetParcour(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SetParcour)
{
    ui->setupUi(this);
}

SetParcour::~SetParcour()
{
    delete ui;
}
int SetParcour::getDD(){
    return ui->DDline->text().toInt();
}
int SetParcour::getDO(){
    return ui->DOline->text().toInt();
}


void SetParcour::on_Apply_clicked()
{

    emit acceptButtonPressed();
}

