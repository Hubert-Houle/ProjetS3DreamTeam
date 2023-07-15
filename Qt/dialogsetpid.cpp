#include "dialogsetpid.h"
#include "ui_dialogsetpid.h"

DialogSetPID::DialogSetPID(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogSetPID)
{
    ui->setupUi(this);
}

DialogSetPID::~DialogSetPID()
{
    delete ui;
}

void DialogSetPID::on_Apply_clicked()
{
    emit acceptButtonPressed();
}


void DialogSetPID::on_Cancel_clicked()
{
    emit cancelButtonPressed();
}
double DialogSetPID::getP(){
    return this->ui->lineEditP->text().toDouble();

}
double DialogSetPID::getI(){
    return this->ui->lineEditI->text().toDouble();

}
double DialogSetPID::getD(){
    return this->ui->lineEditD->text().toDouble();

}
double DialogSetPID::getSP(){
    return this->ui->lineEditSP->text().toDouble();

}
