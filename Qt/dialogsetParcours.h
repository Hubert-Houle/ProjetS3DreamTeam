#ifndef DIALOGSETPARCOURS_H
#define DIALOGSETPARCOURS_H

#include <QDialog>

namespace Ui {
class SetParcour;
}

class SetParcour : public QDialog
{
    Q_OBJECT

public:
    explicit SetParcour(QWidget *parent = nullptr);
    ~SetParcour();
    int getDD();
    int getDO();
    Ui::SetParcour *ui;

signals:
    void acceptButtonPressed();
    void cancelButtonPressed();

private slots:
    void on_Apply_released();
    void on_Apply_clicked();
};


#endif // DIALOGSETPARCOURS_H
