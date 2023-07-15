#ifndef DIALOGSETPID_H
#define DIALOGSETPID_H

#include <QDialog>

namespace Ui {
class DialogSetPID;
}

class DialogSetPID : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSetPID(QWidget *parent = nullptr);
    ~DialogSetPID();
    double getP();
    double getI();
    double getD();
    double getSP();

signals:
    void acceptButtonPressed();
    void cancelButtonPressed();

private slots:
    void on_Apply_clicked();
    void on_Cancel_clicked();

private:
    Ui::DialogSetPID *ui;
};

#endif // DIALOGSETPID_H
