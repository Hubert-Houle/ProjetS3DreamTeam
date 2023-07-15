#ifndef DIALOGELECTRIQUE_H
#define DIALOGELECTRIQUE_H

#include <QDialog>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts>

namespace Ui {
class DialogElectrique;
}

class DialogElectrique : public QDialog
{
    Q_OBJECT

public:
    explicit DialogElectrique(double *courant, double *tension, double *puissance, double *energie, double *time, QWidget *parent = nullptr);
    ~DialogElectrique();
    void setActive();
    void setInactive();

private:
    Ui::DialogElectrique *ui;
    double *Courant;
    double *Tension;
    double *Puissance;
    double *Energie;
    double *Time;
    bool active = false;


    QLineSeries seriesCourant;
    QChart chartCourant;
    QLineSeries seriesTension;
    QChart chartTension;
    QLineSeries seriesPuissance;
    QChart chartPuissance;
    QLineSeries seriesEnergie;
    QChart chartEnergie;

signals:
    void quitPressed();

private slots:
    void updateInfo();

};

#endif // DIALOGELECTRIQUE_H
