#include "dialogelectrique.h"
#include "ui_dialogelectrique.h"

DialogElectrique::DialogElectrique(double *courant, double *tension, double *puissance, double *energie, double *time, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogElectrique)
{
    ui->setupUi(this);
    Courant = courant;
    Tension = tension;
    Puissance = puissance;
    Energie = energie;
    Time = time;

    ui->graphCourant->setChart(&chartCourant);
    chartCourant.setTitle("Courant");
    chartCourant.legend()->hide();
    chartCourant.addSeries(&seriesCourant);

    ui->graphTension->setChart(&chartTension);
    chartTension.setTitle("Tension");
    chartTension.legend()->hide();
    chartTension.addSeries(&seriesTension);

    ui->graphPuissance->setChart(&chartPuissance);
    chartPuissance.setTitle("Puissance");
    chartPuissance.legend()->hide();
    chartPuissance.addSeries(&seriesPuissance);

    ui->graphEnergie->setChart(&chartEnergie);
    chartEnergie.setTitle("Energie Consomme");
    chartEnergie.legend()->hide();
    chartEnergie.addSeries(&seriesEnergie);
}

DialogElectrique::~DialogElectrique()
{
    delete ui;
}

void DialogElectrique::updateInfo()
{
    seriesCourant.append(*Time, *Courant);
    // Mise en forme du graphique (non optimal)
    chartCourant.removeSeries(&seriesCourant);
    chartCourant.addSeries(&seriesCourant);
    chartCourant.createDefaultAxes();

    seriesTension.append(*Time, *Tension);
    // Mise en forme du graphique (non optimal)
    chartTension.removeSeries(&seriesTension);
    chartTension.addSeries(&seriesTension);
    chartTension.createDefaultAxes();

    seriesPuissance.append(*Time, *Puissance);
    // Mise en forme du graphique (non optimal)
    chartPuissance.removeSeries(&seriesPuissance);
    chartPuissance.addSeries(&seriesPuissance);
    chartPuissance.createDefaultAxes();

    seriesEnergie.append(*Time, *Energie);
    // Mise en forme du graphique (non optimal)
    chartEnergie.removeSeries(&seriesEnergie);
    chartEnergie.addSeries(&seriesEnergie);
    chartEnergie.createDefaultAxes();

    ui->labelCourant->setText(QString::number(*Courant));
    ui->labelTension->setText(QString::number(*Tension));
    ui->labelPuissance->setText(QString::number(*Puissance));
    ui->labelEnergie->setText(QString::number(*Energie));
}

void DialogElectrique::setActive()
{
    active = true;
}

void DialogElectrique::setInactive()
{
    active = false;
}
