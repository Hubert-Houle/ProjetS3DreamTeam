#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSerialPortInfo>
#include <QString>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts>

// Propres librairies
#include "csvwriter.h"
#include "serialprotocol.h"
#include "dialogsetParcours.h"
#include "dialogsetpid.h"
#include "dialogelectrique.h"

// Classe definissant l'application
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    const qint32 BAUD_RATE = 115200;
    // variable parcour
    int distanceDepot = 0;
    int distanceObstacle = 0;

    // Variable electrique
    double newTime_ = 0;
    double lastTime_ = 0;
    double energieUtilise = 0;
    double courant = 0;
    double tension = 0;
    double puissance = 0;

    // Variable PID
    double setpoint = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    SerialProtocol* serialCom_=nullptr;

    //dialog
    SetParcour *dialogSetParcour;
    DialogSetPID *dialogSetPID;
    DialogElectrique *dialogElectrique;

    explicit MainWindow(int updateRate, QWidget *parent = nullptr);
    explicit MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow() override;
    void closeEvent(QCloseEvent *event) override;

    void sendMessage(QString msg);
    void setUpdateRate(int rateMs);

    void onPeriodicUpdate();
    void onMessageReceived(QString);

signals:
    void updateElectrique();


private slots:
    void receiveFromSerial(QString);
    void sendPulseSetting();
    void sendPulseStart();
    void manageRecording(int);
    void changeJsonKeyValue();
    void startSerialCom(QString);
    void sendPID();
    void dialogParaPaAccepted();
    void dialogParaPaCancel();
    void dialogParaPIDAccepted();
    void dialogParaPIDCancel();
    void dialogElectriqueQuit();

    void on_actionParametre_Parcour_triggered();

    void on_actionParametre_PID_triggered();

    void on_actionCommunication_triggered();

    void on_actionElectrique_triggered();

    void on_actionElectrique_triggered(bool checked);

private:
    void connectTimers(int updateRate);
    void connectButtons();
    void connectSerialPortRead();
    void connectSpinBoxes();
    void connectDialog();
    void startRecording();
    void stopRecording();
    void connectTextInputs();
    void connectComboBox();
    void portCensus();
    void sendParcourSetting(int distanceObstacle, int distanceDepot);

    bool record = false;
    CsvWriter* writer_;
    QTimer updateTimer_;
    QString msgReceived_{""};
    QString msgBuffer_{""};



    QString JsonKey_;
    QLineSeries series_;
    QChart chart_;


protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
