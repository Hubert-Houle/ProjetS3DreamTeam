#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int updateRate, QWidget *parent):
    QMainWindow(parent)
{
    // Constructeur de la classe
    // Initialisation du UI
    ui = new Ui::MainWindow;
    dialogSetParcour = new SetParcour();
    dialogSetPID = new DialogSetPID();
    dialogElectrique = new DialogElectrique(&courant, &tension, &puissance, &energieUtilise, &newTime_);
    ui->setupUi(this);

    // Initialisation du graphique
    ui->graph->setChart(&chart_);
    chart_.setTitle("Donnees brutes");
    chart_.legend()->hide();
    chart_.addSeries(&series_);

    // Fonctions de connections events/slots
    connectTimers(updateRate);
    connectButtons();
    connectSpinBoxes();
    connectTextInputs();
    connectComboBox();
    connectDialog();

    // Recensement des ports
    portCensus();

    // initialisation du timer
    updateTimer_.start();

    // init des dialogs

}

MainWindow::~MainWindow(){
    // Destructeur de la classe
    updateTimer_.stop();
    if(serialCom_!=nullptr){
      delete serialCom_;
    }
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){
    // Fonction appelee lorsque la fenetre est detruite
    event->accept();
}

void MainWindow::receiveFromSerial(QString msg){
    // Fonction appelee lors de reception sur port serie
    // Accumulation des morceaux de message
    msgBuffer_ += msg;

    //Si un message est termine
    if(msgBuffer_.endsWith('\n')){
        // Passage ASCII vers structure Json
        QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer_.toUtf8());

        // Analyse du message Json
        if(!jsonResponse.isEmpty()){
            QJsonObject jsonObj = jsonResponse.object();
            QString buff = jsonResponse.toJson(QJsonDocument::Indented);

            // Affichage des messages Json
            ui->textBrowser->setText(buff.mid(2,buff.length()-4));

            // Affichage des valeurs electriques
            if(jsonObj.contains("current") && jsonObj.contains("voltage") && jsonObj.contains("time")){
                courant = jsonObj["current"].toDouble();
                tension = jsonObj["voltage"].toDouble();
                puissance = courant * tension;
                newTime_ = jsonObj["time"].toDouble();
                energieUtilise+=  (newTime_ - lastTime_)/1000 * puissance;
                lastTime_ = newTime_;
                emit updateElectrique();
            }

            // Affichage des donnees dans le graph
            if(jsonObj.contains(JsonKey_)){
                double time = jsonObj["time"].toDouble();
                series_.append(time, jsonObj[JsonKey_].toDouble());
                // Mise en forme du graphique (non optimal)
                chart_.removeSeries(&series_);
                chart_.addSeries(&series_);
                chart_.createDefaultAxes();
            }

            // Fonction de reception de message (vide pour l'instant)
            msgReceived_ = msgBuffer_;
            onMessageReceived(msgReceived_);

            // Si les donnees doivent etre enregistrees
            if(record){
                writer_->write(jsonObj);
            }
        }
        // Reinitialisation du message tampon
        msgBuffer_ = "";
    }
}

void MainWindow::connectTimers(int updateRate){
    // Fonction de connection de timers
    connect(&updateTimer_, &QTimer::timeout, this, [this]{onPeriodicUpdate();});
    updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead(){
    // Fonction de connection au message de la classe (serialProtocol)
    connect(serialCom_, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectButtons(){
    // Fonction de connection du boutton Send
    connect(ui->pulseButton, SIGNAL(clicked()), this, SLOT(sendPulseStart()));
    connect(ui->checkBox, SIGNAL(stateChanged(int)), this, SLOT(manageRecording(int)));
}

void MainWindow::connectSpinBoxes(){
    // Fonction de connection des spin boxes
    connect(ui->DurationBox, SIGNAL(valueChanged(int)), this, SLOT(sendPulseSetting()));
    connect(ui->PWMBox, SIGNAL(valueChanged(double)), this, SLOT(sendPulseSetting()));
}

void MainWindow::connectTextInputs(){
    // Fonction de connection des entrees de texte
    connect(ui->JsonKey, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->JsonKey->text();
}
void MainWindow::connectDialog(){
    // bouton dialog
    connect(dialogSetParcour, SIGNAL(acceptButtonPressed()),this, SLOT(dialogParaPaAccepted()));
    connect(dialogSetPID, SIGNAL(acceptButtonPressed()),this,SLOT(dialogParaPIDAccepted()));
    connect(dialogElectrique, SIGNAL(quitPressed()),this,SLOT(dialogElectriqueQuit()));

    // update
    connect(this,SIGNAL(updateElectrique()),dialogElectrique, SLOT(updateInfo()));

}

void MainWindow::connectComboBox(){
    // Fonction de connection des entrees deroulantes
    connect(ui->comboBoxPort, SIGNAL(activated(QString)), this, SLOT(startSerialCom(QString)));
}

void MainWindow::portCensus(){
    // Fonction pour recenser les ports disponibles
    ui->comboBoxPort->clear();
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->comboBoxPort->addItem(port.portName());
    }
}

void MainWindow::startSerialCom(QString portName){
    // Fonction SLOT pour demarrer la communication serielle
    qDebug().noquote() << "Connection au port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    connectSerialPortRead();
}

void MainWindow::changeJsonKeyValue(){
    // Fonction SLOT pour changer la valeur de la cle Json
    series_.clear();
    JsonKey_ = ui->JsonKey->text();
}
void MainWindow::sendPID(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    // pour minimiser le nombre de decimales( QString::number)

    QJsonArray array = { QString::number(kp, 'f', 2),
                         QString::number(ki, 'f', 2),
                         QString::number(kd, 'f', 2),
                         QString::number(setpoint, 'f', 2)
                       };
    QJsonObject jsonObject
    {
        {"setGoal", array}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}
void MainWindow::sendPulseSetting(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    double PWM_val = ui->PWMBox->value();
    int duration_val = ui->DurationBox->value();
    QJsonObject jsonObject
    {// pour minimiser le nombre de decimales( QString::number)
        {"pulsePWM", QString::number(PWM_val)},
        {"pulseTime", duration_val}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendParcourSetting(int distanceObstacle, int distanceDepot){
    QJsonObject jsonObject
    {// pour minimiser le nombre de decimales( QString::number)
        {"DistanceObstacle", QString::number(distanceObstacle)},
        {"DistanceDepot", distanceDepot}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendPulseStart(){
    // Fonction SLOT pour envoyer la commande de pulse
    QJsonObject jsonObject
    {
        {"pulse", 1}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendMessage(QString msg){
    // Fonction SLOT d'ecriture sur le port serie
    if(serialCom_==nullptr){
        qDebug().noquote() <<"Erreur aucun port serie !!!";
        return;
    }
    serialCom_->sendMessage(msg);
    qDebug().noquote() <<"Message du RPI: "  <<msg;
}

void MainWindow::setUpdateRate(int rateMs){
    // Fonction d'initialisation du chronometre
    updateTimer_.start(rateMs);
}

void MainWindow::manageRecording(int stateButton){
    // Fonction SLOT pour determiner l'etat du bouton d'enregistrement
    if(stateButton == 2){
        startRecording();
    }
    if(stateButton == 0){
        stopRecording();
    }
}

void MainWindow::startRecording(){
    // Fonction SLOT pour creation d'un nouveau fichier csv
    record = true;
    writer_ = new CsvWriter("/home/pi/Desktop/");
    ui->label_pathCSV->setText(writer_->folder+writer_->filename);
}

void MainWindow::stopRecording(){
    // Fonction permettant d'arreter l'ecriture du CSV
    record = false;
    delete writer_;
}
void MainWindow::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
    qDebug().noquote() << "Message du Arduino: " << msg;
}

void MainWindow::onPeriodicUpdate(){
    // Fonction SLOT appelee a intervalle definie dans le constructeur
    qDebug().noquote() << "*";
}


void MainWindow::on_actionParametre_Parcour_triggered(){
    dialogSetParcour->show();
}
void MainWindow::on_actionParametre_PID_triggered()
{
    dialogSetPID->show();
}

void MainWindow::dialogParaPaCancel(){
    dialogSetParcour->hide();
}
void MainWindow::dialogParaPaAccepted(){
     qDebug().noquote() << "Parcour Setting accepted";

     distanceDepot = dialogSetParcour->getDD();
     distanceObstacle = dialogSetParcour->getDO();
     sendParcourSetting(distanceObstacle,distanceObstacle);
     dialogSetParcour->hide();
}
void MainWindow::dialogParaPIDCancel(){
     qDebug().noquote() << "PID Setting accepted";
     dialogSetPID->hide();
}
void MainWindow::dialogParaPIDAccepted(){
     qDebug().noquote() << "PID Setting accepted";

     kp = dialogSetPID->getP();
     ki = dialogSetPID->getI();
     kd = dialogSetPID->getD();
     setpoint = dialogSetPID->getSP();
     dialogSetPID->hide();
     sendPID();
}

void MainWindow::dialogElectriqueQuit()
{
    dialogElectrique->setInactive();
    dialogElectrique->hide();
}




void MainWindow::on_actionElectrique_triggered()
{
    qDebug().noquote() << "Electrique Open";
    dialogElectrique->show();
    dialogElectrique->setActive();
}


void MainWindow::on_actionElectrique_triggered(bool checked)
{
    bool newt = checked;
    checked = newt;
    dialogElectrique->show();
    dialogElectrique->setActive();
}

