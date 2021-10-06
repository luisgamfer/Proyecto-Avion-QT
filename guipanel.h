#ifndef GUIPANEL_H
#define GUIPANEL_H

#include <QWidget>
#include <QSerialPort>
#include <QMessageBox>
#include <qwt_dial_needle.h>
#include <qwt_analog_clock.h>
#include <QTimer>
#include <QTime>

namespace Ui {
class GUIPanel;
}


class GUIPanel : public QWidget
{
    Q_OBJECT

public:
    //GUIPanel(QWidget *parent = 0);
    explicit GUIPanel(QWidget *parent = 0);
    ~GUIPanel(); // Da problemas

private slots:
    void readRequest();
    void on_pingButton_clicked();
    void on_runButton_clicked();
    void on_statusButton_clicked();

    void changeValue(void);

    void on_ControlVelocidad_sliderReleased();

    void disminucionPitch();

private: // funciones privadas
    void pingDevice();
    void startSlave();
    void processError(const QString &s);
    void activateRunButton();
    void pingResponseReceived();
    QPixmap rotatePixmap(const QPixmap thePixmax, int angle);
    void disableWidgets();
    void enableWidgets();
    void initPitchCompass();
    unsigned short convertScale(unsigned short value, unsigned short min, unsigned short max);
    void initRuedaVelocidad();
    void initReloj();
    void initDeposito();
    void initPanelAltitud();

private:
    Ui::GUIPanel *ui;
    int transactionCount;
    bool fConnected;
    QSerialPort serial;
    QByteArray incommingDataBuffer;
    QString LastError;
    QMessageBox ventanaPopUp;
    QPixmap originalPixmap;
    QTimer *VelocidadTimer;
    bool pedalLiberado;
    QTimer *timerPitch;
    int valor_pitch1;
    int valor_pitch2;
};

#endif // GUIPANEL_H
