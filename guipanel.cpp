#include "guipanel.h"
#include "ui_guipanel.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie

#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos

extern "C" {
#include "serial2USBprotocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

#include "usb_messages_table.h"

#include <QPainter>       // colores diferentes para los componentes
#include <QTimer>
#include <QGraphicsPixmapItem>
#include <QString>

#include <qwt_dial_needle.h>
#include <qwt_round_scale_draw.h>
#include <qwt_scale_draw.h>
#include <qwt_scale_div.h>
#include <qwt_scale_engine.h>
#include <qwt_compass.h>

#if QT_VERSION < 0x040000
#include <QColorGroup>    // Cabeceras para definir espacios de
typedef QColorGroup Palette;
#else
typedef QPalette Palette;
#endif

GUIPanel::GUIPanel(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Simulador de vuelo (2020/2021)")); // Título de la ventana

    // Conexion por el puerto serie-USB
    fConnected=false;                 // Todavía no hemos establecido la conexión USB
    ui->serialPortComboBox->clear(); // Vacía de componentes la comboBox
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        // La identificación nos permite que SOLO aparezcan los interfaces tipo USB serial de Texas Instrument
        if ((info.vendorIdentifier()==0x1CBE) && (info.productIdentifier()==0x0002))
        {
            ui->serialPortComboBox->addItem(info.portName());
        }
    ui->serialPortComboBox->setFocus();   // Componente del GUI seleccionado de inicio
    // Las funciones CONNECT son la base del funcionamiento de QT; conectan dos componentes
    // o elementos del sistema; uno que GENERA UNA SEÑAL; y otro que EJECUTA UNA FUNCION (SLOT) al recibir dicha señal.
    // En el ejemplo se conecta la señal readyRead(), que envía el componente que controla el puerto USB serie (serial),
    // con la propia clase PanelGUI, para que ejecute su funcion readRequest() en respuesta.
    // De esa forma, en cuanto el puerto serie esté preparado para leer, se lanza una petición de datos por el
    // puerto serie.El envío de readyRead por parte de "serial" es automatico, sin necesidad de instrucciones
    // del programador
    connect(&serial, SIGNAL(readyRead()), this, SLOT(readRequest()));

    ui->pingButton->setEnabled(false);    // Se deshabilita el botón de ping del interfaz gráfico, hasta que
    // se haya establecido conexión

    //Inicializa la ventana pop-up PING
    ventanaPopUp.setIcon(QMessageBox::Information);
    ventanaPopUp.setText(tr("Status: RESPUESTA A PING RECIBIDA"));
    ventanaPopUp.setStandardButtons(QMessageBox::Ok);
    ventanaPopUp.setWindowTitle(tr("Evento"));
    ventanaPopUp.setParent(this,Qt::Popup);

    // Inicializa componentes para los controles y mandos
    initPitchCompass(); // Pinta y configura el componente del control de angulo de ataque (Pitch)
    initRuedaVelocidad(); // Inicializacion para "tunear" la esfera
    initReloj(); // Iniciamos el reloj
    initDeposito(); // Iniciamos el deposito
    initPanelAltitud();

    // Configura otros controles e indicadores del GUI

    // Deshabilita controles hasta que nos conectemos
     // Configuración inicial del indicadores varios
    disableWidgets();
    originalPixmap=(QPixmap) *(ui->drone->pixmap());

    ui->ControlVelocidad->setSingleSteps(2); // Tiene que ser divisor del "factor de inercia" (para que no hay oscilacion)
    ui->ControlVelocidad->setTotalSteps(ui->RuedaVelocidad->upperBound()/2); // Para que haya una coincidencia de escalas en el dial y el Slider

    // Inicializacion de la variable del timer para el ajuste retardado de velocidad
    VelocidadTimer = new QTimer(this);
    VelocidadTimer->connect(VelocidadTimer, SIGNAL(timeout()), this, SLOT(changeValue()));

    //Inicialización de variables auxiliares
    valor_pitch1 = 0;
    valor_pitch2 = 0;

    //Ocultamos el cristal roto
    ui->CristalRoto->setVisible(false);

}

GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

void GUIPanel::readRequest()
{
    int StopCharPosition,StartCharPosition,tam;   // Solo uso notacin hungara en los elementos que se van a
    // intercambiar con el micro - para control de tamaño -
    uint8_t *pui8Frame; // Puntero a zona de memoria donde reside la trama recibida
    void *ptrtoparam;
    uint8_t ui8Message; // Para almacenar el mensaje de la trama entrante


    incommingDataBuffer.append(serial.readAll()); // Añade el contenido del puerto serie USB al array de bytes 'incommingDataBuffer'
    // así vamos acumulando  en el array la información que va llegando

    // Busca la posición del primer byte de fin de trama (0xFD) en el array. Si no estuviera presente,
    // salimos de la funcion, en caso contrario, es que ha llegado al menos una trama.
    // Hay que tener en cuenta que pueden haber llegado varios paquetes juntos.
    StopCharPosition=incommingDataBuffer.indexOf((char)STOP_FRAME_CHAR,0);
    while (StopCharPosition>=0)
    {
        //Ahora buscamos el caracter de inicio correspondiente.
        StartCharPosition=incommingDataBuffer.lastIndexOf((char)START_FRAME_CHAR,0); //Este seria el primer caracter de inicio que va delante...

        if (StartCharPosition<0)
        {
            //En caso de que no lo encuentre, no debo de hacer nada, pero debo vaciar las primeras posiciones hasta STOP_FRAME_CHAR (inclusive)
            incommingDataBuffer.remove(0,StopCharPosition+1);
            LastError=QString("Status:Fallo trozo paquete recibido");
        } else
        {
            incommingDataBuffer.remove(0,StartCharPosition); //Si hay datos anteriores al caracter de inicio, son un trozo de trama incompleto. Los tiro.
            tam=StopCharPosition-StartCharPosition+1;//El tamanio de la trama es el numero de bytes desde inicio hasta fin, ambos inclusive.
            if (tam>=MINIMUM_FRAME_SIZE)
            {
                pui8Frame=(uint8_t*)incommingDataBuffer.data(); // Puntero de trama al inicio del array de bytes
                pui8Frame++; //Nos saltamos el caracter de inicio.
                tam-=2; //Descontamos los bytes de inicio y fin del tamanio del paquete

                // Paso 1: Destuffing y cálculo del CRC. Si todo va bien, obtengo la trama
                // con valores actualizados y sin bytes de CRC.
                tam=destuff_and_check_checksum((unsigned char *)pui8Frame,tam);
                if (tam>=0)
                {
                    //El paquete está bien, luego procedo a tratarlo.
                    ui8Message=decode_message_type(pui8Frame); // Obtencion del byte de Mensaje
                    tam=get_message_param_pointer(pui8Frame,tam,&ptrtoparam);
                    switch(ui8Message) // Segun el mensaje tengo que hacer cosas distintas
                    {
                    /* A PARTIR AQUI ES DONDE SE DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS MENSAJES QUE SE ENVIEN DESDE LA TIVA */
                    case MENSAJE_PING:  // Algunos mensajes no tiene parametros
                        // Crea una ventana popup con el texto indicado
                        pingResponseReceived();
                        break;

                    case MENSAJE_POTENCIOMETRO:
                    {
                        PARAM_MENSAJE_POTENCIOMETRO giro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(giro),&giro)>0)
                        {
                            giro.roll = giro.roll & 0xFFF;
                            giro.pitch = giro.pitch & 0xFFF;
                            giro.yaw = giro.yaw & 0xFFF;

                            // Configuracion del yaw a nivel visual
                            ui->ElementoYaw->setHeading((float)convertScale((unsigned)giro.yaw,0,360)-180);
                            ui->ElementoYaw->update();

                            // Configuracion del roll a nivel visual ( se pone el pitch porque el elemento permite ambos)
                            ui->ElementoRoll->setRoll(convertScale((unsigned)giro.roll,0,360)-180);

                            // Configuracion del pitch a nivel visual
                            ui->drone->setPixmap(rotatePixmap(*(ui->drone->pixmap()),convertScale((unsigned)giro.pitch,0,180)-90));
                            ui->ElementoRoll->setPitch(-convertScale((unsigned)giro.pitch,0,180)+90);
                            ui->ElementoRoll->update();
                            valor_pitch1 = convertScale((unsigned)giro.pitch,0,180)-90;
                            valor_pitch2 = -convertScale((unsigned)giro.pitch,0,180)+90;

                        }
                    }
                        break;
                    case MENSAJE_RELOJ:
                    {
                        PARAM_MENSAJE_RELOJ valor_reloj;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(valor_reloj),&valor_reloj.reloj)>0)
                        {
                            ui->Reloj->setValue((double)valor_reloj.reloj*60.0); //Se actualiza el reloj moviendose cada segundo como si pasara una min
                        }
                    }
                        break;

                    case MENSAJE_COMBUSTIBLE:
                    {

                        PARAM_MENSAJE_COMBUSTIBLE combustible_restante;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(combustible_restante),&combustible_restante.combustible)>0){

                            if(combustible_restante.combustible > 0){

                                ui->Deposito->setValue(combustible_restante.combustible); //Actualización del depósito

                            }else{

                                ui->Deposito->setValue(0.0); //Si no hay combustible, ponemos el depósito a 0

                                // Deshabilitamos la palanca de control de velocidad
                                ui->ControlVelocidad->setDisabled(true);

                                VelocidadTimer->stop();
                                ui->RuedaVelocidad->setValue(0); //Ponemos el velocímetro a 0

                                timerPitch = new QTimer(this);
                                connect(timerPitch, SIGNAL(timeout()), this, SLOT(disminucionPitch()));
                                timerPitch->start(50);

                            }
                        }

                    }
                        break;

                    case MENSAJE_ALTURA:
                    {

                        PARAM_MENSAJE_ALTURA altitud;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(altitud),&altitud.altura)>0){

                                ui->PanelAltitud->setValue((int)altitud.altura); //Actualizamos el valor de la altura

                        }

                    }

                        break;

                    case MENSAJE_COLISION:
                    {

                               ui->PanelAltitud->setValue(0); //Ponemos el altímetro a 0
                               ui->CristalRoto->setVisible(true); //Mostramos la imagen del cristal roto
                               disableWidgets(); //Deshabilitamos los widgets
                               ui->groupBox->setEnabled(false); //Deshabilitamos los widgets del groupbox


                    }
                        break;

                    case MENSAJE_MSG_RADIO:
                    {

                        PARAM_MENSAJE_MSG_RADIO mensaje_radio;

                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(mensaje_radio),&mensaje_radio.caracteres)>0){

                                ui->statusLabel->setText(tr(mensaje_radio.caracteres)); //Se muestra el mensaje enviado por el interfaz

                        }

                    }

                        break;

                    case MENSAJE_NO_IMPLEMENTADO:
                    {
                        // En otros mensajes hay que extraer los parametros de la trama y copiarlos
                        // a una estructura para poder procesar su informacion
                        PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                        if (check_and_extract_message_param(ptrtoparam, tam, sizeof(parametro),&parametro)>0)
                        {
                            // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                            ui->statusLabel->setText(tr("  Mensaje rechazado,"));
                        }
                        else
                        {
                            // TRATAMIENTO DE ERRORES
                        }
                    }
                        break;

                        //Falta por implementar la recepcion de mas tipos de mensajes
                        //habria que decodificarlos y emitir las señales correspondientes con los parametros que correspondan

                    default:
                        //Este error lo notifico mediante la señal statusChanged
                        LastError=QString("Status: Recibido paquete inesperado");
                        ui->statusLabel->setText(tr("  Recibido paquete inesperado,"));
                        break;
                    }
                }
                else
                {
                    LastError=QString("Status: Error de stuffing o CRC");
                    ui->statusLabel->setText(tr(" Error de stuffing o CRC"));
                 }
            }
            else
            {

                // B. La trama no está completa o no tiene el tamano adecuado... no lo procesa
                //Este error lo notifico mediante la señal statusChanged
                LastError=QString("Status: Error trozo paquete recibido");
                ui->statusLabel->setText(tr(" Fallo trozo paquete recibido"));
            }
            incommingDataBuffer.remove(0,StopCharPosition-StartCharPosition+1); //Elimino el trozo que ya he procesado
        }

        StopCharPosition=incommingDataBuffer.indexOf((char)STOP_FRAME_CHAR,0); //Compruebo si el se ha recibido alguna trama completa mas. (Para ver si tengo que salir del bucle o no
    } //Fin del while....
}

// Funciones auxiliares a la gestión comunicación USB

// Establecimiento de la comunicación USB serie a través del interfaz seleccionado en la comboBox, tras pulsar el
// botón RUN del interfaz gráfico. Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
void GUIPanel::startSlave()
{
    if (serial.portName() != ui->serialPortComboBox->currentText()) {
        serial.close();
        serial.setPortName(ui->serialPortComboBox->currentText());

        if (!serial.open(QIODevice::ReadWrite)) {
            processError(tr("No puedo abrir el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setBaudRate(9600)) {
            processError(tr("No puedo establecer tasa de 9600bps en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setDataBits(QSerialPort::Data8)) {
            processError(tr("No puedo establecer 8bits de datos en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setParity(QSerialPort::NoParity)) {
            processError(tr("NO puedo establecer parida en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setStopBits(QSerialPort::OneStop)) {
            processError(tr("No puedo establecer 1bitStop en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
            processError(tr("No puedo establecer el control de flujo en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }
    }

    ui->runButton->setEnabled(false);

    // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
    ui->statusLabel->setText(tr("Estado: Ejecucion, conectado al puerto %1.")
                             .arg(ui->serialPortComboBox->currentText()));

    // Y se habilitan los controles
    ui->pingButton->setEnabled(true);

    // Variable indicadora de conexión a TRUE, para que se permita enviar mensajes en respuesta
    // a eventos del interfaz gráfico
    fConnected=true;

}

// Funcion auxiliar de procesamiento de errores de comunicación (usada por startSlave)
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
}

// Funciones SLOT que se crean automaticamente desde QTDesigner al activar una señal de un Widget del interfaz gráfico
// Se suelen asociar a funciones auxiliares, en muchos caso, por comodidad.

// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked()
{
    uint8_t pui8Frame[MAX_FRAME_SIZE];

    // Se rellenan los parametros del paquete (en este caso, el brillo)
    int size;

    // Timer que controla el movimiento retardado de la aguja de velocidad
    VelocidadTimer->start(50);
    startSlave();
    enableWidgets();

    size=create_frame((uint8_t *)pui8Frame, MENSAJE_INICIO, NULL, 0, MAX_FRAME_SIZE);

    // Si se pudo crear correctamente, se envia la trama
    if (size>0) serial.write((char *)pui8Frame,size);

}

// SLOT asociada a pulsación del botón PING
void GUIPanel::on_pingButton_clicked()
{
    pingDevice();
}

// SLOT asociada al borrado del mensaje de estado al pulsar el boton
void GUIPanel::on_statusButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

// Funciones de usuario asociadas a la respuesta a mensajes. La estructura va a ser muy parecida en casi todos los
// casos. Se va a crear una trama de un tamaño maximo (100), y se le van a introducir los elementos de
// num_secuencia, mensaje, y parametros.

// Envío de un mensaje PING

void GUIPanel::pingDevice()
{
    uint8_t paquete[MAX_FRAME_SIZE];
    int size;

    if (fConnected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // El mensaje PING no necesita parametros; de ahí el NULL, y el 0 final.
        // No vamos a usar el mecanismo de numeracion de tramas; pasamos un 0 como n de trama
        size=create_frame(paquete, MENSAJE_PING, nullptr, 0, MAX_FRAME_SIZE);
        // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
        if (size>0) serial.write((const char*)paquete,size);
    }
}

void GUIPanel::pingResponseReceived()

{
    // Ventana popUP para el caso de mensaje PING; no te deja definirla en un "caso"
    ventanaPopUp.setStyleSheet("background-color: lightgrey");
    ventanaPopUp.setModal(true);
    ventanaPopUp.show();
}

QPixmap GUIPanel::rotatePixmap(QPixmap thePixmax, int angle){
    QSize size = originalPixmap.size();
    QPixmap rotatedPixmap(size);
    rotatedPixmap.fill(QColor::fromRgb(0, 0, 0, 0)); //the new pixmap must be transparent.
    QPainter* p = new QPainter(&rotatedPixmap);
    p->translate(size.height()/2,size.height()/2);
    p->rotate(angle);
    p->translate(-size.height()/2,-size.height()/2);
    p->drawPixmap(0, 0, originalPixmap);
    p->end();
    delete p;
    return rotatedPixmap;
}


// Deshabilita los widgets mientras no queramos que funcionen
void GUIPanel::disableWidgets(){

    // Deshabilita giros del avion
    ui->PitchCompass->setEnabled(false);
    ui->ElementoRoll->setEnabled(false);
    ui->ElementoYaw->setEnabled(false);

    // Deshabilita el control de velocidad
    ui->ControlVelocidad->setEnabled(false);
    ui->RuedaVelocidad->setEnabled(false);

    // Deshabilita el reloj
    ui->Reloj->setEnabled(false);

    // Deshabilita el deposito
    ui->Deposito->setEnabled(false);

    // Deshabilita el panel de altitud
    ui->PanelAltitud->setEnabled(false);

}

// Habilita los widgets para poder utilizarlos
void GUIPanel::enableWidgets(){

    // Habilita giros del avion
    ui->PitchCompass->setEnabled(true);
    ui->ElementoRoll->setEnabled(true);
    ui->ElementoYaw->setEnabled(true);

    // Habilida el control de velocidad
    ui->ControlVelocidad->setEnabled(true);
    ui->RuedaVelocidad->setEnabled(true);

    // Habilita el reloj
    ui->Reloj->setEnabled(true);

    // Habilita el deposito
    ui->Deposito->setEnabled(true);

    // Habilita el panel de altitud
    ui->PanelAltitud->setEnabled(true);



}

// Configuracion e inicialización de la esfera de "angulo de ataque" (PITCH) del avion

void GUIPanel::initPitchCompass(){
    // Color del fondo de la esfera
    Palette colorGroup;
    colorGroup.setColor(Palette::Base, Qt::darkBlue);
    colorGroup.setColor(Palette::Foreground,
                        QColor(Qt::blue).dark(120));
    colorGroup.setColor(Palette::Text, Qt::white);
    ui->PitchCompass->setPalette(colorGroup);
    ui->PitchCompass->setMode(QwtDial::RotateNeedle); // Modo rotacion escala

    // Escala para la esfera
    // Etiquetas y ticks/marcas
    QMap<double, QString> map; // Indicamos que etiquetas queremos que salgan
    QString label;
    label.sprintf("90");
    map.insert(0, label);
    label.sprintf("45");
    map.insert(45, label);
    label.sprintf("0");
    map.insert(90, label);
    label.sprintf("-45");
    map.insert(135, label);
    label.sprintf("-90");
    map.insert(180, label);

    QwtCompassScaleDraw *pitchScDraw = new QwtCompassScaleDraw(map);

    // Longitud de las marcas
    pitchScDraw ->setTickLength(QwtScaleDiv::MinorTick,0);  // Solo queremos
    pitchScDraw->setTickLength(QwtScaleDiv::MediumTick,0); // que aparezcan las
    pitchScDraw ->setTickLength(QwtScaleDiv::MajorTick,10);  // marcas grandes
    // Establecemos donde irán los Ticks (0 esta al "Norte" de la "brujula"
    QList< double > ticks;
    ticks.append(0.0);
    ticks.append(45.0);
    ticks.append(90.0);
    ticks.append(135.0);
    ticks.append(180.0);

    // Divisiones de la escala
    QwtScaleDiv myScDiv=pitchScDraw->scaleDiv(); // Trabajamos con la escala

    myScDiv.setInterval(QwtInterval(0,360)); // Rango 0-360º
    // Solo aparecen ticks en la escala "mayor" (el resto está vacío)
    myScDiv.setTicks(QwtScaleDiv::MinorTick,QList<double>());
    myScDiv.setTicks(QwtScaleDiv::MediumTick,QList<double>());
    myScDiv.setTicks(QwtScaleDiv::MajorTick,ticks);
    // Indicamos que aparezca el "circulo interior",etiquetas y  marcas
    pitchScDraw->enableComponent(QwtAbstractScaleDraw::Labels,true);
    pitchScDraw->enableComponent(QwtAbstractScaleDraw::Ticks,true);
    pitchScDraw->enableComponent(QwtAbstractScaleDraw::Backbone,true);
    pitchScDraw->setSpacing(8); // Distancia al centro de la corona de ticks
    // Asignamos el componente de escala al compas
    ui->PitchCompass->setScaleDraw(pitchScDraw);
    // .. y el componente de divisiones a la escala
    pitchScDraw->setScaleDiv(myScDiv); //(si se hace en otro orden NO funciona)

    ui->PitchCompass->setReadOnly(true);
    ui->PitchCompass->setWrapping(false); // La aguja no puede superar los valores máximo o minimo
}

// Convierte un valor en la escala 0-4096 (centrado en 2048) a otro en la escala (min,max),
// centrado en (max-min)/2
unsigned short GUIPanel::convertScale(unsigned short value, unsigned short min, unsigned short max){
    float temp = ((float)value)/4096;
    temp = (temp * (max - min)) + min;
    return temp;
}

// Configuracion de la esfera que marca la velocidad de 0 a 200 km/h
void GUIPanel::initRuedaVelocidad(){

    // Aspecto de la escala
    QwtRoundScaleDraw *scaleDraw = new QwtRoundScaleDraw();
    scaleDraw->setSpacing(2);
    scaleDraw->setTickLength( QwtScaleDiv::MinorTick, 0 );
    scaleDraw->setTickLength( QwtScaleDiv::MediumTick, 4 );
    scaleDraw->setTickLength( QwtScaleDiv::MajorTick, 8 );
    scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, true);
    scaleDraw->enableComponent(QwtAbstractScaleDraw::Labels, true);
    ui->RuedaVelocidad->setScaleDraw(scaleDraw);
    ui->RuedaVelocidad->setWrapping(false);  // La aguja no puede superar los valores
    // maximo o minimo
    ui->RuedaVelocidad->setReadOnly(true); // La aguja no se puede mover con el raton

    // Para que no dibuje la esfera completa, sino un arco de 270º centrado en la mitad
    ui->RuedaVelocidad->setOrigin(135);
    ui->RuedaVelocidad->setScaleArc(0.0, 270.0);

    // Configuracion de la aguja del instrumento, y de los ticks
    QwtDialSimpleNeedle *needle = new QwtDialSimpleNeedle(
                QwtDialSimpleNeedle::Arrow, true, Qt::red,
                QColor(Qt::gray).light(130));
    ui->RuedaVelocidad->setNeedle(needle);
    ui->RuedaVelocidad->setScale(0.0, 200.0); // Rango de valores del instrumento
    ui->RuedaVelocidad->setScaleMaxMajor(12); // Numero max de marcas de etiqueta "grandes"
    ui->RuedaVelocidad->setScaleMaxMinor(5); // Numero max de marcas de etiqueta "pequeñas"
    ui->RuedaVelocidad->scaleDraw()->setSpacing(2); // Distancia ticks del borde exterior
    // de la esfera y las etiquetas
}

// Slot que se encarga del movimiento retardado de la aguja de velocidad
void GUIPanel::changeValue(void) // Slider Controlled
{
    double finalValue=0;
    static double posOffset = 2; // Ojo a que el tamaño de paso de SLider->step sea multiplo de estos
    static double negOffset = 4;
    if(ui->ControlVelocidad->value() > 0 )
        finalValue = ui->ControlVelocidad->value();

    double actualValue = ui->RuedaVelocidad->value();
    if(actualValue != finalValue){
        if ( (actualValue < finalValue))
            ui->RuedaVelocidad->setValue(actualValue + posOffset);
        else if (actualValue > finalValue)
        {
            ui->RuedaVelocidad->setValue(actualValue - negOffset);
        }
    }
}

// Slot que reacciona cuando se suelta la palanca que controla la velocidad y envia ese valor en km/h como mensaje
void GUIPanel::on_ControlVelocidad_sliderReleased()
{
    PARAM_MENSAJE_VELOCIDAD velocidad;
    uint8_t pui8Frame[MAX_FRAME_SIZE];

    // Se rellenan los parametros del paquete (en este caso, el brillo)
    int size;

    velocidad.bIntensity = (float)ui->ControlVelocidad->value(); //Obtenemos el valor de la barra y lo almacenamos en el valor de intensidad

    size=create_frame((uint8_t *)pui8Frame, MENSAJE_VELOCIDAD, &velocidad, sizeof(velocidad), MAX_FRAME_SIZE);

    // Si se pudo crear correctamente, se envia la trama
    if (size>0) serial.write((char *)pui8Frame,size);
}

void GUIPanel::initReloj()
{
    // Configuración del objeto reloj (a nivel gráfico)
    ui->Reloj->setLineWidth(6);
    ui->Reloj->setFrameShadow(QwtDial::Sunken);
    ui->Reloj->setHand(QwtAnalogClock::SecondHand,NULL);
}

void GUIPanel::initDeposito(){

    // Configuracion deposito (QwtThermo)
    ui->Deposito->setPipeWidth(50);   // Anchura del componente
    ui->Deposito->setValue(100); // Nivel inicial del deposito
    // Color del deposito
    QColor naranja(255, 125, 0, 180); // R,G,B, opacidad
    QBrush pincel( naranja , Qt::SolidPattern ); // Pincel del deposito
    //QBrush pincel(QPixmap(":/images/Agua.jpeg")); // se podrían usar incluso texturas), mediante imagenes almacenadas como recursos en .qrc
    ui->Deposito->setFillBrush(pincel);
}

void GUIPanel::disminucionPitch(){

    double finalValue1=45;
    double finalValue2 = -45;
    static double posOffset = 1; // Ojo a que el tamaño de paso de SLider->step sea multiplo de estos
    static double negOffset = 1;

    double actualValue1 = (double)valor_pitch1;
    double actualValue2 = (double)valor_pitch2;

    if(actualValue1 != finalValue1){
        if ( (actualValue1 < finalValue1)){
           ui->drone->setPixmap(rotatePixmap(*(ui->drone->pixmap()),actualValue1 + posOffset)); //Vamos actualizando el valor del pitch con el offset

           valor_pitch1 = actualValue1+posOffset; //Incrementamos el valor del pitch

        }

        else if (actualValue1 > finalValue1)
        {
           ui->drone->setPixmap(rotatePixmap(*(ui->drone->pixmap()),actualValue1 - negOffset));

           valor_pitch1 = actualValue1 - negOffset;
        }
    }

    if(actualValue2 != finalValue2){
        if ( (actualValue2 < finalValue2)){

           ui->ElementoRoll->setPitch(actualValue2 + posOffset);
           ui->ElementoRoll->update();

           valor_pitch2 = actualValue2+posOffset;

        }

        else if (actualValue2 > finalValue2)
        {
           ui->ElementoRoll->setPitch(actualValue2 - negOffset);
           ui->ElementoRoll->update();

           valor_pitch2 = actualValue2 - negOffset;
        }
    }
}

void GUIPanel::initPanelAltitud(){

    ui->PanelAltitud->setValue(3000); //Inicialización del panel de altitud a 3000m

}

