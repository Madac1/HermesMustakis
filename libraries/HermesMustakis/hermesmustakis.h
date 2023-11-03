
#ifndef HERMESMUSTAKIS_H

#include <QTRSensors.h>
#include <pitches.h>

#define DEBUG true                //
#define DELAY_NOTA 200            //delay de las notas de feedback
#define NUM_SENSOR 6              //Cantidad de sensores principales
#define NUM_SAMPLES_PER_SENSOR 4  //Muestras por sensor
#define EMITTER_PIN 11            //Pin emisor

//Pines puente H
#define AIN1 7                    //Establece el pin de la primera fase A del puente H
#define AIN2 8                    //Establece el pin de la segunda fase A del puente H
#define APWM 6                    //Establece el pin de la alimentación A del puente H
#define BIN1 9                    //Establece el pin de la segunda fase B del puente H
#define BIN2 4                    //Establece el pin de la primera fase B del puente H
#define BPWM 5                    //Establece el pin de la alimentación B del puente H

//Pines extras
#define LED_INTEGRADO 13
#define BUZZER 10                 //Establece el pin del Buzzer
#define BOTON 12                  //Establece el pin del Boton

#define SENSOR_LATERAL_IZQ A7     //Establece el pin del sensor lateral izquierdo
#define SENSOR_LATERAL_DER A6     //Establece el pin del sensor lateral derecho

class hermesMustakis
{
    private:
        int lastError;
        unsigned long integral;
        unsigned int umbral;
        unsigned int suma_hitos_izq;
        unsigned int fin;
        unsigned int geo;
        unsigned const int limite;
        unsigned int geo_aux;
        int ref;
        const unsigned char* pines;

    public:
        QTRSensorsAnalog qtra;
        unsigned int* sensorValues;
        void mover(int velocidadIzquierda, int velocidadDerecha);
        void moverMotor(int vel, unsigned int IN1, unsigned int IN2, unsigned int PWM);
        void tono_subida();
        void tono_bajada();
        void tono_bajo();
        void tono_alto();
        int calibrar(int milisegundos);
        void seguidor(float Kp, float Ki, float Kd, int Tp);
        void hitos();
        bool leerSensor(int pin, int umbral);
        int geoActual(bool sensor_izq, bool sensor_der);
        void cruce();
        void hitoDerecho();
        void hitoIzquierdo();
        void init();
        void verbosidad_sensores(unsigned int sensor[], bool toggle);
        void verbosidad_variables(int proporcional, int integral, int derivada, int giro, bool toggle);
        int leerPosicion(QTRSensorsAnalog sensor);
        hermesMustakis(){
            unsigned long integral = 0;       //Es la sumatoria de los errores
            unsigned int geo = 0, geo_aux = 0;
            unsigned int umbral = 0;
            unsigned int fin = 0;
            unsigned int suma_hitos_izq = 0;
            unsigned const int limite = 250;
            unsigned char pines[] = {A5, A4, A3, A2, A1, A0};
            QTRSensorsAnalog qtra(pines, NUM_SENSOR, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
        };
};

#endif