#include <QTRSensors.h>
#include "pitches.h"

#define NUM_SENSOR 6              //Cantidad de sensores principales
#define NUM_SAMPLES_PER_SENSOR 4  //Muestras por sensor
#define EMITTER_PIN 11            //Pin emisor

//Pines puente H
#define AIN1 7                    //Establece el pin de la primera fase A del puente H
#define AIN2 8                    //Establece el pin de la segunda fase A del puente H
#define PWMA 6                    //Establece el pin de la alimentación A del puente H
#define BIN2 9                    //Establece el pin de la segunda fase B del puente H
#define BIN1 4                    //Establece el pin de la primera fase B del puente H
#define BPWM 5                    //Establece el pin de la alimentación B del puente H

//Pines extras
#define BUZZER 10                 //Establece el pin del Buzzer
#define BOTON 12                  //Establece el pin del Boton

#define SENSOR_LATERAL_IZQ A7     //Establece el pin del sensor lateral izquierdo
#define SENSOR_LATERAL_DER A6     //Establece el pin del sensor lateral derecho

const float Kp = 0.25;  // Regula el factor proporcional (Proporcional a la posición)
const float Ki = 5;     // Regula el factor integrante (Sumatoria de los errores)
const float Kd = 100;   // Regula el factor derivado (Taza de cambio de posición)
const float Tp = 25;    // Guarda la velocidad base
const int ref = 0;      // Guarda el punto central del sensor de linea

int error;              //Es proporcional a la posición
int integral = 0;       //Es la sumatoria de los errores
int derivada = 0;       //Es la taza con la que cambia la posición
int lastError;          //Auxiliar que guarda el error para un ciclo futuro

int giro;               //Regula la cantidad de giro de las ruedas
int posicion;           //Guarda la posición de la linea en el arreglo de sensores
int velocidadIzq;       //Guarda la velocidad del motor derecho
int velocidadDer;       //Guarda la velocidad del motor izquierdo

int sensor_lateral_derecho;
int sensor_lateral_izquierdo;

int geo = 0, geo1 = 0, geo2 = 0, geo3 = 0, geo4 = 0, geo5 = 0;
int umbral = 750;
int fin = 0;

QTRSensorsAnalog qtra((unsigned char[]){ A5, A4, A3, A2, A1, A0 }, NUM_SENSOR, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSOR];

void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BOTON, INPUT);

  while(digitalRead(BOTON) == 0);

  tone(BUZZER, NOTE_A6 /*1760*/, 200);
  delay(200);
  tone(BUZZER, NOTE_E7 /*2637*/, 200);
  delay(200);

  for (int i = 0; i < 400; i++) {
    qtra.calibrate();
  }

  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  tone(BUZZER, NOTE_A6, 200);
  delay(200);

  while(digitalRead(BOTON) == 0);
}

void moverMotores(int velocidadIzquierda, int velocidadDerecha) {
  //Hay que usar velocidades enteras (int) ya que analogWrite solo acepta enteros.
  if(velocidadIzquierda >= 0)
  //Separar en casos de velocidaddes mayores que 0 y menores que 0 (en reversa) para programar
  //correctamente la polaridad de los motores y su dirección
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, velocidadIzquierda);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -velocidadIzquierda); 
    //Hay que invertir el valor de velocidad 
    //ya que analogWrite solo acepta enteros positivos
  }
  if(velocidadDerecha >= 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(BPWM, velocidadDerecha);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(BPWM, -velocidadDerecha);
  }
}

void loop() {
  posicion = qtra.readLine(sensorValues, true, true); //Primero leer los valores de los sensores
  posicion = map(posicion, 0, 5000, -255, 255); //Para luego mapearlos con la función

  error = posicion - ref;
  integral = integral + error;
  derivada = error - lastError;
  lastError = error;

  giro = Kp * error + Ki * integral + Kd * derivada;

  velocidadIzq = Tp + giro;
  velocidadDer = Tp - giro;

  moverMotores(velocidadIzq, velocidadDer);
  sensor_lateral_derecho = analogRead(SENSOR_LATERAL_DER);
  sensor_lateral_izquierdo = analogRead(SENSOR_LATERAL_IZQ);
  
}


void hitos() {
  int Hiz = analogRead(SENSOR_LATERAL_IZQ);
  int Hde = analogRead(SENSOR_LATERAL_DER);

  if (Hiz < umbral) {
    Hiz = 1;
  } else {
    Hiz = 0;
  }

  if (Hde < umbral) {
    Hde = 1;
  } else {
    Hde = 0;
  }
  
  if (Hiz == 0 && Hde == 0) {
    geo = 0;
  }
  if (Hiz == 1 && Hde == 0) {
    geo = 1;
  }
  if (Hiz == 0 && Hde == 1) {
    geo = 2;
  }
  if (Hiz == 1 && Hde == 1) {
    geo = 3;
  }

  if (geo1 != geo) {
    if (geo == 0 && geo1 == 1 && geo2 == 0) {
      tone(BUZZER, 1000, 100);
       //funcionHitoIz();
    }
    if (geo == 0 && geo1 == 2 && geo2 == 0) {
      
      fin++;
      funcionHitoDe();
    }
    if (geo == 0 && ((geo1 == 3) || (geo2 == 3) || (geo3 == 3))) {

      funcionCruce();
    }
    geo5 = geo4;
    geo4 = geo3;
    geo3 = geo2;
    geo2 = geo1;
    geo1 = geo;
  }
}

void funcionCruce() {
  Serial.println("Intersection");
  tone(BUZZER, 2000, 500);
}

void funcionHitoDe() {

  tone(BUZZER, 2000, 5000);
  if (fin >= 2) {
    //aqui va el codigo para frenar
  }
}

void funcionHitoIz() {

  tone(BUZZER, 2000, 1000);
  //aqui va el codigo para los cambios de curvatura
}