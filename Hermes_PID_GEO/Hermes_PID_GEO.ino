#include <QTRSensors.h>
#include "pitches.h"

#define DEBUG true                //
#define DELAY_NOTA 200            //delay de las notas de feedback
#define NUM_SENSOR 6              //Cantidad de sensores principales
#define NUM_SAMPLES_PER_SENSOR 4  //Muestras por sensor
#define EMITTER_PIN 11            //Pin emisor

//Pines puente H
#define AIN1 7                    //Establece el pin de la primera fase A del puente H
#define AIN2 8                    //Establece el pin de la segunda fase A del puente H
#define APWM 6                    //Establece el pin de la alimentación A del puente H
#define BIN2 9                    //Establece el pin de la segunda fase B del puente H
#define BIN1 4                    //Establece el pin de la primera fase B del puente H
#define BPWM 5                    //Establece el pin de la alimentación B del puente H

//Pines extras
#define LED_INTEGRADO 13
#define BUZZER 10                 //Establece el pin del Buzzer
#define BOTON 12                  //Establece el pin del Boton

#define SENSOR_LATERAL_IZQ A7     //Establece el pin del sensor lateral izquierdo
#define SENSOR_LATERAL_DER A6     //Establece el pin del sensor lateral derecho

const float Kp = 0.4;  // Regula el factor proporcional (Proporcional a la posición)
const float Ki = 0.5;     // Regula el factor integrante (Sumatoria de los errores)
const float Kd = 80;   // Regula el factor derivado (Taza de cambio de posición)
const int Tp = 80;    // Guarda la velocidad base
const int ref = 0;      // Guarda el punto central del sensor de linea

int error2, error3, error4, error5, error6;
int integral = 0;       //Es la sumatoria de los errores
int lastError;          //Auxiliar que guarda el error para un ciclo futuro

int geo = 0, geo_aux = 0, geo2 = 0, geo3 = 0, geo4 = 0, geo5 = 0;
int umbral = 0;
int fin = 0;
int suma_hitos_izq = 0;
const int limite = 255;

QTRSensorsAnalog qtra((unsigned char[]){ A5, A4, A3, A2, A1, A0 }, NUM_SENSOR, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSOR];

void mover(int velocidadIzquierda, int velocidadDerecha) {
  //Hay que usar velocidades enteras (int) ya que analogWrite solo acepta enteros.
  moverMotor(velocidadIzquierda, AIN1, AIN2, APWM);
  moverMotor(velocidadDerecha, BIN1, BIN2, BPWM);
}

void moverMotor(int vel, unsigned int IN1, unsigned int IN2, unsigned int PWM)
{
  if(vel >= 0)
  //Separar en casos de velocidaddes mayores que 0 y menores que 0 (en reversa) para programar
  //correctamente la polaridad de los motores y su dirección
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, vel);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM, -vel); 
    //Hay que invertir el valor de velocidad 
    //ya que analogWrite solo acepta enteros positivos
  }
}

void tono_subida()
{
  tone(BUZZER, NOTE_A6 /*1760*/, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_E7 /*2637*/, DELAY_NOTA);
  delay(DELAY_NOTA);
}

void tono_bajada()
{
  tone(BUZZER, NOTE_E7, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_A6, DELAY_NOTA);
  delay(DELAY_NOTA);
}

void tono_bajo()
{
  tone(BUZZER, NOTE_A6, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_A6, DELAY_NOTA);
  delay(DELAY_NOTA);
}

void tono_alto()
{
  tone(BUZZER, NOTE_E7, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_E7, DELAY_NOTA);
  delay(DELAY_NOTA);
}

int calibrar(int milisegundos)
{
  unsigned long milisegundos_anteriores = millis();
  unsigned int mayor, menor;
  while (millis() - milisegundos_anteriores <= milisegundos)
  {
    qtra.calibrate();
    int sensor_izquierdo = analogRead(SENSOR_LATERAL_IZQ);
    int sensor_derecho = analogRead(SENSOR_LATERAL_DER);
    if(sensor_izquierdo < menor || sensor_derecho < menor)
    {
      if(sensor_izquierdo <= sensor_derecho)
      {
        menor = sensor_izquierdo;
      }
      else
      {
        menor = sensor_derecho;
      }
    }
    if(sensor_izquierdo > mayor || sensor_derecho > mayor)
    {
      if(sensor_izquierdo >= sensor_derecho)
      {
        mayor = sensor_izquierdo;
      }
      else
      {
        mayor = sensor_derecho;
      }
    }
  }
  return (mayor + menor)/2; 
}

void seguidor(float Kp, float Ki, float Kd, int Tp, int lim)
{
  int posicion = leerPosicion(qtra);

  int error = posicion - ref;
  integral = integral + error + error2 + error3 + error4 + error5;
  int derivada = error - lastError;
  lastError = error;

  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = lastError; 

  int giro = Kp * error + Ki * integral + Kd * derivada;
  if (giro > lim)
  {
    giro = lim;
  }
  else if (giro < -lim)
  {
    giro = -lim;
  }

  int velocidadIzq = Tp + giro;
  int velocidadDer = Tp - giro;

  mover(velocidadIzq, velocidadDer);
  
  verbosidad_sensores(sensorValues, DEBUG);
  Serial.print(" || ");
  verbosidad_variables(error, integral, derivada, giro, DEBUG);
  Serial.println();
}

void hitos() 
{
  bool sensor_izq = leerSensor(SENSOR_LATERAL_IZQ, umbral);
  bool sensor_der = leerSensor(SENSOR_LATERAL_DER, umbral);
  
  geo = geoActual(sensor_izq, sensor_der);

  if (geo_aux != geo) {
    if (geo == 0 && geo_aux == 1) 
    {
      funcionHitoIz();
    }
    if (geo == 0 && geo_aux == 2) 
    {
      fin++;
      funcionHitoDe();
    }
    if (geo == 0 && ((geo_aux == 3))) 
    {
      funcionCruce();
    }
    geo_aux = geo;
  }
}

bool leerSensor(int pin, int umbral)
{
  int sensor = analogRead(pin);
  if (sensor < umbral) 
  {
    sensor = 1;
  } 
  else 
  {
    sensor = 0;
  }
  return (bool) sensor;
}

int geoActual(bool sensor_izq, bool sensor_der)
{
  int geo;
  if (sensor_izq == 0 && sensor_der == 0) 
  {
    geo = 0;
  }
  if (sensor_izq == 1 && sensor_der == 0) 
  {
    geo = 1;
  }
  if (sensor_izq == 0 && sensor_der == 1) 
  {
    geo = 2;
  }
  if (sensor_izq == 1 && sensor_der == 1) 
  {
    geo = 3;
  }
  return geo;
}

void funcionCruce() {
  tone(BUZZER, NOTE_A6, DELAY_NOTA * 2);
}

void funcionHitoDe() 
{
  tone(BUZZER, NOTE_A6, DELAY_NOTA * 2);
  if (fin >= 2) {
    while(digitalRead(BOTON) == 0){
      mover(0,0);
    };
    
  }
}

void funcionHitoIz() 
{
  tone(BUZZER, NOTE_E7, DELAY_NOTA * 2);
  suma_hitos_izq++;
}

void hermesPinSet()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BOTON, INPUT);
}

void verbosidad_sensores(unsigned int sensor[], bool toggle)
{
  if(toggle)
  {
    for(int i = 0; i < 5; i++)
    {
      Serial.print(sensor[i]);
      if(i<4)
      {
        Serial.print(" ");
      }
    }
    Serial.print(" | ");
    Serial.print(leerSensor(SENSOR_LATERAL_DER, umbral));
    Serial.print(" ");
    Serial.print(leerSensor(SENSOR_LATERAL_IZQ, umbral));
  }
}

void verbosidad_variables(int proporcional, int integral, int derivada, int giro, bool toggle)
{
  if(toggle)
  {
    Serial.print(proporcional);
    Serial.print(" ");
    Serial.print(integral);
    Serial.print(" ");
    Serial.print(derivada);
    Serial.print(" | ");
    Serial.print(umbral);
    Serial.print(" ");
    Serial.print(giro);
  }
}

int leerPosicion(QTRSensorsAnalog sensor)
{
  int pos = qtra.readLine(sensorValues, true, true); //Primero leer los valores de los sensores
  pos = map(pos, 0, 5000, -255, 255); //Para luego mapearlos con la función
  return pos;
}

void setup() 
{
  Serial.begin(9600);

  hermesPinSet();

  while(digitalRead(BOTON) == 0);
  
  tono_subida();
  delay(800);
  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  
  mover(30,-30);
  umbral = calibrar(10000);
  mover(30,-30);
  while(leerPosicion(qtra) > 5 || leerPosicion(qtra) < -5);
  mover(0,0);
  tono_bajada();

  while(digitalRead(BOTON) == 0);
  
  tono_bajada();
  delay(400);
  tono_alto();
  delay(400);
  tono_alto();
  delay(400);
  tono_alto();

  while(digitalRead(BOTON) == 0)
  {
    seguidor(Kp, Ki, Kd, Tp, limite);
    hitos();
  }

  mover(0,0);
  tono_subida();
  tono_alto();
  tono_bajada();
}

void loop() 
{}