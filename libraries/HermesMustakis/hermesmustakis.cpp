#include <QTRSensors.h>
#include <Arduino.h>
#include <Arduino_BuiltIn.h>
#include "./HermesMustakis.h"


void hermesMustakis::mover(int velocidadIzquierda, int velocidadDerecha) {
  //Hay que usar velocidades enteras (int) ya que analogWrite solo acepta enteros.
  moverMotor(velocidadIzquierda, AIN1, AIN2, APWM);
  moverMotor(velocidadDerecha, BIN1, BIN2, BPWM);
}

void hermesMustakis::moverMotor(int vel, unsigned int IN1, unsigned int IN2, unsigned int PWM)
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

void hermesMustakis::tono_subida()
{
  tone(BUZZER, NOTE_A6 /*1760*/, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_E7 /*2637*/, DELAY_NOTA);
  delay(DELAY_NOTA);
}

void hermesMustakis::tono_bajada()
{
  tone(BUZZER, NOTE_E7, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_A6, DELAY_NOTA);
  delay(DELAY_NOTA);
}

void hermesMustakis::tono_bajo()
{
  tone(BUZZER, NOTE_A6, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_A6, DELAY_NOTA);
  delay(DELAY_NOTA);
}

void hermesMustakis::tono_alto()
{
  tone(BUZZER, NOTE_E7, DELAY_NOTA);
  delay(DELAY_NOTA);
  tone(BUZZER, NOTE_E7, DELAY_NOTA);
  delay(DELAY_NOTA);
}

int hermesMustakis::calibrar(int milisegundos)
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
  umbral = (mayor + menor) / 2; 
}

void hermesMustakis::seguidor(float Kp, float Ki, float Kd, int Tp)
{
  int posicion = leerPosicion(qtra);

  int error = posicion - ref;
  integral = integral + error;
  int derivada = error - lastError;
  lastError = error;

  int giro = Kp * error + Ki * integral + Kd * derivada;
  if (giro > limite)
  {
    giro = limite;
  }
  else if (giro < -limite)
  {
    giro = -limite;
  }

  int velocidadIzq = Tp + giro;
  int velocidadDer = Tp - giro;

  mover(velocidadIzq, velocidadDer);
  
  verbosidad_sensores(sensorValues, DEBUG);
  Serial.print(" || ");
  verbosidad_variables(error, integral, derivada, giro, DEBUG);
  Serial.println();
}

void hermesMustakis::hitos() 
{
  bool sensor_izq = leerSensor(SENSOR_LATERAL_IZQ, umbral);
  bool sensor_der = leerSensor(SENSOR_LATERAL_DER, umbral);
  
  geo = geoActual(sensor_izq, sensor_der);

  if (geo_aux != geo) {
    if (geo == 0 && geo_aux == 1) 
    {
      hitoIzquierdo();
    }
    if (geo == 0 && geo_aux == 2) 
    {
      fin++;
      hitoDerecho();
    }
    if (geo == 0 && ((geo_aux == 3))) 
    {
      cruce();
    }
    geo_aux = geo;
  }
}

bool hermesMustakis::leerSensor(int pin, int umbral)
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

int hermesMustakis::geoActual(bool sensor_izq, bool sensor_der)
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

void hermesMustakis::cruce() {
  tone(BUZZER, NOTE_A6, DELAY_NOTA * 2);
}

void hermesMustakis::hitoDerecho() 
{
  tone(BUZZER, NOTE_A6, DELAY_NOTA * 2);
  if (fin >= 2) {
    while(digitalRead(BOTON) == 0){
      mover(0,0);
    };
    
  }
}

void hermesMustakis::hitoIzquierdo() 
{
  tone(BUZZER, NOTE_E7, DELAY_NOTA * 2);
  suma_hitos_izq++;
}

void hermesMustakis::init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BOTON, INPUT);
}

void hermesMustakis::verbosidad_sensores(unsigned int sensor[], bool toggle)
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

void hermesMustakis::verbosidad_variables(int proporcional, int integral, int derivada, int giro, bool toggle)
{
  if(toggle)
  {
    Serial.print(proporcional);
    Serial.print(" ");
    Serial.print(integral);
    Serial.print(" ");
    Serial.print(derivada);
    Serial.print(" ");
    Serial.print(giro);
  }
}

int hermesMustakis::leerPosicion(QTRSensorsAnalog sensor)
{
  int pos = qtra.readLine(sensorValues, true, true); //Primero leer los valores de los sensores
  pos = map(pos, 0, 5000, -255, 255); //Para luego mapearlos con la función
  return pos;
}