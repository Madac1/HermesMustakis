#include <QTRSensors.h>

//Definición sensor Sigue Lineas
#define NUM_SENSORS             6
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN 11
QTRSensorsAnalog qtra((unsigned char[]) {A5, A4, A3, A2, A1, A0},NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

//Mapeo de pines
//Motor A
#define AIN1 7
#define AIN2 8
#define PWMA 6
//Motor B
#define BIN1 9
#define BIN2 4
#define PWMB 5
//boton
#define BOTON 12

//Constantes
float KP = 0.2; //Constante Proporcional
float KI = 0;    //Constante Integral
float KD = 0.2;  //Constante Derivada

//Variables
int error = 0;      //Proporcional
int integral = 0;   //Integral
int lastError = 0;  //Ultimo error
int derivada = 0;   //Derivada

//referencia y velocidad base
int ref = 0;
int Tp = 80;

char com;
int mod;

void setup() {
  //Seteo pines motor  
  pinMode(BIN2  , OUTPUT);
  pinMode(BIN1  , OUTPUT);
  pinMode(PWMB  , OUTPUT);
  pinMode(AIN1  , OUTPUT);
  pinMode(AIN2  , OUTPUT);
  pinMode(PWMA  , OUTPUT);
  //seteo pin boton
  pinMode(BOTON, INPUT);

  //calibracion sensor
  while(digitalRead(BOTON) == 0);
  Motor(40,-40);
  for ( int i = 0; i < 150; i++) {    
    qtra.calibrate();    
  }
  Motor(0,0);

  //Se pausa el programa hasta que se presione el boton
  while(digitalRead(BOTON) == 0);
  Serial.begin(9600);
}

void loop() {
  //Lectura posicion
  int posicion = qtra.readLine(sensorValues, true, true);
  posicion = map(posicion, 0, 5000, -255, 255);
  

  //Cálculo PID
  error = posicion - ref;
  integral = integral + error;
  derivada = error - lastError;

  //Cálculo giro
  int giro = (error * KP) + (integral * KI) + (derivada * KD);

  //Seteo velocidad ruedas
  int velizq = Tp - giro;
  int velder = Tp + giro;

  //Movimiento
  Motor(velizq,velder);

  //Se actualiza el último error
  lastError = error; 
/*  
  while(Serial.available() > 0)
  {
    com = Serial.read();
    if (com == 4) {
      mod = 4;
    }
    if (com == 3) {
      mod = 3;
    }
    if (com == 2) {
      mod = 2;
    }
    if(com == 1 && mod == 4)
    {
      KD = KD + 0.02;
    }
    if(com == 0 && mod == 4)
    {
      KD = KD - 0.02;
    }
    if(com == 1 && mod == 3)
    {
      KI = KI + 0.02;
    }
    if(com == 0 && mod == 3)
    {
      KI = KI - 0.02;
    }
    if(com == 1 && mod == 2)
    {
      KP = KP + 0.02;
    }
    if(com == 0 && mod == 2)
    {
      KP = KP - 0.02;
    }
    Serial.flush();
  }
  
  Serial.print(posicion); 
  Serial.print(" "); 
  Serial.print(giro); 
  Serial.print(" | "); 
  Serial.print(error); 
  Serial.print(" "); 
  Serial.print(integral); 
  Serial.print(" "); 
  Serial.print(derivada);
  Serial.print(" | "); 
  Serial.print(velizq);
  Serial.print(" "); 
  Serial.print(velder);
  Serial.print(" | "); 
  Serial.print(KP);
  Serial.print(" "); 
  Serial.print(KI);
  Serial.print(" "); 
  Serial.print(KD);
  Serial.println();
  */
}
//accionamiento motor izquierdo
void Motoriz(int value) {
  if ( value >= 0 ) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMB, value);
}
//accionamiento motor derecho
void Motorde(int value) {
  if ( value >= 0 ) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, value);
}

//Accionamiento de motores
void Motor(int left, int righ) {
  Motoriz(left);
  Motorde(righ);
}