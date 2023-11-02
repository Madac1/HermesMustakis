#include <HermesMustakis.h>

const float Kp = 0.4;  // Regula el factor proporcional (Proporcional a la posición)
const float Ki = 0.5;     // Regula el factor integrante (Sumatoria de los errores)
const float Kd = 80;   // Regula el factor derivado (Taza de cambio de posición)
const int Tp = 80;    // Guarda la velocidad base
const int ref = 0;      // Guarda el punto central del sensor de linea

hermesMustakis hermes;

void setup() 
{
  Serial.begin(9600);

  hermes.init();

  while(digitalRead(BOTON) == 0);
  
  hermes.tono_subida();
  delay(800);
  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  tone(BUZZER, NOTE_E7, 200);
  delay(200);
  
  hermes.mover(30,-30);
  hermes.calibrar(10000);
  hermes.mover(30,-30);
  while(hermes.leerPosicion(hermes.qtra) > 5 || hermes.leerPosicion(hermes.qtra) < -5);
  hermes.mover(0,0);
  hermes.tono_bajada();

  while(digitalRead(BOTON) == 0);
  
  hermes.tono_bajada();
  delay(400);
  hermes.tono_alto();
  delay(400);
  hermes.tono_alto();
  delay(400);
  hermes.tono_alto();

  while(digitalRead(BOTON) == 0)
  {
    hermes.seguidor(Kp, Ki, Kd, Tp);
    hermes.hitos();
  }

  hermes.mover(0,0);
  hermes.tono_subida();
  hermes.tono_alto();
  hermes.tono_bajada();
}

void loop() 
{}