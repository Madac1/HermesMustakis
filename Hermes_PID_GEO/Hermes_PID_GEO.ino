#include <ArduControllerBluetooth.h>
#include <HermesMustakis.h>

#define TX_BT 0
#define RX_BT 1
#define BRATE_BT 9600

const float Kp = 0.08;  // Regula el factor proporcional (Proporcional a la posición)
const float Ki = 0;     // Regula el factor integrante (Sumatoria de los errores)
const float Kd = 0;   // Regula el factor derivado (Taza de cambio de posición)
const int Tp = 80;    // Guarda la velocidad base
const int ref = 0;      // Guarda el punto central del sensor de linea

hermesMustakis hermes;
ArduControllerBluetooth bt(RX_BT, TX_BT, 9600);

void setup()
{
  Serial.begin(9600);
  Serial.println("Inicializando...");
  hermes.init();
  bt.begin();  
  bt.sendATCommand("AT+NAMEHermesTemuco");

  while(digitalRead(BOTON) == 0);
  
  hermes.tono_subida();
  delay(800);
  tone(BUZZER, NOTE_E7, 200);
  delay(400);
  tone(BUZZER, NOTE_E7, 200);
  delay(400);
  tone(BUZZER, NOTE_E7, 200);
  delay(400);
  
  Serial.println("Comenzando calibración.");
  hermes.mover(30,-30);
  hermes.calibrar(10000);
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

  Serial.println("Comenzando seguimiento de linea.");
  while(digitalRead(BOTON) == 0)
  {
    bt.communication();
    hermes.seguidor(Kp, Ki, Kd, Tp);
    hermes.hitos();
  }

  hermes.mover(0,0);
  hermes.tono_subida();
  hermes.tono_bajada();
}

void loop() 
{}