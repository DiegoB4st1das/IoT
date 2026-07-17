#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define LORA_FREQ 433E6
#define SF 9
#define BW 125E3
#define CR 5

float valor = 20.0;
int contadorPaquetes = 0;
long ultimaLatenciaReal = 0; // Aquí guardamos el tiempo real de vuelo

void setupLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERROR: RA-02 no responde.");
    while (true);
  }
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.setTxPower(14);
  LoRa.setSyncWord(0xF5);
  Serial.println("LoRa TX listo 433 MHz, SF9, BW125");
}

void setup() {
  Serial.begin(115200);
  setupLoRa();
}

void loop() {
  valor += 0.3;
  if (valor > 35.0) valor = 20.0;
  contadorPaquetes++;

  StaticJsonDocument<128> doc;
  doc["v"] = valor;
  doc["id"] = contadorPaquetes;
  doc["lat"] = ultimaLatenciaReal; // Le mandamos el dato real calculado al RX

  char payload[80];
  serializeJson(doc, payload);

  unsigned long tiempoSalida = millis(); // Empezamos el cronómetro

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  Serial.print("TX [");
  Serial.print(contadorPaquetes);
  Serial.print("]: ");
  Serial.println(payload);

  // Nos quedamos escuchando un ratito para atrapar el ECO del receptor
  unsigned long inicioEspera = millis();
  bool ecoAtrapado = false;
 
  while (millis() - inicioEspera < 1000) {
    int tamanoEco = LoRa.parsePacket();
    if (tamanoEco > 0) {
      // Calculamos: (Tiempo Total de ida y vuelta) / 2
      ultimaLatenciaReal = (millis() - tiempoSalida) / 2;
      ecoAtrapado = true;
      break;
    }
  }

  // Si conectaste mal el receptor y no hubo eco, la latencia es 0
  if (!ecoAtrapado) ultimaLatenciaReal = 0;

  // Calculamos el delay para mantener el ritmo de 2 segundos de tu maestra
  long tiempoTranscurrido = millis() - tiempoSalida;
  if (tiempoTranscurrido < 2000) {
    delay(2000 - tiempoTranscurrido);
  }
}
