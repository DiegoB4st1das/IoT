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

void setupLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERROR: RA-02 no responde. Verifica conexiones y antena.");
    while (true);
  }
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.setSyncWord(0xF5);
  Serial.println("LoRa RX listo esperando paquetes...");
}

void procesarPaquete(int tamano) {
  // 1. PRIMERO LEEMOS EL PAQUETE (Antes de que el chip se borre)
  String jsonStr = "";
  for (int i = 0; i < tamano; i++) {
    jsonStr += (char)LoRa.read();
  }

  // 2. MANDAMOS EL ECO AL TRANSMISOR
  // (Le damos 20ms de respiro al TX para que alcance a cambiar a modo escucha)
  delay(20);
  LoRa.beginPacket();
  LoRa.print("ACK");
  LoRa.endPacket();

  // 3. AHORA SÍ PROCESAMOS LA INFORMACIÓN QUE GUARDAMOS
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    Serial.print("Error al parsear JSON: ");
    Serial.println(err.c_str());
    return;
  }

  float valor = doc["v"];
  int id = doc["id"];
  long lat = doc["lat"]; // Aquí extraemos la latencia real que nos mandó el TX

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // SE IMPRIME EXACTAMENTE IGUAL QUE EL ORIGINAL
  Serial.print("PKT "); Serial.print(id);
  Serial.print(" | Val: "); Serial.print(valor, 1);
  Serial.print(" | RSSI: "); Serial.print(rssi);
  Serial.print(" dBm | SNR: "); Serial.print(snr, 1);
  Serial.print(" dB | Lat: "); Serial.print(lat);
  Serial.println(" ms");
}

void setup() {
  Serial.begin(115200);
  setupLoRa();
}

void loop() {
  int tamano = LoRa.parsePacket();
  if (tamano > 0) {
    procesarPaquete(tamano);
  }
}

