#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "d9193b5a-0f97-4356-8bb1-67b0bd3b2e10"
#define CHAR_RPM_UUID       "5580e27f-0976-449a-bac7-2f82fddb0388"
#define CHAR_SETPOINT_UUID  "f1e221af-4201-4645-91b7-ee65c66e8eb9"

// Variables globales
BLEServer* pServer = nullptr;
BLECharacteristic* pRpmChar = nullptr;

bool deviceConnected = false;
float rpm = 0.0;
float setpoint = 0.0;

// Servidor Callbacks (Maneja conexiones y desconexiones)
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println(">>> Central conectado!");
  }

  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    // Es crítico reiniciar el advertising para que otros se puedan conectar al desconectarse el anterior
    BLEDevice::startAdvertising(); 
    Serial.println(">>> Central desconectado!");
  }
};

// Característica Callbacks (Se ejecuta cuando el teléfono escribe un Setpoint)
class SetpointCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c)  {
    String val = c->getValue();

    if (val.length() > 0) {
      setpoint = val.toFloat();
      Serial.printf("Setpoint : %.1f RPM\n", setpoint);
    }
  }
};

void setup() {
  Serial.begin(115200);

  // --- PASO 1: Inicializar el dispositivo BLE ---
  BLEDevice::init("Motor-DBS");

  // --- PASO 2: Crear el servidor y registrar callbacks ---
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // --- PASO 3: Crear el servicio ---
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // --- PASO 4: Crear característica RPM (Notify) ---
  pRpmChar = pService->createCharacteristic(
      CHAR_RPM_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  
  pRpmChar->addDescriptor(new BLE2902());

  // --- PASO 5: Crear característica Setpoint (Write) ---
  BLECharacteristic* pSpChar = pService->createCharacteristic(
      CHAR_SETPOINT_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  
  pSpChar->setCallbacks(new SetpointCallbacks());

  // Nota: el setpoint NO necesita BLE2902 porque no usa Notify

  // --- PASO 6: Iniciar servicio y advertising ---
  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);

  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE listo - esperando conexion...");
}

void loop() {
  if (deviceConnected) {
    rpm += 5.0;
    if (rpm > 600.0) rpm = 0.0;

    char buf[10];
    sprintf(buf, "%.1f", rpm);
    pRpmChar->setValue(buf);

    pRpmChar->notify();

    Serial.printf("RPM: %.1f | setpoint: %.1f\n", rpm, setpoint);
  } 
  delay(500);
}
