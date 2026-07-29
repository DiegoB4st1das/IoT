#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── Credenciales WiFi y MQTT (TUS DATOS) ────────────────────────────────────
const char* WIFI_SSID     = "Diego B";
const char* WIFI_PASSWORD = "german123";
const char* MQTT_BROKER   = "10.35.146.178";
const int MQTT_PORT       = 1883;

// Topics oficiales
const char* TOPIC_TELEMETRIA = "iot/motor/telemetria";
const char* TOPIC_SETPOINT   = "iot/motor/setpoint";
const char* TOPIC_DIRECCION  = "iot/motor/direccion";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
bool mqttListo = false;

// ── Pines del Puente H y Encoder ─────────────────────────────────────────────
#define AIN1 26 
#define AIN2 13 
#define ENCA 27 
#define ENCB 4  

// ── PINES DE LOS BOTONES FÍSICOS ────────────────────────────────────────────
#define BTN_PARO   32
#define BTN_INICIO 33
#define BTN_GIRO   25

// Variable de intención de giro (lo que queremos que haga el puente H)
bool sentidoHorario = true;

// NUEVA VARIABLE: Lo que el encoder REALMENTE está midiendo (1 = CW, 0 = CCW)
int direccion_real_encoder = 1; 

// Variables para el antirrebote (Debounce) sin usar delay()
uint32_t tDebounceParo   = 0;
uint32_t tDebounceInicio = 0;
uint32_t tDebounceGiro   = 0;
const int TIEMPO_DEBOUNCE = 250; // ms para evitar dobles clics

// ── Ganancias del PID y Variables ───────────────────────────────────────────
float Kp = 3.0;
float Ki = 45.0;
float Kd = 0.0;

float setpoint_RPM = 90.0; 
float integral = 0;
float error_previo = 0;
float pwmSalida = 0;
float rpm_actual_global = 0;
float error_porcentaje_global = 0;

// ── Temporizadores oficiales ────────────────────────────────────────────────
#define T_PID_MS 10     // Loop del PID cada 10 ms (100 Hz)
#define T_MQTT_MS 500   // Publicar a MQTT cada 500 ms

uint32_t tPID = 0;
uint32_t tMQTT = 0;

// ── Variables del Motor ─────────────────────────────────────────────────────
const float PPM = 11.0;         
const float GEAR_RATIO = 34.0;  
const float PPR = PPM * GEAR_RATIO; 

volatile long pulsos = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ── Interrupción protegida ──────────────────────────────────────────────────
void IRAM_ATTR contarPulsos() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(ENCB) > 0) pulsos++;
  else pulsos--;
  portEXIT_CRITICAL_ISR(&mux);
}

// ── Leer RPM protegido ──────────────────────────────────────────────────────
float calcularRPM(float dt_s) {
  portENTER_CRITICAL(&mux);
  long pulsos_actuales = pulsos;
  pulsos = 0;
  portEXIT_CRITICAL(&mux);
  
  return (pulsos_actuales / PPR) * (60.0 / dt_s);
}

// ── Mover Motor (Con Inversión de Giro) ─────────────────────────────────────
void aplicarPWM(float valor) {
  int pwm_int = (int)valor;
  if (pwm_int > 255) pwm_int = 255;
  if (pwm_int < 0) pwm_int = 0;

  // Zona muerta para superar fricción estática
  if (pwm_int > 0 && pwm_int < 60) pwm_int = 60;

  if (pwm_int > 0) {
    if (sentidoHorario) {
      analogWrite(AIN1, pwm_int);
      digitalWrite(AIN2, LOW);
    } else {
      digitalWrite(AIN1, LOW);
      analogWrite(AIN2, pwm_int);
    }
  } else {
    analogWrite(AIN1, 0); 
    analogWrite(AIN2, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  
  pwmSalida = pwm_int;
}

// ── Conexión WiFi ───────────────────────────────────────────────────────────
void conectarWiFi() {
  Serial.printf("Conectando a WiFi %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int n = 0;
  while (WiFi.status() != WL_CONNECTED && n++ < 40) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n OK IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n FALLO WiFi - continuando sin MQTT");
  }
}

// ── Callback MQTT: Recibir Setpoint y Dirección ─────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  
  if (String(topic) == TOPIC_SETPOINT) {
    float sp = msg.toFloat();
    if (sp >= 0 && sp <= 700) {
      setpoint_RPM = sp;
      integral = 0; 
      Serial.printf("< Nuevo setpoint desde Web: %.1f RPM\n", setpoint_RPM);
    }
  } else if (String(topic) == TOPIC_DIRECCION) {
    sentidoHorario = !sentidoHorario;
    integral = 0; // Reset integral para transición suave
    Serial.printf("< Cambio de giro desde Web: %s\n", sentidoHorario ? "Horario" : "Antihorario");
  }
}

// ── Conexión MQTT ───────────────────────────────────────────────────────────
void conectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  int n = 0;
  while (!mqttClient.connected() && n++ < 5) {
    Serial.print("Conectando MQTT...");
    if (mqttClient.connect("ESP32-Motor-PID")) {
      mqttClient.subscribe(TOPIC_SETPOINT);
      mqttClient.subscribe(TOPIC_DIRECCION);
      mqttListo = true;
      Serial.println(" OK");
    } else {
      Serial.printf(" fallo (rc=%d)\n", mqttClient.state());
      delay(1000);
    }
  }
}

// ── Publicar Telemetría Oficial ─────────────────────────────────────────────
void publicarTelemetria() {
  if (!mqttListo || !mqttClient.connected()) return;
  
  StaticJsonDocument<200> doc;
  
  doc["rpm"] = round(rpm_actual_global * 10.0f) / 10.0f; 
  doc["setpoint"] = setpoint_RPM;
  doc["error"] = round((setpoint_RPM - rpm_actual_global) * 10.0f) / 10.0f;
  doc["pwm"] = round(pwmSalida);
  doc["nodo"] = "ESP32-Motor";
  doc["err_pct"] = round(error_porcentaje_global * 10.0f) / 10.0f;
  
  // MANDAMOS LA LECTURA REAL DEL ENCODER (1 = CW, 0 = CCW)
  doc["dir"] = direccion_real_encoder;

  char buf[200];
  serializeJson(doc, buf);
  mqttClient.publish(TOPIC_TELEMETRIA, buf);
  
  Serial.printf("-> rpm:%.1f sp:%.1f err:%.1f pwm:%.0f dir:%s\n", 
                rpm_actual_global, setpoint_RPM, setpoint_RPM - rpm_actual_global, 
                pwmSalida, direccion_real_encoder == 1 ? "CW" : "CCW");
}

// ── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  
  // Botones físicos con Pull-up
  pinMode(BTN_PARO, INPUT_PULLUP);
  pinMode(BTN_INICIO, INPUT_PULLUP);
  pinMode(BTN_GIRO, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA), contarPulsos, RISING);

  aplicarPWM(0);

  conectarWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    conectarMQTT();
  }

  tPID = millis();
  tMQTT = millis();

  Serial.println("==================================================");
  Serial.println("Sistema listo - PID + MQTT Oficial + Inversión de Giro");
  Serial.printf("Setpoint inicial: %.1f RPM\n", setpoint_RPM);
  Serial.println("==================================================");
}

// ── LOOP PRINCIPAL ──────────────────────────────────────────────────────────
void loop() {
  if (mqttListo) mqttClient.loop();
  
  uint32_t ahora = millis();

  // 0. LECTURA DE BOTONES FÍSICOS (Antirrebote sin retardar PID)
  if (digitalRead(BTN_PARO) == LOW && (ahora - tDebounceParo) > TIEMPO_DEBOUNCE) {
      setpoint_RPM = 0.0;
      integral = 0;
      Serial.println("¡PARO DE EMERGENCIA FÍSICO ACTIVADO!");
      tDebounceParo = ahora;
  }
  
  if (digitalRead(BTN_INICIO) == LOW && (ahora - tDebounceInicio) > TIEMPO_DEBOUNCE) {
      setpoint_RPM = 90.0;
      integral = 0;
      Serial.println("¡INICIO FÍSICO ACTIVADO (90 RPM)!");
      tDebounceInicio = ahora;
  }

  if (digitalRead(BTN_GIRO) == LOW && (ahora - tDebounceGiro) > TIEMPO_DEBOUNCE) {
      sentidoHorario = !sentidoHorario;
      integral = 0;
      Serial.printf("¡CAMBIO DE GIRO FÍSICO ACTIVADO! Sentido objetivo: %s\n", sentidoHorario ? "Horario" : "Antihorario");
      tDebounceGiro = ahora;
  }

  // 1. LAZO DEL PID (Cada 10ms)
  if ((ahora - tPID) >= T_PID_MS) {
    float dt_s = (ahora - tPID) / 1000.0f;
    
    // Leemos las RPM CRUDAS con todo y su signo (positivo o negativo)
    float rpm_crudo = calcularRPM(dt_s);
    
    // Actualizamos la dirección real basada en la lectura del encoder
    if (rpm_crudo > 0.5) {
        direccion_real_encoder = 0; // Girando a la derecha
    } else if (rpm_crudo < -0.5) {
        direccion_real_encoder = 1; // Girando a la izquierda
    }
    
    // Le quitamos el signo para que el PID trabaje con pura magnitud absoluta
    rpm_actual_global = fabs(rpm_crudo);
    float error = setpoint_RPM - rpm_actual_global;
    
    if (setpoint_RPM != 0) {
        error_porcentaje_global = (error / setpoint_RPM) * 100.0;
    } else {
        error_porcentaje_global = 0.0;
    }

    float P = Kp * error;
    
    integral += (error * dt_s); 
    if (integral > (255.0 / Ki)) integral = (255.0 / Ki);
    if (integral < 0) integral = 0; 
    
    float I = Ki * integral;
    float D = Kd * ((error - error_previo) / dt_s);
    
    float u = P + I + D;
    aplicarPWM(u);

    error_previo = error;
    tPID = ahora;
  }

  // 2. LAZO MQTT (Cada 500ms)
  if ((ahora - tMQTT) >= T_MQTT_MS) {
    publicarTelemetria();
    tMQTT = ahora;
    
    if (mqttListo && !mqttClient.connected()) {
      conectarMQTT();
    }
  }
}
