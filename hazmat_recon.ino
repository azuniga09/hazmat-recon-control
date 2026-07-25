/*
  ============================================================
  HAZMAT-RECON — Robot explorador de gases
  ============================================================
  Control:    iPad (volante por inclinación) vía Web Bluetooth
              -> HM-10 (BLE) -> SoftwareSerial
  Sensores:   HC-SR04 (ultrasónico) + MQ-2 (humo / gas combustible)
  Actuadores: 2 motores DC vía L293D (tracción diferencial tipo tanque)
              + bocina (claxon) + LED de alarma de gas

  ------------------------------------------------------------
  MAPA DE PINES (Arduino UNO / Nano)
  ------------------------------------------------------------
  HM-10 (BLE) — SoftwareSerial
    HM-10 TX  -> Arduino pin 10 (RX)
    HM-10 RX  <- Arduino pin 11 (TX)
    *** IMPORTANTE: HM-10 usa lógica de 3.3V. Poner un divisor
        de voltaje en la línea Arduino-TX -> HM-10-RX:
        Arduino pin11 --[R 1k]--+--[R 2k]-- GND
                                 |
                            HM-10 RX
        (la línea HM-10 TX -> Arduino RX sí se puede conectar
        directo, la mayoría de módulos toleran 5V en esa entrada,
        pero si tu módulo es estricto usa divisor en ambas líneas)

  L293D — Motor izquierdo
    ENA (PWM) -> pin 5
    IN1       -> pin 4
    IN2       -> pin 2

  L293D — Motor derecho
    ENB (PWM) -> pin 6
    IN3       -> pin 7
    IN4       -> pin 8

  HC-SR04 (ultrasónico)
    TRIG -> pin 12
    ECHO -> pin 13

  MQ-2 (humo / gas combustible)
    AOUT -> A0
    *** El MQ-2 necesita 1-2 minutos de calentamiento para dar
        lecturas estables. Enciende el robot y espera antes de
        confiar en la alarma de gas.
    *** El MQ-2 detecta bien humo, GLP, metano y gases
        combustibles. Si necesitas detectar CO específicamente
        usa un MQ-7 adicional, y para calidad de aire general
        un MQ-135, cada uno en su propio pin analógico (A1, A2...).

  Bocina (buzzer activo o pasivo)
    -> pin 3

  LED indicador de alarma de gas (opcional, quítalo si no lo usas)
    -> A1

  Pin 9 queda libre para expansión (servo de mástil de sensor, etc).
  ------------------------------------------------------------
  PROTOCOLO (texto plano, terminado en '\n')
  ------------------------------------------------------------
  iPad -> Arduino:  C,<steer>,<throttle>,<horn>
                     steer:    -100..100  (negativo = izquierda)
                     throttle: -100..100  (negativo = reversa)
                     horn:     0 o 1

  Arduino -> iPad:  T,<dist_cm>,<gas_raw>,<gas_pct>,<alarma>
                     dist_cm:  distancia en cm, -1 si sin eco
                     gas_raw:  lectura analógica cruda (0-1023)
                     gas_pct:  % relativo al baseline calibrado
                     alarma:   0 o 1
  ============================================================
*/

#include <SoftwareSerial.h>

SoftwareSerial bleSerial(10, 11); // RX, TX

// ---------- Pines motores ----------
const uint8_t ENA = 5, IN1 = 4, IN2 = 2;   // motor izquierdo
const uint8_t ENB = 6, IN3 = 7, IN4 = 8;   // motor derecho

// ---------- Pines sensores / actuadores ----------
const uint8_t TRIG_PIN = 12, ECHO_PIN = 13;
const uint8_t GAS_PIN  = A0;
const uint8_t HORN_PIN = 3;
const uint8_t ALARM_LED_PIN = A1;

// ---------- Parámetros ajustables ----------
const unsigned long CMD_TIMEOUT_MS = 600;   // failsafe: sin comando -> frenar
const unsigned long TELEMETRY_MS   = 300;   // cada cuánto se envía telemetría
const unsigned long OBSTACLE_STOP_CM = 5;   // corte de emergencia por choque inminente
const int GAS_BASELINE_DEFAULT = 250;       // referencia en aire limpio (se recalibra al encender)
const int GAS_ALARM_DELTA      = 250;       // cuánto debe subir sobre el baseline para alarmar

// ---------- Estado de control ----------
int steer = 0;
int throttle = 0;
bool hornOn = false;
unsigned long lastCmdMillis = 0;
unsigned long lastTelemetryMillis = 0;
int gasBaseline = GAS_BASELINE_DEFAULT;
long lastDistanceCm = -1;
String rxBuffer = "";

void setup() {
  Serial.begin(9600);     // monitor serie USB, solo para depurar
  bleSerial.begin(9600);  // baud por defecto de fábrica del HM-10

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);

  stopMotors();

  // Calibración rápida del MQ-2 en aire limpio (promedio de 20 lecturas)
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(GAS_PIN); delay(20); }
  gasBaseline = sum / 20;

  Serial.println(F("HAZMAT-RECON listo."));
  Serial.print(F("Baseline gas: ")); Serial.println(gasBaseline);
}

void loop() {
  readIncomingCommands();
  applyFailsafe();
  driveMotors();
  handleHorn();
  sendTelemetryIfDue();
}

// ================= Comunicación BLE =================
void readIncomingCommands() {
  while (bleSerial.available()) {
    char c = bleSerial.read();
    if (c == '\n') {
      parseCommand(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 40) rxBuffer = ""; // protección contra basura/ruido
    }
  }
}

// Formato: C,<steer>,<throttle>,<horn>
void parseCommand(String line) {
  if (line.length() == 0 || line.charAt(0) != 'C') return;

  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  int p3 = line.indexOf(',', p2 + 1);
  if (p1 < 0 || p2 < 0 || p3 < 0) return;

  steer    = constrain(line.substring(p1 + 1, p2).toInt(), -100, 100);
  throttle = constrain(line.substring(p2 + 1, p3).toInt(), -100, 100);
  hornOn   = line.substring(p3 + 1).toInt() == 1;

  lastCmdMillis = millis();
}

// Si se pierde la conexión BLE, detener el robot (importante en zonas
// de difícil acceso: un robot "a la deriva" sin control es un riesgo).
void applyFailsafe() {
  if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    steer = 0;
    throttle = 0;
    hornOn = false;
  }
}

// ================= Motores (tracción diferencial) =================
void driveMotors() {
  int left  = constrain(throttle + steer, -200, 200);
  int right = constrain(throttle - steer, -200, 200);

  left  = map(left,  -200, 200, -255, 255);
  right = map(right, -200, 200, -255, 255);

  // Corte de emergencia si hay un obstáculo pegado al frente y se
  // intenta seguir avanzando (no bloquea reversa ni giros en el sitio).
  long dist = lastDistanceCm;
  if (dist > 0 && dist < OBSTACLE_STOP_CM && throttle > 0) {
    left = 0;
    right = 0;
  }

  setMotor(ENA, IN1, IN2, left);
  setMotor(ENB, IN3, IN4, right);
}

void setMotor(uint8_t enPin, uint8_t in1, uint8_t in2, int speedVal) {
  bool fwd = speedVal >= 0;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  analogWrite(enPin, constrain(abs(speedVal), 0, 255));
}

void stopMotors() {
  setMotor(ENA, IN1, IN2, 0);
  setMotor(ENB, IN3, IN4, 0);
}

// ================= Bocina =================
void handleHorn() {
  if (hornOn) tone(HORN_PIN, 1200);
  else noTone(HORN_PIN);
}

// ================= Sensores + telemetría =================
long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // timeout ~25ms (~4m)
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}

void sendTelemetryIfDue() {
  if (millis() - lastTelemetryMillis < TELEMETRY_MS) return;
  lastTelemetryMillis = millis();

  lastDistanceCm = readDistanceCm();
  int gasRaw = analogRead(GAS_PIN);
  int delta = gasRaw - gasBaseline;
  int gasPct = constrain(map(delta, 0, GAS_ALARM_DELTA * 2, 0, 100), 0, 100);
  bool alarm = delta > GAS_ALARM_DELTA;

  digitalWrite(ALARM_LED_PIN, alarm ? HIGH : LOW);

  bleSerial.print("T,");
  bleSerial.print(lastDistanceCm);
  bleSerial.print(",");
  bleSerial.print(gasRaw);
  bleSerial.print(",");
  bleSerial.print(gasPct);
  bleSerial.print(",");
  bleSerial.println(alarm ? 1 : 0);

  // Espejo por USB para depurar con el Monitor Serie
  Serial.print("dist="); Serial.print(lastDistanceCm);
  Serial.print(" gasRaw="); Serial.print(gasRaw);
  Serial.print(" gasPct="); Serial.print(gasPct);
  Serial.print(" alarm="); Serial.println(alarm);
}
