/*
  ============================================================
  NOVA-RECON — Sonda rover de detección de gases
  ============================================================
  Control:    iPad (acelerómetro, 2 ejes) vía Web Bluetooth
              -> HM-10 (BLE) -> SoftwareSerial
              Eje X del iPad (adelante/atrás) -> avance/reversa
              Eje Y del iPad (lado a lado)    -> giro izq/der
  Sensores:   HC-SR04  (ultrasónico, distancia)
              MQ-2     (humo / gas combustible general)
              MQ-7     (monóxido de carbono, CO)
              MQ-6     (butano / GLP, más selectivo que el MQ-2)
              MQ-3     (vapor de alcohol/etanol — informativo, no es
                        un gas de riesgo típico en este contexto)
              MH-Z19   (CO2 real en ppm, sensor NDIR por infrarrojo)
  Actuadores: 2 motores DC vía L293D (tracción diferencial)
              + bocina (claxon) + LED de alarma de gas

  ------------------------------------------------------------
  MAPA DE PINES (Arduino UNO / Nano)
  ------------------------------------------------------------
  HM-10 (BLE) — SoftwareSerial
    HM-10 TX  -> Arduino pin 10 (RX)
    HM-10 RX  <- Arduino pin 11 (TX)
    *** HM-10 usa lógica de 3.3V. Poner divisor de voltaje en la
        línea Arduino-TX -> HM-10-RX:
        Arduino pin11 --[R 1k]--+--[R 2k]-- GND
                                 |
                            HM-10 RX

  L293D — Motor izquierdo          L293D — Motor derecho
    ENA (PWM) -> pin 5               ENB (PWM) -> pin 6
    IN1       -> pin 4               IN3       -> pin 7
    IN2       -> pin 2               IN4       -> pin 8

  HC-SR04 (ultrasónico)
    TRIG -> pin 12
    ECHO -> pin 13

  MH-Z19 (CO2, salida PWM — NO usamos su UART a propósito)
    PWM  -> pin 9
    *** Usamos la salida PWM en vez de UART para no tener que
        correr un SEGUNDO puerto serie por software compitiendo
        con el del HM-10 (poco confiable en un Uno/Nano). La
        medición se hace sondeando el pin en cada vuelta de
        loop() sin bloquear (ver pollCO2Pwm()) — un pulseIn()
        normal se quedaría bloqueado hasta ~1s por ciclo, lo cual
        rompería el control del robot y dispararía el failsafe
        de conexión por falsos positivos.
    *** El MH-Z19 necesita unos 3 minutos de precalentamiento
        antes de dar lecturas confiables. Antes de eso, la app
        va a mostrar "calentando sensor".

  MQ-2 (humo / combustible)   MQ-7 (CO)          MQ-6 (butano/GLP)
    AOUT -> A0                  AOUT -> A2          AOUT -> A3

  MQ-3 (vapor de alcohol)
    AOUT -> A4
    *** El MQ-3 no alimenta el LED físico de alarma ni el failsafe —
        detectar alcohol no es en sí un peligro tipo fuga/incendio en
        este contexto, así que lo tratamos como dato informativo en
        la app, no como disparador de alarma de hardware. Si tu caso
        de uso sí lo amerita (ej. derrame de etanol, riesgo de
        inflamabilidad), puedes sumarlo a la lógica de anyAlarm más
        abajo.

    *** Los MQ-2/3/6/7 necesitan calentamiento (~1-2 min) antes de
        dar lecturas estables, y son sensores RELATIVOS (se
        calibra un baseline en aire limpio al encender) — no dan
        ppm certificados, a diferencia del MH-Z19.
    *** El MQ-7 en particular necesitaría un ciclo de calentamiento
        alterno (5V / 1.4V) para ppm de CO precisos según su
        datasheet; aquí va a 5V fijo como la mayoría de módulos
        hobby, así que su lectura es una tendencia/alarma relativa,
        no un valor certificado.

  Bocina (buzzer)      -> pin 3
  LED alarma de gas    -> A1  (se enciende si CUALQUIER sensor alarma)

  *** Nota de alimentación: con 3 sensores MQ (heaters) + el
      MH-Z19 + 2 motores, el consumo total sube bastante frente a
      la versión anterior. Alimenta la lógica/sensores desde una
      fuente de 5V estable separada de la batería de los motores,
      para evitar caídas de voltaje que reinicien el Arduino o den
      lecturas de gas erráticas.

  A4, A5 quedan libres para expansión futura.
  ------------------------------------------------------------
  PROTOCOLO (texto plano, terminado en '\n')
  ------------------------------------------------------------
  iPad -> Arduino:  C,<steer>,<throttle>,<horn>
                     steer:    -100..100  (negativo = izquierda)
                     throttle: -100..100  (negativo = reversa)
                     horn:     0 o 1

  Arduino -> iPad:  T,<dist_cm>,<g1_raw>,<g1_pct>,<g1_alarm>,
                       <g2_raw>,<g2_pct>,<g2_alarm>,
                       <g3_raw>,<g3_pct>,<g3_alarm>,
                       <g4_raw>,<g4_pct>,<g4_alarm>,
                       <co2_ppm>,<co2_alarm>
                     dist_cm:  distancia en cm, -1 si sin eco
                     g1_*:     MQ-2  (humo / combustible)
                     g2_*:     MQ-7  (monóxido de carbono)
                     g3_*:     MQ-6  (butano / GLP)
                     g4_*:     MQ-3  (vapor de alcohol, informativo)
                     *_pct:    % relativo al baseline calibrado al encender
                     co2_ppm:  ppm real de CO2, -1 si el sensor aún
                               no completó su primer ciclo (calentando)
                     *_alarm:  0 o 1
  ============================================================
*/

#include <SoftwareSerial.h>

SoftwareSerial bleSerial(10, 11); // RX, TX

// ---------- Pines motores ----------
const uint8_t ENA = 5, IN1 = 4, IN2 = 2;   // motor izquierdo
const uint8_t ENB = 6, IN3 = 7, IN4 = 8;   // motor derecho

// ---------- Pines sensores / actuadores ----------
const uint8_t TRIG_PIN = 12, ECHO_PIN = 13;
const uint8_t GAS1_PIN = A0;  // MQ-2  (humo / combustible)
const uint8_t GAS2_PIN = A2;  // MQ-7  (CO)
const uint8_t GAS3_PIN = A3;  // MQ-6  (butano / GLP)
const uint8_t GAS4_PIN = A4;  // MQ-3  (vapor de alcohol)
const uint8_t CO2_PWM_PIN = 9; // MH-Z19 (CO2, PWM)
const uint8_t HORN_PIN = 3;
const uint8_t ALARM_LED_PIN = A1;

// ---------- Parámetros ajustables ----------
const unsigned long CMD_TIMEOUT_MS = 600;   // failsafe: sin comando -> frenar
const unsigned long TELEMETRY_MS   = 300;   // cada cuánto se envía telemetría
const unsigned long OBSTACLE_STOP_CM = 5;   // corte de emergencia por choque inminente

const int GAS1_BASELINE_DEFAULT = 250;
const int GAS1_ALARM_DELTA      = 250;
const int GAS2_BASELINE_DEFAULT = 250;
const int GAS2_ALARM_DELTA      = 200;
const int GAS3_BASELINE_DEFAULT = 250;
const int GAS3_ALARM_DELTA      = 250;
const int GAS4_BASELINE_DEFAULT = 250;
const int GAS4_ALARM_DELTA      = 250; // informativo, ajusta o ignora según tu caso de uso

// Umbral de alarma de CO2 en ppm. 5000 ppm = límite de exposición
// ocupacional de 8 horas (OSHA PEL). Ajusta según el criterio que
// quieras seguir para tu caso de uso.
const long CO2_ALARM_PPM = 5000;

// ---------- Estado de control ----------
int steer = 0;
int throttle = 0;
bool hornOn = false;
unsigned long lastCmdMillis = 0;
unsigned long lastTelemetryMillis = 0;
int gas1Baseline = GAS1_BASELINE_DEFAULT;
int gas2Baseline = GAS2_BASELINE_DEFAULT;
int gas3Baseline = GAS3_BASELINE_DEFAULT;
int gas4Baseline = GAS4_BASELINE_DEFAULT;
long lastDistanceCm = -1;
String rxBuffer = "";

// ---------- Estado del sondeo no bloqueante de CO2 (PWM) ----------
int co2PinLastState = LOW;
unsigned long co2RiseMicros = 0;
long co2Ppm = -1; // -1 = aún sin una lectura completa (calentando/sin señal)

void setup() {
  Serial.begin(9600);     // monitor serie USB, solo para depurar
  bleSerial.begin(9600);  // baud por defecto de fábrica del HM-10

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);
  pinMode(CO2_PWM_PIN, INPUT);

  stopMotors();

  // Calibración rápida de los 4 sensores MQ en aire limpio
  long sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
  for (int i = 0; i < 20; i++) {
    sum1 += analogRead(GAS1_PIN);
    sum2 += analogRead(GAS2_PIN);
    sum3 += analogRead(GAS3_PIN);
    sum4 += analogRead(GAS4_PIN);
    delay(20);
  }
  gas1Baseline = sum1 / 20;
  gas2Baseline = sum2 / 20;
  gas3Baseline = sum3 / 20;
  gas4Baseline = sum4 / 20;

  Serial.println(F("NOVA-RECON listo."));
  Serial.print(F("Baseline MQ-2: ")); Serial.println(gas1Baseline);
  Serial.print(F("Baseline MQ-7: ")); Serial.println(gas2Baseline);
  Serial.print(F("Baseline MQ-6: ")); Serial.println(gas3Baseline);
  Serial.print(F("Baseline MQ-3: ")); Serial.println(gas4Baseline);
  Serial.println(F("MH-Z19: precalentando (~3 min para lecturas confiables)."));
}

void loop() {
  readIncomingCommands();
  applyFailsafe();
  driveMotors();
  handleHorn();
  pollCO2Pwm();       // sondeo ligero, no bloqueante, en cada vuelta
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

// Si se pierde la conexión BLE, detener el robot.
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

  if (lastDistanceCm > 0 && lastDistanceCm < OBSTACLE_STOP_CM && throttle > 0) {
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

// ================= CO2 (MH-Z19, PWM no bloqueante) =================
// Decodifica el ciclo PWM del MH-Z19 sondeando el pin cada vuelta de
// loop() en vez de usar pulseIn() (que bloquearía hasta ~1s y rompería
// el control del robot). Formula del datasheet: ppm = 5000*(Th-2)/1000,
// con Th = duración del pulso alto en milisegundos.
void pollCO2Pwm() {
  int state = digitalRead(CO2_PWM_PIN);
  unsigned long now = micros();

  if (state == HIGH && co2PinLastState == LOW) {
    co2RiseMicros = now; // flanco de subida: empieza el pulso alto
  } else if (state == LOW && co2PinLastState == HIGH) {
    long thMs = (long)((now - co2RiseMicros) / 1000UL); // duración del pulso alto
    long ppm = 5000L * (thMs - 2) / 1000L;
    co2Ppm = (ppm < 0) ? 0 : ppm;
  }
  co2PinLastState = state;
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

  int g1raw = analogRead(GAS1_PIN);
  int g1delta = g1raw - gas1Baseline;
  int g1pct = constrain(map(g1delta, 0, GAS1_ALARM_DELTA * 2, 0, 100), 0, 100);
  bool g1alarm = g1delta > GAS1_ALARM_DELTA;

  int g2raw = analogRead(GAS2_PIN);
  int g2delta = g2raw - gas2Baseline;
  int g2pct = constrain(map(g2delta, 0, GAS2_ALARM_DELTA * 2, 0, 100), 0, 100);
  bool g2alarm = g2delta > GAS2_ALARM_DELTA;

  int g3raw = analogRead(GAS3_PIN);
  int g3delta = g3raw - gas3Baseline;
  int g3pct = constrain(map(g3delta, 0, GAS3_ALARM_DELTA * 2, 0, 100), 0, 100);
  bool g3alarm = g3delta > GAS3_ALARM_DELTA;

  int g4raw = analogRead(GAS4_PIN);
  int g4delta = g4raw - gas4Baseline;
  int g4pct = constrain(map(g4delta, 0, GAS4_ALARM_DELTA * 2, 0, 100), 0, 100);
  bool g4alarm = g4delta > GAS4_ALARM_DELTA;

  bool co2alarm = co2Ppm >= 0 && co2Ppm > CO2_ALARM_PPM;

  // El MQ-3 (alcohol) es informativo: no enciende el LED físico de alarma.
  digitalWrite(ALARM_LED_PIN, (g1alarm || g2alarm || g3alarm || co2alarm) ? HIGH : LOW);

  bleSerial.print("T,");
  bleSerial.print(lastDistanceCm); bleSerial.print(",");
  bleSerial.print(g1raw);          bleSerial.print(",");
  bleSerial.print(g1pct);          bleSerial.print(",");
  bleSerial.print(g1alarm ? 1 : 0);bleSerial.print(",");
  bleSerial.print(g2raw);          bleSerial.print(",");
  bleSerial.print(g2pct);          bleSerial.print(",");
  bleSerial.print(g2alarm ? 1 : 0);bleSerial.print(",");
  bleSerial.print(g3raw);          bleSerial.print(",");
  bleSerial.print(g3pct);          bleSerial.print(",");
  bleSerial.print(g3alarm ? 1 : 0);bleSerial.print(",");
  bleSerial.print(g4raw);          bleSerial.print(",");
  bleSerial.print(g4pct);          bleSerial.print(",");
  bleSerial.print(g4alarm ? 1 : 0);bleSerial.print(",");
  bleSerial.print(co2Ppm);         bleSerial.print(",");
  bleSerial.println(co2alarm ? 1 : 0);

  // Espejo por USB para depurar con el Monitor Serie
  Serial.print("dist="); Serial.print(lastDistanceCm);
  Serial.print(" mq2="); Serial.print(g1pct);
  Serial.print("% mq7="); Serial.print(g2pct);
  Serial.print("% mq6="); Serial.print(g3pct);
  Serial.print("% mq3="); Serial.print(g4pct);
  Serial.print("% co2="); Serial.print(co2Ppm);
  Serial.println("ppm");
}
