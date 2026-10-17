// =====================================================================
//  Wokwi: бортовой регистратор энергии — "цифровой двойник" рейса
//  Два поезда едут по одному графику: алюминий 145 т и сталь 180 т.
//  Регистратор считает кВт·ч/км для каждого и экономию алюминия в %.
//
//  Ручки:
//    POT1 (GPIO34) — расстояние между остановками, 1..100 км
//    POT2 (GPIO35) — рекуперация (возврат энергии при торможении), 0..60 %
//  Кнопки:
//    STOP  (GPIO13) — внеплановая остановка (красный сигнал)
//    RESET (GPIO14) — сбросить счётчики (жать после поворота ручки)
//  Светодиоды: зелёный — тяга, красный — торможение.
//  Serial: JSON-телеметрия раз в секунду (так данные ушли бы на сервер по MQTT).
//
//  Время ускорено в 20 раз. Физика — упрощённая модель, а не замер.
//  В реальном устройстве вместо модели стоят датчики тока и напряжения.
// =====================================================================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int POT_SPACING = 34, POT_REGEN = 35, BTN_STOP = 13, BTN_RESET = 14;
const int LED_TR = 26, LED_BR = 27;

const float A_MAX = 0.8, B_DEC = 0.8, V_MAX = 140.0 / 3.6;
const float ROT = 1.08, CDA = 10.0, RHO = 1.2, ROLL = 12.0, ROLL_V = 0.1, ETA_TR = 0.85;
const float M_ALU = 145.0, M_STEEL = 180.0;
const float DT = 1.0;           // секунд модели за шаг
const int   STEP_MS = 50;       // 1 с модели = 50 мс реального времени (x20)
const float DWELL_S = 60.0;     // стоянка на станции, с

float v = 0, x = 0, segLen = 10000, dwell = 0;
bool braking = false, forceStop = false;
double eAlu = 0, eSteel = 0, dist = 0;
int stops = 0;
float spacingKm = 10, regen = 0.3;
unsigned long lastStep = 0, lastShow = 0, lastJson = 0;

float resistance(float mt, float vv) {
  return mt * (ROLL + ROLL_V * vv * 3.6) + 0.5 * RHO * CDA * vv * vv;
}

void addEnergy(double &e, float mt, float a, float vv) {
  float m = mt * 1000.0 * ROT;
  float F = m * a + resistance(mt, vv);
  if (F > 0) e += F * vv * DT / ETA_TR;   // тяга: берём из сети
  else       e += F * vv * DT * regen;    // торможение: часть возвращаем
}

void resetCounters() {
  eAlu = eSteel = dist = 0;
  stops = 0;
}

void readInputs() {
  spacingKm = 1.0 + analogRead(POT_SPACING) / 4095.0 * 99.0;
  regen = analogRead(POT_REGEN) / 4095.0 * 0.6;
  if (digitalRead(BTN_STOP) == LOW) forceStop = true;
  if (digitalRead(BTN_RESET) == LOW) resetCounters();
}

void stepModel() {
  if (dwell > 0) {                         // стоим на станции
    dwell -= DT;
    digitalWrite(LED_TR, LOW); digitalWrite(LED_BR, LOW);
    if (dwell <= 0) { x = 0; segLen = spacingKm * 1000.0; }
    return;
  }
  float brakeDist = v * v / (2 * B_DEC);
  if (!braking && (segLen - x <= brakeDist || forceStop)) braking = true;

  float a;
  if (braking) {
    a = -B_DEC;
    if (v + a * DT <= 0) a = -v / DT;
  } else if (v < V_MAX) {
    a = min(A_MAX, (V_MAX - v) / DT);
  } else {
    a = 0;
  }

  addEnergy(eAlu, M_ALU, a, v);
  addEnergy(eSteel, M_STEEL, a, v);

  digitalWrite(LED_TR, (!braking && a > 0) ? HIGH : LOW);
  digitalWrite(LED_BR, braking ? HIGH : LOW);

  v += a * DT;
  if (v < 0.01 && braking) {               // остановились
    v = 0; braking = false; forceStop = false;
    dwell = DWELL_S; stops++;
  }
  x += v * DT;
  dist += v * DT;
}

void show() {
  char buf[21];
  double km = dist / 1000.0;
  double kA = km > 0.1 ? eAlu / 3.6e6 / km : 0;
  double kS = km > 0.1 ? eSteel / 3.6e6 / km : 0;
  double save = kS > 0 ? 100.0 * (kS - kA) / kS : 0;

  snprintf(buf, sizeof(buf), "Stops/%3dkm Reg%2d%% ", (int)spacingKm, (int)(regen * 100));
  lcd.setCursor(0, 0); lcd.print(buf);
  snprintf(buf, sizeof(buf), "V%3dkm/h St%3d %4dk", (int)(v * 3.6), stops, (int)km);
  lcd.setCursor(0, 1); lcd.print(buf);
  snprintf(buf, sizeof(buf), "Alu145t %5.2fkWh/km", kA);
  lcd.setCursor(0, 2); lcd.print(buf);
  snprintf(buf, sizeof(buf), "St180t %5.2f -%4.1f%%", kS, save);
  lcd.setCursor(0, 3); lcd.print(buf);
}

void telemetry() {
  double km = dist / 1000.0;
  Serial.printf("{\"km\":%.1f,\"v_kmh\":%.0f,\"stops\":%d,\"spacing_km\":%.0f,"
                "\"regen\":%.2f,\"kwh_alu\":%.1f,\"kwh_steel\":%.1f}\n",
                km, v * 3.6, stops, spacingKm, regen, eAlu / 3.6e6, eSteel / 3.6e6);
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(LED_TR, OUTPUT);
  pinMode(LED_BR, OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Energy recorder");
  lcd.setCursor(0, 1); lcd.print("Alu vs Steel");
  delay(1000);
  lcd.clear();
  readInputs();
  segLen = spacingKm * 1000.0;
}

void loop() {
  unsigned long now = millis();
  if (now - lastStep >= STEP_MS) { lastStep = now; readInputs(); stepModel(); }
  if (now - lastShow >= 300)     { lastShow = now; show(); }
  if (now - lastJson >= 1000)    { lastJson = now; telemetry(); }
}
