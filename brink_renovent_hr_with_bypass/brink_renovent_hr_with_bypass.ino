#include <Arduino.h>
#include <OpenTherm.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// this part shall be changed ----------------------------------
const char* wifiSsid = "Your WiFi SSID";
const char* wifiPassword = "Your WiFi password";
const char* mqttServer = "Your MQTT broker IP address";
const char* mqttClientId = "brink";  // Unique client id
const int mqttPort = 1883;
const char* mqttUser = "Your MQTT username";
const char* mqttPassword = "Your MQTT password";
float maxVent = 2.96;                            // it means 296 m/h3 - max available flow in my Brink - not used/needed
const unsigned long readPeriod = 1500;           // 1000 = every second; set between 1000 - 5000
const unsigned long readPeriod_bypass = 120000;  // Set +15000 - OT disconnection needed for bypass work
//-----------------------------------------------------------------------

unsigned long startTime;
unsigned long currentTime;
unsigned long readOT;

const int HWCPin = 14;  // Option: HW circulation pump D5
const int inPin = 4;    // ESP8266 D2
const int outPin = 5;   // ESP8266 D1

OpenTherm ot(inPin, outPin);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

int cVol, cVol_old = 0;          // No.1 Current position/outlet volume [m3/h]
bool bp, bp_old = 1;             // No.2 Bypass status
bool sem_bp;                     // semaphore for bypass workaround
float tP, tP_old = 0;            // No.3 Temperature from atmosphere [°C]
float tS, tS_old = 0;            // No.4 Temperature from indoors [°C]
int inVol, inVol_old = 0;        // No.6 Current input volume [m3/h]
int outVol, outVol_old = 0;      // No.7 Current output volume [m3/h]
int pressIn, pressIn_old = 0;    // No.8 Current pressure input duct [Pa]
int pressOut, pressOut_old = 0;  // No.9 Current pressure output duct [Pa]
int frost, frost_old = 0;        // No.10 Status frost protection
int u4, u4_old = 1;              // U4 Minimum atmospheric temperature bypass
int u5, u5_old = 1;              // U5 Minimum indoor temperature bypass
int i1, i1_old = 100;            // I1 Fixed imbalance
int fCode, fCode_old = 0;        // Fault code
bool fault, fault_old = 1;       // Fault indication?
bool vMode, vMode_old = 0;       // Ventilation mode
bool filter, filter_old = 1;     // Filter dirty?
long rssi, rssi_old = 0;         // Wifi signal strength [dB]

void ICACHE_RAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

void MqttReconnect() {
  while (!mqttClient.connected()) {
    delay(1000);

    if (mqttClient.connect(mqttClientId, mqttUser, mqttPassword)) {
      mqttClient.setBufferSize(512);
      Serial.println("MQTT> Connected");
      if (mqttClient.subscribe("brink/+/set")) {
        Serial.println("MQTT> Subscribed to brink/+/set");
      }
      MqttPublishAutoDiscovery();
    } else {
      Serial.print("MQTT> failed: ");
      Serial.print(mqttClient.state());
      Serial.println(", trying again ...");
      delay(5000);
    }
  }
}

void MqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = 0;

  // No.1. Current position/outlet volume [m3/h]
  if (strcmp(topic, "brink/cvol/set") == 0) {
    ot.setVentilation(atoi((char*)payload));  // uint8_t
    delay(100);
    refreshAll();
  }

  // State
  if (strcmp(topic, "brink/vmode/set") == 0) {
    if (strcmp((char*)payload, "1") == 0) ot.setVentilation(17);
    if (strcmp((char*)payload, "0") == 0) ot.setVentilation(0);
    delay(100);
    refreshAll();
  }

  // U4. Minimum Outside Temperature
  if (strcmp(topic, "brink/u4/set") == 0) {
    ot.setBrinkTSP(U4, atoi((char*)payload) * 2);
  }

  // U5. Minimum Indoor Temperature
  if (strcmp(topic, "brink/u5/set") == 0) {
    ot.setBrinkTSP(U5, atoi((char*)payload) * 2);
    delay(100);
    refreshAll();  // change of U5 triggers refresh of other parameters
  }
}

void MqttPublishAutoDiscovery() {
  StaticJsonDocument<400> entity;
  // never changing items
  entity["val_tpl"] = "{{ value_json }}";

  // Temperatures
  entity["dev_cla"] = "temperature";
  entity["unit_of_meas"] = "°C";

  // No.3. Temperature from atmosphere [°C]
  entity["uniq_id"] = "brink_supply_in_temperature";
  entity["name"] = "Temperature from atmosphere";
  entity["stat_t"] = "brink/tp/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_supply_in_temperature/config", entity);

  // No.4. Temperature from indoors [°C]
  entity["uniq_id"] = "brink_exhaust_in_temperature";
  entity["name"] = "Temperature from indoors";
  entity["stat_t"] = "brink/ts/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_exhaust_in_temperature/config", entity);

  // U4. Minimum Outside Temperature
  entity["uniq_id"] = "brink_minimum_atmospheric_temperature_bypass";
  entity["name"] = "Minimum atmospheric temperature bypass";
  entity["stat_t"] = "brink/u4/get";
  entity["cmd_t"] = "brink/u4/set";
  entity["min"] = 5;
  entity["max"] = 20;
  entity["step"] = 1;
  MqttPublishAutoDiscoveryMessage("homeassistant/number/brink_minimum_atmospheric_temperature_bypass/config", entity);

  // U5. Minimum Indoor Temperature
  entity["uniq_id"] = "brink_minimum_indoor_temperature_bypass";
  entity["name"] = "Minimum indoor temperature bypass";
  entity["stat_t"] = "brink/u5/get";
  entity["cmd_t"] = "brink/u5/set";
  entity["min"] = 18;
  entity["max"] = 30;
  entity["step"] = 1;
  MqttPublishAutoDiscoveryMessage("homeassistant/number/brink_minimum_indoor_temperature_bypass/config", entity);
  entity.remove("cmd_t");
  entity.remove("min");
  entity.remove("max");
  entity.remove("step");

  // Pressures
  entity["dev_cla"] = "pressure";
  entity["unit_of_meas"] = "Pa";

  // No.8. Current pressure input duct [Pa]
  entity["uniq_id"] = "brink_current_pressure_input_duct";
  entity["name"] = "Current pressure input duct";
  entity["stat_t"] = "brink/pressin/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_pressure_input_duct/config", entity);

  // No.9. Current pressure output duct [Pa]
  entity["uniq_id"] = "brink_current_pressure_output_duct";
  entity["name"] = "Current pressure output duct";
  entity["stat_t"] = "brink/pressout/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_pressure_output_duct/config", entity);

  // Fan
  entity.remove("dev_cla");

  // No.1. Current position/outlet volume [m3/h]
  entity.remove("val_tpl");
  entity["uniq_id"] = "brink_current_position_outlet_volume";
  entity["name"] = "Current position/outlet volume";
  entity["stat_t"] = "brink/vmode/get";
  entity["cmd_t"] = "brink/vmode/set";
  entity["pl_on"] = "1";
  entity["pl_off"] = "0";
  entity["pct_stat_t"] = "brink/cvol/get";
  entity["pct_cmd_t"] = "brink/cvol/set";
  MqttPublishAutoDiscoveryMessage("homeassistant/fan/brink_current_position_outlet_volume/config", entity);
  entity["val_tpl"] = "{{ value_json }}";
  entity.remove("cmd_t");
  entity.remove("pl_on");
  entity.remove("pl_off");
  entity.remove("pct_stat_t");
  entity.remove("pct_cmd_t");

  // Flows
  entity["unit_of_meas"] = "m³/h";
  entity["ic"] = "mdi:wind-power";

  // No.7. Current input volume [m3/h]
  entity["uniq_id"] = "brink_current_input_volume";
  entity["name"] = "Current input volume";
  entity["stat_t"] = "brink/invol/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_input_volume/config", entity);

  // No.8. Current output volume [m3/h]
  entity["uniq_id"] = "brink_current_output_volume";
  entity["name"] = "Current output volume";
  entity["stat_t"] = "brink/outvol/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_output_volume/config", entity);
  entity.remove("unit_of_meas");
  entity.remove("ic");

  // Faults
  entity["ic"] = "mdi:alert-circle";

  // 1.4.1 Fault Indication
  entity["dev_cla"] = "problem";
  entity["uniq_id"] = "brink_fault_indication";
  entity["name"] = "Fault Indication";
  entity["stat_t"] = "brink/fault/get";
  entity["val_tpl"] = "{% if value_json == '1' %}ON{% else %}OFF{% endif %}";
  MqttPublishAutoDiscoveryMessage("homeassistant/binary_sensor/brink_fault_indication/config", entity);
  entity["val_tpl"] = "{{ value_json }}";
  entity.remove("dev_cla");
  entity.remove("pl_on");
  entity.remove("pl_off");

  // Fault Code
  entity["uniq_id"] = "brink_fault_code";
  entity["name"] = "Fault Code";
  entity["stat_t"] = "brink/fcode/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_fault_code/config", entity);
  entity.remove("ic");

  // Strings
  // Ventilation Mode
  entity["uniq_id"] = "brink_ventilation_mode";
  entity["name"] = "Ventilation Mode";
  entity["stat_t"] = "brink/vmode/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_ventilation_mode/config", entity);

  // No.2. Bypass Status
  // 0 = bypass valve shut
  // 1 = bypass valve automatic
  // 2 = input at minimum
  entity["dev_cla"] = "enum";
  entity["ops"][0] = "bypass valve shut";
  entity["ops"][1] = "bypass valve automatic";
  entity["ops"][2] = "input at minimum";
  entity["uniq_id"] = "brink_bypass_status";
  entity["name"] = "Bypass Status";
  entity["stat_t"] = "brink/bp/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_bypass_status/config", entity);
  entity.remove("dev_cla");
  entity.remove("ops");

  // I1. Fixed imbalance
  entity["uniq_id"] = "brink_i1";
  entity["name"] = "Fixed imbalance";
  entity["stat_t"] = "brink/i1/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_i1/config", entity);

  // No.10 Status frost protection
  entity["uniq_id"] = "brink_frost_protection";
  entity["name"] = "Status frost protection";
  entity["stat_t"] = "brink/frost/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_frost_protection/config", entity);

  // Binary
  // Filter Dirty
  entity["dev_cla"] = "problem";
  entity["uniq_id"] = "brink_filter_dirty";
  entity["name"] = "Filter Dirty";
  entity["stat_t"] = "brink/filter/get";
  entity["val_tpl"] = "{% if value_json == '1' %}ON{% else %}OFF{% endif %}";
  MqttPublishAutoDiscoveryMessage("homeassistant/binary_sensor/brink_filter_dirty/config", entity);
  entity["val_tpl"] = "{{ value_json }}";
  entity.remove("payload_on");
  entity.remove("payload_off");

  // Signal Strengths
  // Wifi signal strength
  entity["dev_cla"] = "signal_strength";
  entity["unit_of_meas"] = "dB";
  entity["uniq_id"] = "brink_wifi_rsi";
  entity["name"] = "WiFi RSI";
  entity["stat_t"] = "brink/wifi/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_wifi_rssi/config", entity);
}

void MqttPublishAutoDiscoveryMessage(const char* topic, JsonDocument& payload) {
  payload["dev"]["ids"][0] = "brink_renovent_hr";
  payload["dev"]["mf"] = "Brink";
  payload["dev"]["mdl"] = "Renovent HR";
  payload["dev"]["name"] = "Brink Renovent HR";

  char buffer[512];
  size_t n = serializeJson(payload, buffer);
  mqttClient.publish(topic, buffer);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start of Brink Renovent HR with Bypass program");

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.print("WiFi> Connected to WiFi. Current IP Address is: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(MqttCallback);
  MqttReconnect();

  startTime = millis();
  ot.begin(handleInterrupt);

  ReadBrinkParameters();
  refreshAll();
  sem_bp = bp;
  if (bp == 1) readOT = readPeriod_bypass;
  else readOT = readPeriod;
}

// refresh slow changing parameters on request by every change of U5
void refreshAll() {
  // No.1 Current position/outlet volume [m3/h]
  mqttClient.publish("brink/cvol/get", String(int(floor(cVol / maxVent))).c_str());
  // No.2 Bypass status
  mqttClient.publish("brink/bp/get", String(bp).c_str());
  // No.3 Temperature from atmosphere [°C]
  mqttClient.publish("brink/tp/get", String(tP).c_str());
  // No.4 Temperature to atmosphere [°C]
  mqttClient.publish("brink/ts/get", String(tS).c_str());
  // No.6 Current input volume [m3/h]
  mqttClient.publish("brink/invol/get", String(inVol * maxVent).c_str());
  // No.7 Current output volume [m3/h]
  mqttClient.publish("brink/outvol/get", String(outVol * maxVent).c_str());
  // No.8 Current pressure input duct [Pa]
  mqttClient.publish("brink/pressin/get", String(pressIn).c_str());
  // No.9 Current pressure output duct [Pa]
  mqttClient.publish("brink/pressout/get", String(pressOut).c_str());
  // No.10 Status frost protection
  mqttClient.publish("brink/frost/get", String(frost).c_str());

  // U4 Minimum atmospheric temperature bypass
  mqttClient.publish("brink/u4/get", String(u4 / 2).c_str());
  // U5 Minimum indoor temperature bypass
  mqttClient.publish("brink/u5/get", String(u5 / 2).c_str());

  // I1 Fixed imbalance
  mqttClient.publish("brink/i1/get", String(i1).c_str());

  // Fault code
  mqttClient.publish("brink/fcode/get", String(fCode).c_str());
  // Fault indication?
  mqttClient.publish("brink/fault/get", String(fault).c_str());

  // Ventilation mode
  mqttClient.publish("brink/vmode/get", String(vMode).c_str());

  // Filter Dirty?
  mqttClient.publish("brink/filter/get", String(filter).c_str());

  // Wifi signal strength
  mqttClient.publish("brink/wifi/get", String(rssi).c_str());
}

void loop() {
  // if not connected to MQTT broker reconnect
  if (!mqttClient.connected()) {
    MqttReconnect();
  }
  mqttClient.loop();

  currentTime = millis();
  if (currentTime - startTime >= readOT) {
    ReadBrinkParameters();

    // Wifi signal strength
    rssi = WiFi.RSSI();
    if (abs(rssi - rssi_old) > 2) {
      mqttClient.publish("brink/wifi/get", String(rssi).c_str());
      rssi_old = rssi;
    }

    // Workaround for bypass change and keeping change when U4 and U5 conditions are met
    // Bypass is CLOSED
    if (sem_bp == 0) {
      if ((tS > u5 / 2) && (tP > u4 / 2) && (tP < tS)) {  // if true open bypass,
        delay(150000);                                    // stop 2,5 min
        readOT = readPeriod_bypass;
        sem_bp = 1;
      }
    }

    // Bypass is OPEN
    if (sem_bp == 1) {
      if ((tS < u5 / 2) || (tP < u4 / 2) || (tP > tS)) {  // if true close bypass
        delay(150000);                                    // stop 2,5 min
        readOT = readPeriod;
        sem_bp = 0;
      }
    }
    startTime = currentTime;
  }
}

// Reading all requested Brink HR parameters, MQTT publication only happens if a value has changed
void ReadBrinkParameters() {
  // No.1 Current position/outlet volume [m3/h]
  cVol = ot.getBrinkTSP(CurrentVol);
  if (cVol != cVol_old) {
    mqttClient.publish("brink/cvol/get", String(int(floor(cVol / maxVent))).c_str());
    cVol_old = cVol;
  }

  // No.2 Bypass status
  bp = ot.getBrinkTSP(BypassStatus);
  if (bp != bp_old) {
    mqttClient.publish("brink/bp/get", String(bp).c_str());
    bp_old = bp;
  }

  // No.3 Temperature from atmosphere [°C]
  tP = ot.getVentSupplyInTemperature();
  if (abs(tP - tP_old) > 0.2) {  // Reduce data publication due frequent slight changes of temp
    mqttClient.publish("brink/tp/get", String(tP).c_str());
    tP_old = tP;
  }

  // No.4 Temperature from indoors [°C]
  tS = ot.getVentExhaustInTemperature();
  if (abs(tS - tS_old) > 0.2) {  // Reduce data publication due frequent slight changes of temp
    mqttClient.publish("brink/ts/get", String(tS).c_str());
    tS_old = tS;
  }

  // No.6 Current input volume [m3/h]
  inVol = ot.getBrinkTSP(CurrentInputVol);
  if (inVol != inVol_old) {
    mqttClient.publish("brink/invol/get", String(inVol * maxVent).c_str());
    inVol_old = inVol;
  }

  // No.7 Current output volume [m3/h]
  outVol = ot.getBrinkTSP(CurrentOutputVol);
  if (outVol != outVol_old) {
    mqttClient.publish("brink/outvol/get", String(outVol * maxVent).c_str());
    outVol_old = outVol;
  }

  // No.8 Current pressure input duct [Pa]
  pressIn = ot.getBrink2TSP(CPID);
  if (abs(pressIn - pressIn_old) > 1) {  // Reduce data publication due to slight changes of pressure
    mqttClient.publish("brink/pressin/get", String(pressIn).c_str());
    pressIn_old = pressIn;
  }

  // No.9 Current pressure output duct [Pa]
  pressOut = ot.getBrink2TSP(CPOD);
  if (abs(pressOut - pressOut_old) > 1) {  // Reduce data publication due to slight changes of pressure
    mqttClient.publish("brink/pressout/get", String(pressOut).c_str());
    pressOut_old = pressOut;
  }

  // No.10 Status frost protection
  frost = ot.getBrinkTSP(FrostStatus);
  if (frost != frost_old) {
    mqttClient.publish("brink/frost/get", String(frost).c_str());
    frost_old = frost;
  }

  // U4 Minimum atmospheric temperature bypass
  u4 = ot.getBrinkTSP(U4);
  if (u4 != u4_old) {
    mqttClient.publish("brink/u4/get", String(u4 / 2).c_str());
    u4_old = u4;
  }

  // U5 Minimum indoor temperature bypass
  u5 = ot.getBrinkTSP(U5);
  if (u5 != u5_old) {
    mqttClient.publish("brink/u5/get", String(u5 / 2).c_str());
    u5_old = u5;
  }

  // I1 Fixed imbalance
  i1 = ot.getBrinkTSP(I1);
  if (i1 != i1_old) {
    mqttClient.publish("brink/i1/get", String(i1).c_str());
    i1_old = i1;
  }

  // Fault code
  fCode = ot.getVentFaultCode();
  if (fCode != fCode_old) {
    mqttClient.publish("brink/fcode/get", String(fCode).c_str());
    fCode_old = fCode;
  }

  // Fault indication?
  fault = ot.getFaultIndication();
  if (fault != fault_old) {
    mqttClient.publish("brink/fault/get", String(fault).c_str());
    fault_old = fault;
  }

  // Ventilation mode
  vMode = ot.getVentilationMode();
  if (vMode != vMode_old) {
    mqttClient.publish("brink/vmode/get", String(vMode).c_str());
    vMode_old = vMode;
  }

  // Filter dirty?
  filter = ot.getDiagnosticIndication();
  if (filter != filter_old) {
    mqttClient.publish("brink/filter/get", String(filter).c_str());
    filter_old = filter;
  }
}
