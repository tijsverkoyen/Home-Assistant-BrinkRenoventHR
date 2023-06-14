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

const char* mqttTopicIn = "brink/+/set";  // subscribe commands
unsigned long startTime;
unsigned long currentTime;
unsigned long readOT;

const int HWCPin = 14;  // Option: HW circulation pump D5
const int inPin = 4;    // ESP8266 D2
const int outPin = 5;   // ESP8266 D1
OpenTherm ot(inPin, outPin);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

float tIn, tIn_old = 0;       // temp external
float tOut, tOut_old = 0;     // temp internal
bool fault, fault_old = 1;    // fault code
bool vmode, vmode_old = 0;    // ventilation mode
bool bypass, bypass_old = 1;  // bypass mode
bool filter, filter_old = 1;  // filter replacement indicator

int pressin, pressin_old = 0;    // pressure input duct [Pa]
int pressout, pressout_old = 0;  // pressure output duct [Pa]
int vstep1, vstep1_old = 50;     // U1
int vstep2, vstep2_old = 150;    // U2
int vstep3, vstep3_old = 300;    // U3
int tU4, tU4_old = 1;            // U4 - atmospheric temp threshold for bypass
int tU5, tU5_old = 1;            // U5 - inside temp threshold for bypass
int cvol, cvol_old = 0;          // current ventilation capacity (out) [m/h3]
int RPMin, RPMin_old;            // RPM in - not used
int RPMout, RPMout_old;          // RPM out - not used
int fcode, fcode_old = 0;        // fault code
int msg, msg_old = 0;            // C-operation message
int param1, param1_old = 100;    // I1- imbalance parameter
long lRssi, lRssi_old = 0;       // Wifi signal level
bool sem_bypass;                 // semaphore for bypass workaround
int invol, invol_old = 0;        // No.6 Current input volume
int outvol, outvol_old = 0;      // No.7 Current output volume
int frost, frost_old = 0;        // No.10 Status frost protection

void ICACHE_RAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

void MqttReconnect() {
  while (!mqttClient.connected()) {
    delay(1000);

    if (mqttClient.connect(mqttClientId, mqttUser, mqttPassword)) {
      mqttClient.setBufferSize(512);
      Serial.println("MQTT> Connected");
      mqttClient.subscribe(mqttTopicIn);
      Serial.print("MQTT> Subscribed");
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

  // Setting/Changing selected Brink Renovent HR parameters
  if (strcmp(topic, "brink/VentNomValue/set") == 0) ot.setVentilation(atoi((char*)payload));  // uint8_t
  if (strcmp(topic, "brink/U4/set") == 0) ot.setBrinkTSP(U4, atoi((char*)payload) * 2);
  if (strcmp(topic, "brink/U5/set") == 0) {
    ot.setBrinkTSP(U5, atoi((char*)payload) * 2);
    delay(100);
    refreshAll();  // change of U5 triggers refresh of other parameters
  }
}

void MqttPublishAutoDiscovery() {
  StaticJsonDocument<400> entity;
  // never changing items
  entity["value_template"] = "{{ value_json }}";

  // 1. Diagnostic entities
  entity["entity_category"] = "diagnostic";

  // 1.1 Temperatures
  entity["device_class"] = "temperature";
  entity["unit_of_measurement"] = "°C";

  // 1.1.1 No.3. Temperature from atmosphere
  entity["unique_id"] = "brink_supply_in_temperature";
  entity["name"] = "Temperature from atmosphere";
  entity["state_topic"] = "brink/SupplyInTemperature/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_supply_in_temperature/config", entity);

  // 1.1.2 No.4. Temperature from indoors
  entity["unique_id"] = "brink_exhaust_in_temperature";
  entity["name"] = "Temperature from indoors";
  entity["state_topic"] = "brink/ExhaustInTemperature/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_exhaust_in_temperature/config", entity);

  // 1.1.3 U4. Minimum Outside Temperature
  // @todo: number entity, want instelbaar
  entity["unique_id"] = "brink_minimum_atmospheric_temperature_bypass";
  entity["name"] = "Minimum atmospheric temperature bypass";
  entity["state_topic"] = "brink/U4/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_minimum_atmospheric_temperature_bypass/config", entity);

  // 1.1.4 U5. Minimum Indoor Temperature
  // @todo: number entity, want instelbaar
  entity["unique_id"] = "brink_minimum_indoor_temperature_bypass";
  entity["name"] = "Minimum indoor temperature bypass";
  entity["state_topic"] = "brink/U5/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_minimum_indoor_temperature_bypass/config", entity);

  // 1.2 Pressures
  entity["device_class"] = "pressure";
  entity["unit_of_measurement"] = "Pa";

  // 1.2.1 No.8. Current pressure input duct
  entity["unique_id"] = "brink_current_pressure_input_duct";
  entity["name"] = "Current pressure input duct";
  entity["state_topic"] = "brink/CPID/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_pressure_input_duct/config", entity);

  // 1.2.1 No.9. Current Pressure Output Duct
  entity["unique_id"] = "brink_current_pressure_output_duct";
  entity["name"] = "Current pressure output duct";
  entity["state_topic"] = "brink/CPOD/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_pressure_output_duct/config", entity);

  // 1.3 Flows
  entity.remove("device_class");
  entity["unit_of_measurement"] = "m³/h";
  entity["icon"] = "mdi:wind-power";

  // 1.3.1 No.1. Current position/outlet volume
  // @todo Fan entity
  entity["unique_id"] = "brink_current_position_outlet_volume";
  entity["name"] = "Current position/outlet volume";
  entity["state_topic"] = "brink/CurrentVolume/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_position_outlet_volume/config", entity);

  // 1.3.2 No.7. Current input volume
  entity["unique_id"] = "brink_current_input_volume";
  entity["name"] = "Current input volume";
  entity["state_topic"] = "brink/CurrentInputVol/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_input_volume/config", entity);

  // 1.3.3 No.8. Current output volume
  entity["unique_id"] = "brink_current_output_volume";
  entity["name"] = "Current output volume";
  entity["state_topic"] = "brink/CurrentOutputVol/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_current_output_volume/config", entity);

  entity.remove("unit_of_measurement");
  entity.remove("icon");

  // 1.4 Fault
  entity["icon"] = "mdi:alert-circle";
  // 1.4.1 Fault Indication
  entity["unique_id"] = "brink_fault_indication";
  entity["name"] = "Fault Indication";
  entity["state_topic"] = "brink/FaultIndication/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_fault_indication/config", entity);

  // 1.4.2 Fault Code
  entity["unique_id"] = "brink_fault_code";
  entity["name"] = "Fault Code";
  entity["state_topic"] = "brink/FaultCode/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_fault_code/config", entity);
  entity.remove("icon");

  // 1.5 Strings
  // 1.5.1 Ventilation Mode
  entity["unique_id"] = "brink_ventilation_mode";
  entity["name"] = "Ventilation Mode";
  entity["state_topic"] = "brink/VentilationMode/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_ventilation_mode/config", entity);

  // 1.5.2 No.2. Bypass Status
  // 0 = bypass valve shut
  // 1 = bypass valve automatic
  // 2 = input at minimum
  entity["device_class"] = "enum";
  entity["options"][0] = "bypass valve shut";
  entity["options"][1] = "bypass valve automatic";
  entity["options"][2] = "input at minimum";
  entity["unique_id"] = "brink_bypass_status";
  entity["name"] = "Bypass Status";
  entity["state_topic"] = "brink/BypassStatus/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_bypass_status/config", entity);
  entity.remove("device_class");
  entity.remove("options");
  
   // 1.5.4 No.0. Message code operating condition
  entity["unique_id"] = "brink_message_code_operating_condition";
  entity["name"] = "Message code operating condition";
  entity["state_topic"] = "brink/MCOC/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_message_code_operating_condition/config", entity);

  // 1.5.5 I1. Fixed imbalance
  entity["unique_id"] = "brink_i1";
  entity["name"] = "Fixed imbalance";
  entity["state_topic"] = "brink/I1/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_i1/config", entity);

  // 1.5.6 Status frost protection
  entity["unique_id"] = "brink_frost_protection";
  entity["name"] = "Status frost protection";
  entity["state_topic"] = "brink/FrostStatus/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_frost_protection/config", entity);

  // 1.6 Binary
  // 1.6.1 Filter Dirty
  entity["device_class"] = "problem";
  entity["unique_id"] = "brink_filter_dirty";
  entity["name"] = "Filter Dirty";
  entity["state_topic"] = "brink/FilterDirty/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/binary_sensor/brink_filter_dirty/config", entity);

  // 1.x Signal Strengths
  // 1.x.1 WiFi Signal Strenght
  entity["device_class"] = "signal_strength";
  entity["unit_of_measurement"] = "dB";
  entity["unique_id"] = "brink_wifi_rsi";
  entity["name"] = "WiFi RSI";
  entity["state_topic"] = "brink/Wifi/get";
  MqttPublishAutoDiscoveryMessage("homeassistant/sensor/brink_wifi_rssi/config", entity);
}

void MqttPublishAutoDiscoveryMessage(const char* topic, JsonDocument& payload) {
  payload["device"]["identifiers"][0] = "brink_renovent_hr";
  payload["device"]["manufacturer"] = "Brink";
  payload["device"]["model"] = "Renovent HR";
  payload["device"]["name"] = "Brink Renovent HR";

  char buffer[512];
  size_t n = serializeJson(payload, buffer);
  mqttClient.publish(topic, buffer);
  Serial.print("Brink> Publishing message");
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
  Serial.println("");

  // Connect to MQTT Broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(MqttCallback);
  MqttReconnect();
  MqttPublishAutoDiscovery();

  startTime = millis();
  ot.begin(handleInterrupt);

  ReadBrinkParameters();
  refreshAll();
  sem_bypass = bypass;
  if (bypass == 1) readOT = readPeriod_bypass;
  else readOT = readPeriod;
}

// refresh slow changing parameters on request by every change of U5
void refreshAll() {
  mqttClient.publish("brink/SupplyInTemperature/get", String(tIn).c_str());
  mqttClient.publish("brink/ExhaustInTemperature/get", String(tOut).c_str());
  mqttClient.publish("brink/FaultIndication/get", String(fault).c_str());
  mqttClient.publish("brink/VentilationMode/get", String(vmode).c_str());
  mqttClient.publish("brink/BypassStatus/get", String(bypass).c_str());
  mqttClient.publish("brink/FilterDirty/get", String(filter).c_str());
  mqttClient.publish("brink/CurrentVolume/get", String(cvol * maxVent).c_str());
  mqttClient.publish("brink/FaultCode/get", String(fcode).c_str());
  mqttClient.publish("brink/OperationMsg/get", String(msg).c_str());
  mqttClient.publish("brink/I1/get", String(param1).c_str());
  mqttClient.publish("brink/U1/get", String(vstep1).c_str());
  mqttClient.publish("brink/U2/get", String(vstep2).c_str());
  mqttClient.publish("brink/U3/get", String(vstep3).c_str());
  mqttClient.publish("brink/U4/get", String(tU4 / 2).c_str());
  mqttClient.publish("brink/U5/get", String(tU5 / 2).c_str());
  mqttClient.publish("brink/Wifi/get", String(lRssi).c_str());
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

    // Publish WiFi signal strength
    lRssi = WiFi.RSSI();
    if (abs(lRssi - lRssi_old) > 2) {
      mqttClient.publish("brink/Wifi/get", String(lRssi).c_str());
      lRssi_old = lRssi;
    }

    // Workaround for bypass change and keeping change when U4 and U5 conditions are met
    // Bypass is CLOSED
    if (sem_bypass == 0) {
      if ((tOut > tU5 / 2) && (tIn > tU4 / 2) && (tIn < tOut)) {  // if true open bypass,
        delay(150000);                                            // stop 2,5 min
        readOT = readPeriod_bypass;
        sem_bypass = 1;
      }
    }

    // Bypass is OPEN
    if (sem_bypass == 1) {
      if ((tOut < tU5 / 2) || (tIn < tU4 / 2) || (tIn > tOut)) {  // if true close bypass
        delay(150000);                                            // stop 2,5 min
        readOT = readPeriod;
        sem_bypass = 0;
      }
    }
    startTime = currentTime;
  }
}

// Reading all requested Brink HR parameters, MQTT publication only happens if a value has changed
void ReadBrinkParameters() {
  tIn = ot.getVentSupplyInTemperature();
  if (abs(tIn - tIn_old) > 0.2) {  // Reduce data publication due frequent slight changes of temp
    mqttClient.publish("brink/SupplyInTemperature/get", String(tIn).c_str());
    tIn_old = tIn;
  }

  tOut = ot.getVentExhaustInTemperature();
  if (abs(tOut - tOut_old) > 0.2) {  // Reduce data publication due frequent slight changes of temp
    mqttClient.publish("brink/ExhaustInTemperature/get", String(tOut).c_str());
    tOut_old = tOut;
  }

  fault = ot.getFaultIndication();
  if (fault != fault_old) {
    mqttClient.publish("brink/FaultIndication/get", String(fault).c_str());
    fault_old = fault;
  }

  vmode = ot.getVentilationMode();
  if (vmode != vmode_old) {
    mqttClient.publish("brink/VentilationMode/get", String(vmode).c_str());
    vmode_old = vmode;
  }

  bypass = ot.getBrinkTSP(BypassStatus);  // ot.getBypassStatus() - this method does not work
  if (bypass != bypass_old) {
    mqttClient.publish("brink/BypassStatus/get", String(bypass).c_str());
    bypass_old = bypass;
  }

  filter = ot.getDiagnosticIndication();
  if (filter != filter_old) {
    mqttClient.publish("brink/FilterDirty/get", String(filter).c_str());
    filter_old = filter;
  }

  pressin = ot.getBrink2TSP(CPID);
  if (abs(pressin - pressin_old) > 1) {  // Reduce data publication due frequent slight changes of pressure
    mqttClient.publish("brink/CPID/get", String(pressin).c_str());
    pressin_old = pressin;
  }

  pressout = ot.getBrink2TSP(CPOD);
  if (abs(pressout - pressout_old) > 1) {  // Reduce data publication due frequent slight changes of pressure
    mqttClient.publish("brink/CPOD/get", String(pressout).c_str());
    pressout_old = pressout;
  }

  cvol = ot.getBrinkTSP(CurrentVol);
  if (cvol != cvol_old) {
    mqttClient.publish("brink/CurrentVolume/get", String(cvol).c_str());
    cvol_old = cvol;
  }

  fcode = ot.getVentFaultCode();
  if (fcode != fcode_old) {
    mqttClient.publish("brink/FaultCode/get", String(fcode).c_str());
    fcode_old = fcode;
  }

  vstep1 = ot.getBrinkTSP(U1);
  if (vstep1 != vstep1_old) {
    mqttClient.publish("brink/U1/get", String(vstep1).c_str());
    vstep1_old = vstep1;
  }

  vstep2 = ot.getBrinkTSP(U2);
  if (vstep2 != vstep2_old) {
    mqttClient.publish("brink/U2/get", String(vstep2).c_str());
    vstep2_old = vstep2;
  }

  vstep3 = ot.getBrink2TSP(U3);
  if (vstep3 != vstep3_old) {
    mqttClient.publish("brink/U3/get", String(vstep3).c_str());
    vstep3_old = vstep3;
  }

  tU4 = ot.getBrinkTSP(U4);
  if (tU4 != tU4_old) {
    mqttClient.publish("brink/U4/get", String(tU4 / 2).c_str());
    tU4_old = tU4;
  }

  tU5 = ot.getBrinkTSP(U5);
  if (tU5 != tU5_old) {
    mqttClient.publish("brink/U5/get", String(tU5 / 2).c_str());
    tU5_old = tU5;
  }

  msg = ot.getBrinkTSP(MsgOperation);
  if (msg != msg_old) {
    mqttClient.publish("brink/MCOC/get", String(msg).c_str());
    msg_old = msg;
  }

  param1 = ot.getBrinkTSP(I1);
  if (param1 != param1_old) {
    mqttClient.publish("brink/I1/get", String(param1).c_str());
    param1_old = param1;
  }

  invol = ot.getBrinkTSP(CurrentInputVol);
  if (invol != invol_old) {
    mqttClient.publish("brink/InputVolume/get", String(invol * maxVent).c_str());
    invol_old = invol;
  }

  outvol = ot.getBrinkTSP(CurrentOutputVol);
  if (outvol != outvol_old) {
    mqttClient.publish("brink/OutputVolume/get", String(invol * maxVent).c_str());
    outvol_old = outvol;
  }

  frost = ot.getBrinkTSP(FrostStatus);
  if (frost != frost_old) {
    mqttClient.publish("brink/FrostStatus/get", String(frost).c_str());
    frost_old = frost;
  }
}
