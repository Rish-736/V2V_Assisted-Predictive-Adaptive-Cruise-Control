// =====================================================================
//  mac_address.ino  --  utility sketch
//
//  Flash to either board and open the serial monitor at 115200.
//  Prints the STA MAC address and the current WiFi channel.
//
//  NOTE: the main sketches use ESP-NOW BROADCAST, so you do not need
//  these MACs to get the link running. You only need them if you set
//  V2V_USE_UNICAST = 1 in lead_node.ino to get per-packet delivery
//  callbacks for a link-reliability experiment.
//
//  Keep the printout anyway -- it is a useful screenshot for Review 1
//  ("here are our two nodes, here is the link coming up").
// =====================================================================
#include <WiFi.h>
#include <esp_wifi.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" V2V Predictive ACC -- node identity");
  Serial.println("==========================================");
  Serial.print  (" STA MAC  : "); Serial.println(WiFi.macAddress());
  Serial.print  (" Channel  : "); Serial.println(primary);
  Serial.print  (" Chip     : "); Serial.println(ESP.getChipModel());
  Serial.print  (" Cores    : "); Serial.println(ESP.getChipCores());
  Serial.print  (" Free heap: "); Serial.println(ESP.getFreeHeap());
  Serial.println("==========================================");
}

void loop() {
  delay(5000);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}
