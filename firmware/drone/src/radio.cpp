#include "radio.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static uint8_t           sNode = 0;
static portMUX_TYPE      sMux = portMUX_INITIALIZER_UNLOCKED;
static RadioCmd          sBuf;   // guarded by sMux
static volatile bool     sFresh = false;
static volatile uint32_t sLastRxMs = 0;
static volatile uint32_t sLastSeq = 0;
static volatile uint16_t sLost = 0;
static uint8_t           sGsMac[6] = {0};
static volatile bool     sGsMacValid = false;
static bool              sGsPeerAdded = false;
static volatile int8_t   sRssi = 0;

#if ESP_ARDUINO_VERSION_MAJOR < 3
// Promiscuous callback only used to grab RSSI of ESP-NOW frames on core 2.x
static void IRAM_ATTR promiscCb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  auto* p = (wifi_promiscuous_pkt_t*)buf;
  sRssi = p->rx_ctrl.rssi;
}
#endif

static void handleFrame(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(SwarmFrame)) return;
  const SwarmFrame* f = (const SwarmFrame*)data;
  if (f->magic != SWARM_MAGIC || f->version != SWARM_VERSION) return;

  uint32_t now = micros();
  portENTER_CRITICAL(&sMux);
  if (sLastSeq && f->seq > sLastSeq + 1) sLost += (uint16_t)(f->seq - sLastSeq - 1);
  sLastSeq = f->seq;
  memcpy(&sBuf.cmd, &f->cmd[sNode], sizeof(DroneCmd));
  sBuf.frameFlags = f->frame_flags;
  sBuf.seq        = f->seq;
  sBuf.gsTimeUs   = f->gs_time_us;
  sBuf.rxMicros   = now;
  sBuf.rssi       = sRssi;
  sFresh = true;
  sLastRxMs = millis();
  portEXIT_CRITICAL(&sMux);

  if (!sGsMacValid) {            // learn the ground station MAC from 1st frame
    memcpy(sGsMac, mac, 6);
    sGsMacValid = true;
  }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info->rx_ctrl) sRssi = info->rx_ctrl->rssi;
  handleFrame(info->src_addr, data, len);
}
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  handleFrame(mac, data, len);
}
#endif

bool Radio::begin(uint8_t nodeId) {
  sNode = nodeId < SWARM_MAX_NODES ? nodeId : 0;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);                       // no modem sleep = no added latency
  esp_wifi_set_channel(SWARM_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
#if ESP_ARDUINO_VERSION_MAJOR < 3
  // Core 2.x has no RSSI in the ESP-NOW callback; sniff it from MGMT frames.
  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(promiscCb);
  esp_wifi_set_promiscuous(true);
#endif

  if (esp_now_init() != ESP_OK) return false;
#if ESP_ARDUINO_VERSION_MAJOR < 3
  esp_wifi_config_espnow_rate(WIFI_IF_STA, SWARM_PHY_RATE);  // same rate as the ground station
#endif
  esp_now_register_recv_cb(onRecv);
  return true;
}

bool Radio::latest(RadioCmd& out) {
  bool fresh;
  portENTER_CRITICAL(&sMux);
  fresh = sFresh;
  if (fresh) {
    out = sBuf;
    sFresh = false;
  }
  portEXIT_CRITICAL(&sMux);
  return fresh;
}

uint32_t Radio::lastRxMillis() { return sLastRxMs; }

uint16_t Radio::takeLostFrames() {
  portENTER_CRITICAL(&sMux);
  uint16_t l = sLost;
  sLost = 0;
  portEXIT_CRITICAL(&sMux);
  return l;
}

void Radio::sendTelemetry(const Telemetry& t) {
  if (!sGsMacValid) return;
  if (!sGsPeerAdded) {           // add the peer here, not in the Wi-Fi callback
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, sGsMac, 6);
    peer.channel = SWARM_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) return;
    sGsPeerAdded = true;
  }
  esp_now_send(sGsMac, (const uint8_t*)&t, sizeof(t));
}
