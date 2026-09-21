// ============================================================
//  esp32-c6-ch375-printserver.ino  (v4 — параллельный режим CH375 +
//  фичи, перенесённые из проекта ESP32-S3-PrintServer: LPR/LPD,
//  веб-портал со статусом (RU/EN), SSDP/UPnP + mDNS, опрос статуса
//  принтера по PJL, WPA2-Enterprise, опциональный OLED-дисплей)
//  ----------------------------------------------------------------
//  ЖЕЛЕЗО (см. даташит CH375, разделы 4, 6.2.1, 7.3, 8.1):
//    ESP32-C6 Super Mini  <-- 8-бит параллельная шина -->  CH375B  <-- USB -->  принтер
//
//    CH375B D0..D7  -> ESP32-C6 GPIO18, GPIO19, GPIO7, GPIO6, GPIO3, GPIO2, GPIO1, GPIO0
//    CH375B RD#     -> ESP32-C6 GPIO23
//    CH375B WR#     -> ESP32-C6 GPIO22
//    CH375B A0      -> ESP32-C6 GPIO21
//    CH375B INT#    -> ESP32-C6 GPIO20
//    CH375B CS#     -> GND (жёстко, постоянно выбран — другого устройства на шине нет)
//    CH375B TXD     -> GND через резистор ~1кОм (ВАЖНО: strapping-пин,
//                       выбирающий параллельный режим при старте чипа;
//                       раздел 8.1 даташита)
//    CH375B RXD     -> не подключен
//    CH375B RST     -> не подключен (это выход чипа, не вход)
//
//  Пины сверены с распиновкой конкретной платы (ESP32-C6 Super Mini):
//  GP8 занят встроенным RGB LED (WS2812 DIN), GP9 — кнопка BOOT (она же
//  используется в этом скетче как RESET_BUTTON_PIN), GP15 занят обычным
//  LED-индикатором, TX/RX зарезервированы под консоль/прошивку. ВАЖНО:
//  GP12/GP13 тоже не используются — на ESP32-C6 это фиксированные на
//  уровне кристалла линии встроенного USB Serial/JTAG (D-/D+), даже если
//  на схеме платы это никак не отмечено — переопределение этих пинов под
//  GPIO обрывает COM-порт сразу после того, как код доходит до setup().
//
//  Опциональный OLED (SSD1306, I2C): GP4 = SDA, GP5 = SCL — свободные
//  пины на этой плате. Включается флагом ENABLE_OLED_DISPLAY ниже.
//  Требует библиотеки Adafruit_GFX и Adafruit_SSD1306 (Library Manager).
// ============================================================

// Раскомментируйте, если на плату физически впаян SSD1306:
// #define ENABLE_OLED_DISPLAY

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "esp_wifi.h"
#include "esp_eap_client.h"

#ifdef ENABLE_OLED_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

#include <Adafruit_NeoPixel.h>

// ==================== НАСТРОЙКИ ЖЕЛЕЗА ====================

#define CH375_D0_PIN 18
#define CH375_D1_PIN 19
#define CH375_D2_PIN 7
#define CH375_D3_PIN 6
#define CH375_D4_PIN 3
#define CH375_D5_PIN 2
#define CH375_D6_PIN 1
#define CH375_D7_PIN 0
#define CH375_RD_PIN 23
#define CH375_WR_PIN 22
#define CH375_A0_PIN 21
#define CH375_INT_PIN 20

const uint8_t CH375_D_PINS[8] = {
  CH375_D0_PIN, CH375_D1_PIN, CH375_D2_PIN, CH375_D3_PIN,
  CH375_D4_PIN, CH375_D5_PIN, CH375_D6_PIN, CH375_D7_PIN
};

const int RESET_BUTTON_PIN = 9;
const uint32_t RESET_HOLD_MS = 10000;
const uint16_t RAW_PORT = 9100;
const uint16_t LPR_PORT = 515;
const uint16_t SSDP_PORT = 1900;

// Встроенный WS2812 RGB LED на этой плате (уже упоминался как занятый GP8).
#define RGB_LED_PIN 8
Adafruit_NeoPixel pixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

enum LedState {
  LED_STATE_OFF,
  LED_STATE_WIFI_CONNECTING,  // зелёный/красный, чередуются
  LED_STATE_AP_CONFIG,        // синий, мигает
  LED_STATE_WAITING_PRINTER,  // красный, горит — принтер выключен/не подключен
  LED_STATE_IDLE_READY,       // зелёный, горит
  LED_STATE_PRINTING,         // зелёный, мигает
  LED_STATE_RESET_HELD,       // красный, мигает — приоритет надо всем
  LED_STATE_PRINTER_ERROR     // жёлтый, мигает — ошибка принтера ИЛИ статус не определён
};
LedState currentLedState = LED_STATE_OFF;
bool resetHeldOverrideLed = false;

#ifdef ENABLE_OLED_DISPLAY
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 5
#define OLED_WIDTH 128
#define OLED_HEIGHT 32
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledAvailable = false;
#endif

// ==================== CH375: КОМАНДЫ И СТРУКТУРЫ ====================

#define CH375_CMD_GET_IC_VER     0x01
#define CH375_CMD_CHECK_EXIST    0x06
#define CH375_CMD_SET_USB_ADDR   0x13
#define CH375_CMD_SET_USB_MODE   0x15
#define CH375_CMD_SET_ENDP6      0x1C
#define CH375_CMD_SET_ENDP7      0x1D
#define CH375_CMD_GET_STATUS     0x22
#define CH375_CMD_RD_USB_DATA    0x28
#define CH375_CMD_WR_USB_DATA7   0x2B
#define CH375_CMD_SET_ADDRESS    0x45
#define CH375_CMD_GET_DESCR      0x46
#define CH375_CMD_SET_CONFIG     0x49
#define CH375_CMD_ISSUE_TOKEN    0x4F
#define CH375_CMD_RET_SUCCESS    0x51

#define CH375_USB_MODE_HOST       0x06
#define CH375_USB_MODE_HOST_RESET 0x07

#define CH375_USB_INT_SUCCESS  0x14
#define CH375_USB_INT_CONNECT  0x15

#define CH375_USB_DEVICE_DESCRIPTOR        0x01
#define CH375_USB_CONFIGURATION_DESCRIPTOR 0x02
#define CH375_USB_STRING_DESCRIPTOR        0x03
#define CH375_USB_INTERFACE_DESCRIPTOR     0x04

#define USB_PID_OUT 0x01
#define USB_PID_IN  0x09

#pragma pack(push, 1)
struct USBDeviceDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
};
struct USBConfigurationDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
};
struct USBInterfaceDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
};
struct USBEndpointDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
};
struct USBConfigurationDescriptorFull {
  USBConfigurationDescriptor configuration;
  USBInterfaceDescriptor interface;
  USBEndpointDescriptor endpoints[4];
};
#pragma pack(pop)

// ==================== CH375: НИЗКОУРОВНЕВЫЕ ФУНКЦИИ (ПАРАЛЛЕЛЬНАЯ ШИНА) ====================

bool ch375ToggleSend = false;    // toggle DATA0/DATA1 для host OUT эндпоинта
bool ch375ToggleReceive = false; // toggle DATA0/DATA1 для host IN эндпоинта

enum Ch375BusDir { CH375_BUS_UNSET, CH375_BUS_OUTPUT, CH375_BUS_INPUT };
Ch375BusDir ch375BusCurrentDir = CH375_BUS_UNSET;

// pinMode() на ESP32 заметно дороже digitalWrite() (перенастройка матрицы
// GPIO), а направление шины почти всегда не меняется от байта к байту —
// переключаем направление только когда оно реально отличается от текущего.
void ch375BusSetOutput() {
  if (ch375BusCurrentDir == CH375_BUS_OUTPUT) return;
  for (uint8_t i = 0; i < 8; i++) pinMode(CH375_D_PINS[i], OUTPUT);
  ch375BusCurrentDir = CH375_BUS_OUTPUT;
}
void ch375BusSetInput() {
  if (ch375BusCurrentDir == CH375_BUS_INPUT) return;
  for (uint8_t i = 0; i < 8; i++) pinMode(CH375_D_PINS[i], INPUT);
  ch375BusCurrentDir = CH375_BUS_INPUT;
}
void ch375BusWrite(uint8_t value) {
  for (uint8_t i = 0; i < 8; i++) digitalWrite(CH375_D_PINS[i], (value >> i) & 1);
}
uint8_t ch375BusRead() {
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; i++) if (digitalRead(CH375_D_PINS[i])) value |= (1 << i);
  return value;
}

void ch375WriteByte(uint8_t value, bool isCommand) {
  digitalWrite(CH375_A0_PIN, isCommand ? HIGH : LOW);
  delayMicroseconds(3); // TAS
  ch375BusSetOutput();
  ch375BusWrite(value);
  delayMicroseconds(3); // TIS
  digitalWrite(CH375_WR_PIN, LOW);
  delayMicroseconds(3); // TWW
  digitalWrite(CH375_WR_PIN, HIGH);
  delayMicroseconds(3); // TIH/TAH
}

uint8_t ch375ReadByte(bool isCommand) {
  digitalWrite(CH375_A0_PIN, isCommand ? HIGH : LOW);
  delayMicroseconds(3); // TAS
  ch375BusSetInput();
  digitalWrite(CH375_RD_PIN, LOW);
  delayMicroseconds(3); // TON
  uint8_t value = ch375BusRead();
  digitalWrite(CH375_RD_PIN, HIGH);
  delayMicroseconds(3); // TAH
  return value;
}

void ch375SendCommand(uint8_t b) { ch375WriteByte(b, true); }
void ch375SendData(uint8_t b) { ch375WriteByte(b, false); }

bool ch375Receive(uint8_t &outByte, uint32_t timeoutMs = 1000) {
  delayMicroseconds(30);
  outByte = ch375ReadByte(false);
  return true;
}

bool ch375CheckExist(uint8_t testByte) {
  ch375SendCommand(CH375_CMD_CHECK_EXIST);
  ch375SendData(testByte);
  uint8_t resp;
  if (!ch375Receive(resp, 500)) return false;
  return resp == (uint8_t)(~testByte);
}

uint8_t ch375GetChipVersion() {
  ch375SendCommand(CH375_CMD_GET_IC_VER);
  uint8_t resp = 0;
  ch375Receive(resp, 500);
  return resp;
}

bool ch375ExecCommand(uint8_t cmd, uint8_t arg, uint32_t timeoutMs = 1000) {
  ch375SendCommand(cmd);
  ch375SendData(arg);
  uint8_t resp;
  if (!ch375Receive(resp, timeoutMs)) return false;
  return resp == CH375_CMD_RET_SUCCESS;
}

bool ch375WaitInterrupt(uint8_t &status, uint32_t timeoutMs = 3000) {
  unsigned long start = millis();
  while (digitalRead(CH375_INT_PIN) != LOW) {
    if (millis() - start > timeoutMs) return false;
    yield();
  }
  ch375SendCommand(CH375_CMD_GET_STATUS);
  return ch375Receive(status, timeoutMs);
}

bool ch375GetDescriptor(uint8_t descriptorType) {
  ch375SendCommand(CH375_CMD_GET_DESCR);
  ch375SendData(descriptorType);
  uint8_t status;
  if (!ch375WaitInterrupt(status)) return false;
  return status == CH375_USB_INT_SUCCESS;
}

// ЭКСПЕРИМЕНТАЛЬНО: чтение строкового дескриптора (Manufacturer/Product/
// SerialNumber). В отличие от device/configuration дескрипторов, для
// строки нужен ещё индекс — команда GET_DESCR исторически проверена
// только с одним байтом данных (тип), поведение со вторым байтом (индекс)
// не подтверждено документацией. Обёрнуто так, чтобы неудача просто
// давала пустую строку, не трогая остальную работу принтера.
bool ch375GetStringDescriptor(uint8_t index, char *out, size_t outSize) {
  out[0] = 0;
  if (index == 0) return false;
  ch375SendCommand(CH375_CMD_GET_DESCR);
  ch375SendData(CH375_USB_STRING_DESCRIPTOR);
  ch375SendData(index);
  uint8_t status;
  if (!ch375WaitInterrupt(status, 1000) || status != CH375_USB_INT_SUCCESS) return false;
  uint8_t raw[64];
  uint8_t len = ch375RdUsbDataInto(raw, sizeof(raw));
  if (len < 4 || raw[1] != CH375_USB_STRING_DESCRIPTOR) return false;
  // Стандартный формат: raw[0]=bLength, raw[1]=bDescriptorType, дальше
  // символы в UTF-16LE. Берём только код-пойнты до 127 (латиница/цифры —
  // этого достаточно для типичных Manufacturer/Model/SerialNumber строк).
  size_t o = 0;
  for (uint8_t i = 2; i + 1 < len && o + 1 < outSize; i += 2) {
    uint16_t ch = raw[i] | (raw[i + 1] << 8);
    out[o++] = (ch > 0 && ch < 128) ? (char)ch : '?';
  }
  out[o] = 0;
  return o > 0;
}

void ch375RdUsbData(uint8_t *buf, uint8_t maxLen) {
  ch375SendCommand(CH375_CMD_RD_USB_DATA);
  uint8_t len = 0;
  ch375Receive(len, 500);
  for (uint8_t i = 0; i < len; i++) {
    uint8_t data = 0;
    ch375Receive(data, 500);
    if (i < maxLen) buf[i] = data;
  }
}

// Как ch375RdUsbData, но возвращает реальную длину полученных данных —
// нужно для чтения ответов через bulk IN (PJL-статус), где длина заранее
// не известна.
uint8_t ch375RdUsbDataInto(uint8_t *buf, uint8_t maxLen) {
  ch375SendCommand(CH375_CMD_RD_USB_DATA);
  uint8_t len = 0;
  ch375Receive(len, 500);
  for (uint8_t i = 0; i < len; i++) {
    uint8_t data = 0;
    ch375Receive(data, 500);
    if (i < maxLen) buf[i] = data;
  }
  return len > maxLen ? maxLen : len;
}

void ch375WrUsbData(const uint8_t *buf, uint8_t len) {
  ch375SendCommand(CH375_CMD_WR_USB_DATA7);
  ch375SendData(len);
  for (uint8_t i = 0; i < len; i++) {
    ch375SendData(buf[i]);
  }
}

void ch375ToggleHostEndpoint(uint8_t setEndpointCommand, bool tog) {
  ch375SendCommand(setEndpointCommand);
  ch375SendData(tog ? 0xC0 : 0x80);
  delayMicroseconds(10); // TE3 по даташиту: 2-4мкс максимум, запас x2-5
}

bool ch375IssueToken(uint8_t targetEndpoint, uint8_t pid, uint32_t waitMs = 20000) {
  if ((targetEndpoint & 0xF0) || (pid & 0xF0)) return false;
  ch375SendCommand(CH375_CMD_ISSUE_TOKEN);
  ch375SendData((targetEndpoint << 4) | pid);
  uint8_t status;
  // CH375 сам, аппаратно, бесконечно ретраит NAK от устройства (раздел
  // 5.11 даташита) — прерывание придёт только когда реально закончит,
  // успехом или настоящей ошибкой. Поэтому один длинный вызов ожидания,
  // а не повторная отправка ISSUE_TOKEN снаружи.
  if (!ch375WaitInterrupt(status, waitMs)) return false;
  return status == CH375_USB_INT_SUCCESS;
}

bool ch375DoBulkOutTransfer(uint8_t targetEndpoint, const uint8_t *buf, uint8_t len) {
  if (len > 64) return false;
  ch375ToggleHostEndpoint(CH375_CMD_SET_ENDP7, ch375ToggleSend);
  ch375WrUsbData(buf, len);
  if (ch375IssueToken(targetEndpoint, USB_PID_OUT, 20000)) {
    ch375ToggleSend = !ch375ToggleSend;
    return true;
  }
  return false;
}

// Bulk IN — для чтения ответов принтера (PJL-статус). waitMs короткий и
// настраиваемый: в отличие от OUT, здесь "нет данных прямо сейчас" —
// нормальная ситуация при поллинге, а не ошибка, которую нужно ждать
// секундами.
bool ch375DoBulkInTransfer(uint8_t targetEndpoint, uint8_t *buf, uint8_t maxLen, uint8_t *actualLen, uint32_t waitMs = 300) {
  ch375ToggleHostEndpoint(CH375_CMD_SET_ENDP6, ch375ToggleReceive);
  if (!ch375IssueToken(targetEndpoint, USB_PID_IN, waitMs)) return false;
  ch375ToggleReceive = !ch375ToggleReceive;
  *actualLen = ch375RdUsbDataInto(buf, maxLen);
  return true;
}

bool ch375ResetAndGetDeviceDescriptor(USBDeviceDescriptor *result) {
  if (!ch375ExecCommand(CH375_CMD_SET_USB_MODE, CH375_USB_MODE_HOST)) return false;
  uint8_t status;
  if (!ch375WaitInterrupt(status, 5000) || status != CH375_USB_INT_CONNECT) return false;

  if (!ch375ExecCommand(CH375_CMD_SET_USB_MODE, CH375_USB_MODE_HOST_RESET)) return false;
  delay(10);
  if (!ch375ExecCommand(CH375_CMD_SET_USB_MODE, CH375_USB_MODE_HOST)) return false;
  delay(100);
  if (!ch375WaitInterrupt(status, 5000) || status != CH375_USB_INT_CONNECT) return false;
  delay(200);

  if (!ch375GetDescriptor(CH375_USB_DEVICE_DESCRIPTOR)) return false;
  ch375RdUsbData((uint8_t *)result, sizeof(USBDeviceDescriptor));

  return result->bLength >= 18
      && result->bDescriptorType == CH375_USB_DEVICE_DESCRIPTOR
      && result->bNumConfigurations == 1;
}

bool ch375SetAddress(uint8_t address) {
  if (address & 0b10000000) return false;
  ch375SendCommand(CH375_CMD_SET_ADDRESS);
  ch375SendData(address);
  uint8_t status;
  if (!ch375WaitInterrupt(status) || status != CH375_USB_INT_SUCCESS) return false;

  ch375SendCommand(CH375_CMD_SET_USB_ADDR);
  ch375SendData(address);
  delay(5);
  return true;
}

bool ch375GetFullConfigurationDescriptor(USBConfigurationDescriptorFull *result) {
  if (!ch375GetDescriptor(CH375_USB_CONFIGURATION_DESCRIPTOR)) return false;
  ch375RdUsbData((uint8_t *)result, sizeof(USBConfigurationDescriptorFull));
  return result->configuration.bDescriptorType == CH375_USB_CONFIGURATION_DESCRIPTOR
      && result->configuration.bNumInterfaces >= 1
      && result->interface.bDescriptorType == CH375_USB_INTERFACE_DESCRIPTOR
      && result->interface.bNumEndpoints <= 4;
}

bool ch375SetConfiguration(uint8_t configuration) {
  ch375SendCommand(CH375_CMD_SET_CONFIG);
  ch375SendData(configuration);
  uint8_t status;
  if (!ch375WaitInterrupt(status)) return false;
  return status == CH375_USB_INT_SUCCESS;
}

void ch375ResetChip() {
  // Намеренно пусто — RST чипа это выход, не вход, внешний сброс не нужен.
}

// ==================== ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ====================

WiFiServer rawServer(RAW_PORT);
WiFiServer lprServer(LPR_PORT);
WebServer configServer(80);
WiFiUDP ssdpUdp;
Preferences prefs;

String savedSsid, savedPassword;
bool savedIsEnterprise = false;
String savedEapIdentity, savedEapUsername, savedEapPassword;
String portalPassword; // пусто = защита портала выключена

volatile bool printerReady = false;
volatile bool jobInProgress = false; // печать идёт прямо сейчас — не лезем с PJL-опросом
uint8_t printerOutEndpoint = 0;
uint16_t printerOutMaxPacket = 64;
uint8_t printerInEndpoint = 0;
bool printerHasInEndpoint = false;

int printerStatusCode = -1;
String printerStatusDisplay = "";
bool printerStatusOnline = false;
unsigned long lastPjlPollMs = 0;

String printerManufacturer, printerModel, printerSerial;
uint8_t printerUsbAddress = 0;

unsigned long resetButtonPressStart = 0;
unsigned long resetButtonLastReport = 0;

String ssdpUuid;
unsigned long lastSsdpNotifyMs = 0;

unsigned long totalJobsPrinted = 0;
unsigned long totalBytesPrinted = 0;

// ==================== NVS ====================

bool loadWifiCredentials() {
  prefs.begin("wifi", true);
  savedSsid = prefs.getString("ssid", "");
  savedPassword = prefs.getString("pass", "");
  savedIsEnterprise = prefs.getBool("ent", false);
  savedEapIdentity = prefs.getString("eapId", "");
  savedEapUsername = prefs.getString("eapUser", "");
  savedEapPassword = prefs.getString("eapPass", "");
  prefs.end();

  prefs.begin("portal", true);
  portalPassword = prefs.getString("pass", "");
  prefs.end();

  return savedSsid.length() > 0;
}

void saveWifiCredentials(const String &ssid, const String &pass, bool isEnterprise,
                          const String &eapId, const String &eapUser, const String &eapPass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putBool("ent", isEnterprise);
  prefs.putString("eapId", eapId);
  prefs.putString("eapUser", eapUser);
  prefs.putString("eapPass", eapPass);
  prefs.end();
  Serial.printf("[NVS] Сохранены новые WiFi-настройки: SSID=\"%s\" enterprise=%d\n", ssid.c_str(), isEnterprise);
}

void clearWifiCredentials() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
  prefs.begin("portal", false);
  prefs.clear();
  prefs.end();
  Serial.println("[NVS] WiFi-настройки и пароль портала очищены.");
}

void savePortalPassword(const String &pass) {
  prefs.begin("portal", false);
  if (pass.length() == 0) prefs.remove("pass");
  else prefs.putString("pass", pass);
  prefs.end();
  portalPassword = pass;
}

// ==================== USB: ПОИСК ЭНДПОИНТОВ И ИНИЦИАЛИЗАЦИЯ ПРИНТЕРА ====================

bool findBulkOutEndpoint(const USBConfigurationDescriptorFull &cfg, uint8_t *epOut, uint16_t *maxPacket) {
  uint8_t count = cfg.interface.bNumEndpoints;
  if (count > 4) count = 4;
  for (uint8_t i = 0; i < count; i++) {
    const USBEndpointDescriptor &ep = cfg.endpoints[i];
    bool isOut = (ep.bEndpointAddress & 0x80) == 0;
    bool isBulk = (ep.bmAttributes & 0x03) == 0x02;
    if (isOut && isBulk) {
      // ВАЖНО: ch375IssueToken() ожидает "голый" номер эндпоинта (4 бита,
      // без старшего бита направления) — bEndpointAddress его несёт, для
      // OUT он и так 0, поэтому раньше маскирование не требовалось, но
      // для консистентности маскируем и здесь.
      *epOut = ep.bEndpointAddress & 0x0F;
      *maxPacket = ep.wMaxPacketSize;
      return true;
    }
  }
  return false;
}

bool findBulkInEndpoint(const USBConfigurationDescriptorFull &cfg, uint8_t *epIn) {
  uint8_t count = cfg.interface.bNumEndpoints;
  if (count > 4) count = 4;
  for (uint8_t i = 0; i < count; i++) {
    const USBEndpointDescriptor &ep = cfg.endpoints[i];
    bool isIn = (ep.bEndpointAddress & 0x80) != 0;
    bool isBulk = (ep.bmAttributes & 0x03) == 0x02;
    if (isIn && isBulk) {
      // ВАЖНО: та же маскировка, что и для OUT — но здесь она критична:
      // bEndpointAddress для IN несёт установленный старший бит (обычно
      // 0x81), а ch375IssueToken() отбраковывает targetEndpoint, если в
      // нём есть что-то за пределами младших 4 бит. Без маски запрос
      // проваливался ещё ДО обращения к чипу — именно поэтому PJL-ответ
      // не приходил вообще никогда.
      *epIn = ep.bEndpointAddress & 0x0F;
      return true;
    }
  }
  return false;
}

bool initPrinter(bool verbose = true) {
  USBDeviceDescriptor devDesc;
  if (verbose) Serial.println("[USB] Сброс шины и ожидание подключения устройства...");
  if (!ch375ResetAndGetDeviceDescriptor(&devDesc)) {
    if (verbose) Serial.println("[USB] Не удалось получить Device Descriptor (таймаут или неверные данные).");
    return false;
  }
  Serial.printf("[USB] Устройство найдено: VID=%04X PID=%04X класс=%02X\n",
                devDesc.idVendor, devDesc.idProduct, devDesc.bDeviceClass);

  if (!ch375SetAddress(1)) {
    if (verbose) Serial.println("[USB] Не удалось назначить USB-адрес устройству.");
    return false;
  }
  printerUsbAddress = 1;

  char strBuf[64];
  if (ch375GetStringDescriptor(devDesc.iManufacturer, strBuf, sizeof(strBuf))) printerManufacturer = strBuf;
  else printerManufacturer = (devDesc.idVendor == 0x03F0) ? "Hewlett-Packard" : "";
  if (ch375GetStringDescriptor(devDesc.iProduct, strBuf, sizeof(strBuf))) printerModel = strBuf;
  else printerModel = (devDesc.idVendor == 0x03F0 && devDesc.idProduct == 0x3817) ? "HP LaserJet P2015 Series" : "";
  if (ch375GetStringDescriptor(devDesc.iSerialNumber, strBuf, sizeof(strBuf))) printerSerial = strBuf;
  else printerSerial = "";

  USBConfigurationDescriptorFull cfgDesc;
  if (!ch375GetFullConfigurationDescriptor(&cfgDesc)) {
    if (verbose) Serial.println("[USB] Не удалось прочитать Configuration Descriptor.");
    return false;
  }
  Serial.printf("[USB] Интерфейс: класс=%02X подкласс=%02X протокол=%02X, эндпоинтов=%d\n",
                cfgDesc.interface.bInterfaceClass, cfgDesc.interface.bInterfaceSubClass,
                cfgDesc.interface.bInterfaceProtocol, cfgDesc.interface.bNumEndpoints);

  if (!ch375SetConfiguration(cfgDesc.configuration.bConfigurationValue)) {
    if (verbose) Serial.println("[USB] Не удалось выставить Configuration.");
    return false;
  }

  if (!findBulkOutEndpoint(cfgDesc, &printerOutEndpoint, &printerOutMaxPacket)) {
    if (verbose) Serial.println("[USB] Не найден bulk OUT эндпоинт.");
    return false;
  }
  Serial.printf("[USB] Bulk OUT эндпоинт: 0x%02X, максимальный пакет: %u байт\n",
                printerOutEndpoint, printerOutMaxPacket);

  printerHasInEndpoint = findBulkInEndpoint(cfgDesc, &printerInEndpoint);
  if (printerHasInEndpoint) {
    Serial.printf("[USB] Bulk IN эндпоинт: 0x%02X (для опроса PJL-статуса)\n", printerInEndpoint);
  } else {
    Serial.println("[USB] Bulk IN эндпоинт не найден — опрос PJL-статуса недоступен.");
  }

  ch375ToggleSend = false;
  ch375ToggleReceive = false;
  return true;
}

void checkPrinterConnection() {
  static unsigned long lastCheckMs = 0;
  static unsigned long lastEnumAttemptMs = 0;
  static int enumFailStreak = 0;
  const unsigned long CHECK_INTERVAL_MS = 2000;
  if (millis() - lastCheckMs < CHECK_INTERVAL_MS) return;
  lastCheckMs = millis();

  bool alive = ch375CheckExist(0x5A);
  if (alive && !printerReady) {
    // Пока принтер физически выключен, CH375 всё равно жив и отвечает —
    // без бэкоффа мы долбили бы полную энумерацию (и лог) каждые 2с
    // бесконечно. После нескольких неудач подряд разрежаем попытки и
    // логируем только первую и затем изредка, а не каждый раз.
    unsigned long retryIntervalMs = (enumFailStreak >= 3) ? 10000 : 2000;
    if (millis() - lastEnumAttemptMs < retryIntervalMs) return;
    lastEnumAttemptMs = millis();

    bool verbose = (enumFailStreak == 0) || (enumFailStreak % 18 == 0); // напоминание примерно раз в 3 минуты
    if (verbose) Serial.println("[USB] CH375 отвечает, пробуем enumeration принтера...");
    printerReady = initPrinter(verbose);
    if (printerReady) {
      Serial.println("[USB] Принтер готов.");
      enumFailStreak = 0;
    } else {
      enumFailStreak++;
    }
    recomputeIdleLedState();
  } else if (!alive && printerReady) {
    Serial.println("[USB] Связь с CH375 потеряна — сбрасываем состояние принтера.");
    printerReady = false;
    printerHasInEndpoint = false;
    printerStatusCode = -1;
    printerStatusDisplay = "";
    printerStatusOnline = false;
    printerManufacturer = ""; printerModel = ""; printerSerial = ""; printerUsbAddress = 0;
    enumFailStreak = 0;
    recomputeIdleLedState();
  } else if (alive && printerReady) {
    enumFailStreak = 0;
  }
}

bool sendToPrinter(const uint8_t *data, size_t len) {
  if (!printerReady) return false;
  size_t offset = 0;
  while (offset < len) {
    size_t chunk = len - offset;
    if (chunk > printerOutMaxPacket) chunk = printerOutMaxPacket;
    if (chunk > 255) chunk = 255;
    if (!ch375DoBulkOutTransfer(printerOutEndpoint, data + offset, (uint8_t)chunk)) {
      Serial.println("[USB] Ошибка bulk-передачи — прерываем задание.");
      printerReady = false;
      return false;
    }
    offset += chunk;
  }
  return true;
}

// ==================== RGB LED ====================

// Неблокирующая индикация: чистая функция без задержек, считает фазу
// мигания через millis(). Вызывается на каждой итерации loop() и внутри
// всех блокирующих циклов (ожидание WiFi, AP-портал, передача задания),
// чтобы мигание не "подвисало" вместе с остальным кодом.
void updateLed() {
  LedState effective = resetHeldOverrideLed ? LED_STATE_RESET_HELD : currentLedState;

  uint32_t color = 0;
  uint32_t altColor = 0; // если не 0 — мигание идёт между color и altColor, не гаснет совсем
  bool blink = false;
  uint32_t periodMs = 300;

  switch (effective) {
    case LED_STATE_WIFI_CONNECTING:
      color = pixel.Color(0, 255, 0);
      altColor = pixel.Color(255, 0, 0);
      blink = true; periodMs = 300;
      break;
    case LED_STATE_AP_CONFIG:
      color = pixel.Color(0, 80, 255);
      blink = true; periodMs = 400;
      break;
    case LED_STATE_WAITING_PRINTER:
      color = pixel.Color(255, 0, 0);
      blink = false;
      break;
    case LED_STATE_IDLE_READY:
      color = pixel.Color(0, 255, 0);
      blink = false;
      break;
    case LED_STATE_PRINTING:
      color = pixel.Color(0, 255, 0);
      blink = true; periodMs = 150;
      break;
    case LED_STATE_RESET_HELD:
      color = pixel.Color(255, 0, 0);
      blink = true; periodMs = 200;
      break;
    case LED_STATE_PRINTER_ERROR:
      color = pixel.Color(255, 200, 0);
      blink = true; periodMs = 350;
      break;
    case LED_STATE_OFF:
    default:
      color = 0;
      blink = false;
      break;
  }

  bool on = true;
  if (blink) on = (millis() % (periodMs * 2)) < periodMs;

  uint32_t finalColor;
  if (blink && altColor != 0) finalColor = on ? color : altColor;
  else finalColor = on ? color : 0;

  pixel.setPixelColor(0, finalColor);
  pixel.show();
}

// Пересчитывает "фоновое" состояние по актуальному факту WiFi+принтера.
// Вызывается после: подключения к WiFi, изменения состояния принтера,
// обновления PJL-статуса, завершения задания печати.
void recomputeIdleLedState() {
  if (WiFi.status() != WL_CONNECTED) return; // о подключении сигнализирует сам connectWiFi()
  if (!printerReady) { currentLedState = LED_STATE_WAITING_PRINTER; return; }
  // CODE=0 — единственный однозначный маркер "всё в порядке" у PJL HP;
  // любое другое значение (включая -1 "статус не определён", если
  // опрос ещё не проходил или принтер не поддерживает bulk IN) — сигнал
  // не начинать новую печать, пока не прояснится.
  if (printerHasInEndpoint && printerStatusCode != 0) { currentLedState = LED_STATE_PRINTER_ERROR; return; }
  currentLedState = LED_STATE_IDLE_READY;
}

// Принтер готов принять новое задание: аппаратно на связи и не занят
// другим заданием прямо сейчас. ВАЖНО: сюда намеренно НЕ подмешан PJL-
// статус — опрос через bulk IN пока не проверен настолько, чтобы им
// блокировать реальную печать (SET_ENDP6, используемый для toggle-бита
// приёма, не задокументирован официально и может работать не так, как
// предполагалось). Статус остаётся чисто информационным — для LED и
// дашборда, а не как автоматический блокиратор заданий.
bool printerCanAcceptJob() {
  return printerReady && !jobInProgress;
}

// ==================== PJL: ОПРОС СТАТУСА ПРИНТЕРА ====================

int lastLoggedPjlCode = -999;
String lastLoggedPjlDisplay = "\x01"; // заведомо непохоже на реальный DISPLAY

void pollPrinterStatus() {
  if (!printerReady || !printerHasInEndpoint || jobInProgress) return;
  if (millis() - lastPjlPollMs < 2000) return; // раз в пару секунд, как на ESP32-S3
  lastPjlPollMs = millis();

  const char query[] = "\x1B%-12345X@PJL INFO STATUS\n\x1B%-12345X";
  if (!sendToPrinter((const uint8_t *)query, sizeof(query) - 1)) return;

  String response;
  unsigned long start = millis();
  uint8_t buf[64];
  while (millis() - start < 2000) {
    uint8_t got = 0;
    if (!ch375DoBulkInTransfer(printerInEndpoint, buf, sizeof(buf), &got, 250)) break;
    if (got == 0) break;
    for (uint8_t i = 0; i < got; i++) response += (char)buf[i];
    if (got < sizeof(buf)) break; // короткий пакет = конец ответа
  }

  if (response.length() == 0) {
    printerStatusCode = -1; // отсутствие ответа тоже считается проблемой, не оставляем старый статус
    if (lastLoggedPjlCode != printerStatusCode) {
      lastLoggedPjlCode = printerStatusCode;
      Serial.println("[PJL] Нет ответа на @PJL INFO STATUS — статус не определён.");
    }
    recomputeIdleLedState();
    return;
  }

  int codeIdx = response.indexOf("CODE=");
  if (codeIdx >= 0) printerStatusCode = response.substring(codeIdx + 5).toInt();
  int dispIdx = response.indexOf("DISPLAY=\"");
  if (dispIdx >= 0) {
    int endQuote = response.indexOf('"', dispIdx + 9);
    if (endQuote > dispIdx) printerStatusDisplay = response.substring(dispIdx + 9, endQuote);
  }
  printerStatusOnline = response.indexOf("ONLINE=TRUE") >= 0;
  recomputeIdleLedState();

  static String lastLoggedDisplay = "\x01"; // заведомо непохоже на реальный DISPLAY
  bool changed = (printerStatusCode != lastLoggedPjlCode) || (printerStatusDisplay != lastLoggedDisplay);
  if (changed) {
    lastLoggedPjlCode = printerStatusCode;
    lastLoggedDisplay = printerStatusDisplay;
    if (printerStatusCode == 0) {
      Serial.printf("[PJL] CODE=0 -> в норме | Сырой ответ (%u байт): %s\n", response.length(), response.c_str());
    } else {
      Serial.printf("[PJL] CODE=%d -> ЕСТЬ ПРОБЛЕМА | Сырой ответ (%u байт): %s\n",
                    printerStatusCode, response.length(), response.c_str());
    }
  }
}

// ==================== КНОПКА СБРОСА ====================

void checkResetButton() {
  bool down = (digitalRead(RESET_BUTTON_PIN) == LOW);
  resetHeldOverrideLed = down;
  if (down) {
    if (resetButtonPressStart == 0) {
      resetButtonPressStart = millis();
      resetButtonLastReport = resetButtonPressStart;
      Serial.printf("[RESET] Кнопка нажата, удерживайте %lu с для сброса WiFi-настроек...\n",
                    (unsigned long)RESET_HOLD_MS / 1000);
    } else {
      unsigned long held = millis() - resetButtonPressStart;
      if (millis() - resetButtonLastReport > 5000) {
        resetButtonLastReport = millis();
        Serial.printf("[RESET] Удержание: %lu / %lu с\n",
                      (unsigned long)held / 1000, (unsigned long)RESET_HOLD_MS / 1000);
      }
      if (held >= RESET_HOLD_MS) {
        Serial.println("[RESET] Сброс WiFi-настроек и перезагрузка!");
        clearWifiCredentials();
        delay(300);
        ESP.restart();
      }
    }
  } else {
    resetButtonPressStart = 0;
  }
  updateLed(); // вызывается на каждой итерации loop() — LED никогда не "подвисает"
}

// ==================== WiFi ====================

bool connectWiFi() {
  if (savedSsid.length() == 0) return false;
  Serial.printf("[WiFi] Подключение к \"%s\" ...\n", savedSsid.c_str());
  currentLedState = LED_STATE_WIFI_CONNECTING;
  WiFi.mode(WIFI_STA);

  if (savedIsEnterprise) {
    esp_eap_client_set_identity((uint8_t *)savedEapIdentity.c_str(), savedEapIdentity.length());
    esp_eap_client_set_username((uint8_t *)savedEapUsername.c_str(), savedEapUsername.length());
    esp_eap_client_set_password((uint8_t *)savedEapPassword.c_str(), savedEapPassword.length());
    esp_wifi_sta_enterprise_enable();
    WiFi.begin(savedSsid.c_str());
  } else {
    WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
  }

  int retries = 30;
  while (WiFi.status() != WL_CONNECTED && retries-- > 0) {
    updateLed();
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Подключено. IP: %s\n", WiFi.localIP().toString().c_str());
    recomputeIdleLedState();
    return true;
  }
  Serial.println("[WiFi] Не удалось подключиться.");
  return false;
}

// ==================== SSDP / UPnP ====================

void ssdpBegin() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char uuidBuf[40];
  snprintf(uuidBuf, sizeof(uuidBuf), "38323636-4558-4530-b8a5-%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ssdpUuid = String(uuidBuf);

  ssdpUdp.beginMulticast(IPAddress(239, 255, 255, 250), SSDP_PORT);
  Serial.printf("[SSDP] Слушаю multicast 239.255.255.250:%u, UUID=%s\n", SSDP_PORT, ssdpUuid.c_str());
}

void ssdpSendNotify() {
  String notify = "NOTIFY * HTTP/1.1\r\n"
                   "HOST: 239.255.255.250:1900\r\n"
                   "CACHE-CONTROL: max-age=1800\r\n"
                   "LOCATION: http://" + WiFi.localIP().toString() + "/description.xml\r\n"
                   "SERVER: ESP32/1.0 UPnP/1.0 PrintServer/1.0\r\n"
                   "NT: upnp:rootdevice\r\n"
                   "NTS: ssdp:alive\r\n"
                   "USN: uuid:" + ssdpUuid + "::upnp:rootdevice\r\n\r\n";
  ssdpUdp.beginPacket(IPAddress(239, 255, 255, 250), SSDP_PORT);
  ssdpUdp.write((const uint8_t *)notify.c_str(), notify.length());
  ssdpUdp.endPacket();
  Serial.println("[SSDP] Отправлен NOTIFY ssdp:alive.");
}

void ssdpLoop() {
  int packetSize = ssdpUdp.parsePacket();
  if (packetSize > 0) {
    char buf[600];
    int len = ssdpUdp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = 0;
      String req(buf);
      if (req.indexOf("M-SEARCH") >= 0 &&
          (req.indexOf("ssdp:all") >= 0 || req.indexOf("upnp:rootdevice") >= 0 || req.indexOf("ssdp:discover") >= 0)) {
        IPAddress remoteIp = ssdpUdp.remoteIP();
        uint16_t remotePort = ssdpUdp.remotePort();
        String resp = "HTTP/1.1 200 OK\r\n"
                       "CACHE-CONTROL: max-age=1800\r\n"
                       "EXT:\r\n"
                       "LOCATION: http://" + WiFi.localIP().toString() + "/description.xml\r\n"
                       "SERVER: ESP32/1.0 UPnP/1.0 PrintServer/1.0\r\n"
                       "ST: upnp:rootdevice\r\n"
                       "USN: uuid:" + ssdpUuid + "::upnp:rootdevice\r\n\r\n";
        ssdpUdp.beginPacket(remoteIp, remotePort);
        ssdpUdp.write((const uint8_t *)resp.c_str(), resp.length());
        ssdpUdp.endPacket();
        Serial.printf("[SSDP] M-SEARCH от %s:%u -> отвечаю\n", remoteIp.toString().c_str(), remotePort);
      }
    }
  }
  if (millis() - lastSsdpNotifyMs > 60000) {
    lastSsdpNotifyMs = millis();
    ssdpSendNotify();
  }
}

// ==================== OLED (опционально) ====================

#ifdef ENABLE_OLED_DISPLAY
void oledInit() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledAvailable = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledAvailable) {
    Serial.println("[OLED] Дисплей не найден на I2C (0x3C) — пропускаю.");
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("CH375 Print Server");
  oled.println("Запуск...");
  oled.display();
}

void oledUpdate() {
  if (!oledAvailable) return;
  static unsigned long lastUpdateMs = 0;
  if (millis() - lastUpdateMs < 1000) return;
  lastUpdateMs = millis();

  oled.clearDisplay();
  oled.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    oled.println(WiFi.localIP().toString());
  } else {
    oled.println("WiFi: нет связи");
  }
  oled.print("Printer: ");
  oled.println(printerReady ? "ready" : "off");
  if (printerReady && printerStatusDisplay.length() > 0) {
    oled.println(printerStatusDisplay);
  }
  oled.printf("Jobs: %lu", totalJobsPrinted);
  oled.display();
}
#endif

// ==================== ВЕБ-ПОРТАЛ ====================

bool checkPortalAuth() {
  if (portalPassword.length() == 0) return true;
  if (!configServer.authenticate("admin", portalPassword.c_str())) {
    configServer.requestAuthentication();
    return false;
  }
  return true;
}

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Print Server</title>
<style>
*{box-sizing:border-box}
body{font-family:sans-serif;max-width:900px;margin:24px auto;padding:0 16px;
     background:#0f172a;color:#e2e8f0}
header{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:18px}
h1{font-size:1.3rem;margin:0}
.sub{color:#94a3b8;font-size:.85rem;margin-top:2px}
#langBtn{background:#7f1d1d;border:none;color:#fecaca;padding:7px 16px;
         border-radius:8px;cursor:pointer;font-weight:600}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media (max-width:640px){.grid{grid-template-columns:1fr}}
.card{background:#1e293b;border:1px solid #334155;border-radius:10px;padding:16px 18px}
.card h3{margin:0 0 10px;font-size:.78rem;letter-spacing:.06em;color:#94a3b8;text-transform:uppercase}
.row{display:flex;justify-content:space-between;padding:5px 0;font-size:.92rem;border-bottom:1px solid #24324a}
.row:last-child{border-bottom:none}
.row span:first-child{color:#94a3b8}
.pill{padding:2px 10px;border-radius:999px;font-size:.8rem;font-weight:600}
.pill.ok{background:#14532d;color:#4ade80}
.pill.bad{background:#450a0a;color:#f87171}
.pjl{margin-top:10px;font-size:.75rem;color:#64748b;white-space:pre-wrap;font-family:monospace}
input{width:100%;padding:9px;margin-top:6px;box-sizing:border-box;
      background:#0f172a;border:1px solid #475569;border-radius:8px;color:#e2e8f0}
button.wide{margin-top:10px;width:100%;padding:10px;background:#3b82f6;color:#fff;
       border:none;border-radius:8px;font-size:.92rem;cursor:pointer}
.hint{color:#64748b;font-size:.75rem;margin-top:6px}
a.btn{display:block;text-align:center;margin-top:10px;padding:10px;
      background:#7f1d1d;color:#fecaca;border-radius:8px;text-decoration:none;font-weight:600}
.instructions{margin-top:16px}
.instructions .grid{grid-template-columns:1fr 1fr}
.instructions ol{padding-left:20px;font-size:.88rem;line-height:1.6}
.instructions code{background:#0f172a;padding:1px 5px;border-radius:4px}
.foot{color:#64748b;font-size:.78rem;margin-top:10px;text-align:center}
</style></head><body>
<header>
  <div>
    <h1 id="title">ESP32 Print Server</h1>
    <div class="sub" id="subtitle">WiFi → USB bridge</div>
  </div>
  <button id="langBtn" onclick="toggleLang()">RU</button>
</header>

<div class="grid">
  <div class="card">
    <h3 id="l_printerCard">Printer</h3>
    <div class="row"><span id="l_status">Status</span><span id="v_pStatus">-</span></div>
    <div class="row"><span id="l_manufacturer">Manufacturer</span><span id="v_manufacturer">-</span></div>
    <div class="row"><span id="l_model">Model</span><span id="v_model">-</span></div>
    <div class="pjl" id="v_pjlRaw"></div>
  </div>
  <div class="card">
    <h3 id="l_usbCard">USB Connection</h3>
    <div class="row"><span id="l_status2">Status</span><span id="v_uStatus">-</span></div>
    <div class="row"><span id="l_devAddr">Device address</span><span id="v_devAddr">-</span></div>
    <div class="row"><span id="l_bulkOut">Bulk OUT</span><span id="v_bulkOut">-</span></div>
    <div class="row"><span id="l_bulkIn">Bulk IN</span><span id="v_bulkIn">-</span></div>
    <div class="row"><span id="l_pktSize">Packet size</span><span id="v_pktSize">-</span></div>
  </div>
  <div class="card">
    <h3 id="l_wifiCard">Wi-Fi</h3>
    <div class="row"><span id="l_network">Network</span><span id="v_network">-</span></div>
    <div class="row"><span id="l_ip">IP address</span><span id="v_ip">-</span></div>
    <div class="row"><span id="l_mac">MAC</span><span id="v_mac">-</span></div>
    <div class="row"><span id="l_signal">Signal</span><span id="v_signal">-</span></div>
    <a class="btn" href="/wifi" id="wifiBtn">Reset WiFi settings</a>
  </div>
  <div class="card">
    <h3 id="l_sysCard">System</h3>
    <div class="row"><span id="l_uptime">Uptime</span><span id="v_uptime">-</span></div>
    <div class="row"><span id="l_freemem">Free memory</span><span id="v_freemem">-</span></div>
    <label id="l_portalPass" style="display:block;margin-top:12px;font-size:.85rem;color:#94a3b8">Portal password</label>
    <input type="password" id="portalPassInput" placeholder="••••••••">
    <div class="hint" id="l_portalHint">Leave blank and save to remove protection. After setting it, the browser will ask for a login/password next time you open the portal (login: admin).</div>
    <button class="wide" onclick="savePortalPassword()" id="l_savePass">Save password</button>
  </div>
</div>

<div class="card instructions">
  <h3 id="l_addCard">Adding the printer on a computer</h3>
  <div class="grid">
    <div>
      <div style="font-weight:600;margin-bottom:6px" id="l_win">Windows 10 / 11</div>
      <ol id="l_winSteps"></ol>
    </div>
    <div>
      <div style="font-weight:600;margin-bottom:6px" id="l_mac">macOS</div>
      <ol id="l_macSteps"></ol>
    </div>
  </div>
  <div class="foot" id="l_footnote">Both protocols (Raw and LPR) work at the same time on this board — there is no difference in print quality.</div>
</div>

<script>
const dict = {
  en: {
    title:"ESP32 Print Server", subtitle:"WiFi → USB bridge", printerCard:"Printer",
    status:"Status", manufacturer:"Manufacturer", model:"Model", serial:"Serial number",
    usbCard:"USB Connection", status2:"Status", devAddr:"Device address", bulkOut:"Bulk OUT",
    bulkIn:"Bulk IN", pktSize:"Packet size", wifiCard:"Wi-Fi", network:"Network", ip:"IP address",
    mac:"MAC", signal:"Signal", wifiBtn:"Reset WiFi settings", sysCard:"System", uptime:"Uptime",
    freemem:"Free memory", portalPass:"Portal password",
    portalHint:"Leave blank and save to remove protection. After setting it, the browser will ask for a login/password next time you open the portal (login: admin).",
    savePass:"Save password", addCard:"Adding the printer on a computer", win:"Windows 10 / 11", mac2:"macOS",
    footnote:"Both protocols (Raw and LPR) work at the same time on this board — there is no difference in print quality.",
    ready:"Ready", notready:"Not ready", connected:"Connected", disconnected:"Disconnected",
    winSteps:[
      "First install the official driver for <b>{model}</b>",
      "Settings → Bluetooth & devices → Printers & scanners → <b>Add device</b>",
      "Click <b>\"Add manually\"</b> → <b>\"Add a printer using a TCP/IP address or hostname\"</b>",
      "Device type — <b>TCP/IP Device</b>, hostname/IP — <b>{ip}</b>",
      "Protocol — either <b>HP Jetdirect - Socket</b> (Raw, 9100) or <b>LPD</b> (LPR, 515)",
      "If Windows won't let you pick LPR directly — add it via \"Special\", protocol LPR",
      "Make sure <b>\"SNMP Status Enabled\"</b> is unchecked — the printer doesn't support it",
      "For the driver, pick the already-installed <b>{model}</b>"
    ],
    macSteps:[
      "First install the official driver for <b>{model}</b>",
      "System Settings → Printers & Scanners → <b>Add Printer</b>",
      "Switch to the <b>IP</b> tab",
      "Protocol — <b>HP Jetdirect - Socket</b> (Raw, 9100) or <b>LPD</b> (LPR, 515)",
      "Address — <b>{ip}</b>; for LPD, type any name into the \"Queue\" field",
      "Under Use, pick <b>Select Software</b> → find <b>{model}</b>"
    ]
  },
  ru: {
    title:"ESP32 Print Server", subtitle:"WiFi → USB мост", printerCard:"Принтер",
    status:"Статус", manufacturer:"Производитель", model:"Модель", serial:"Серийный номер",
    usbCard:"USB-подключение", status2:"Статус", devAddr:"Адрес устройства", bulkOut:"Bulk OUT",
    bulkIn:"Bulk IN", pktSize:"Размер пакета", wifiCard:"Wi-Fi", network:"Сеть", ip:"IP-адрес",
    mac:"MAC", signal:"Сигнал", wifiBtn:"Сбросить WiFi-настройки", sysCard:"Система", uptime:"Аптайм",
    freemem:"Свободная память", portalPass:"Пароль портала",
    portalHint:"Оставьте пустым и сохраните, чтобы убрать защиту. После установки браузер при следующем открытии портала запросит логин/пароль (логин: admin).",
    savePass:"Сохранить пароль", addCard:"Добавление принтера на компьютере", win:"Windows 10 / 11", mac2:"macOS",
    footnote:"Оба протокола (Raw и LPR) работают на этой плате одновременно — разницы в качестве печати нет.",
    ready:"Готов", notready:"Не готов", connected:"Подключен", disconnected:"Отключен",
    winSteps:[
      "Сначала установите официальный драйвер для <b>{model}</b>",
      "Параметры → Bluetooth и устройства → Принтеры и сканеры → <b>Добавить устройство</b>",
      "Нажмите <b>«Добавить вручную»</b> → <b>«Добавить принтер по TCP/IP-адресу или имени узла»</b>",
      "Тип устройства — <b>Устройство TCP/IP</b>, имя узла/IP — <b>{ip}</b>",
      "Протокол — либо <b>HP Jetdirect - Socket</b> (Raw, 9100), либо <b>LPD</b> (LPR, 515)",
      "Если Windows не даёт выбрать LPR напрямую — добавьте через «Особый», протокол LPR",
      "Убедитесь, что <b>«Включить состояние SNMP»</b> снято — принтер это не поддерживает",
      "В качестве драйвера выберите уже установленный <b>{model}</b>"
    ],
    macSteps:[
      "Сначала установите официальный драйвер для <b>{model}</b>",
      "Системные настройки → Принтеры и сканеры → <b>Добавить принтер</b>",
      "Перейдите на вкладку <b>IP</b>",
      "Протокол — <b>HP Jetdirect - Socket</b> (Raw, 9100) или <b>LPD</b> (LPR, 515)",
      "Адрес — <b>{ip}</b>; для LPD впишите любое имя в поле «Очередь»",
      "В поле «Использовать» выберите <b>«Выбрать ПО»</b> → найдите <b>{model}</b>"
    ]
  }
};
let lang = localStorage.getItem('psLang') || 'ru';
let lastModel = 'HP LaserJet P2015 Series';
let lastIp = '-';

function fillSteps(listEl, steps){
  listEl.innerHTML = '';
  steps.forEach(s=>{
    const li = document.createElement('li');
    li.innerHTML = s.replace(/{model}/g, lastModel).replace(/{ip}/g, lastIp);
    listEl.appendChild(li);
  });
}

function applyLang(){
  const d = dict[lang];
  for (const key in d) {
    const el = document.getElementById('l_' + key);
    if (el && typeof d[key] === 'string') el.innerHTML = d[key];
  }
  document.getElementById('title').textContent = d.title;
  document.getElementById('subtitle').textContent = d.subtitle + ' · ' + lastIp;
  document.getElementById('langBtn').textContent = lang === 'ru' ? 'EN' : 'RU';
  fillSteps(document.getElementById('l_winSteps'), d.winSteps);
  fillSteps(document.getElementById('l_macSteps'), d.macSteps);
}

function toggleLang(){ lang = lang === 'ru' ? 'en' : 'ru'; localStorage.setItem('psLang', lang); applyLang(); }

function savePortalPassword(){
  const val = document.getElementById('portalPassInput').value;
  fetch('/portal-password', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'newpass=' + encodeURIComponent(val)})
    .then(()=>{ document.getElementById('portalPassInput').value=''; });
}

function fmtUptime(s){
  const h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = s%60;
  return h+'h '+m+'m '+sec+'s';
}

function refresh(){
  fetch('/api/status').then(r=>r.json()).then(j=>{
    const d = dict[lang];
    lastIp = j.ip; lastModel = j.model || lastModel;
    document.getElementById('subtitle').textContent = d.subtitle + ' · ' + j.ip;

    document.getElementById('v_pStatus').innerHTML = j.printerReady
      ? '<span class="pill ok">'+d.ready+'</span>' : '<span class="pill bad">'+d.notready+'</span>';
    document.getElementById('v_manufacturer').textContent = j.manufacturer || '-';
    document.getElementById('v_model').textContent = j.model || '-';
    document.getElementById('v_pjlRaw').textContent = j.printerReady
      ? '@PJL INFO STATUS\nCODE=' + j.printerStatusCode + '\nDISPLAY="' + j.printerStatus + '"\nONLINE=' + (j.printerOnline ? 'TRUE' : 'FALSE')
      : '';

    document.getElementById('v_uStatus').innerHTML = j.printerReady
      ? '<span class="pill ok">'+d.connected+'</span>' : '<span class="pill bad">'+d.disconnected+'</span>';
    document.getElementById('v_devAddr').textContent = j.usbAddress || '-';
    document.getElementById('v_bulkOut').textContent = j.bulkOut || '-';
    document.getElementById('v_bulkIn').textContent = j.bulkIn || '-';
    document.getElementById('v_pktSize').textContent = j.packetSize ? (j.packetSize + ' bytes') : '-';

    document.getElementById('v_network').textContent = j.ssid || '-';
    document.getElementById('v_ip').textContent = j.ip;
    document.getElementById('v_mac').textContent = j.mac;
    document.getElementById('v_signal').textContent = j.rssi + ' dBm';

    document.getElementById('v_uptime').textContent = fmtUptime(j.uptime);
    document.getElementById('v_freemem').textContent = Math.round(j.freeHeap/1024) + ' KB';

    fillSteps(document.getElementById('l_winSteps'), d.winSteps);
    fillSteps(document.getElementById('l_macSteps'), d.macSteps);
  }).catch(()=>{});
}
applyLang();
refresh();
setInterval(refresh, 4000);
</script>
</body></html>
)HTML";

const char WIFI_SETUP_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Настройка WiFi</title>
<style>
body{font-family:sans-serif;max-width:360px;margin:40px auto;padding:0 16px;
     background:#0f172a;color:#e2e8f0}
h2{font-size:1.2rem;margin-bottom:16px}
label{display:block;margin-top:14px;font-size:.85rem;color:#94a3b8}
input,select{width:100%;padding:10px;margin-top:6px;box-sizing:border-box;
      background:#1e293b;border:1px solid #475569;border-radius:8px;color:#e2e8f0}
button{margin-top:20px;width:100%;padding:12px;background:#3b82f6;color:#fff;
       border:none;border-radius:8px;font-size:1rem;cursor:pointer}
.chk{display:flex;align-items:center;gap:8px;margin-top:14px}
.chk input{width:auto}
#entFields{display:none}
small{color:#64748b}
</style></head><body>
<h2>ESP32-C6 + CH375 Print Server</h2>
<form action="/save" method="POST">
<label>Сеть (SSID) <small id="scanStatus"></small></label>
<select id="ssidSelect" onchange="document.getElementById('ssid').value=this.value">
  <option value="">-- просканировать --</option>
</select>
<input type="text" name="ssid" id="ssid" placeholder="или введите вручную" required>
<label>Пароль</label><input type="password" name="pass" id="pass">
<div class="chk">
  <input type="checkbox" name="enterprise" id="enterprise" onchange="toggleEnt()">
  <label style="margin:0">WPA2-Enterprise</label>
</div>
<div id="entFields">
  <label>Identity</label><input type="text" name="eapId">
  <label>Username</label><input type="text" name="eapUser">
  <label>Enterprise-пароль</label><input type="password" name="eapPass">
</div>
<button type="submit">Сохранить и подключиться</button>
</form>
<script>
function toggleEnt(){
  document.getElementById('entFields').style.display =
    document.getElementById('enterprise').checked ? 'block' : 'none';
}
fetch('/scan').then(r=>r.json()).then(list=>{
  const sel = document.getElementById('ssidSelect');
  document.getElementById('scanStatus').textContent = '(' + list.length + ' найдено)';
  list.forEach(s=>{
    const opt = document.createElement('option');
    opt.value = s; opt.textContent = s;
    sel.appendChild(opt);
  });
}).catch(()=>{ document.getElementById('scanStatus').textContent = '(сканирование не удалось)'; });
</script>
</body></html>
)HTML";

void handleRoot() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!checkPortalAuth()) return;
    configServer.send(200, "text/html", FPSTR(DASHBOARD_HTML));
  } else {
    configServer.send(200, "text/html", FPSTR(WIFI_SETUP_HTML));
  }
}

void handleWifiPage() {
  configServer.send(200, "text/html", FPSTR(WIFI_SETUP_HTML));
}

void handleScan() {
  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "\"" + WiFi.SSID(i) + "\"";
  }
  json += "]";
  WiFi.scanDelete();
  configServer.send(200, "application/json", json);
}

void handleSave() {
  String ssid = configServer.arg("ssid");
  String pass = configServer.arg("pass");
  bool enterprise = configServer.hasArg("enterprise");
  String eapId = configServer.arg("eapId");
  String eapUser = configServer.arg("eapUser");
  String eapPass = configServer.arg("eapPass");
  if (ssid.length() == 0) {
    configServer.send(400, "text/plain", "SSID не может быть пустым");
    return;
  }
  saveWifiCredentials(ssid, pass, enterprise, eapId, eapUser, eapPass);
  configServer.send(200, "text/html",
    "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head>"
    "<body style=\"font-family:sans-serif;background:#0f172a;color:#e2e8f0\">"
    "<h3>Сохранено. Перезагрузка...</h3></body></html>");
  delay(800);
  ESP.restart();
}

void handleApiStatus() {
  if (!checkPortalAuth()) return;
  char hexBuf[8];
  String json = "{";
  json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"ssid\":\"" + WiFi.SSID() + "\",";
  json += "\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("-")) + "\",";
  json += "\"mac\":\"" + WiFi.macAddress() + "\",";
  json += "\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + ",";
  json += "\"printerReady\":" + String(printerReady ? "true" : "false") + ",";
  json += "\"manufacturer\":\"" + printerManufacturer + "\",";
  json += "\"model\":\"" + printerModel + "\",";
  json += "\"serial\":\"" + printerSerial + "\",";
  json += "\"usbAddress\":" + String(printerUsbAddress) + ",";
  snprintf(hexBuf, sizeof(hexBuf), "0x%X", printerOutEndpoint);
  json += "\"bulkOut\":\"" + String(printerReady ? hexBuf : "") + "\",";
  snprintf(hexBuf, sizeof(hexBuf), "0x%X", printerInEndpoint | (printerHasInEndpoint ? 0x80 : 0));
  json += "\"bulkIn\":\"" + String(printerHasInEndpoint ? hexBuf : "") + "\",";
  json += "\"packetSize\":" + String(printerReady ? printerOutMaxPacket : 0) + ",";
  json += "\"printerStatus\":\"" + printerStatusDisplay + "\",";
  json += "\"printerStatusCode\":" + String(printerStatusCode) + ",";
  json += "\"printerOnline\":" + String(printerStatusOnline ? "true" : "false") + ",";
  json += "\"totalJobs\":" + String(totalJobsPrinted) + ",";
  json += "\"totalBytes\":" + String(totalBytesPrinted) + ",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap());
  json += "}";
  configServer.send(200, "application/json", json);
}

void handleDescriptionXml() {
  String ip = WiFi.localIP().toString();
  String xml = "<?xml version=\"1.0\"?>\r\n"
               "<root xmlns=\"urn:schemas-upnp-org:device-1-0\">\r\n"
               "<specVersion><major>1</major><minor>0</minor></specVersion>\r\n"
               "<URLBase>http://" + ip + "/</URLBase>\r\n"
               "<device>\r\n"
               "<deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>\r\n"
               "<friendlyName>ESP32-C6 Print Server</friendlyName>\r\n"
               "<manufacturer>DIY</manufacturer>\r\n"
               "<manufacturerURL>https://github.com</manufacturerURL>\r\n"
               "<modelName>CH375 Print Server</modelName>\r\n"
               "<modelDescription>WiFi to USB printer bridge (ESP32-C6 + CH375B)</modelDescription>\r\n"
               "<presentationURL>http://" + ip + "/</presentationURL>\r\n"
               "<UDN>uuid:" + ssdpUuid + "</UDN>\r\n"
               "</device>\r\n"
               "</root>\r\n";
  configServer.send(200, "text/xml", xml);
}

void handleSetPortalPassword() {
  if (!checkPortalAuth()) return;
  String newPass = configServer.arg("newpass");
  savePortalPassword(newPass);
  configServer.send(200, "text/plain", "OK");
}

void webPortalBegin() {
  configServer.on("/", HTTP_GET, handleRoot);
  configServer.on("/wifi", HTTP_GET, handleWifiPage);
  configServer.on("/save", HTTP_POST, handleSave);
  configServer.on("/scan", HTTP_GET, handleScan);
  configServer.on("/api/status", HTTP_GET, handleApiStatus);
  configServer.on("/description.xml", HTTP_GET, handleDescriptionXml);
  configServer.on("/portal-password", HTTP_POST, handleSetPortalPassword);
  configServer.begin();
  Serial.println("[Portal] Веб-портал запущен (работает постоянно).");
}

void servePrintClients();
void serveLprClients();

void startConfigPortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("CH375-PrintServer-Setup", "12345678");
  Serial.println("[AP] Точка доступа поднята: 192.168.4.1");
  currentLedState = LED_STATE_AP_CONFIG;
  webPortalBegin();
  while (true) {
    configServer.handleClient();
    checkResetButton(); // также обновляет LED каждую итерацию
    checkPrinterConnection();
    servePrintClients();
    serveLprClients();
    delay(2);
  }
}

// ==================== TCP: RAW ====================

void handleRawClient(WiFiClient &client) {
  Serial.printf("[RAW] Клиент подключился: %s\n", client.remoteIP().toString().c_str());
  jobInProgress = true;
  currentLedState = LED_STATE_PRINTING;
  unsigned long jobStartMs = millis();
  size_t jobBytes = 0;
  uint8_t buf[512];

  while (true) {
    int avail = client.available();
    if (avail > 0) {
      int toRead = avail > (int)sizeof(buf) ? (int)sizeof(buf) : avail;
      int n = client.read(buf, toRead);
      if (n <= 0) break;
      jobBytes += n;
      if (!sendToPrinter(buf, n)) {
        Serial.println("[RAW] Ошибка отправки на принтер — прерываем задание.");
        break;
      }
      updateLed();
      continue;
    }
    if (!client.connected()) break;
    updateLed();
    delay(1);
  }
  client.stop();

  if (printerReady) {
    if (ch375DoBulkOutTransfer(printerOutEndpoint, nullptr, 0)) {
      Serial.println("[RAW] Отправлен завершающий пакет нулевой длины (ZLP).");
    } else {
      Serial.println("[RAW] Не удалось отправить ZLP — принтер может не завершить задание.");
    }
  }

  unsigned long elapsedMs = millis() - jobStartMs;
  float kbPerSec = elapsedMs > 0 ? (jobBytes / 1024.0f) / (elapsedMs / 1000.0f) : 0;
  Serial.printf("[RAW] Клиент отключился. Job: %u байт.\n", (unsigned)jobBytes);
  Serial.printf("[RAW] Время передачи: %lu мс (%.1f КБ/с).\n", elapsedMs, kbPerSec);

  if (jobBytes > 0) { totalJobsPrinted++; totalBytesPrinted += jobBytes; }
  jobInProgress = false;
  recomputeIdleLedState();
}

void servePrintClients() {
  WiFiClient client = rawServer.available();
  if (!client) return;
  if (!printerCanAcceptJob()) {
    Serial.println("[RAW] Принтер не готов принять задание (не подключен/занят/ошибка) — отклоняю соединение.");
    client.stop();
    return;
  }
  handleRawClient(client);
}

// ==================== TCP: LPR/LPD (порт 515) ====================

bool lprReadLine(WiFiClient &client, String &line, uint32_t timeoutMs = 5000) {
  line = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') return true;
      line += c;
    } else {
      if (!client.connected()) return false;
      delay(1);
    }
  }
  return false;
}

void lprSendAck(WiFiClient &client) {
  uint8_t zero = 0;
  client.write(&zero, 1);
}

void handleLprClient(WiFiClient &client) {
  Serial.printf("[LPR] Клиент подключился: %s\n", client.remoteIP().toString().c_str());
  jobInProgress = true;
  currentLedState = LED_STATE_PRINTING;

  int cmd = -1;
  {
    unsigned long start = millis();
    while (millis() - start < 3000 && !client.available()) {
      if (!client.connected()) { client.stop(); jobInProgress = false; return; }
      delay(1);
    }
    if (client.available()) cmd = client.read();
  }
  if (cmd != 0x02) {
    client.stop();
    jobInProgress = false;
    return;
  }
  String queue;
  lprReadLine(client, queue);
  Serial.printf("[LPR] Очередь: \"%s\"\n", queue.c_str());
  lprSendAck(client);

  size_t totalJobBytes = 0;

  while (client.connected() || client.available()) {
    updateLed();
    if (!client.available()) {
      if (!client.connected()) break;
      delay(1);
      continue;
    }
    int sub = client.read();
    if (sub == 0x01) {
      Serial.println("[LPR] Клиент запросил abort.");
      break;
    }
    if (sub != 0x02 && sub != 0x03) break;

    String header;
    if (!lprReadLine(client, header)) break;
    int sp = header.indexOf(' ');
    long declaredSize = sp > 0 ? header.substring(0, sp).toInt() : -1;

    if (sub == 0x02) {
      // Control file — нам не нужно его содержимое, просто вычитываем и отбрасываем.
      Serial.printf("[LPR] Control file, размер=%ld: %s\n", declaredSize, header.c_str());
      lprSendAck(client);
      long remaining = declaredSize;
      uint8_t buf[256];
      while (remaining > 0) {
        if (!client.available()) {
          if (!client.connected()) break;
          delay(1);
          continue;
        }
        int toRead = remaining < (long)sizeof(buf) ? (int)remaining : (int)sizeof(buf);
        int n = client.read(buf, toRead);
        if (n <= 0) break;
        remaining -= n;
      }
      unsigned long tstart = millis();
      while (millis() - tstart < 500) {
        if (client.available()) { client.read(); break; }
        if (!client.connected()) break;
        delay(1);
      }
      lprSendAck(client);
    } else {
      // Data file — реальное задание печати. Windows LPR-клиент печально
      // известен тем, что может врать про размер (шлёт огромное число,
      // не зная реальный объём заранее) — поэтому не доверяем размеру
      // слепо: если он выглядит неправдоподобно большим, переключаемся
      // в потоковый режим (льём до отключения клиента), и каждый кусок
      // сразу уходит в принтер, а не после накопления целого чанка.
      Serial.printf("[LPR] Data file (задание печати), размер=%ld: %s\n", declaredSize, header.c_str());
      lprSendAck(client);

      bool streaming = (declaredSize <= 0) || (declaredSize > 100L * 1024 * 1024);
      long remaining = declaredSize;
      size_t received = 0;
      uint8_t buf[512];
      while (true) {
        if (!client.available()) {
          if (!client.connected()) break;
          if (!streaming && remaining <= 0) break;
          delay(1);
          continue;
        }
        int want = streaming ? (int)sizeof(buf) : (remaining < (long)sizeof(buf) ? (int)remaining : (int)sizeof(buf));
        if (want <= 0) break;
        int n = client.read(buf, want);
        if (n <= 0) break;
        received += n;
        totalJobBytes += n;
        if (!streaming) remaining -= n;
        if (!sendToPrinter(buf, n)) {
          Serial.println("[LPR] Ошибка отправки на принтер — прерываем задание.");
          break;
        }
        if (!streaming && remaining <= 0) break;
      }
      Serial.printf("[LPR] Data file принят: %u байт.\n", (unsigned)received);

      unsigned long tstart = millis();
      while (millis() - tstart < 500) {
        if (client.available()) { client.read(); break; }
        if (!client.connected()) break;
        delay(1);
      }
      lprSendAck(client);
    }
  }

  if (printerReady) {
    ch375DoBulkOutTransfer(printerOutEndpoint, nullptr, 0);
  }
  client.stop();
  jobInProgress = false;
  recomputeIdleLedState();
  Serial.printf("[LPR] Job завершён. Байт задания печати: %u\n", (unsigned)totalJobBytes);
  Serial.println("[LPR] Соединение закрыто.");

  if (totalJobBytes > 0) { totalJobsPrinted++; totalBytesPrinted += totalJobBytes; }
}

void serveLprClients() {
  WiFiClient client = lprServer.available();
  if (!client) return;
  if (!printerCanAcceptJob()) {
    client.stop();
    return;
  }
  handleLprClient(client);
}

// ==================== SETUP / LOOP ====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32-C6 + CH375B Print Server (параллельный режим) ===");

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CH375_INT_PIN, INPUT_PULLUP);

  pixel.begin();
  pixel.setBrightness(60);
  pixel.show(); // гасим на всякий случай перед стартом

  pinMode(CH375_A0_PIN, OUTPUT);
  digitalWrite(CH375_A0_PIN, LOW);
  pinMode(CH375_RD_PIN, OUTPUT);
  digitalWrite(CH375_RD_PIN, HIGH);
  pinMode(CH375_WR_PIN, OUTPUT);
  digitalWrite(CH375_WR_PIN, HIGH);
  ch375BusSetInput();

  ch375ResetChip();
  delay(100);

#ifdef ENABLE_OLED_DISPLAY
  oledInit();
#endif

  Serial.println("[USB] Проверяем связь с CH375 (параллельный режим)...");
  {
    uint8_t testByte = 0x5A;
    ch375SendCommand(CH375_CMD_CHECK_EXIST);
    ch375SendData(testByte);
    uint8_t resp = 0;
    ch375Receive(resp, 500);
    if (resp == (uint8_t)(~testByte)) {
      Serial.printf("[USB] CH375 версия чипа: 0x%02X. Связь в порядке.\n", ch375GetChipVersion());
    } else {
      Serial.printf("[USB] CH375 не отвечает — отправили 0x%02X, ожидали 0x%02X, получили 0x%02X.\n",
                    testByte, (uint8_t)(~testByte), resp);
      Serial.println("[USB] Проверьте: TXD чипа на GND (режим Parallel), CS# на GND, распайку D0-D7/RD#/WR#/A0.");
    }
  }

  loadWifiCredentials();
  bool connected = connectWiFi();
  if (!connected) startConfigPortal(); // не вернётся, кроме как через reboot

  if (MDNS.begin("ESP32-C6-PrintServer")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] Доступно как ESP32-C6-PrintServer.local");
  }

  webPortalBegin();
  ssdpBegin();

  rawServer.begin();
  lprServer.begin();
  Serial.printf("[TCP] RAW print server слушает порт %u\n", RAW_PORT);
  Serial.printf("[TCP] LPR/LPD print server слушает порт %u\n", LPR_PORT);
}

void loop() {
  checkResetButton();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Соединение потеряно, переподключаемся...");
    if (!connectWiFi()) startConfigPortal();
  }
  checkPrinterConnection();
  pollPrinterStatus();
  configServer.handleClient();
  ssdpLoop();
  servePrintClients();
  serveLprClients();
#ifdef ENABLE_OLED_DISPLAY
  oledUpdate();
#endif
  delay(2);
}
