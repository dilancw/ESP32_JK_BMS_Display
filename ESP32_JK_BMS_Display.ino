/**
   A BLE client example that is rich in capabilities.
   There is a lot new capabilities implemented.
   author unknown
   updated by chegewara
*/

#include "BLEDevice.h"

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//#include "BLEScan.h"
//JK_B2A24S15P, Address: c8:47:80:03:b5:b5, manufacturer data: 650b88a0c8478003b5b5, serviceNotifyUuid: 0000ffe0-0000-1000-8000-00805f9b34fb, serviceNotifyUuid: 0000fee7-0000-1000-8000-00805f9b34fb

// The remote service we wish to connect to.
static BLEUUID serviceNotifyUuid("0000ffe0-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID    charReadUUID("ffe1");
static boolean notify_received = false;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristicRead;
static BLEAdvertisedDevice* myDevice;

static uint32_t last_cell_update = 0;

static const uint8_t COMMAND_CELL_INFO = 0x96;
static const uint8_t COMMAND_DEVICE_INFO = 0x97;
//static const uint8_t COMMAND_DEVICE_INFO = 0x96;
static const uint8_t FRAME_VERSION_JK04 = 0x01;
static const uint8_t FRAME_VERSION_JK02_24S = 0x02;
static const uint8_t FRAME_VERSION_JK02_32S = 0x03;

static const uint16_t MIN_RESPONSE_SIZE = 298;
static const uint16_t MAX_RESPONSE_SIZE = 320;



float cell_voltages[8];
float average_cell_voltage_sensor, delta_cell_voltage_sensor;
float total_voltage, current , power;
float state_of_charge_sensor, capacity_remaining_sensor;



#define TAG 0
std::vector<uint8_t> frame_buffer_;

#ifdef ESP_LOGE
#undef ESP_LOGE
#endif
#ifdef ESP_LOGW
#undef ESP_LOGW
#endif
#ifdef ESP_LOGI
#undef ESP_LOGI
#endif
#ifdef ESP_LOGD
#undef ESP_LOGD
#endif
#ifdef ESP_LOGV
#undef ESP_LOGV
#endif

#define ESP_LOGE(tag, ...) print_helper(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) print_helper(tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) print_helper(tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) print_helper(tag, __VA_ARGS__)
#define ESP_LOGCONFIG(tag, ...) print_helper(tag, __VA_ARGS__)
#define ESP_LOGV(tag, ...) print_helper(tag, __VA_ARGS__)
#define ESP_LOGVV(tag, ...) print_helper(tag, __VA_ARGS__)

std::string str_snprintf(const char *fmt, size_t len, ...) {
  std::string str;
  va_list args;

  str.resize(len);
  va_start(args, len);
  size_t out_length = vsnprintf(&str[0], len + 1, fmt, args);
  va_end(args);

  if (out_length < len)
    str.resize(out_length);

  return str;
}
std::string to_string(int value) {
  return str_snprintf("%d", 32, value);  // NOLINT
}
std::string to_string(long value) {
  return str_snprintf("%ld", 32, value);  // NOLINT
}
std::string to_string(long long value) {
  return str_snprintf("%lld", 32, value);  // NOLINT
}
std::string to_string(unsigned value) {
  return str_snprintf("%u", 32, value);  // NOLINT
}
std::string to_string(unsigned long value) {
  return str_snprintf("%lu", 32, value);  // NOLINT
}
std::string to_string(unsigned long long value) {
  return str_snprintf("%llu", 32, value);  // NOLINT
}
std::string to_string(float value) {
  return str_snprintf("%f", 32, value);
}
std::string to_string(double value) {
  return str_snprintf("%f", 32, value);
}
std::string to_string(long double value) {
  return str_snprintf("%Lf", 32, value);
}

static char format_hex_char(uint8_t v) {
  return v >= 10 ? 'a' + (v - 10) : '0' + v;
}
std::string format_hex(const uint8_t *data, size_t length) {
  std::string ret;
  ret.resize(length * 2);
  for (size_t i = 0; i < length; i++) {
    ret[2 * i] = format_hex_char((data[i] & 0xF0) >> 4);
    ret[2 * i + 1] = format_hex_char(data[i] & 0x0F);
  }
  return ret;
}
std::string format_hex(const std::vector<uint8_t> &data) {
  return format_hex(data.data(), data.size());
}

static char format_hex_pretty_char(uint8_t v) {
  return v >= 10 ? 'A' + (v - 10) : '0' + v;
}
std::string format_hex_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  for (size_t i = 0; i < length; i++) {
    ret[3 * i] = format_hex_pretty_char((data[i] & 0xF0) >> 4);
    ret[3 * i + 1] = format_hex_pretty_char(data[i] & 0x0F);
    if (i != length - 1)
      ret[3 * i + 2] = '.';
  }
  if (length > 4)
    return ret + " (" + to_string(length) + ")";
  return ret;
}
std::string format_hex_pretty(const std::vector<uint8_t> &data) {
  return format_hex_pretty(data.data(), data.size());
}

void print_helper(int tag, const char* format, ...)
{
  char buffer[1024];
  va_list args;
  va_start (args, format);
  sprintf (buffer, format, args);
  Serial.print(buffer);
  va_end (args);
}
float ieee_float_(uint32_t f) {
  static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
  float ret;
  memcpy(&ret, &f, sizeof(float));
  return ret;
}
void publish_state_(String string , float val)
{
  Serial.print(string);
  Serial.print(" = ");
  Serial.println(val);
}
void publish_state_i(uint8_t i, String string , float val)
{
  Serial.print(string);
  Serial.print(i);
  Serial.print(" = ");
  Serial.println(val);
}


void decode_jk02_cell_info_(const std::vector<uint8_t> &data) {
  auto jk_get_16bit = [&](size_t i) -> uint16_t { return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0); };
  auto jk_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(jk_get_16bit(i + 2)) << 16) | (uint32_t(jk_get_16bit(i + 0)) << 0);
  };

  uint8_t frame_version = FRAME_VERSION_JK02_24S;
  uint8_t offset = 0;
  if (true) {
    frame_version = FRAME_VERSION_JK02_32S;
    offset = 16;
  }

  //ESP_LOGI(TAG, "Cell info frame (version %d, %d bytes) received", frame_version, data.size());
  // ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), 150).c_str());
  // ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front() + 150, data.size() - 150).c_str());

  // 6 example responses (128+128+44 = 300 bytes per frame)
  //
  //
  // 55.AA.EB.90.02.8C.FF.0C.01.0D.01.0D.FF.0C.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.01.0D.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.FF.FF.00.00.00.0D.00.00.00.00.9D.01.96.01.8C.01.87.01.84.01.84.01.83.01.84.01.85.01.81.01.83.01.86.01.82.01.82.01.83.01.85.01.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.03.D0.00.00.00.00.00.00.00.00
  // 00.00.BE.00.BF.00.D2.00.00.00.00.00.00.54.8E.0B.01.00.68.3C.01.00.00.00.00.00.3D.04.00.00.64.00.79.04.CA.03.10.00.01.01.AA.06.00.00.00.00.00.00.00.00.00.00.00.00.07.00.01.00.00.00.D5.02.00.00.00.00.AE.D6.3B.40.00.00.00.00.58.AA.FD.FF.00.00.00.01.00.02.00.00.EC.E6.4F.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00
  // 00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.CD
  //
  // 55.AA.EB.90.02.8D.FF.0C.01.0D.01.0D.01.0D.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.FF.0C.FF.0C.01.0D.01.0D.01.0D.01.0D.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.FF.FF.00.00.00.0D.00.00.00.00.9D.01.96.01.8C.01.87.01.84.01.84.01.83.01.84.01.85.01.81.01.83.01.86.01.82.01.82.01.83.01.85.01.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.04.D0.00.00.00.00.00.00.00.00
  // 00.00.BE.00.BF.00.D2.00.00.00.00.00.00.54.8E.0B.01.00.68.3C.01.00.00.00.00.00.3D.04.00.00.64.00.79.04.CA.03.10.00.01.01.AA.06.00.00.00.00.00.00.00.00.00.00.00.00.07.00.01.00.00.00.D5.02.00.00.00.00.AE.D6.3B.40.00.00.00.00.58.AA.FD.FF.00.00.00.01.00.02.00.00.F0.E6.4F.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00
  // 00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.D3
  //
  // 55.AA.EB.90.02.8E.FF.0C.01.0D.01.0D.FF.0C.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.FF.0C.FF.0C.01.0D.01.0D.01.0D.01.0D.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.FF.FF.00.00.00.0D.00.00.00.00.9D.01.96.01.8C.01.87.01.84.01.84.01.83.01.84.01.85.01.81.01.83.01.86.01.82.01.82.01.83.01.85.01.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.04.D0.00.00.00.00.00.00.00.00
  // 00.00.BE.00.BF.00.D2.00.00.00.00.00.00.54.8E.0B.01.00.68.3C.01.00.00.00.00.00.3D.04.00.00.64.00.79.04.CA.03.10.00.01.01.AA.06.00.00.00.00.00.00.00.00.00.00.00.00.07.00.01.00.00.00.D5.02.00.00.00.00.AE.D6.3B.40.00.00.00.00.58.AA.FD.FF.00.00.00.01.00.02.00.00.F5.E6.4F.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00
  // 00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.D6
  //
  // 55.AA.EB.90.02.91.FF.0C.FF.0C.01.0D.FF.0C.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.01.0D.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.FF.FF.00.00.00.0D.00.00.00.00.9D.01.96.01.8C.01.87.01.84.01.84.01.83.01.84.01.85.01.81.01.83.01.86.01.82.01.82.01.83.01.85.01.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.01.D0.00.00.00.00.00.00.00.00
  // 00.00.BF.00.C0.00.D2.00.00.00.00.00.00.54.8E.0B.01.00.68.3C.01.00.00.00.00.00.3D.04.00.00.64.00.79.04.CC.03.10.00.01.01.AA.06.00.00.00.00.00.00.00.00.00.00.00.00.07.00.01.00.00.00.D5.02.00.00.00.00.AE.D6.3B.40.00.00.00.00.58.AA.FD.FF.00.00.00.01.00.02.00.00.01.E7.4F.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00
  // 00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.E7
  //
  // 55.AA.EB.90.02.92.01.0D.01.0D.01.0D.01.0D.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.01.0D.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.FF.FF.00.00.00.0D.00.00.00.00.9D.01.96.01.8C.01.87.01.84.01.84.01.83.01.84.01.85.01.81.01.83.01.86.01.82.01.82.01.83.01.85.01.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.03.D0.00.00.00.00.00.00.00.00
  // 00.00.BF.00.C0.00.D2.00.00.00.00.00.00.54.8E.0B.01.00.68.3C.01.00.00.00.00.00.3D.04.00.00.64.00.79.04.CC.03.10.00.01.01.AA.06.00.00.00.00.00.00.00.00.00.00.00.00.07.00.01.00.00.00.D5.02.00.00.00.00.AE.D6.3B.40.00.00.00.00.58.AA.FD.FF.00.00.00.01.00.02.00.00.06.E7.4F.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00
  // 00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.F8
  //
  // 55.AA.EB.90.02.93.FF.0C.01.0D.01.0D.01.0D.01.0D.01.0D.FF.0C.01.0D.01.0D.01.0D.FF.0C.FF.0C.01.0D.01.0D.01.0D.01.0D.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.FF.FF.00.00.00.0D.00.00.00.00.9D.01.96.01.8C.01.87.01.84.01.84.01.83.01.84.01.85.01.81.01.83.01.86.01.82.01.82.01.83.01.85.01.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.04.D0.00.00.00.00.00.00.00.00
  // 00.00.BE.00.C0.00.D2.00.00.00.00.00.00.54.8E.0B.01.00.68.3C.01.00.00.00.00.00.3D.04.00.00.64.00.79.04.CD.03.10.00.01.01.AA.06.00.00.00.00.00.00.00.00.00.00.00.00.07.00.01.00.00.00.D5.02.00.00.00.00.AE.D6.3B.40.00.00.00.00.58.AA.FD.FF.00.00.00.01.00.02.00.00.0A.E7.4F.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00
  // 00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.00.F8
  //
  // Byte Len  Payload                Content              Coeff.      Unit        Example value
  // 0     2   0x55 0xAA 0xEB 0x90    Header
  // 4     1   0x02                   Record type
  // 5     1   0x8C                   Frame counter
  // 6     2   0xFF 0x0C              Voltage cell 01       0.001        V
  // 8     2   0x01 0x0D              Voltage cell 02       0.001        V
  // 10    2   0x01 0x0D              Voltage cell 03       0.001        V
  // ...
  uint8_t cells = 24 + (offset / 2);
  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  for (uint8_t i = 0; i < cells; i++) {
    float cell_voltage = (float) jk_get_16bit(i * 2 + 6) * 0.001f;
    float cell_resistance = (float) jk_get_16bit(i * 2 + 64 + offset) * 0.001f;
    if (cell_voltage > 0 && cell_voltage < min_cell_voltage) {
      min_cell_voltage = cell_voltage;
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
    }
    //publish_state_i(i, "cell voltage", cell_voltage);
    if (i < 8)
    {
      cell_voltages[i] = cell_voltage;
    }
    //publish_state_i(i, "cell_resistance", cell_resistance);
  }
  //publish_state_("min_cell_voltage_sensor_", min_cell_voltage);
  //publish_state_("max_cell_voltage_sensor_", max_cell_voltage);

  // 54    4   0xFF 0xFF 0x00 0x00    Enabled cells bitmask
  //           0x0F 0x00 0x00 0x00    4 cells enabled
  //           0xFF 0x00 0x00 0x00    8 cells enabled
  //           0xFF 0x0F 0x00 0x00    12 cells enabled
  //           0xFF 0x1F 0x00 0x00    13 cells enabled
  //           0xFF 0xFF 0x00 0x00    16 cells enabled
  //           0xFF 0xFF 0xFF 0x00    24 cells enabled
  //           0xFF 0xFF 0xFF 0xFF    32 cells enabled
  // ESP_LOGV(TAG, "Enabled cells bitmask: 0x%02X 0x%02X 0x%02X 0x%02X", data[54 + offset], data[55 + offset],data[56 + offset], data[57 + offset]);

  // 58    2   0x00 0x0D              Average Cell Voltage  0.001        V
  //publish_state_("average_cell_voltage_sensor_", (float) jk_get_16bit(58 + offset) * 0.001f);
  average_cell_voltage_sensor = (float) jk_get_16bit(58 + offset) * 0.001f;
  // 60    2   0x00 0x00              Delta Cell Voltage    0.001        V
  //publish_state_("delta_cell_voltage_sensor_", (float) jk_get_16bit(60 + offset) * 0.001f);
  delta_cell_voltage_sensor = (float) jk_get_16bit(60 + offset) * 0.001f;
  // 62    1   0x00                   Max voltage cell      1
  //publish_state_("max_voltage_cell_sensor_", (float) data[62 + offset] + 1);
  // 63    1   0x00                   Min voltage cell      1
  //publish_state_("min_voltage_cell_sensor_", (float) data[63 + offset] + 1);
  // 64    2   0x9D 0x01              Resistance Cell 01    0.001        Ohm
  // 66    2   0x96 0x01              Resistance Cell 02    0.001        Ohm
  // 68    2   0x8C 0x01              Resistance Cell 03    0.001        Ohm
  // ...
  // 110   2   0x00 0x00              Resistance Cell 24    0.001        Ohm

  offset = offset * 2;

  // 112   2   0x00 0x00              Unknown112
  //  if (frame_version == FRAME_VERSION_JK02_32S) {
  //    publish_state_("power_tube_temperature_sensor_", (float) ((int16_t) jk_get_16bit(112 + offset)) * 0.1f);
  //  } else {
  //    ESP_LOGD(TAG, "Unknown112: 0x%02X 0x%02X", data[112 + offset], data[113 + offset]);
  //  }

  // 114   4   0x00 0x00 0x00 0x00    Wire resistance warning bitmask (each bit indicates a warning per cell / wire)
  //  ESP_LOGD(TAG, "Wire resistance warning bitmask: 0x%02X 0x%02X 0x%02X 0x%02X", data[114 + offset], data[115 + offset],
  //           data[116 + offset], data[117 + offset]);

  // 118   4   0x03 0xD0 0x00 0x00    Battery voltage       0.001        V
  total_voltage = (float) jk_get_32bit(118 + offset) * 0.001f;
  //publish_state_("total_voltage_sensor_", total_voltage);

  // 122   4   0x00 0x00 0x00 0x00    Battery power         0.001        W
  // 126   4   0x00 0x00 0x00 0x00    Charge current        0.001        A
  current = (float) ((int32_t) jk_get_32bit(126 + offset)) * 0.001f;
  //publish_state_("current_sensor_", (float) ((int32_t) jk_get_32bit(126 + offset)) * 0.001f);

  // Don't use byte 122 because it's unsigned
  // float power = (float) ((int32_t) jk_get_32bit(122 + offset)) * 0.001f;
  power = total_voltage * current;
  //  publish_state_("power_sensor_", power);
  //  publish_state_("charging_power_sensor_", std::max(0.0f, power));               // 500W vs 0W -> 500W
  //  publish_state_("discharging_power_sensor_", std::abs(std::min(0.0f, power)));  // -500W vs 0W -> 500W

  // 130   2   0xBE 0x00              Temperature Sensor 1  0.1          °C
  //  publish_state_("temperatures_[0].temperature_sensor_",
  //                 (float) ((int16_t) jk_get_16bit(130 + offset)) * 0.1f);

  // 132   2   0xBF 0x00              Temperature Sensor 2  0.1          °C
  //  publish_state_("temperatures_[1].temperature_sensor_",
  //                 (float) ((int16_t) jk_get_16bit(132 + offset)) * 0.1f);
  //
  //  // 134   2   0xD2 0x00              MOS Temperature       0.1          °C
  //  if (frame_version == FRAME_VERSION_JK02_32S) {
  //    uint16_t raw_errors_bitmask = (uint16_t(data[134 + offset]) << 8) | (uint16_t(data[134 + 1 + offset]) << 0);
  //    publish_state_("errors_bitmask_sensor_", (float) raw_errors_bitmask);
  //    //publish_state_("errors_text_sensor_", error_bits_to_string_(raw_errors_bitmask));
  //  } else {
  //    publish_state_("power_tube_temperature_sensor_", (float) ((int16_t) jk_get_16bit(134 + offset)) * 0.1f);
  //  }

  // 136   2   0x00 0x00              System alarms
  //           0x00 0x01                Charge overtemperature               0000 0000 0000 0001
  //           0x00 0x02                Charge undertemperature              0000 0000 0000 0010
  //           0x00 0x04                                                     0000 0000 0000 0100
  //           0x00 0x08                Cell Undervoltage                    0000 0000 0000 1000
  //           0x00 0x10                                                     0000 0000 0001 0000
  //           0x00 0x20                                                     0000 0000 0010 0000
  //           0x00 0x40                                                     0000 0000 0100 0000
  //           0x00 0x80                                                     0000 0000 1000 0000
  //           0x01 0x00                                                     0000 0001 0000 0000
  //           0x02 0x00                                                     0000 0010 0000 0000
  //           0x04 0x00                Cell count is not equal to settings  0000 0100 0000 0000
  //           0x08 0x00                Current sensor anomaly               0000 1000 0000 0000
  //           0x10 0x00                Cell Over Voltage                    0001 0000 0000 0000
  //           0x20 0x00                                                     0010 0000 0000 0000
  //           0x40 0x00                                                     0100 0000 0000 0000
  //           0x80 0x00                                                     1000 0000 0000 0000
  //
  //           0x14 0x00                Cell Over Voltage +                  0001 0100 0000 0000
  //                                    Cell count is not equal to settings
  //           0x04 0x08                Cell Undervoltage +                  0000 0100 0000 1000
  //                                    Cell count is not equal to settings
  //  if (frame_version != FRAME_VERSION_JK02_32S) {
  //    uint16_t raw_errors_bitmask = (uint16_t(data[136 + offset]) << 8) | (uint16_t(data[136 + 1 + offset]) << 0);
  //    publish_state_("errors_bitmask_sensor_", (float) raw_errors_bitmask);
  //    //publish_state_(errors_text_sensor_, error_bits_to_string_(raw_errors_bitmask));
  //  }
  //
  //  // 138   2   0x00 0x00              Balance current      0.001         A
  //  publish_state_("balancing_current_sensor_", (float) ((int16_t) jk_get_16bit(138 + offset)) * 0.001f);
  //
  // 140   1   0x00                   Balancing action                   0x00: Off
  //                                                                     0x01: Charging balancer
  //                                                                     0x02: Discharging balancer
  //  publish_state_("balancing_sensor_", (data[140 + offset]));
  //  publish_state_("balancing_binary_sensor_", (data[140 + offset] != 0x00));
  //ESP_LOGD(TAG, " Balancing indicator (legacy): %s", (data[140 + offset] != 0x00));

  // 141   1   0x54                   State of charge in   1.0           %
  //publish_state_("state_of_charge_sensor_", (float) data[141 + offset]);
  state_of_charge_sensor =  (float) data[141 + offset];
  // 142   4   0x8E 0x0B 0x01 0x00    Capacity_Remain      0.001         Ah
  //publish_state_("capacity_remaining_sensor_", (float) jk_get_32bit(142 + offset) * 0.001f);
  capacity_remaining_sensor = jk_get_32bit(142 + offset) * 0.001f;
  // 146   4   0x68 0x3C 0x01 0x00    Nominal_Capacity     0.001         Ah
  //  publish_state_("total_battery_capacity_setting_sensor_", (float) jk_get_32bit(146 + offset) * 0.001f);
  //
  //  // 150   4   0x00 0x00 0x00 0x00    Cycle_Count          1.0
  //  publish_state_("charging_cycles_sensor_", (float) jk_get_32bit(150 + offset));
  //
  //  // 154   4   0x3D 0x04 0x00 0x00    Cycle_Capacity       0.001         Ah
  //  publish_state_("total_charging_cycle_capacity_sensor_", (float) jk_get_32bit(154 + offset) * 0.001f);
  //
  //  // 158   1   0x64                   SOH                  1.0           %
  //  ESP_LOGD(TAG, "State of health: %d %%", data[158 + offset]);
  //
  //  // 159   1   0x00                   Precharge
  //  ESP_LOGD(TAG, "Precharge: %s", (data[159 + offset]));
  //
  //  // 160   2   0x79 0x04              User alarm
  //  ESP_LOGD(TAG, "User alarm: 0x%02X 0x%02X (always 0xC5 0x09?)", data[160 + offset], data[161 + offset]);
  //
  //  // 162   4   0xCA 0x03 0x10 0x00    Total runtime in seconds           s
  //  publish_state_("total_runtime_sensor_", (float) jk_get_32bit(162 + offset));
  //  publish_state_("total_runtime_formatted_text_sensor_", (float)(jk_get_32bit(162 + offset)));
  //
  //  // 166   1   0x01                   Charging mosfet enabled                      0x00: off, 0x01: on
  //  publish_state_("charging_binary_sensor_", (bool) data[166 + offset]);
  //
  //  // 167   1   0x01                   Discharging mosfet enabled                   0x00: off, 0x01: on
  //  publish_state_("discharging_binary_sensor_", (bool) data[167 + offset]);
  //
  //  // 168   1   0x01                   Precharging                                  0x00: off, 0x01: on
  //  publish_state_("precharging_binary_sensor_", (bool) data[168 + offset]);
  //
  //  // 169   1   0x01                   Balancer working                             0x00: off, 0x01: on
  //  // publish_state_("balancing_binary_sensor_", (bool) data[169 + offset]);
  //  ESP_LOGD(TAG, " Balancing indicator (new): %s", ((bool) data[169 + offset]));
  //
  //  ESP_LOGD(TAG, "Discharge overcurrent protection release timer: %d", jk_get_16bit(170 + offset));
  //  ESP_LOGD(TAG, "Discharge short circuit protection release timer: %d", jk_get_16bit(172 + offset));
  //  ESP_LOGD(TAG, "Charge overcurrent protection release timer: %d", jk_get_16bit(174 + offset));
  //  ESP_LOGD(TAG, "Charge short circuit protection release timer: %d", jk_get_16bit(176 + offset));
  //  ESP_LOGD(TAG, "Undervoltage protection release timer: %d", jk_get_16bit(178 + offset));
  //  ESP_LOGD(TAG, "Overvoltage protection release timer: %d", jk_get_16bit(180 + offset));
  //  ESP_LOGD(TAG, "Temperature sensor absent bitmask: %d", jk_get_16bit(182 + offset));
  //  // bit0: Mosfet temperature sensor
  //  // bit1: Temperature sensor 1
  //  // bit2: Temperature sensor 2
  //  // bit3: Temperature sensor 3
  //  // bit4: Temperature sensor 4
  //  // bit5: Temperature sensor 5
  //
  //  ESP_LOGD(TAG, "Heating: %s", ((bool) data[183 + offset]));
  //  ESP_LOGD(TAG, "Time emergency: %d s", jk_get_16bit(186 + offset));
  //  ESP_LOGD(TAG, "Discharge current correction factor: %d", jk_get_16bit(188 + offset));
  //  ESP_LOGD(TAG, "Charging current sensor voltage: %.3f", jk_get_16bit(190 + offset) * 0.001f);
  //  ESP_LOGD(TAG, "Discharging current sensor voltage: %.3f", jk_get_16bit(192 + offset) * 0.001f);
  //  ESP_LOGD(TAG, "Battery voltage correction factor: %f", (float) jk_get_32bit(194 + offset) * 1.0f);
  //
  //  ESP_LOGD(TAG, "Battery voltage: %.3f", (float) ieee_float_(jk_get_32bit(202 + offset)));
  //  ESP_LOGD(TAG, "Heating current: %d mA", jk_get_16bit(204 + offset));
  //
  //  ESP_LOGD(TAG, "Charger Plugged: %s", ((bool) data[213 + offset]));
  //  ESP_LOGD(TAG, "Temperature sensor 3: %.1f", (float) jk_get_16bit(222 + offset));
  //  ESP_LOGD(TAG, "Temperature sensor 4: %.1f", (float) jk_get_16bit(224 + offset));
  //  ESP_LOGD(TAG, "Temperature sensor 5: %.1f", (float) jk_get_16bit(226 + offset));
  //  ESP_LOGD(TAG, "Time enter sleep: %u s", jk_get_32bit(238 + offset));
  //  ESP_LOGD(TAG, "PCL Module State: %s", ((bool) data[242 + offset]));
  //
  //  // 192   1   0x01                   Heating status          0x00: off, 0x01: on
  //  publish_state_("heating_binary_sensor_", (bool) data[192 + offset]);
  //
  //  // 204   2   0x01 0xFD              Heating current         0.001         A
  //  publish_state_("heating_current_sensor_", (float) ((int16_t) jk_get_16bit(204 + offset)) * 0.001f);
  //
  //  if (frame_version == FRAME_VERSION_JK02_32S) {
  //    uint16_t raw_emergency_time_countdown = jk_get_16bit(186 + offset);
  //    ESP_LOGI(TAG, "  Emergency switch: %s", (raw_emergency_time_countdown > 0));
  //    publish_state_("emergency_switch_", raw_emergency_time_countdown > 0);
  //    publish_state_("emergency_time_countdown_sensor_", (float) raw_emergency_time_countdown * 1.0f);
  //
  //    publish_state_("temperatures_[4].temperature_sensor_",
  //                   (float) ((int16_t) jk_get_16bit(222 + offset)) * 0.1f);
  //    publish_state_("temperatures_[3].temperature_sensor_",
  //                   (float) ((int16_t) jk_get_16bit(224 + offset)) * 0.1f);
  //    publish_state_("temperatures_[2].temperature_sensor_",
  //                   (float) ((int16_t) jk_get_16bit(226 + offset)) * 0.1f);
  //  }

  // 299   1   0xCD                   CRC

}
void decode_(const std::vector<uint8_t> &data) {
  //reset_online_status_tracker_();
  Serial.print("In decoding data");
  uint8_t frame_type = data[4];
  Serial.println(frame_type);
  Serial.println(format_hex_pretty(&data.front(), 150).c_str());
  switch (frame_type) {
    case 0x01:
      //decode_jk02_settings_(data);
      break;
    case 0x02:
      decode_jk02_cell_info_(data);
      last_cell_update = millis();
      break;
    case 0x03:
      //decode_device_info_(data);
      break;
    default:
      Serial.print("Unsupported message type (0x%02X)");
      Serial.println(data[4], HEX);
  }
}
void assemble_data(const uint8_t *data, uint16_t length) {
  //  Serial.println("In assmeble data");
  //  Serial.print("frame_buffer_ size");
  //  Serial.println(frame_buffer_.size());
  if (frame_buffer_.size() > MAX_RESPONSE_SIZE) {
    Serial.println("Frame dropped because of invalid length");
    frame_buffer_.clear();
  }

  // Flush buffer on every preamble
  if (data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
    frame_buffer_.clear();
    Serial.println("frame buffer clear");
  }

  frame_buffer_.insert(frame_buffer_.end(), data, data + length);
  //Serial.println(format_hex_pretty(&frame_buffer_.front(), 150).c_str());
  if (frame_buffer_.size() > MIN_RESPONSE_SIZE) {
    const uint8_t *raw = &frame_buffer_[0];
    // Even if the frame is 320 bytes long the CRC is at position 300 in front of 0xAA 0x55 0x90 0xEB
    const uint16_t frame_size = 300;  // frame_buffer_.size();

    uint8_t computed_crc = crc(raw, frame_size - 1);
    uint8_t remote_crc = raw[frame_size - 1];
    if (computed_crc != remote_crc) {
      Serial.println("CRC check failed! 0x%02X != 0x%02X");
      frame_buffer_.clear();
      return;
    }

    std::vector<uint8_t> data(frame_buffer_.begin(), frame_buffer_.end());

    decode_(data);
    frame_buffer_.clear();
  }
}


static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  //  Serial.print("Notify callback for characteristic ");
  //  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
  //    Serial.print("data: ");
  //    Serial.println((char*)pData);
  assemble_data(pData, length);
  notify_received = true;
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      Serial.println("onDisconnect");
    }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pServiceNotify = pClient->getService(serviceNotifyUuid);
  if (pServiceNotify == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceNotifyUuid.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristicRead = pServiceNotify->getCharacteristic(charReadUUID);
  if (pRemoteCharacteristicRead == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charReadUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  // if(pRemoteCharacteristicRead->canRead()) {
  //   std::string value = pRemoteCharacteristicRead->readValue();
  //   Serial.print("The characteristic value was: ");
  //   Serial.println(value.c_str());
  // }
  uint8_t enable_read_handle[] = {0x01, 0x00};

  pRemoteCharacteristicRead->writeValue(&enable_read_handle[0], 2);


  if (pRemoteCharacteristicRead->canNotify())
    pRemoteCharacteristicRead->registerForNotify(notifyCallback);
  connected = true;

  return true;
}
/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceNotifyUuid)) {

        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;

      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  lcd.init();
  lcd.backlight();
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
} // End of setup.

uint8_t crc(const uint8_t data[], const uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc + data[i];
  }
  return crc;
}


bool write_register(uint8_t address, uint32_t value, uint8_t length) {
  uint8_t frame[20];
  frame[0] = 0xAA;     // start sequence
  frame[1] = 0x55;     // start sequence
  frame[2] = 0x90;     // start sequence
  frame[3] = 0xEB;     // start sequence
  frame[4] = address;  // holding register
  frame[5] = length;   // size of the value in byte
  frame[6] = value >> 0;
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;

  //  frame[4] = 0x96;  // holding register
  //  frame[5] = 0x00;   // size of the value in byte
  //  frame[6] = 0x00;
  //  frame[7] = 0x00;
  //  frame[8] = 0x00;
  //  frame[9] = 0x00;

  frame[10] = 0x00;
  frame[11] = 0x00;
  frame[12] = 0x00;
  frame[13] = 0x00;
  frame[14] = 0x00;
  frame[15] = 0x00;
  frame[16] = 0x00;
  frame[17] = 0x00;
  frame[18] = 0x00;
  frame[19] = crc(frame, sizeof(frame) - 1);
  Serial.println("frame Generated");
  Serial.println(format_hex_pretty(&frame[0], 20).c_str());
  //delay(1000);

  pRemoteCharacteristicRead->writeValue(&frame[0], 20);
  Serial.println("write frame done");
  frame_buffer_.clear();
  //delay(1000);
  //ESP_LOGD(TAG, "Write register: %s", format_hex_pretty(frame, sizeof(frame)).c_str());
  //  auto status =
  //      esp_ble_gattc_write_char(parent_->get_gattc_if(), parent_->get_conn_id(), char_handle_,
  //                               sizeof(frame), frame, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  //
  //  if (status) {
  //    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", parent_->address_str().c_str(), status);
  //  }
  //
  //  return (status == 0);
  return true;
}

// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    Serial.println("Requesting cell info");
    if (notify_received == false)
    {
      Serial.println("Trying to get device info");
      write_register(COMMAND_DEVICE_INFO, 0x00000000, 0x00);
      delay(5000);
    } else {
      if ((millis() - last_cell_update) > 10000)
      {
        Serial.println("Trying to COMMAND_CELL_INFO");
        write_register(COMMAND_CELL_INFO, 0x00000000, 0x00);
      }
    }
    // Set the characteristic's value to be the array of bytes that is actually a string.

  } else if (doScan) {
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }

  delay(1000); // Delay a second between loops.
  display();
} // End of loop


void display()
{
  lcd.setCursor(0, 0);
  // print message

  for (int ii =0; ii < 8; ii++)
  {
    Serial.print("Cell ");
    Serial.print(ii);
    Serial.print(" ");
    Serial.println(cell_voltages[ii], 4);
  }
  Serial.print("Total Voltage = ");
  Serial.print( total_voltage);
  Serial.println("V");
  
  lcd.print(total_voltage);
  lcd.print("V ");
  
  Serial.print("current =");
  Serial.println(current, 2);
  Serial.print("A");

  lcd.print(current);
  lcd.print("A ");
  
  Serial.print("power =");
  Serial.println(power, 2);
  Serial.print("w");

  Serial.print("state_of_charge =");
  Serial.println(state_of_charge_sensor, 2);
  Serial.print("%");

   lcd.setCursor(0, 1);

  lcd.print(state_of_charge_sensor);
  lcd.print("% ");
  
  Serial.print("capacity_remaining_sensor =");
  Serial.println(capacity_remaining_sensor, 2);
  Serial.print("Ah");

    lcd.print(capacity_remaining_sensor);
  lcd.print("Ah ");

}
