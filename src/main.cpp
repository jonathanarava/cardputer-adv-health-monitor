/*
 * Cardputer ADV (Stamp-S3A) - sensor scanner + full EXT-header GPIO console
 *
 * Purpose: dump readings from every I2C sensor you've wired up, plus the raw
 * state of ALL 14 EXT 2.54-14P header positions (including the 5V/GND rails),
 * in the same physical order they appear on the header. Emits a self-
 * describing JSON line each cycle for the companion web dashboard, and a
 * human-readable block for plain-terminal use.
 *
 * Pinmap source (official): https://docs.m5stack.com/en/core/Cardputer-Adv
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <Adafruit_MPU6050.h>
#include <VL53L0X.h>
#include <SparkFun_BMI270_Arduino_Library.h>

// ---------------------------------------------------------------------
// Fixed I2C bus on Stamp-S3A. Shared by the onboard BMI270 IMU,
// ES8311 audio codec, and TCA8418 keyboard controller.
// ---------------------------------------------------------------------
static const uint8_t I2C_SDA_PIN = 8;
static const uint8_t I2C_SCL_PIN = 9;

// ---------------------------------------------------------------------
// EXT 2.54-14P header - ALL 14 physical positions, in header order.
// Matches the official M5 pinmap table exactly:
//   FUNC   PIN | LEFT  RIGHT | PIN  FUNC
//   RESET  G3  |  1     2   | 5VIN
//   INT    G4  |  3     4   | GND
//   BUSY   G6  |  5     6   | 5VOUT
//   SCK    G40 |  7     8   | G8   I2C_SDA
//   MOSI   G14 |  9    10   | G9   I2C_SCL
//   MISO   G39 | 11    12   | G13  UART_RX
//   CS     G5  | 13    14   | G15  UART_TX
// ---------------------------------------------------------------------
enum PinKind
{
  KIND_FREE,
  KIND_I2C,
  KIND_SPI_SD,
  KIND_UART,
  KIND_PWR_IN,
  KIND_PWR_OUT,
  KIND_GND
};

struct ExtPin
{
  uint8_t headerPos;     // 1-14, physical position on the connector
  const char *funcLabel; // silkscreen label from the M5 pinmap
  int8_t gpio;           // -1 if this position isn't a GPIO (power/gnd)
  PinKind kind;
  bool adc1; // ADC1-capable: reliable even with Wi-Fi/BT on
  bool adc2; // ADC2-capable: unreliable once Wi-Fi/BT is active
};

static const ExtPin extHeader[14] = {
    {1, "RESET", 3, KIND_FREE, true, false},
    {2, "5VIN", -1, KIND_PWR_IN, false, false},
    {3, "INT", 4, KIND_FREE, true, false},
    {4, "GND", -1, KIND_GND, false, false},
    {5, "BUSY", 6, KIND_FREE, true, false},
    {6, "5VOUT", -1, KIND_PWR_OUT, false, false},
    {7, "SCK", 40, KIND_SPI_SD, false, false},
    {8, "I2C_SDA", 8, KIND_I2C, false, false},
    {9, "MOSI", 14, KIND_SPI_SD, false, false},
    {10, "I2C_SCL", 9, KIND_I2C, false, false},
    {11, "MISO", 39, KIND_SPI_SD, false, false},
    {12, "UART_RX", 13, KIND_UART, false, true},
    {13, "CS", 5, KIND_FREE, true, false},
    {14, "UART_TX", 15, KIND_UART, false, true},
};
static const size_t extHeaderCount = 14;

// ---------------------------------------------------------------------
// Sensor objects
// ---------------------------------------------------------------------
BMI270 bmi270; // onboard IMU
Adafruit_SHT31 sht31;
Adafruit_BMP280 bmp280;
Adafruit_BME280 bme280;
BH1750 bh1750;
Adafruit_MPU6050 mpu6050;
VL53L0X vl53l0x;

bool has_bmi270 = false, has_sht31 = false, has_bmp280 = false;
bool has_bme280 = false, has_bh1750 = false, has_mpu6050 = false, has_vl53l0x = false;

// Cached I2C scan results (rescanned periodically, not every frame - a
// full 127-address scan is too slow to run at the dashboard's frame rate)
static const uint8_t MAX_I2C_HITS = 16;
uint8_t i2cAddrs[MAX_I2C_HITS];
uint8_t i2cAddrCount = 0;
unsigned long lastI2CScanMs = 0;
const unsigned long I2C_RESCAN_INTERVAL_MS = 5000;

// ---------------------------------------------------------------------
void i2cScan(bool verbose)
{
  i2cAddrCount = 0;
  if (verbose)
    Serial.println(F("\n--- I2C bus scan (SDA=G8 / SCL=G9) ---"));
  for (uint8_t addr = 1; addr < 127 && i2cAddrCount < MAX_I2C_HITS; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      i2cAddrs[i2cAddrCount++] = addr;
      if (verbose)
        Serial.printf("  0x%02X responded\n", addr);
    }
  }
  if (verbose && i2cAddrCount == 0)
    Serial.println(F("  nothing responded"));
  lastI2CScanMs = millis();
}

// ---------------------------------------------------------------------
void initSensors()
{
  Serial.println(F("\n--- Sensor init ---"));

  if (bmi270.beginI2C(BMI2_I2C_PRIM_ADDR) == BMI2_OK ||
      bmi270.beginI2C(BMI2_I2C_SEC_ADDR) == BMI2_OK)
  {
    has_bmi270 = true;
    Serial.println(F("[OK] BMI270 (onboard IMU)"));
  }
  else
  {
    Serial.println(F("[--] BMI270 not responding"));
  }

  has_sht31 = sht31.begin(0x44) || sht31.begin(0x45);
  Serial.println(has_sht31 ? F("[OK] SHT31") : F("[--] SHT31 not found"));

  has_bmp280 = bmp280.begin(0x76) || bmp280.begin(0x77);
  Serial.println(has_bmp280 ? F("[OK] BMP280") : F("[--] BMP280 not found"));

  has_bme280 = bme280.begin(0x76) || bme280.begin(0x77);
  Serial.println(has_bme280 ? F("[OK] BME280") : F("[--] BME280 not found"));

  has_bh1750 = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23) ||
               bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C);
  Serial.println(has_bh1750 ? F("[OK] BH1750") : F("[--] BH1750 not found"));

  has_mpu6050 = mpu6050.begin(0x68) || mpu6050.begin(0x69);
  Serial.println(has_mpu6050 ? F("[OK] MPU6050") : F("[--] MPU6050 not found"));
  // Note: 0x68 is also the BMI270's default address. If both "detect",
  // trust the BMI270 (it's the one physically on this board) and treat
  // the MPU6050 hit as a false positive unless you've wired one in externally.

  has_vl53l0x = vl53l0x.init();
  if (has_vl53l0x)
    vl53l0x.setTimeout(500);
  Serial.println(has_vl53l0x ? F("[OK] VL53L0X") : F("[--] VL53L0X not found"));
}

// ---------------------------------------------------------------------
void readSensors()
{
  Serial.println(F("\n--- Sensor readings ---"));

  if (has_bmi270)
  {
    bmi270.getSensorData();
    Serial.printf("BMI270   accel(g): x=%.3f y=%.3f z=%.3f  gyro(dps): x=%.2f y=%.2f z=%.2f\n",
                  bmi270.data.accelX, bmi270.data.accelY, bmi270.data.accelZ,
                  bmi270.data.gyroX, bmi270.data.gyroY, bmi270.data.gyroZ);
  }

  if (has_sht31)
  {
    Serial.printf("SHT31    temp=%.2f C  hum=%.2f %%\n",
                  sht31.readTemperature(), sht31.readHumidity());
  }

  if (has_bmp280)
  {
    Serial.printf("BMP280   temp=%.2f C  pres=%.2f hPa  alt=%.2f m\n",
                  bmp280.readTemperature(), bmp280.readPressure() / 100.0f,
                  bmp280.readAltitude(1013.25f));
  }

  if (has_bme280)
  {
    Serial.printf("BME280   temp=%.2f C  hum=%.2f %%  pres=%.2f hPa\n",
                  bme280.readTemperature(), bme280.readHumidity(), bme280.readPressure() / 100.0f);
  }

  if (has_bh1750)
  {
    Serial.printf("BH1750   lux=%.1f\n", bh1750.readLightLevel());
  }

  if (has_mpu6050)
  {
    sensors_event_t a, g, t;
    mpu6050.getEvent(&a, &g, &t);
    Serial.printf("MPU6050  accel(m/s^2): x=%.2f y=%.2f z=%.2f  gyro(rad/s): x=%.2f y=%.2f z=%.2f  temp=%.2f C\n",
                  a.acceleration.x, a.acceleration.y, a.acceleration.z,
                  g.gyro.x, g.gyro.y, g.gyro.z, t.temperature);
  }

  if (has_vl53l0x)
  {
    uint16_t mm = vl53l0x.readRangeSingleMillimeters();
    if (vl53l0x.timeoutOccurred())
    {
      Serial.println(F("VL53L0X  timeout"));
    }
    else
    {
      Serial.printf("VL53L0X  dist=%u mm\n", mm);
    }
  }

  if (!has_bmi270 && !has_sht31 && !has_bmp280 && !has_bme280 &&
      !has_bh1750 && !has_mpu6050 && !has_vl53l0x)
  {
    Serial.println(F("  (nothing detected - check wiring / addresses)"));
  }
}

// ---------------------------------------------------------------------
// Human-readable dump of all 14 EXT header positions, for plain-terminal
// use. GPIO/UART pins get a live read; I2C/SPI/power/GND positions are
// labeled only - we don't touch pins that already belong to another bus
// or that carry 5V (analogRead on a 5V-capable rail would be meaningless
// and pinMode'ing a power pin as INPUT is pointless/risky).
// ---------------------------------------------------------------------
void gpioDeveloperView()
{
  Serial.println(F("\n--- EXT header (all 14 positions, header order) ---"));

  for (size_t i = 0; i < extHeaderCount; i++)
  {
    const ExtPin &p = extHeader[i];
    Serial.printf("  PIN%-2u %-8s", p.headerPos, p.funcLabel);

    switch (p.kind)
    {
    case KIND_FREE:
    case KIND_UART:
    {
      pinMode(p.gpio, INPUT);
      int d = digitalRead(p.gpio);
      Serial.printf(" G%-2d digital=%d", p.gpio, d);
      if (p.adc1 || p.adc2)
      {
        int raw = analogRead(p.gpio);
        Serial.printf("  analog=%4d (%.2fV)%s", raw, raw * 3.3f / 4095.0f,
                      p.adc2 ? "  [ADC2 - unreliable if Wi-Fi/BT is on]" : "");
      }
      break;
    }
    case KIND_I2C:
      Serial.printf(" G%-2d [shared: onboard I2C bus]", p.gpio);
      break;
    case KIND_SPI_SD:
      Serial.printf(" G%-2d [shared: onboard microSD]", p.gpio);
      break;
    case KIND_PWR_IN:
      Serial.print(F(" [power rail - not a data pin]"));
      break;
    case KIND_PWR_OUT:
      Serial.print(F(" [power rail - not a data pin]"));
      break;
    case KIND_GND:
      Serial.print(F(" [ground]"));
      break;
    }
    Serial.println();
  }
}

// ---------------------------------------------------------------------
// One self-describing JSON line per cycle for the web dashboard. Every
// field the UI needs (position, label, gpio, kind, live value) travels
// in the line itself, so the dashboard never has to hardcode header
// layout assumptions independently of this firmware.
// ---------------------------------------------------------------------
void plotFrame()
{
  if (millis() - lastI2CScanMs > I2C_RESCAN_INTERVAL_MS)
  {
    i2cScan(false); // keep the device list fresh if something gets hot-plugged
  }

  float ax = has_bmi270 ? bmi270.data.accelX : 0;
  float ay = has_bmi270 ? bmi270.data.accelY : 0;
  float az = has_bmi270 ? bmi270.data.accelZ : 0;
  float gx = has_bmi270 ? bmi270.data.gyroX : 0;
  float gy = has_bmi270 ? bmi270.data.gyroY : 0;
  float gz = has_bmi270 ? bmi270.data.gyroZ : 0;

  Serial.print(F("PLOT:{"));
  Serial.printf("\"t\":%lu,", millis());
  Serial.printf("\"accel\":[%.3f,%.3f,%.3f],", ax, ay, az);
  Serial.printf("\"gyro\":[%.2f,%.2f,%.2f],", gx, gy, gz);

  Serial.print(F("\"sensors\":{"));
  Serial.printf("\"bmi270\":%s,", has_bmi270 ? "true" : "false");
  Serial.printf("\"sht31\":%s,", has_sht31 ? "true" : "false");
  Serial.printf("\"bmp280\":%s,", has_bmp280 ? "true" : "false");
  Serial.printf("\"bme280\":%s,", has_bme280 ? "true" : "false");
  Serial.printf("\"bh1750\":%s,", has_bh1750 ? "true" : "false");
  Serial.printf("\"mpu6050\":%s,", has_mpu6050 ? "true" : "false");
  Serial.printf("\"vl53l0x\":%s", has_vl53l0x ? "true" : "false");
  Serial.print(F("},"));

  Serial.print(F("\"i2c\":["));
  for (uint8_t i = 0; i < i2cAddrCount; i++)
  {
    if (i)
      Serial.print(',');
    Serial.printf("\"0x%02X\"", i2cAddrs[i]);
  }
  Serial.print(F("],"));

  Serial.print(F("\"gpio\":["));
  for (size_t i = 0; i < extHeaderCount; i++)
  {
    const ExtPin &p = extHeader[i];
    if (i)
      Serial.print(',');
    Serial.printf("{\"pos\":%u,\"label\":\"%s\"", p.headerPos, p.funcLabel);

    const char *kindStr =
        p.kind == KIND_FREE ? "free" : p.kind == KIND_I2C   ? "i2c"
                                   : p.kind == KIND_SPI_SD  ? "spi_sd"
                                   : p.kind == KIND_UART    ? "uart"
                                   : p.kind == KIND_PWR_IN  ? "power_in"
                                   : p.kind == KIND_PWR_OUT ? "power_out"
                                                            : "gnd";
    Serial.printf(",\"kind\":\"%s\"", kindStr);

    if (p.gpio >= 0)
      Serial.printf(",\"gpio\":\"G%d\"", p.gpio);

    if (p.kind == KIND_FREE || p.kind == KIND_UART)
    {
      pinMode(p.gpio, INPUT);
      int d = digitalRead(p.gpio);
      Serial.printf(",\"d\":%d", d);
      if (p.adc1 || p.adc2)
      {
        int a = analogRead(p.gpio);
        Serial.printf(",\"a\":%d,\"adc2\":%s", a, p.adc2 ? "true" : "false");
      }
    }
    Serial.print('}');
  }
  Serial.print(F("]}"));
  Serial.println();
}

// ---------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(1500); // give the USB CDC time to enumerate before we print anything

  Serial.println(F("\n=== Cardputer ADV (Stamp-S3A) sensor + GPIO dev console ==="));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  i2cScan(true);
  initSensors();
}

void loop()
{
  readSensors();
  gpioDeveloperView();
  plotFrame();
  Serial.println(F("=================================================="));
  delay(150); // ~6-7 Hz - smooth enough for live graphs, light enough on the link
}