#include <Arduino.h>
#include <SPI.h>

#include "ICM45686.h"
#include "ICM45686_FlightAdapter.h"

/*
 * ICM45686 飞控适配层 SPI 示例。
 *
 * 这个示例只演示飞控主循环需要的最小数据路径：
 * 1. 初始化 SPI 和 ICM45686。
 * 2. 以固定 ODR 启动 accel/gyro。
 * 3. 读取已换算的 g、dps、温度和统计信息。
 * 4. 检查数据新鲜度、错误计数和原始值饱和。
 *
 * 后续接入飞控时，坐标系旋转、安装方向补偿、滤波和 EKF 不应写在驱动库里，
 * 应放在飞控自己的传感器标定层、姿态解算层和导航层。
 */

#ifndef ICM45686_EXAMPLE_SPI_SCK
#define ICM45686_EXAMPLE_SPI_SCK 12
#endif

#ifndef ICM45686_EXAMPLE_SPI_MISO
#define ICM45686_EXAMPLE_SPI_MISO 13
#endif

#ifndef ICM45686_EXAMPLE_SPI_MOSI
#define ICM45686_EXAMPLE_SPI_MOSI 11
#endif

#ifndef ICM45686_EXAMPLE_SPI_CS
#define ICM45686_EXAMPLE_SPI_CS 10
#endif

#ifndef ICM45686_EXAMPLE_SPI_HZ
#define ICM45686_EXAMPLE_SPI_HZ 6000000
#endif

static ICM456xx imu(SPI, ICM45686_EXAMPLE_SPI_CS, ICM45686_EXAMPLE_SPI_HZ);
static ICM45686FlightAdapter imu_fc(imu);

void setup()
{
  Serial.begin(115200);
  delay(1500);

  pinMode(ICM45686_EXAMPLE_SPI_CS, OUTPUT);
  digitalWrite(ICM45686_EXAMPLE_SPI_CS, HIGH);
  SPI.begin(ICM45686_EXAMPLE_SPI_SCK, ICM45686_EXAMPLE_SPI_MISO, ICM45686_EXAMPLE_SPI_MOSI,
            ICM45686_EXAMPLE_SPI_CS);

  ICM45686FlightConfig cfg;
  cfg.accel_odr_hz = 200;
  cfg.gyro_odr_hz = 200;
  cfg.accel_fsr_g = 16;
  cfg.gyro_fsr_dps = 2000;
  cfg.stale_after_us = 10000;

  const int status = imu_fc.begin(cfg);
  Serial.print("imu_fc.begin=");
  Serial.println(status);
  if (status != 0) {
    Serial.println("初始化失败，请检查供电、GND、SPI 接线、CS 和 SPI mode 3。");
  }
}

void loop()
{
  static uint32_t last_print_ms = 0;
  if (millis() - last_print_ms < 20) {
    return;
  }
  last_print_ms = millis();

  ICM45686FlightSample sample;
  const int status = imu_fc.readSample(sample);
  if (status != 0) {
    const ICM45686FlightStats &stats = imu_fc.stats();
    Serial.print("read_error status=");
    Serial.print(status);
    Serial.print(" count=");
    Serial.println(stats.read_errors);
    return;
  }

  Serial.print("t_us=");
  Serial.print(sample.timestamp_us);
  Serial.print(" accel_g=");
  Serial.print(sample.accel_g[0], 5);
  Serial.print(',');
  Serial.print(sample.accel_g[1], 5);
  Serial.print(',');
  Serial.print(sample.accel_g[2], 5);
  Serial.print(" gyro_dps=");
  Serial.print(sample.gyro_dps[0], 5);
  Serial.print(',');
  Serial.print(sample.gyro_dps[1], 5);
  Serial.print(',');
  Serial.print(sample.gyro_dps[2], 5);
  Serial.print(" norm_g=");
  Serial.print(sample.accel_norm_g, 5);
  Serial.print(" temp_c=");
  Serial.print(sample.temperature_c, 2);
  Serial.print(" saturated=");
  Serial.print(sample.saturated);
  Serial.print(" fresh=");
  Serial.println(imu_fc.hasFreshSample(micros()));
}
