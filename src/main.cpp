#include <BluetoothSerial.h>
#include <driver/i2s.h>

BluetoothSerial bluetoothSerial;

// Define the I2S pins according to ESP32-XIAO pinout
#define I2S_WS 10     // Word Select (LRCLK)
#define I2S_SCK 8     // Serial Clock (BCLK)
#define I2S_SD 9      // Serial Data (DIN)
#define I2S_MCK -1    // Master Clock is not used with INMP441
#define SAMPLE_RATE 16000
#define SAMPLE_BITS 16
#define BUFFER_SIZE 64

void setup() {
  Serial.begin(115200);
  bluetoothSerial.begin("ESP32_MIC"); // name of ble device

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");
}

void loop() {
  int16_t buffer[BUFFER_SIZE];
  size_t bytesRead;
  i2s_read(I2S_NUM_0, &buffer, BUFFER_SIZE * sizeof(int16_t), &bytesRead, portMAX_DELAY);
  int amplitude = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    amplitude += abs(buffer[i]);
  }
  amplitude /= BUFFER_SIZE;

  if (amplitude > 1000) { // 设定阈值为1000，根据实际情况调整
    bluetoothSerial.println(amplitude);
  }
  delay(100);
}
