#include <driver/i2s.h>

#include "WM8960.h"

#define I2C_SDA 23
#define I2C_SCL 22

// // I2S pins
#define IIS_SCLK 21
#define IIS_WSLC 27
#define IIS_DSIN 17  // speaker
#define IIS_DSOU 16  // mic

#define CONFIG_I2S_BCK_PIN IIS_SCLK
#define CONFIG_I2S_LRCK_PIN IIS_WSLC
#define CONFIG_I2S_DATA_PIN IIS_DSIN
#define CONFIG_I2S_DATA_IN_PIN IIS_DSOU

#define SPEAKER_I2S_NUMBER I2S_NUM_0
#define MIC_I2S_NUMBER I2S_NUM_1

class Respeaker2 : public Device {
 public:
  Respeaker2();
  void init();
  void updateColors(int colors);
  void updateBrightness(int brightness);
  void setReadMode();
  void setWriteMode(int sampleRate, int bitDepth, int numChannels);
  void writeAudio(uint8_t *data, size_t size, size_t *bytes_written);
  bool readAudio(uint8_t *data, size_t size);
  bool isHotwordDetected();
  void setVolume(uint16_t volume);
  void muteOutput(bool mute);
  int readSize = 512;
  int writeSize = 512;
  int width = 2;

 private:
  void InitI2SSpeakerOrMic(int mode);
  WM8960 dac;
};

Respeaker2::Respeaker2() {}

void Respeaker2::init() {
  if (!dac.begin(I2C_SDA, I2C_SCL, 100000)) {
    ESP_LOGE(TAG, "Error setting up dac. System halted");
    while (1) {
      delay(500);
    }
  }

  dac.outputSelect(HEADPHONE);
  dac.volume(1, 0);
  // dac.mono(true);
}

bool Respeaker2::isHotwordDetected() { return false; }

void Respeaker2::updateColors(int colors){};

void Respeaker2::updateBrightness(int brightness) {}

void Respeaker2::InitI2SSpeakerOrMic(int mode) {
  esp_err_t err = ESP_OK;

  i2s_driver_uninstall(SPEAKER_I2S_NUMBER);
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 6,
      .dma_buf_len = 60,
  };

  if (mode == MODE_MIC) {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.communication_format =  i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  }
  else {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = true;
  }

  err += i2s_driver_install(SPEAKER_I2S_NUMBER, &i2s_config, 0, NULL);
  i2s_pin_config_t tx_pin_config;

  tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
  tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
  tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
  tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;

  err += i2s_set_pin(SPEAKER_I2S_NUMBER, &tx_pin_config);
  err += i2s_set_clk(SPEAKER_I2S_NUMBER, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

  return;
}

void Respeaker2::setWriteMode(int sampleRate, int bitDepth, int numChannels) {
  if (mode != MODE_SPK) {
    dac.unmute();
    Respeaker2::InitI2SSpeakerOrMic(MODE_SPK);
    mode = MODE_SPK;
  }
  
  if (sampleRate > 0) {
    i2s_set_clk(SPEAKER_I2S_NUMBER, sampleRate,
                static_cast<i2s_bits_per_sample_t>(bitDepth),
                static_cast<i2s_channel_t>(2));
  }
}

void Respeaker2::setReadMode() {
  if (mode != MODE_MIC) {
    dac.mute();
    Respeaker2::InitI2SSpeakerOrMic(MODE_MIC);
    mode = MODE_MIC;
  }
}

void Respeaker2::writeAudio(uint8_t *data, size_t size, size_t *bytes_written) {
  // int16_t stereo[size];
  // for (int i = 0; i < size; i ++) {
  //   stereo[i] = ((data[i] & 0xff) | (data[i + 1] << 8));
  // }
  // i2s_write(SPEAKER_I2S_NUMBER, stereo, sizeof(stereo), bytes_written,
  // portMAX_DELAY);

  uint8_t sample[2];
  size_t bw;
  for (int i = 0; i < size; i += 2) {
    sample[0] = data[i];
    sample[1] = data[i + 1];
    i2s_write(SPEAKER_I2S_NUMBER, sample, sizeof(sample), &bw, portMAX_DELAY);
    i2s_write(SPEAKER_I2S_NUMBER, sample, sizeof(sample), &bw, portMAX_DELAY);
  }

  bytes_written = &size;

  // i2s_write(SPEAKER_I2S_NUMBER, data, size, bytes_written, portMAX_DELAY);
  // i2s_write_expand(SPEAKER_I2S_NUMBER, data, size, 8, 16, bytes_written,
  //                  portMAX_DELAY);
}

bool Respeaker2::readAudio(uint8_t *data, size_t size) {
  size_t byte_read;
  i2s_read(SPEAKER_I2S_NUMBER, data, size, &byte_read,
           (100 / portTICK_RATE_MS));
  return true;
}

void Respeaker2::setVolume(uint16_t volume) {
  Serial.print("vol: ");
  Serial.println(volume);
  // dac.volume(volume / 100, 0);
}

void Respeaker2::muteOutput(bool mute) {
  // if (mute) {
  //   dac.mute();
  // } else {
  //   dac.unmute();
  // }
}