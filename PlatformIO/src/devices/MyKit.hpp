#include <driver/i2s.h>

// I2S pins
#define IIS_SCLK 2
#define IIS_WSLC 15
#define IIS_DSIN 12
#define IIS_DSOU 22

#define CONFIG_I2S_BCK_PIN IIS_SCLK
#define CONFIG_I2S_LRCK_PIN IIS_WSLC
#define CONFIG_I2S_DATA_PIN IIS_DSIN
#define CONFIG_I2S_DATA_IN_PIN IIS_DSOU

#define SPEAKER_I2S_NUMBER I2S_NUM_0
#define MIC_I2S_NUMBER I2S_NUM_0

#define KEY_LISTEN 16

class MyKit : public Device {
 public:
  MyKit();
  void init();
  void updateLeds(int colors);
  // void updateBrightness(int brightness);
  void setReadMode();
  void setWriteMode(int sampleRate, int bitDepth, int numChannels);
  void writeAudio(uint8_t *data, size_t size, size_t *bytes_written);
  bool readAudio(uint8_t *data, size_t size);
  bool isHotwordDetected();
  void setVolume(uint16_t volume);
  int readSize = 256;
  int writeSize = 512;
  int width = 2;

 private:
  void InitI2SSpeakerOrMic(int mode);
  void initI2S();
};

MyKit::MyKit(){};

void MyKit::init() {
  Serial.println("Init MyKit... ");
  pinMode(KEY_LISTEN, INPUT_PULLDOWN);
  // initI2S();
};

void MyKit::updateLeds(int colors){
    // // turn off LEDs
    // digitalWrite(LED_STREAM, HIGH);
    // digitalWrite(LED_WIFI, HIGH);

    // switch (colors)
    // {
    // case COLORS_HOTWORD:
    //     digitalWrite(LED_STREAM, LOW);
    //     break;
    // case COLORS_WIFI_CONNECTED:
    //     // LED_WIFI is turned off already
    //     break;
    // case COLORS_WIFI_DISCONNECTED:
    //     digitalWrite(LED_WIFI, LOW);
    //     break;
    // case COLORS_IDLE:
    //     // all LEDs are turned off already
    //     break;
    // case COLORS_OTA:
    //     break;
    // }
};

void MyKit::initI2S() {
  esp_err_t err = ESP_OK;

  i2s_driver_uninstall(SPEAKER_I2S_NUMBER);
  i2s_driver_uninstall(MIC_I2S_NUMBER);

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER),
      .sample_rate = 16000,
      .bits_per_sample =
          I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
      .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
      .communication_format =
          I2S_COMM_FORMAT_I2S,  // | I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
  };

  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  i2s_config.communication_format =
      i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  i2s_config.channel_format = I2S_CHANNEL_FMT_ALL_LEFT;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 64;
  i2s_config.use_apll = true;

  err += i2s_driver_install(MIC_I2S_NUMBER, &i2s_config, 0, NULL);

  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = true;

  err += i2s_driver_install(SPEAKER_I2S_NUMBER, &i2s_config, 0, NULL);

  i2s_pin_config_t tx_pin_config = {
        .bck_io_num = CONFIG_I2S_BCK_PIN,
        .ws_io_num = CONFIG_I2S_LRCK_PIN,
        .data_out_num = CONFIG_I2S_DATA_PIN,
        .data_in_num = -1,
  };

  err += i2s_set_pin(SPEAKER_I2S_NUMBER, &tx_pin_config);

  tx_pin_config.data_out_num = -1;
  tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;

  err += i2s_set_pin(MIC_I2S_NUMBER, &tx_pin_config);

  err += i2s_set_clk(MIC_I2S_NUMBER, 16000, I2S_BITS_PER_SAMPLE_16BIT,
                     I2S_CHANNEL_MONO);
}

void MyKit::InitI2SSpeakerOrMic(int mode) {
  esp_err_t err = ESP_OK;

  i2s_driver_uninstall(SPEAKER_I2S_NUMBER);
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER),
      .sample_rate = 16000,
      .bits_per_sample =
          I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format =
          I2S_COMM_FORMAT_I2S,  // | I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
  };
  if (mode == MODE_MIC) {
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.communication_format =
        i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.dma_buf_count = 8;
    i2s_config.dma_buf_len = 64;
    i2s_config.use_apll = true;
  } else {
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
  if (mode == MODE_SPK) {
    err += i2s_set_clk(SPEAKER_I2S_NUMBER, 16000, I2S_BITS_PER_SAMPLE_16BIT,
                       I2S_CHANNEL_MONO);
  }
}

void MyKit::setWriteMode(int sampleRate, int bitDepth, int numChannels) {
  if (mode != MODE_SPK) {
    Serial.println("write mode");
    MyKit::InitI2SSpeakerOrMic(MODE_SPK);
    mode = MODE_SPK;
  }
  if (sampleRate > 0) {
    i2s_set_clk(SPEAKER_I2S_NUMBER, sampleRate,
                static_cast<i2s_bits_per_sample_t>(bitDepth),
                static_cast<i2s_channel_t>(numChannels));
  }
}

void MyKit::setReadMode() {
  if (mode != MODE_MIC) {
    Serial.println("read mode");
    MyKit::InitI2SSpeakerOrMic(MODE_MIC);
    mode = MODE_MIC;
  }
}

// void MyKit::setVolume(uint16_t volume) {
//   if (volume >= 0 && volume >= 100) {
//     this->volume = volume / 100;
//   }
// };

void MyKit::writeAudio(uint8_t *data, size_t size, size_t *bytes_written) {
  // if (this->volume != 1) {
  // for (size_t i = 0; i < size; i++) {
  // *((uint8_t *)(data + i)) = (*((uint8_t *)(data + i))) * .75;
  // }
  // }

  i2s_write(SPEAKER_I2S_NUMBER, data, size, bytes_written, portMAX_DELAY);
}

bool MyKit::readAudio(uint8_t *data, size_t size) {
  size_t byte_read;
  i2s_read(MIC_I2S_NUMBER, data, size, &byte_read, portMAX_DELAY);
  return true;
}

// bool MyKit::setGain(uint16_t gain) {
//   if (f > 4.0) {
//     f = 4.0;
//   } else if (f < 0.0) {
//     f = 0.0;
//   }
//   gainF2P6 = (uint8_t)(f * (1 << 6));
//   return true;
// }

// int16_t MyKit::amplify(int16_t s) {
//   int32_t v = (s * MyKit::gainF2P6) >> 6;
//   if (v < -32767) {
//     return -32767;
//   } else if (v > 32767) {
//     return 32767;
//   } else {
//     return (int16_t)(v & 0xffff);
//   }
// }

// void MyKit::muteOutput(bool mute) {
//   // digitalWrite(GPIO_PA_EN, mute ? LOW : HIGH);
// }

void MyKit::setVolume(uint16_t volume) {}

bool MyKit::isHotwordDetected() {
  // TODOD debounce maybe?
  bool listen = digitalRead(KEY_LISTEN) == HIGH;
  if (listen) {
    Serial.println("MyKit listen ...");
  }
  return listen;
}