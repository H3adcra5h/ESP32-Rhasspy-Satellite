#include "WM8960.h"

#include <Arduino.h>

#include "Wire.h"

// https://github.com/Seeed-Studio/Seeed_Arduino_Audio/blob/master/control_wm8960.cpp

bool WM8960::begin(const uint8_t sda, const uint8_t scl, const uint32_t frequency) {
    ESP_LOGD(TAG, "WM8960 I2C init sda=%i scl=%i frequency=%i", sda, scl, frequency);

    if (!Wire.begin(sda, scl, frequency)) {
        ESP_LOGE(TAG, "Wire setup error");
        return false;
    }

    Wire.beginTransmission(WM8960_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        ESP_LOGE(TAG, "No WM8960 @ I2C address: 0x%X", WM8960_ADDRESS);
        return false;
    }
    delay(100);

    bool success = Write(RESET, 0x0000);
    ESP_LOGD(TAG, "reset: %i", success);
    delay(100);

    success |= enable();
    ESP_LOGD(TAG, "enable: %i", success);

    //Configure clock
    //MCLK->div1->SYSCLK->DAC/ADC sample Freq = 24MHz(MCLK) / 2*256 = 46.875kHz
    success |= Write(CLOCKING_1, 0x0091);
    success |= Write(CLOCKING_2, 0x01ca);
    ESP_LOGD(TAG, "clock: %i", success);

    //Configure ADC/DAC
    success |= Write(ADC_AND_DAC_CONTROL_1, 1 << 2 | 1 << 1);
    ESP_LOGD(TAG, "adc/dac: %i", success);

    //Configure audio interface
    //I2S format 16 bits word length and set to master mode
    // success |= Write(DIGITAL_AUDIO_INTERFACE, 1<<1 | 1<<6);
    success |= Write(DIGITAL_AUDIO_INTERFACE, 1 << 1);
    ESP_LOGD(TAG, "interface: %i", success);

    //Configure HP_L and HP_R OUTPUTS
    success |= Write(LOUT1_VOLUME, 0x007F | 0x0100);  //LOUT1 Volume Set
    success |= Write(ROUT1_VOLUME, 0x007F | 0x0100);  //ROUT1 Volume Set
    ESP_LOGD(TAG, "jack volume: %i", success);

    //Configure SPK_RP and SPK_RN
    success |= Write(LOUT2_VOLUME, 0x7F | 1 << 8);  //Left Speaker Volume
    success |= Write(ROUT2_VOLUME, 0x7F | 1 << 8);  //Right Speaker Volume
    ESP_LOGD(TAG, "speaker volume: %i", success);

    //Enable the OUTPUTS
    success |= Write(CLASS_D_CONTROL_1, 0x00F7);  //Enable Class D Speaker Outputs
    ESP_LOGD(TAG, "speaker: %i", success);

    //Configure DAC volume
    success |= Write(LEFT_DAC_VOLUME, 0x00FF | 0x0100);
    success |= Write(RIGHT_DAC_VOLUME, 0x00FF | 0x0100);
    ESP_LOGD(TAG, "dac volume: %i", success);

    //3D
    //success |= Write(0x10, 0x001F);

    // enable Left DAC to Left Output Mixer
    success |= Write(LEFT_OUT_MIX, 1 << 8);
    success |= Write(RIGHT_OUT_MIX, 1 << 8);
    ESP_LOGD(TAG, "mixer: %i", success);

    // connect LINPUT1 to PGA and set PGA Boost Gain.
    success |= Write(ADCL_SIGNAL_PATH, 0x0020 | 1 << 8 | 1 << 3);
    success |= Write(ADCR_SIGNAL_PATH, 0x0020 | 1 << 8 | 1 << 3);
    ESP_LOGD(TAG, "pga/boost: %i", success);

    // set Input PGA Volume
    success |= Write(LEFT_INPUT_VOLUME, 0x0027 | 0x0100);
    success |= Write(RIGHT_INPUT_VOLUME, 0x0027 | 0x0100);
    ESP_LOGD(TAG, "pga volume: %i", success);

    // set ADC Volume
    success |= Write(LEFT_ADC_VOLUME, 0x00c3 | 0x0100);
    success |= Write(RIGHT_ADC_VOLUME, 0x00c3 | 0x0100);
    ESP_LOGD(TAG, "adc volume: %i", success);

    // disable bypass switch
    success |= Write(BYPASS_1, 0x0000);
    success |= Write(BYPASS_2, 0x0000);
    ESP_LOGD(TAG, "bypass off: %i", success);

    //enable ALC
    success |= Write(ALC1, 0x007B);  // auto channel, gain +30dB
    success |= Write(ALC2, 0x0100);
    success |= Write(ALC3, 0x0032);
    ESP_LOGD(TAG, "alc: %i", success);

    // noise gate
    // success |= Write(NOISE_GATE, 0x0100);  //0 disabled 1 enabled
    // ESP_LOGD(TAG, "noise gate: %i", success);

    //speaker select
    success |= Write(ADDITIONAL_CONTROL_2, 1 << 6 | 0 << 5);  //0 speaker out 1 headphone out
    ESP_LOGD(TAG, "output select speaker: %i", success);

    //enable thermal shutdown
    success |= Write(ADDITIONAL_CONTROL_1, 0x01C3);
    success |= Write(ADDITIONAL_CONTROL_4, 0x0009 | 1 << 6);  //0x000D,0x0005
    ESP_LOGD(TAG, "thermal shutdown: %i", success);

    //config f clock to 44100 hz
    success |= Write(PLL_N, 0x0037);
    success |= Write(PLL_K_1, 0x0086);
    success |= Write(PLL_K_2, 0x00C2);
    success |= Write(PLL_K_3, 0x0027);
    ESP_LOGD(TAG, "pll clock : %i", success);

    return success;
}

bool WM8960::enable(void) {
    //Set Power Source
    bool ret = Write(POWER_MANAGEMENT_1, 1 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2 | 1 << 1);
    ret |= Write(POWER_MANAGEMENT_2, 1 << 8 | 1 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 0);
    ret |= Write(POWER_MANAGEMENT_3, 1 << 3 | 1 << 2 | 1 << 4 | 1 << 5);
    return true;
}

bool WM8960::disable(void) {
    bool ret = Write(POWER_MANAGEMENT_1, 0);
    ret |= Write(POWER_MANAGEMENT_2, 0);
    ret |= Write(POWER_MANAGEMENT_3, 0);
    return ret;
}

bool WM8960::volume(float volume) {
    return WM8960::volume(volume, volume);
}

bool WM8960::volume(float left, float right) {
    leftVolume = left;
    rightVolume = right;

    left = left * 0xff;
    bool ret = Write(LEFT_DAC_VOLUME, (uint8_t)left | 0x0100);
    right = right * 0xff;
    ret |= Write(RIGHT_DAC_VOLUME, (uint8_t)right | 0x0100);
    return ret;

    /*

    //Configure HP_L and HP_R OUTPUTS
    success |= Write(LOUT1_VOLUME, 0x007F | 0x0100);  //LOUT1 Volume Set
    success |= Write(ROUT1_VOLUME, 0x007F | 0x0100);  //ROUT1 Volume Set
    ESP_LOGD(TAG, "jack volume: %i", success);

    //Configure SPK_RP and SPK_RN
    success |= Write(LOUT2_VOLUME, 0x7F | 1 << 8);  //Left Speaker Volume
    success |= Write(ROUT2_VOLUME, 0x7F | 1 << 8);  //Right Speaker Volume
    ESP_LOGD(TAG, "speaker volume: %i", success);
*/
}

bool WM8960::mute() {
    uint16_t reg = Read(ADC_AND_DAC_CONTROL_1);
    // return Write(ADC_AND_DAC_CONTROL_1, reg | 1 << 3);
    bitSet(reg, 3);
    return Write(ADC_AND_DAC_CONTROL_1, reg);

    // bool ret = Write(LEFT_DAC_VOLUME, 0);
    // ret |= Write(RIGHT_DAC_VOLUME, 0);
    // return ret;
}

bool WM8960::unmute() {
    uint16_t reg = Read(ADC_AND_DAC_CONTROL_1);
    // return Write(ADC_AND_DAC_CONTROL_1, reg & ~(1 << 3));
    bitClear(reg, 3);
    return Write(ADC_AND_DAC_CONTROL_1, reg);
    // return volume(leftVolume, rightVolume);
}

bool WM8960::mono(bool mono){
    uint16_t reg = Read(ADDITIONAL_CONTROL_1);
    if(mono){
        bitSet(reg, 4);
    }
    else {
        bitClear(reg, 4);
    }

    return Write(ADDITIONAL_CONTROL_1, reg);
};


bool WM8960::inputSelect(inputSource in) {
    //respekaer cannot support linein input.
    if (LINEIN == in)
        return false;
    return true;
}

bool WM8960::bypass(bool bypass) {
    // enable bypass switch
    uint16_t reg1 = Read(BYPASS_1);
    uint16_t reg2 = Read(BYPASS_2);
    bool ret;
    if (bypass) {
        ret = Write(BYPASS_1, reg1 | 1 << 7);   //enable left input to output
        ret |= Write(BYPASS_2, reg2 | 1 << 7);  //enable right input to output
    } else {
        ret = Write(BYPASS_1, reg1 & ~(1 << 7));   //disable left input to output
        ret |= Write(BYPASS_2, reg2 & ~(1 << 7));  //disable right input to output
    }
    return ret;
}

bool WM8960::inputLevel(float volume) {
    volume = volume * 0x3f;
    bool ret = Write(LEFT_INPUT_VOLUME, (uint8_t)volume | 0x0100);
    ret |= Write(RIGHT_INPUT_VOLUME, (uint8_t)volume | 0x0100);
    return ret;
}

bool WM8960::outputSelect(outputTarget out) {
    uint16_t reg = Read(ADDITIONAL_CONTROL_2);
    if (HEADPHONE == out) {
        return Write(ADDITIONAL_CONTROL_2, reg | 1 << 5);
    } else {
        return Write(ADDITIONAL_CONTROL_2, reg & ~(1 << 5));
    }
}

bool WM8960::Write(uint8_t reg, uint16_t dat) {
    Wire.beginTransmission(WM8960_ADDRESS);
    Wire.write((reg << 1) | ((uint8_t)((dat >> 8) & 0x0001)));
    Wire.write((uint8_t)(dat & 0x00FF));

    uint8_t error = Wire.endTransmission();
    if (error) {
        ESP_LOGD(TAG, "WM8960 transmission error: 0x%X", error);
        return false;
    } else {
        WM8960_REG_VAL[reg] = dat;
    }
    return true;
}

uint16_t WM8960::Read(uint8_t reg) {
    return WM8960_REG_VAL[reg];
}
