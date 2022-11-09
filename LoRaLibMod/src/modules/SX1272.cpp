#include "SX1272.h"

SX1272::SX1272(Module* mod) : SX127x(mod) {
  
}

int16_t SX1272::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint8_t currentLimit, uint16_t preambleLength, uint8_t gain) {
  // execute common part
  int16_t state = SX127x::begin(SX1272_CHIP_VERSION, syncWord, currentLimit, preambleLength);
  if(state != ERR_NONE) {
    return(state);
  }
  
  // configure settings not accessible by API
  state = config();
  if(state != ERR_NONE) {
    return(state);
  }
  
  // mitigation of receiver spurious response
  // see SX1272/73 Errata, section 2.2 for details
  state = _mod->SPIsetRegValue(0x31, 0b10000000, 7, 7);
  if(state != ERR_NONE) {
    return(state);
  }
  
  // configure publicly accessible settings
  state = setFrequency(freq);
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setBandwidth(bw);
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setSpreadingFactor(sf);
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setCodingRate(cr);
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setOutputPower(power);
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setGain(gain);
  if(state != ERR_NONE) {
    return(state);
  }
  
  return(state);
}

int16_t SX1272::beginFSK(float freq, float br, float rxBw, float freqDev, int8_t power, uint8_t currentLimit) {
  // execute common part
  int16_t state = SX127x::beginFSK(SX1272_CHIP_VERSION, br, rxBw, freqDev, currentLimit);
  if(state != ERR_NONE) {
    return(state);
  }
  
  // configure settings not accessible by API
  state = configFSK();
  if(state != ERR_NONE) {
    return(state);
  }
  
  // configure publicly accessible settings
  state = setFrequency(freq);
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setOutputPower(power);
  if(state != ERR_NONE) {
    return(state);
  }
  
  return(state);
}

int16_t SX1272::setFrequency(float freq) {
  // check frequency range
  if((freq < 860.0) || (freq > 1020.0)) {
    return(ERR_INVALID_FREQUENCY);
  }
  
  // set frequency
  return(SX127x::setFrequencyRaw(freq));
}

int16_t SX1272::setBandwidth(float bw) {
  // check active modem
  if(getActiveModem() != SX127X_LORA) {
    return(ERR_WRONG_MODEM);
  }

  uint8_t newBandwidth;
  
  // check alowed bandwidth values
  if(abs(bw - 125.0) <= 0.001) {
    newBandwidth = SX1272_BW_125_00_KHZ;
  } else if(abs(bw - 250.0) <= 0.001) {
    newBandwidth = SX1272_BW_250_00_KHZ;
  } else if(abs(bw - 500.0) <= 0.001) {
    newBandwidth = SX1272_BW_500_00_KHZ;
  } else {
    return(ERR_INVALID_BANDWIDTH);
  }
  
  // set bandwidth and if successful, save the new setting
  int16_t state = SX1272::setBandwidthRaw(newBandwidth);
  if(state == ERR_NONE) {
    SX127x::_bw = bw;
  }
  return(state);
}

int16_t SX1272::setSpreadingFactor(uint8_t sf) {
  // check active modem
  if(getActiveModem() != SX127X_LORA) {
    return(ERR_WRONG_MODEM);
  }

  uint8_t newSpreadingFactor;
  
  // check allowed spreading factor values
  switch(sf) {
    case 6:
      newSpreadingFactor = SX127X_SF_6;
      break;
    case 7:
      newSpreadingFactor = SX127X_SF_7;
      break;
    case 8:
      newSpreadingFactor = SX127X_SF_8;
      break;
    case 9:
      newSpreadingFactor = SX127X_SF_9;
      break;
    case 10:
      newSpreadingFactor = SX127X_SF_10;
      break;
    case 11:
      newSpreadingFactor = SX127X_SF_11;
      break;
    case 12:
      newSpreadingFactor = SX127X_SF_12;
      break;
    default:
      return(ERR_INVALID_SPREADING_FACTOR);
  }
  
  // set spreading factor and if successful, save the new setting
  int16_t state = SX1272::setSpreadingFactorRaw(newSpreadingFactor);
  if(state == ERR_NONE) {
    SX127x::_sf = sf;
  }
  return(state);
}

int16_t SX1272::setCodingRate(uint8_t cr) {
  // check active modem
  if(getActiveModem() != SX127X_LORA) {
    return(ERR_WRONG_MODEM);
  }

  uint8_t newCodingRate;
  
  // check allowed coding rate values
  switch(cr) {
    case 5:
      newCodingRate = SX1272_CR_4_5;
      break;
    case 6:
      newCodingRate = SX1272_CR_4_6;
      break;
    case 7:
      newCodingRate = SX1272_CR_4_7;
      break;
    case 8:
      newCodingRate = SX1272_CR_4_8;
      break;
    default:
      return(ERR_INVALID_CODING_RATE);
  }
  
  // set coding rate and if successful, save the new setting
  int16_t state = SX1272::setCodingRateRaw(newCodingRate);
  if(state == ERR_NONE) {
    SX127x::_cr = cr;
  }
  return(state);
}

int16_t SX1272::setOutputPower(int8_t power) {
  // check allowed power range
  if(!(((power >= -1) && (power <= 17)) || (power == 20))) {
    return(ERR_INVALID_OUTPUT_POWER);
  }
  
  // set mode to standby
  int16_t state = SX127x::standby();
  
  // set output power
  if(power < 2) {
    // power is less than 2 dBm, enable PA0 on RFIO
    state |= _mod->SPIsetRegValue(SX127X_REG_PA_CONFIG, SX127X_PA_SELECT_RFO, 7, 7);
    state |= _mod->SPIsetRegValue(SX127X_REG_PA_CONFIG, (power + 1), 3, 0);
    state |= _mod->SPIsetRegValue(SX1272_REG_PA_DAC, SX127X_PA_BOOST_OFF, 2, 0);
  } else if((power >= 2) && (power <= 17)) {
    // power is 2 - 17 dBm, enable PA1 + PA2 on PA_BOOST
    state |= _mod->SPIsetRegValue(SX127X_REG_PA_CONFIG, SX127X_PA_SELECT_BOOST, 7, 7);
    state |= _mod->SPIsetRegValue(SX127X_REG_PA_CONFIG, (power - 2), 3, 0);
    state |= _mod->SPIsetRegValue(SX1272_REG_PA_DAC, SX127X_PA_BOOST_OFF, 2, 0);
  } else if(power == 20) {
    // power is 20 dBm, enable PA1 + PA2 on PA_BOOST and enable high power control
    state |= _mod->SPIsetRegValue(SX127X_REG_PA_CONFIG, SX127X_PA_SELECT_BOOST, 7, 7);
    state |= _mod->SPIsetRegValue(SX127X_REG_PA_CONFIG, (power - 5), 3, 0);
    state |= _mod->SPIsetRegValue(SX1272_REG_PA_DAC, SX127X_PA_BOOST_ON, 2, 0);
  }
  return(state);
}

int16_t SX1272::setGain(uint8_t gain) {
  // check active modem
  if(getActiveModem() != SX127X_LORA) {
    return(ERR_WRONG_MODEM);
  }

  // check allowed range
  if(gain > 6) {
    return(ERR_INVALID_GAIN);
  }
  
  // set mode to standby
  int16_t state = SX127x::standby();
  
  // set gain
  if(gain == 0) {
    // gain set to 0, enable AGC loop
    state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_2, SX1272_AGC_AUTO_ON, 2, 2);
  } else {
    state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_2, SX1272_AGC_AUTO_OFF, 2, 2);
    state |= _mod->SPIsetRegValue(SX127X_REG_LNA, (gain << 5) | SX127X_LNA_BOOST_ON);
  }
  return(state);
}

int16_t SX1272::setBandwidthRaw(uint8_t newBandwidth) {
  // set mode to standby
  int16_t state = SX127x::standby();
  
  // write register
  state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_1, newBandwidth, 7, 6);
  return(state);
}

int16_t SX1272::setSpreadingFactorRaw(uint8_t newSpreadingFactor) {
  // set mode to standby
  int16_t state = SX127x::standby();
  
  // write registers
  if(newSpreadingFactor == SX127X_SF_6) {
    state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_1, SX1272_HEADER_IMPL_MODE | SX1272_RX_CRC_MODE_OFF, 2, 1);
    state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_2, SX127X_SF_6 | SX127X_TX_MODE_SINGLE, 7, 3);
    state |= _mod->SPIsetRegValue(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_6, 2, 0);
    state |= _mod->SPIsetRegValue(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_6);
  } else {
    state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_1, SX1272_HEADER_EXPL_MODE | SX1272_RX_CRC_MODE_ON,  2, 1);
    state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_2, newSpreadingFactor | SX127X_TX_MODE_SINGLE, 7, 3);
    state |= _mod->SPIsetRegValue(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_7_12, 2, 0);
    state |= _mod->SPIsetRegValue(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_7_12);
  }
  return(state);
}

int16_t SX1272::setCodingRateRaw(uint8_t newCodingRate) {
  // set mode to standby
  int16_t state = SX127x::standby();
  
  // write register
  state |= _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_1, newCodingRate, 5, 3);
  return(state);
}

int16_t SX1272::config() {
  // configure common registers
  int16_t state = SX127x::config();
  if(state != ERR_NONE) {
    return(state);
  }
  
  // calculate symbol length and set low datarate optimization, if needed
  uint16_t base = 1;
  float symbolLength = (float)(base << _sf) / (float)_bw;
  if(symbolLength >= 0.016) {
    state = _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_1, SX1272_LOW_DATA_RATE_OPT_ON, 0, 0);
  } else {
    state = _mod->SPIsetRegValue(SX127X_REG_MODEM_CONFIG_1, SX1272_LOW_DATA_RATE_OPT_OFF, 0, 0);
  }
  return(state);
}

int16_t SX1272::configFSK() {
  // configure common registers
  int16_t state = SX127x::configFSK();
  if(state != ERR_NONE) {
    return(state);
  }
  
  // set data shaping
  state = _mod->SPIsetRegValue(SX127X_REG_PA_RAMP, SX1272_FSK_GAUSSIAN_0_3, 6, 5);
  if(state != ERR_NONE) {
    return(state);
  }
  
  // set fast PLL hop
  state = _mod->SPIsetRegValue(SX1272_REG_PLL_HOP, SX127X_FAST_HOP_ON, 7, 7);
  
  return(state);
}
