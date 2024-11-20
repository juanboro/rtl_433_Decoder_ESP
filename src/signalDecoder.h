/*
  rtl_433_Decoder_ESP - 433.92 MHz protocols library for ESP32
    based on rtl_433_ESP

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with library. If not, see <http://www.gnu.org/licenses/>


  Project Structure

  rtl_433_Decoder - Main Class
  signalDecoder.cpp - Wrapper and interface for the rtl_433 classes
  rtl_433 - subset of rtl_433 package

*/

#ifndef rtl_433_DECODER_H
#define rtl_433_DECODER_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstring>
#include <vector>

extern "C" {
#include "bitbuffer.h"
#include "fatal.h"
#include "list.h"
#include "pulse_analyzer.h"
#include "pulse_detect.h"
#include "r_api.h"
#include "r_private.h"
#include "rtl_433.h"
#include "rtl_433_devices.h"
}

#include "log.h"

/*----------------------------- functions -----------------------------*/

typedef void (*rtl_433_ESPCallBack)(char* message, void* ctx);

class rtl_433_Decoder {
public:
  // construct
  rtl_433_Decoder(bool ookModulation = true) : _ookModulation(ookModulation), g_cfg({0}) {}

  void rtlSetup();
  static void rtl_433_DecoderTask(void* pvParameters);

  void setCallback(rtl_433_ESPCallBack callback, char* messageBuffer,
                   int bufferSize, void* ctx);
  void processSignal(pulse_data_t* rtl_pulses);
  void processRaw(std::vector<int>& rawdata);

private:
  bool _ookModulation = true;
  int unparsedSignals = 0;

  int rtlVerbose = 0;

  r_cfg_t g_cfg; // Global config object
  TaskHandle_t rtl_433_DecoderHandle;
  QueueHandle_t rtl_433_Queue;
};

#endif