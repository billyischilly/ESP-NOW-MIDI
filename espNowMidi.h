#include <sys/_stdint.h>
/*!
 *  @file       espNowMidi.h
 *  Project     ESP NOW MIDI Library
 *  @brief      MIDI Library for the Arduino - Platform
 *  @license    MIT - Copyright (c) 2015 Francois Best
 *  @author     lathoub, Francois Best
 *  @date       09/10/2022
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once

#include <MIDI.h>
#include <esp_now.h>
#include <WiFi.h>

using namespace MIDI_NAMESPACE;

#include "espNowMidi_Namespace.h"

BEGIN_ESPMIDI_NAMESPACE

#define ESP_BUFFER_SIZE 32
static int mRAvailable = 0;
static int mRWIdx = 0;
static int mRRIdx = 0;
static int mRBuf[ESP_BUFFER_SIZE];


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 1) {
    mRAvailable++;
    mRBuf[mRWIdx++] = *incomingData;
    mRWIdx %= ESP_BUFFER_SIZE;
  }
}

class ESPMIDI {

public:
  ESPMIDI(uint8_t *address) {
    receiverMacAddress = address;
  };

public:
  static const bool thruActivated = true;

  void begin() {

    // Set ESP32 as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Initilize ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW for MIDI");
      return;
    }

    // Register the send callback
    esp_now_register_recv_cb(OnDataRecv);

    // Register peer
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  void end() {
  }

  bool beginTransmission(MidiType) {
    return true;
  };

  void write(byte value) {
    esp_now_send(receiverMacAddress, (uint8_t *)&value, sizeof(value));
  };

  void endTransmission(){};

  byte read() {
    // Read data out of the receive buffer
    int val = mRBuf[mRRIdx++];
    mRRIdx %= ESP_BUFFER_SIZE;
    mRAvailable--;
    if (mRAvailable < 0) mRAvailable = 0;
    return val;
  };

  unsigned available() {
    return mRAvailable;
  };

private:
  uint8_t receivedByte;
  uint8_t *receiverMacAddress;
  esp_now_peer_info_t peerInfo;
};

END_MIDI_NAMESPACE

/*! \brief Create an instance of the library attached to a serial port.
 You can use HardwareSerial or SoftwareSerial for the serial port.
 Example: MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midi2);
 Then call midi2.begin(), midi2.read() etc..
 */
#define ESPMIDI_CREATE_INSTANCE(MacAddress, Name) \
  ESPMIDI_NAMESPACE::ESPMIDI ESP##Name(MacAddress); \
  MIDI_NAMESPACE::MidiInterface<ESPMIDI_NAMESPACE::ESPMIDI> Name((ESPMIDI_NAMESPACE::ESPMIDI&)ESP##Name);

/*! \brief Create an instance of the library attached to a serial port with
 custom settings.
 @see DefaultSettings
 @see MIDI_CREATE_INSTANCE
 */
// #define ESPMIDI_CREATE_CUSTOM_INSTANCE(Type, SerialPort, Name, Settings) \
//   MIDI_NAMESPACE::SerialMIDI<Type> serial##Name(SerialPort); \
//   MIDI_NAMESPACE::MidiInterface<MIDI_NAMESPACE::SerialMIDI<Type>, Settings> Name((MIDI_NAMESPACE::SerialMIDI<Type> &)serial##Name);