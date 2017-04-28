/*
 * DecaWave Tag
 *
 * Tags are placed around the area to be mapped. By default, tags are in a ranging mode. Each tag
 * should be compiled with a unique ID, and resopnds to any ranging requests from the anchor with
 * the proper ID.
 *
 * However, without knowing where the tags are, it is not possible to locate the anchor either.
 * Tags also have a second mode where it acts as an anchor and computes distances to all other
 * tags. These distances are stored in EEPROM (TODO). The anchor queries these distances for positioning
 * calculations (TODO).
 *
 * This implementation of ranging is based on examples in the open source DecaWave 1000 library.
 * The original license is reproduced below.
 * https://github.com/thotro/arduino-dw1000/blob/master/examples/RangingTag/RangingTag.ino
 */
/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file RangingTag.ino
 * Use this to test two-way ranging functionality with two DW1000. This is
 * the tag component's code which polls for range computation. Addressing and
 * frame filtering is currently done in a custom way, as no MAC features are
 * implemented yet.
 *
 * Complements the "RangingAnchor" example sketch.
 *
 * @todo
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

#include <SPI.h>
#include <DW1000.h>

/******** CONFIGURATION VARIABLES ********/
#undef        DEBUG       // Debugging
#ifndef DWID
#   define    DWID 1      // DecaWave Ranging ID
#endif
#ifndef NUM_TAGS
#   define    NUM_TAGS 1  // Number of active tags
#endif

// Connection pins
#define PIN_RST 9  // reset pin
#define PIN_IRQ 2  // irq pin
#define PIN_SS  SS // spi select pin
/******** END CONFIGURATION VARIABLES ********/

/******** ENUMS ********/
// Modes available
#define MODE_POLL     'P' // Listen for ranging requests
#define MODE_DETECT   'D' // Act as an anchor and detect distances to other tags
#define MODE_SAVE     'S' // Save detected distances to other tags to EEPROM
#define MODE_SEND     'N' // Listen for requests for distances
#define MODE_INFO     'I' // Display info

void loop_poll();
void loop_detect();
void loop_save();
void loop_send();
void loop_info();

// Messages used in the ranging protocol
#define POLL          0
#define POLL_ACK      1
#define RANGE         2
#define RANGE_REPORT  3
#define POLL_REQ      250
#define RANGE_FAILED  255
/******** END ENUMS ********/

// Current mode function pointer
void (*mode_fn)();

// message flow state
volatile byte expectedMsgId = POLL_REQ;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

// The current tag being polled
uint8_t current_tag = 1;

void setup() {
  // Serial
  Serial.begin(115200);
  delay(500);

  // Initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  // General configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  // Information
  loop_info();

  // Attach callbacks
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  receiver();
  noteActivity();

  mode_fn = loop_poll;
}

// Update activity timestamp
inline void noteActivity() {
  lastActivity = millis();
}

// Status change on sent success
void handleSent() {
  sentAck = true;
}

// Status change on received success
void handleReceived() {
  receivedAck = true;
}

// Sent a poll message
void transmitPoll() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
#ifdef DEBUG
  Serial.println("Sent poll");
#endif
}

// Send range timestamp
void transmitRange() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE;
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data + 1);
  timePollAckReceived.getTimestamp(data + 6);
  timeRangeSent.getTimestamp(data + 11);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
#ifdef DEBUG
  Serial.println(F("Sent range"));
#endif
}

// Receive a new message
void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

void computeRangeAsymmetric() {
  // asymmetric two-way ranging (more computation intense, less error prone)
  DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
  DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
  DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
  DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
  DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
  // set tof timestamp
  timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
  // symmetric two-way ranging (less computation intense, more error prone on clock drift)
  DW1000Time tof = ((timePollAckReceived - timePollSent) - (timePollAckSent - timePollReceived) +
                    (timeRangeReceived - timePollAckSent) - (timeRangeSent - timePollAckReceived)) * 0.25f;
  // set tof timestamp
  timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */

void loop() {
  if (Serial.available() > 0) {
    int c = Serial.read();
    if (c == MODE_POLL) {
      mode_fn = loop_poll;
    }
    else if (c == MODE_DETECT) {
      mode_fn = loop_detect;
    }
    else if (c == MODE_SAVE) {
      loop_save();
    }
    else if (c == MODE_SEND) {
      mode_fn = loop_send;
    }
    else if (c == MODE_INFO) {
      loop_info();
    }
  }
  mode_fn();
}

void loop_poll() {
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (millis() - lastActivity > resetPeriod) {
      expectedMsgId = POLL_REQ;
      receiver();
      noteActivity();
#ifdef DEBUG
      Serial.println(F("Waiting for poll request"));
#endif
    }
    return;
  }

  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
    }
    else if (msgId == RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      // unexpected message, start over again
#ifdef DEBUG
      Serial.print(F("Received wrong message # ")); Serial.println(msgId);
#endif
      expectedMsgId = POLL_REQ;
      return;
    }
    else {
#ifdef DEBUG
      Serial.print(F("Received message # ")); Serial.println(msgId);
#endif
    }

    if (msgId == POLL_REQ) {
      if (data[1] == DWID) { // Make sure this poll is direct to us
        expectedMsgId = POLL_ACK;
        transmitPoll();
        noteActivity();
      }
    }
    else if (msgId == POLL_ACK) {
      DW1000.getReceiveTimestamp(timePollAckReceived);
      expectedMsgId = RANGE_REPORT;
      transmitRange();
      noteActivity();
    }
    else if (msgId == RANGE_REPORT) {
      expectedMsgId = POLL_REQ;
      float curRange;
      memcpy(&curRange, data + 1, 4);
      receiver();
      noteActivity();
    }
    else if (msgId == RANGE_FAILED) {
      expectedMsgId = POLL_REQ;
      receiver();
      noteActivity();
    }
  }
}

void loop_detect() {
  // TODO
  Serial.println(F("TODO detect"));
}

void loop_save() {
  // TODO
  Serial.println(F("TODO save"));
}

void loop_send() {
  // TODO
  Serial.println(F("TODO send"));
}

void loop_info() {
  Serial.println(F("### DW1000 Arduino Ranging Tag ###"));

  // Information
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  Serial.print("Ranging ID: "); Serial.println(DWID);
}
