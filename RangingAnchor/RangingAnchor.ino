/*
 * DecaWave Anchor
 *
 * An anchor polls each tag for a distance, and computes its position. Anchor has an ID of 0.
 *
 * The anchor must first query each tag for distances to other DecaWaves. These distances may be
 * stored in RAM or stored in EEPROM (TODO).
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
 * @file RangingAnchor.ino
 * Use this to test two-way ranging functionality with two
 * DW1000. This is the anchor component's code which computes range after
 * exchanging some messages. Addressing and frame filtering is currently done
 * in a custom way, as no MAC features are implemented yet.
 *
 * Complements the "RangingTag" example sketch.
 *
 * @todo
 *  - weighted average of ranging results based on signal quality
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

#include <SPI.h>
#include <DW1000.h>

/******** CONFIGURATION VARIABLES ********/
#undef        DEBUG       // Debugging
#ifndef DWID
#   define    DWID 0      // DecaWave Ranging ID
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
#define MODE_POLL     'P' // Poll distances
#define MODE_QUERY    'Q' // Query distances to other tags
#define MODE_SAVE     'S' // Save distances to other tags to EEPROM
#define MODE_INFO     'I' // Display info

void loop_poll();
void loop_query();
void loop_save();
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
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
// last computed range/time
DW1000Time timeComputedRange;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

// The current tag begin polled
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
  DW1000.setDeviceAddress(1);
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
  rangingCountPeriod = millis();

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

// Send a poll request message
void transmitPollReq() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL_REQ;
  data[1] = current_tag; // ID to request from
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
#ifdef DEBUG
  Serial.print("Sent poll request "); Serial.println(current_tag);
#endif
}

// Send a poll acknowledgement
void transmitPollAck() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL_ACK;
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
#ifdef DEBUG
  Serial.println("Sent poll ack");
#endif
}

// Send a range report
void transmitRangeReport(float curRange) {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_REPORT;
  memcpy(data + 1, &curRange, 4);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
#ifdef DEBUG
  Serial.println("Sent range report");
#endif
}

// Send a range failure message
void transmitRangeFailed() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_FAILED;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
#ifdef DEBUG
  Serial.println("Sent range failed");
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
    else if (c == MODE_QUERY) {
      mode_fn = loop_query;
    }
    else if (c == MODE_SAVE) {
      loop_save();
    }
    else if (c == MODE_INFO) {
      loop_info();
    }
    sentAck = false;
    receivedAck = false;
    lastActivity = 0;
  }
  mode_fn();
}

void loop_poll() {
  int32_t curMillis = millis();
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (curMillis - lastActivity > resetPeriod) {
      current_tag++;
      if (current_tag > NUM_TAGS) {
        current_tag = 1;
      }
      expectedMsgId = POLL;
      transmitPollReq();
      noteActivity();
    }
    return;
  }

  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL_REQ) {
      noteActivity();
    }
    else if (msgId == POLL_ACK) {
      DW1000.getTransmitTimestamp(timePollAckSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      // unexpected message, start over again (except if already POLL)
      protocolFailed = true;
    }
    if (msgId == POLL) {
      // on POLL we (re-)start, so no protocol failure
      protocolFailed = false;
      DW1000.getReceiveTimestamp(timePollReceived);
      expectedMsgId = RANGE;
      transmitPollAck();
      noteActivity();
    }
    else if (msgId == RANGE) {
      DW1000.getReceiveTimestamp(timeRangeReceived);
      expectedMsgId = POLL;
      if (!protocolFailed) {
        timePollSent.setTimestamp(data + 1);
        timePollAckReceived.setTimestamp(data + 6);
        timeRangeSent.setTimestamp(data + 11);
        // (re-)compute range as two-way ranging is done
        computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
        transmitRangeReport(timeComputedRange.getAsMicroSeconds());
        float distance = timeComputedRange.getAsMeters();
        Serial.print("Tag: "); Serial.print(current_tag);
        Serial.print("\t Range: "); Serial.print(distance); Serial.print(" m");
        Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
        Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
        //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
        //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
        //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
        // update sampling rate (each second)
        successRangingCount++;
        if (curMillis - rangingCountPeriod > 1000) {
          samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
          rangingCountPeriod = curMillis;
          successRangingCount = 0;
        }
      }
      else {
        transmitRangeFailed();
      }

      // Request to poll the next tag
      current_tag++;
      if (current_tag > NUM_TAGS) {
        current_tag = 1;
      }
      expectedMsgId = POLL;
      transmitPollReq();
      noteActivity();
    }
  }
}

void loop_query() {
  // TODO
  Serial.println(F("TODO query"));
}

void loop_save() {
  // TODO
  Serial.println(F("TODO save"));
}

void loop_info() {
  Serial.println(F("### DW1000 Arduino Ranging Anchor ###"));

  // Information
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print(F("Device ID: ")); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print(F("Unique ID: ")); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print(F("Network ID & Device Address: ")); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print(F("Device mode: ")); Serial.println(msg);
  Serial.print("Ranging ID: "); Serial.println(DWID);
}
