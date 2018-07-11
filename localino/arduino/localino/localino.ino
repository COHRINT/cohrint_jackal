/*
  Cycle through all channels 1-7 (!6) after distance calcuation and switch who's the tag and anchor
 */

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId;
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
#define LEN_DATA 18 // last two bytes are [to who, from who]
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint8_t maxNumTimeouts = 4; // wait this number of timeouts if switching frequencies before switching back
uint8_t numTimeouts = 0;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

// Special Protocol Stuff
uint8_t my_number = 2; // 1 indicates it'll start as master (or a tag)
char syncCode = 's';
uint16_t trans_num = 0;
const uint16_t transition_delay = 100;
uint8_t msg_sent = 0;
uint8_t my_freq = 1;
uint8_t switchedFreq = 0;
uint8_t startingFreq = 1;
uint8_t otherLocalino = 0;

typedef enum State_t {
  TAG = 0,
  ANCHOR = 1
} State;

State my_state;

void setup() {
    Serial.begin(9600);
    syncPC();
    getLocalinoNumber();
    initDW1000();
    becomeAnchor(); // start as anchor
}

void initDW1000() {
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  // general configuration
  if ( setDW(startingFreq) ) freezeError(13);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
}

void print_trans_num()
{
  Serial.print("\r(");
  Serial.print(trans_num);
  Serial.print(") ");

}

void becomeTag() {
  msg_sent = 1;
  trans_num += 1;
  print_trans_num();
  Serial.println("------------TAG-----------");
  delay(transition_delay);
  my_state = TAG;
  // transmit a poll msg
  expectedMsgId = POLL_ACK;
  receiver();
  transmitPoll();
  noteActivity();
}

void becomeAnchor() { // can I move the bulk of this to just the initializing state ?
  msg_sent = 1;
  trans_num += 1;
  print_trans_num();
  Serial.println("----------ANCHOR----------");
  delay(transition_delay);
  my_state = ANCHOR;
  // wait for a poll msg
  expectedMsgId = POLL;
  receiver();
  noteActivity();
  // for first time ranging frequency computation
  rangingCountPeriod = millis();
}

int8_t setDW(uint8_t freq) {
  if ( !freq || freq > 7 || freq == 6 ) // check for invalid frequency
    return -1;
  my_freq = freq; // only time we modify is when we actually change
  Serial.print("\rSetting freq = "); Serial.println(freq);
  DW1000.newConfiguration();
  DW1000.setDefaults(); // we'll change this to dyanimically take a 
  DW1000.setDeviceAddress(my_number);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, freq);
  DW1000.commitConfiguration();
  return 0;
}

/*
  Receive Localino's number from the computer
 */
void getLocalinoNumber() {
  delay(250); // let serial intialize in case it hasn't
  while(true) { // block on getting number
    if (Serial.available() > 0) { // check if data available
      myNumber = Serial.read();
      if (myNumber != 0) { // check if we got the go ahead character
	Serial.print("my number= "); Serial.println(myNumber);
	break; // we received our number
      }
    }
  }
}



/*
  Gets in sync with the PC,
  First receive a char from the PC
  Send an ACK char back
 */
void syncPC() {
  delay(250); // let serial intialize in case it hasn't
  char in = 'a';
  while(true) { // block on syncing 
    if (Serial.available() > 0) { // check if data available
      in = Serial.read();
      if (in == syncCode) { // check if we got the go ahead character
	Serial.print(syncCode); // let's send the sync code back
	  break;
	}
      }
  }
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void freezeError(uint8_t num) {
  Serial.print("Error ");
  Serial.println(num);
  while(1);
}

uint8_t incrFreq(uint8_t cur) {
  if (cur == 5)
    return 7;
  else
    return ( ( cur % 7 ) + 1 ); // channels 1 - 7 (excluding 6)
}

uint8_t decrFreq(uint8_t cur) {
  if (cur == 7) return 6;
  else if (cur == 0) return 7;
  else return cur - 1;
}

void resetInactive() {
    // anchor listens for POLL
  if ( my_state == ANCHOR ) {

    // we didn't fully send a message
    if ( msg_sent == 0 ) {
      if ( expectedMsgId == RANGE ) {
	transmitPollAck();
      }
      else {
	freezeError( 1 );	
      }
    }

    // didn't receive a message
    else if (expectedMsgId == POLL) {
	receiver();
    }
    else if (expectedMsgId == RANGE) {
	transmitPollAck();
	receiver(); // do nothing keep waiting on the RANGE
      }
    else
      freezeError(4);
  } // end ANCHOR

  /* ######################## TAG #################################### */

  else {
    // check for didn't send a message
    if ( msg_sent == 0 ) {
      if ( expectedMsgId == POLL_ACK ) {
	transmitPoll();
      }
      else if (expectedMsgId == POLL) {
	transmitRange();
      }
      else
	freezeError(2);
    }

    // didn't receive a msg
    else if (expectedMsgId == POLL_ACK) {
      numTimeouts++;
      if ( numTimeouts >= maxNumTimeouts ) { // switch frequency (back & forth)
	if (switchedFreq) { // check if we've switched freq
	  if ( setDW( decrFreq(my_freq) ) ) freezeError(11) ; // switch backwards
	  switchedFreq = 1;
	}
	else { // switch forwards
	  if ( setDW( incrFreq(my_freq) ) ) freezeError(11) ;
	  switchedFreq = 0;
	}
	numTimeouts = 0;
      }
      transmitPoll();
    }
    else if (expectedMsgId == POLL) { // we receive a poll to indicate a switch to an anchor
      transmitRange();
    }
    else
      freezeError(3);
  }
  noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPoll() {
    msg_sent = 0;  
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitPollAck() {
  msg_sent = 0;
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL_ACK;
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRange() {
  msg_sent = 0;  
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
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void transmitRangeReport(float curRange) {
  msg_sent = 0;
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_REPORT;
  // write final ranging result
  memcpy(data + 1, &curRange, 4);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

/* void transmitRangeFailed() { */
/*     DW1000.newTransmit(); */
/*     DW1000.setDefaults(); */
/*     data[0] = RANGE_FAILED; */
/*     DW1000.setData(data, LEN_DATA); */
/*     DW1000.startTransmit(); */
/* } */

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
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
    switch( my_state ) {
    case TAG:
      loop_tag();
      break;
    case ANCHOR:
      loop_anchor();
      break;
    }
}

void loop_tag() {
  //  Serial.println("Looping Tag\r");
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (millis() - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  // continue on any success confirmation  
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    msg_sent = 1;
    if (msgId == POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
      //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
    } else if (msgId == RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
    }
    noteActivity();
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      // unexpected message, start over again
      /* if ( (expectedMsgId == RANGE_REPORT) && (msgId == POLL) ) { // check if the other localino has already switched over */
      /* 	becomeAnchor(); */
      /* 	noteActivity(); */
      /* }  */
      if (msgId == RANGE) { // other localino hasn't heard our POLL
	transmitPoll();
      }
      else {
	Serial.print("Received wrong message # "); Serial.println(msgId);
	freezeError(6);
      }
	/* expectedMsgId = POLL_ACK; */
	/* transmitPoll(); */
	/* noteActivity(); */
      return;
    }
    if (msgId == POLL_ACK) {
      switchedFreq = 0; // we've successfully changed frequencies, reset this switching scheme
      DW1000.getReceiveTimestamp(timePollAckReceived);
      expectedMsgId = POLL;
      transmitRange();      
      noteActivity();
      print_trans_num();
      Serial.println("POLL_ACK");
    } 
    else if (msgId == POLL) {
      if ( setDW( incrFreq(my_freq) ) ) freezeError(10); // switch frequencies
      becomeAnchor();
    }
    else
      freezeError(9);
    }
  }




/* ################################################################### */

void loop_anchor()
  byte toContact = 0;
  if (Serial.available() > 0) {
    toContact = Serial.read()
  }
  
  int32_t curMillis = millis(); 
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL_ACK) {
      DW1000.getTransmitTimestamp(timePollAckSent);
    }
    noteActivity();
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      if (msgId == POLL) { // likely it didn't receive the POLL_ACK
	transmitPollAck();
      }
      else {
	// unexpected message, start over again (except if already POLL)
	Serial.print("Received wrong message # "); Serial.println(msgId);
	freezeError(7);
	protocolFailed = true;
      }
      return;
    }
    if (msgId == POLL) {
      // on POLL we (re-)start, so no protocol failure
      protocolFailed = false;
      DW1000.getReceiveTimestamp(timePollReceived);
      expectedMsgId = RANGE;
      transmitPollAck();
      noteActivity();
      Serial.println("\rPOLL");
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
	float distance = timeComputedRange.getAsMeters();
	//	transmitRangeReport(distance);
	print_trans_num();
	Serial.print("\rRange: "); Serial.println(distance);
	noteActivity();
	becomeTag();		
      }
    }
  }
}

