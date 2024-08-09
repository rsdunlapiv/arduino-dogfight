#define D1_PIN 5
#define D2_PIN 4
#define D3_PIN 0
#define D4_PIN 2
#define D5_PIN 14
#define D6_PIN 12
#define D7_PIN 13

#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>
#include "TinyIRSender.hpp"

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  

  Serial.begin(115200);
  while (!Serial)
       ; // Wait for Serial to become available. Is optimized away for some cores.

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_TINYIR));
  Serial.print(F("Send IR signals at pin "));
  Serial.println(IR_SEND_PIN);

  IrSender.begin(); // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
  

}

/*
 * Set up the data to be sent.
 * For most protocols, the data is build up with a constant 8 (or 16 byte) address
 * and a variable 8 bit command.
 * There are exceptions like Sony and Denon, which have 5 bit address.
 */
uint8_t sCommand = 0x0;
uint8_t sRepeats = 20;

void loop() {
    /*
     * Print current send values
     */
    Serial.println();
    Serial.print(F("Send now: address=0xDE80, command=0x"));
    Serial.print(sCommand, HEX);
    Serial.print(F(", repeats="));
    Serial.print(sRepeats);
    Serial.println();

    Serial.println(F("Send standard NEC with 8 bit address"));
    Serial.flush();

    // Receiver output for the first loop must be: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    IrSender.sendNEC(0xDE80, sCommand, sRepeats);

    /*
     * If you want to send a raw HEX value directly like e.g. 0xCB340102 you must use sendNECRaw()
     */
//    Serial.println(F("Send 32 bit LSB 0xCB340102 with NECRaw()"));
//    IrSender.sendNECRaw(0xCB340102, sRepeats);

    /*
     * If you want to send an "old" MSB HEX value used by IRremote versions before 3.0 like e.g. 0x40802CD3 you must use sendNECMSB()
     */
//    Serial.println(F("Send old 32 bit MSB 0x40802CD3 with sendNECMSB()"));
//    IrSender.sendNECMSB(0x40802CD3, 32, sRepeats);

   

    delay(500);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
}