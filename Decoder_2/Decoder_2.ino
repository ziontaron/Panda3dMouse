/* 
This code uses the example Debounce code, 
and the Example code for Rotary Encoders from the Arduino Playground
and also the Arduino Core changes done in Anything Arduino episode 5: https://www.youtube.com/watch?v=ubORsI4uWuo
You can watch the build video here: https://youtu.be/7CHIY-KhFfk

Connect button between pin 9 and ground
Connect the encoder to pin 6, 7 and ground on the common pin
 */

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 9;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin
const int encoder0PinA = 7;
const int encoder0PinB = 8;

// Variables will change:
int bstate = 0; //0 is paused, 1 is playing
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
long longpressDelay = 500;    // If we hold it longer than 500ms then it is a long press.
int val;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
long encoderLastValue=0;
//int lastCommand; //0=volup 1=voldown 2=next, 3=prev
int lastDirection; //0=--, 1=++
int n = LOW;
int reading;

void setup() {
  pinMode (encoder0PinA, INPUT_PULLUP);
  pinMode (encoder0PinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  Serial.begin (9600);
}

void loop() {
  //encoder_code
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) { 
      //to make a double check we are going the right direction.
      if (lastDirection == 0) {
        encoder0Pos--;
      }
      lastDirection = 0;
    } else {
      if (lastDirection == 1) {
        encoder0Pos++;
      }
      lastDirection = 1;
    }
  }
  encoder0PinALast = n;
  
  //We make the vol up/down descision with the lastButttonState.
  if (lastButtonState == HIGH) { //((millis() - lastDebounceTime) < longpressDelay) 
    //nothing happens with the button so if the rotary encoder moves now, it is volume. 
    //Well actually this code runs also if the button isnt released, so we need to make sure it isnt a longpress aswell... which we do two rows up.
    if (encoderLastValue > encoder0Pos) {
      //volume down
      //Remote.decrease();
      //Remote.clear();
      Serial.println ("Volume down");
    }
    else if (encoderLastValue < encoder0Pos) {
      //volume up
      //Remote.increase();
      //Remote.clear();
      Serial.println ("Volume up");
    }
  }
  
  //We must make the next/prev descisions before the button press if.
  if (((millis() - lastDebounceTime) > longpressDelay) && (lastButtonState == LOW)) {
    //So if the button has been held longer than 500ms, then if the rotary encoder is turned it is next/prev
    if (encoderLastValue > encoder0Pos) {
      //previous
      //Remote.previous();
      //Remote.clear();
      delay(500);
      Serial.println ("Previous");
    }
    else if (encoderLastValue < encoder0Pos) {
      //next
      //Remote.next();
      //Remote.clear();
      delay(500);
      Serial.println ("Next");
    }
  } 
  
  //button_code
  reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    //The button is (probalby) pressed or released
    if (reading==LOW) {
      //if it indeed is pressed then:
      //Set the start time
      lastDebounceTime = millis();
    } else {
       //if it is high the button was released, check how long time it was pressed...
       if ((millis() - lastDebounceTime) > longpressDelay) {
         //the button was pressed 500ms or longer
         //we cant handle next/prev here because here the button is already released again, se code above for next/prev...
         Serial.println ("Long press");
       }
       else if ((millis() - lastDebounceTime) > debounceDelay) {
          //it was only pressed a short while, but more than 50ms so it is a play/pause press
          Serial.println ("Play/Pause");
          if (bstate == 0) {
             //Remote.play();
             //Remote.clear();
             bstate = 1;
           } else {
             //Remote.pause();
             //Remote.clear();
             bstate = 0;
           }
       } else {
         //Serial.println (millis()-lastDebounceTime);
       }
    }
  }
  
  
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
  encoderLastValue=encoder0Pos;

}
