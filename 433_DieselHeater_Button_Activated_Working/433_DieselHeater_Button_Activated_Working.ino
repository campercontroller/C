

#include <RCSwitch.h>       //Using library from  https://github.com/sui77/rc-switch/

RCSwitch rf433 = RCSwitch();

//On Button
const int button1Pin = 3;     // the number of the pushbutton pin
int button1State = 0;

//Off Button
const int button2Pin = 4;     // the number of the pushbutton pin
int button2State = 0;

//Up Button
const int button3Pin = 5;     // the number of the pushbutton pin
int button3State = 0;

//Down Button
const int button4Pin = 6;     // the number of the pushbutton pin
int button4State = 0;



void setup() {



  rf433.enableTransmit(10);

  rf433.setPulseLength(401);


  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);
  pinMode(button4Pin, INPUT);

 
}

void loop() {

while(Serial1.available()>0) {

    char c = Serial1.read();
    data_from_display += c; 
    delay(50);
    
  }
  
  data_from_display.trim();
  
  if(data_from_display.length() > 0) {
    
    Serial.println(data_from_display);    
   
    data_from_display = ""; // reset this to empty
    
  }
}

  button1State = digitalRead(button1Pin);  // read the state of the pushbutton value:
  button2State = digitalRead(button2Pin);  // read the state of the pushbutton value:
  button3State = digitalRead(button3Pin);  // read the state of the pushbutton value:
  button4State = digitalRead(button4Pin);  // read the state of the pushbutton value:

  //On
  if (button1State == HIGH) {
    rf433.setRepeatTransmit (20);
    rf433.send("011100000101000100011000");        // Transmitter is connected to Arduino Pin #10
    Serial.print ("button1");
  }
  
  {
    //Off
    button2State = digitalRead(button2Pin);  // read the state of the pushbutton value:
    if (button2State == HIGH) {
      rf433.setRepeatTransmit (20);
      rf433.send("011100000101000100010100");        // Transmitter is connected to Arduino Pin #10
      Serial.print ("button2");
    }

    {
      //Up
      button3State = digitalRead(button3Pin);  // read the state of the pushbutton value:
      if (button3State == HIGH) {
        rf433.setRepeatTransmit (1);
        rf433.send("011100000101000100010010");        // Transmitter is connected to Arduino Pin #10
       Serial.print ("button3");
      }
      {
        //Down
        button4State = digitalRead(button4Pin);  // read the state of the pushbutton value:
        if (button4State == HIGH) {
          rf433.setRepeatTransmit (1);
          rf433.send("011100000101000100010001");        // Transmitter is connected to Arduino Pin #10
          Serial.print ("button4");
        }

      }

    }

  }
}
