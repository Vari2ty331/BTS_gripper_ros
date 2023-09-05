/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

const int singleFinger_button_pin = 8;
const int doubleFinger_button_pin = 10;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(singleFinger_button_pin, INPUT);
  pinMode(doubleFinger_button_pin, INPUT);
  
  //Enable the pullup resistor on the button
  digitalWrite(singleFinger_button_pin, HIGH);
  digitalWrite(doubleFinger_button_pin, HIGH);
  
  //The button is a normally button
  //OR 조건 AND 조건 바꿔봐야함
  last_reading = ! (digitalRead(singleFinger_button_pin)||digitalRead(doubleFinger_button_pin));
 
}

void loop()
{
  
  bool reading = ! digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    // pub_button.publish(&pushed_msg);
    published = true;
  }

  pub_button.publish(&pushed_msg);
  last_reading = reading;
  
  nh.spinOnce();
}
