#include "PinChangeInterrupt.h"

#define CH1_PIN 7
#define CH2_PIN 8
#define CH3_PIN 9
#define CH4_PIN 10
#define CH5_PIN 11
#define CH6_PIN 12

float k = 0.7;
float vals[6] = {0, 0, 0, 0, 0, 0};
unsigned long start_times[6] = {0, 0, 0, 0, 0, 0};
bool new_signal[6] = {false, false, false, false, false, false};

void ch1_callback(void) {
  int ind = 0;
  int PIN = CH1_PIN;

  if(digitalRead(PIN) == HIGH)
  { 
    start_times[ind] = micros();
  }
  else
  {
    if(start_times[ind] && (new_signal[ind] == false))
    {
      int new_val = (int)(micros() - start_times[ind]);
      vals[ind] = vals[ind] + k * (new_val - vals[ind]);
      new_signal[ind] = true;
    }
  }  
}

void ch2_callback(void) {
  int ind = 1;
  int PIN = CH2_PIN;

  if(digitalRead(PIN) == HIGH)
  { 
    start_times[ind] = micros();
  }
  else
  {
    if(start_times[ind] && (new_signal[ind] == false))
    {
      int new_val = (int)(micros() - start_times[ind]);
      vals[ind] = vals[ind] + k * (new_val - vals[ind]);
      new_signal[ind] = true;
    }
  }  
}

void ch3_callback(void) {
  int ind = 2;
  int PIN = CH3_PIN;

  if(digitalRead(PIN) == HIGH)
  { 
    start_times[ind] = micros();
  }
  else
  {
    if(start_times[ind] && (new_signal[ind] == false))
    {
      int new_val = (int)(micros() - start_times[ind]);
      vals[ind] = vals[ind] + k * (new_val - vals[ind]);
      new_signal[ind] = true;
    }
  }  
}

void ch4_callback(void) {
  int ind = 3;
  int PIN = CH4_PIN;

  if(digitalRead(PIN) == HIGH)
  { 
    start_times[ind] = micros();
  }
  else
  {
    if(start_times[ind] && (new_signal[ind] == false))
    {
      int new_val = (int)(micros() - start_times[ind]);
      vals[ind] = vals[ind] + k * (new_val - vals[ind]);
      new_signal[ind] = true;
    }
  }  
}

void ch5_callback(void) {
  int ind = 4;
  int PIN = CH5_PIN;

  if(digitalRead(PIN) == HIGH)
  { 
    start_times[ind] = micros();
  }
  else
  {
    if(start_times[ind] && (new_signal[ind] == false))
    {
      int new_val = (int)(micros() - start_times[ind]);
      vals[ind] = vals[ind] + k * (new_val - vals[ind]);
      new_signal[ind] = true;
    }
  }  
}

void ch6_callback(void) {
  int ind = 5;
  int PIN = CH6_PIN;

  if(digitalRead(PIN) == HIGH)
  { 
    start_times[ind] = micros();
  }
  else
  {
    if(start_times[ind] && (new_signal[ind] == false))
    {
      int new_val = (int)(micros() - start_times[ind]);
      vals[ind] = vals[ind] + k * (new_val - vals[ind]);
      new_signal[ind] = true;
    }
  }  
}

unsigned long prev_time;

void setup() {
  Serial.begin(115200);
  pinMode(CH1_PIN, INPUT); 
  pinMode(CH2_PIN, INPUT); 
  pinMode(CH3_PIN, INPUT); 
  pinMode(CH4_PIN, INPUT);  
  pinMode(CH5_PIN, INPUT);  
  pinMode(CH6_PIN, INPUT);  

  attachPCINT(digitalPinToPCINT(CH1_PIN), ch1_callback, CHANGE);
  attachPCINT(digitalPinToPCINT(CH2_PIN), ch2_callback, CHANGE);
  attachPCINT(digitalPinToPCINT(CH3_PIN), ch3_callback, CHANGE);
  attachPCINT(digitalPinToPCINT(CH4_PIN), ch4_callback, CHANGE);
  attachPCINT(digitalPinToPCINT(CH5_PIN), ch5_callback, CHANGE);
  attachPCINT(digitalPinToPCINT(CH6_PIN), ch6_callback, CHANGE);  

  prev_time = millis();
}



void loop() {
  if (millis() - prev_time > 50){
    for (int i = 0; i < 6; i++){
      Serial.print(vals[i]);
      Serial.print('\t');

      new_signal[i] = false;
    }
    
    Serial.print('\n');

    prev_time = millis();
  }
}
