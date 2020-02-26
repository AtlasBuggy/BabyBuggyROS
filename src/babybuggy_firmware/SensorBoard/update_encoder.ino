
#define NUM_ENCODERS        2

unsigned encoder_interrupt[NUM_ENCODERS] = {2, 3};
unsigned encoder_input[NUM_ENCODERS] = {4, 5};
int encoder_vals[NUM_ENCODERS];

void init_encoder() {
  for(int i = 0; i < NUM_ENCODERS; i++) {
    interruptSetup(i);
    pinMode(encoder_input[i], INPUT);
  }
}

void interruptSetup(int i) {
  switch(i) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(encoder_interrupt[i]), update_encoder_0, RISING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(encoder_interrupt[i]), update_encoder_1, RISING);
      break;
    default:
      Serial.println("ERR: undefinied interrupt in encoder.");
      break;
  }
}

void update_encoder_0() {
  update_encoder(0);
}

void update_encoder_1() {
  update_encoder(1);
}

bool update_encoder(int encoder) {
  if(encoder >= NUM_ENCODERS) return false;
  
  int val = digitalRead(encoder_input[encoder]);
  if(val == HIGH) encoder_vals[encoder] = encoder_vals[encoder] + 1;
  else encoder_vals[encoder] = encoder_vals[encoder] - 1;
  
  return true;
}

void write_encoder_vals() {
  for(int i = 0; i < NUM_ENCODERS; i++) {
    Serial.print(encoder_vals[i]);
    Serial.print("\t");
  }
}
