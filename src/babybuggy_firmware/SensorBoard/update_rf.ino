#define CH1_PIN A2
#define CH2_PIN A3
#define CH3_PIN A4
#define CH4_PIN A5
#define CH5_PIN 6
#define CH6_PIN 7

#define _NUM_PULSE_ON_UPDATE_   6
#define _PULSE_TIMEOUT_         4

int rf_vals_buf[6];
int rf_vals[] = {0, 0, 0, 0, 0, 0};
int rf_errs[] = {0, 0, 0, 0, 0, 0};
int rf_pins[] = {CH1_PIN, CH2_PIN, CH3_PIN, CH4_PIN, CH5_PIN, CH6_PIN};
float rf_k = .3;

int index = 0;


void init_rf() {
  for(int i = 0; i < 6; i++) pinMode(rf_pins[i], INPUT);
}

bool takePulse() {
  int new_val = pulseIn(rf_pins[index], HIGH, _PULSE_TIMEOUT_);
  rf_errs[index] = new_val - rf_vals_buf[index];
  rf_vals_buf[index] = rf_vals_buf[index] + rf_k*rf_errs[index];

  if(++index == 6) {
    index = 0;
    return true;
  }
  return false;
}

void update_rf() {
  bool flag = false;
  for(int i = 0; i < _NUM_PULSE_ON_UPDATE_; i++) if(flag = takePulse()) break;
  
  if(flag) for(int i = 0; i < 6; i++) rf_vals[i] = rf_vals_buf[i];

  /*
  for(int i = 0; i < 6; i++){
    int new_val   = pulseIn(rf_pins[i], HIGH, _PULSE_TIMEOUT_);
    rf_errs[i]    = new_val - rf_vals[i];
    rf_vals[i]    = rf_vals[i] + rf_k*rf_errs[i];
  }
  */
}

void write_rf_vals() {
  for(int i = 0; i < 6; i++){
    Serial.print(rf_vals[i]);
    Serial.print("\t");
  }
}
