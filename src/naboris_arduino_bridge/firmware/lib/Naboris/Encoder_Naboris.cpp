#include "Encoder_Naboris.h"

/* ------------------------ *
 * Encoder global variables *
 * ------------------------ */

Encoder rightEncoder(2, 8);
Encoder leftEncoder(3, 12);

long oldLeftPosition = 0;
long newRightPosition = 0;
long oldRightPosition = 0;
long newLeftPosition = 0;
uint32_t prev_enc_time = 0;

char r_enc_print_buffer[R_ENC_BUF_LEN];
char l_enc_print_buffer[L_ENC_BUF_LEN];

void updateEncoders()
{
    uint32_t enc_time = millis();
    newRightPosition = rightEncoder.read();
    newLeftPosition = leftEncoder.read();

    if (newRightPosition != oldRightPosition) {
        // snprintf(r_enc_print_buffer, R_ENC_BUF_LEN, "er%lu\t%li\n", enc_time, newRightPosition);

        oldRightPosition = newRightPosition;
        Serial.print("er");
        Serial.print(enc_time);
        Serial.print('\t');
        Serial.print(newRightPosition);
        Serial.print('\n');
        // Serial.print(r_enc_print_buffer);
    }

    if (newLeftPosition != oldLeftPosition) {
        // snprintf(l_enc_print_buffer, L_ENC_BUF_LEN, "el%lu\t%li\n", enc_time, newLeftPosition);

        oldLeftPosition = newLeftPosition;
        Serial.print("el");
        Serial.print(enc_time);
        Serial.print('\t');
        Serial.print(newLeftPosition);
        Serial.print('\n');
        // Serial.print(l_enc_print_buffer);
    }
}
