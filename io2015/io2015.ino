// ####################################
// Assert macro
// ####################################

#define assert(val, msg) char msg[val ? 0 : -1];
// ####################################
// Libs
// ####################################

#include <inttypes.h>
#include <arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Metro.h>
#include <stdlib.h>

#define _ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <SFE_VL6180X.h>

// ####################################
// Data
// ####################################

typedef struct {
  uint8_t servo_popcorn_g;
  uint8_t servo_popcorn_d;
  uint8_t servo_gobelet_g;
  uint8_t servo_gobelet_d;
  uint8_t servo_tapi_g;
  uint8_t servo_tapi_d;
  uint8_t servo_guide_g;
  uint8_t servo_guide_d;
  uint8_t servo_pince_g;
  uint8_t servo_pince_d;

  uint8_t dout_ascenseur_pwm;
  uint8_t dout_ascenseur_en1;
  uint8_t dout_ascenseur_en2;
  int16_t dout_ascenseur_position;
  uint8_t dout_chenille_ascenseur_pwm;
  uint8_t dout_chenille_ascenseur_en1;
  uint8_t dout_chenille_ascenseur_en2;
  uint8_t dout_chenille_pwm;
} Consignes;

typedef struct {
  uint8_t din_couleur;
  uint8_t din_test;
  uint8_t din_jack;
  uint8_t din_homming_asc;
  uint8_t din_gobelet_g;
  uint8_t din_gobelet_d;
  uint8_t din_chenille_haute;
  uint8_t din_chenille_intermediaire;
  uint8_t din_chenille_basse;
  uint8_t ain_courant_servos;
  uint8_t ain_courant_moteurs;
  int32_t encodeur_ascenseur;
  uint8_t telemetre_ascenseur; // mm
} Output;

assert(sizeof(Consignes) == 19, taille_consignes);
assert(sizeof(Output) == 16, taille_output);

// ####################################
// PINS
// ####################################

#define PIN_LED 6

#define PIN_SERVO_POPCORN_G 8
#define PIN_SERVO_POPCORN_D 9
#define PIN_SERVO_GOBELET_G 10
#define PIN_SERVO_GOBELET_D 11
#define PIN_SERVO_TAPI_G 12
#define PIN_SERVO_TAPI_D 13
#define PIN_SERVO_GUIDE_G 14
#define PIN_SERVO_GUIDE_D 15
#define PIN_SERVO_PINCE_G 16
#define PIN_SERVO_PINCE_D 17

#define PIN_DOUT_BUZZ PIN_F1
#define PIN_DIN_COULEUR 38
#define PIN_DIN_TEST 40
#define PIN_DIN_JACK PIN_D5
#define PIN_DIN_HOMMING_ASC PIN_F3
#define PIN_DIN_GOBELET_G 42
#define PIN_DIN_GOBELET_D 43
#define PIN_DIN_CHENILLE_HAUTE 2
#define PIN_DIN_CHENILLE_INTERMEDIAIRE 3
#define PIN_DIN_CHENILLE_BASSE 7

#define PIN_DOUT_ASCENSEUR_PWM PIN_B7
#define PIN_DOUT_ASCENSEUR_EN1 PIN_B3
#define PIN_DOUT_ASCENSEUR_EN2 PIN_B4

#define PIN_DOUT_CHENILLE_ASCENSEUR_PWM PIN_B6
#define PIN_DOUT_CHENILLE_ASCENSEUR_EN1 PIN_B1
#define PIN_DOUT_CHENILLE_ASCENSEUR_EN2 PIN_B2

#define PIN_DOUT_CHENILLE_PWM PIN_B5

#define PIN_AIN_COURANT_SERVOS 7
#define PIN_AIN_COURANT_MOTEURS 6

#define PIN_ENCODER_ASCENSEUR_A 18
#define PIN_ENCODER_ASCENSEUR_B 19

#define VL6180X_ADDRESS 0x29
#define VL6180X_ADDRESS_2 0x28
#define PIN_DOUT_LASER_ALIM PIN_D4

// ####################################
// Globals
// ####################################

#define SERVO_OFF 255

// 20ms update, Timer 3, PWM 14, 15, 16
Servo servo_popcorn_g;
Servo servo_popcorn_d;
Servo servo_gobelet_g;
Servo servo_gobelet_d;
Servo servo_tapi_g;
Servo servo_tapi_d;
Servo servo_guide_g;
Servo servo_guide_d;
Servo servo_pince_g;
Servo servo_pince_d;

Encoder encoderAscenseur(PIN_ENCODER_ASCENSEUR_A, PIN_ENCODER_ASCENSEUR_B);

VL6180x telemetreAscenseur(VL6180X_ADDRESS);

uint8_t bufferConsignes[sizeof(Consignes)];
int8_t bufferPosition;
Consignes consignes;
Output output;

Metro communicationMetro = Metro(100);
Metro ledMetro = Metro(100);

uint8_t led_state = 1;
uint8_t buzz_state = 1;

// ####################################
// Setup
// ####################################

void setup() {
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_DIN_COULEUR, INPUT_PULLUP);
  pinMode(PIN_DIN_TEST, INPUT_PULLUP);
  pinMode(PIN_DIN_JACK, INPUT_PULLUP);
  pinMode(PIN_DIN_HOMMING_ASC, INPUT_PULLUP);
  pinMode(PIN_DIN_GOBELET_G, INPUT_PULLUP);
  pinMode(PIN_DIN_GOBELET_D, INPUT_PULLUP);
  pinMode(PIN_DIN_CHENILLE_HAUTE, INPUT_PULLUP);
  pinMode(PIN_DIN_CHENILLE_INTERMEDIAIRE, INPUT_PULLUP);
  pinMode(PIN_DIN_CHENILLE_BASSE, INPUT_PULLUP);

  pinMode(PIN_DOUT_BUZZ, OUTPUT);
  pinMode(PIN_DOUT_ASCENSEUR_PWM, OUTPUT);
  pinMode(PIN_DOUT_ASCENSEUR_EN1, OUTPUT);
  pinMode(PIN_DOUT_ASCENSEUR_EN2, OUTPUT);
  pinMode(PIN_DOUT_CHENILLE_ASCENSEUR_PWM, OUTPUT);
  pinMode(PIN_DOUT_CHENILLE_ASCENSEUR_EN1, OUTPUT);
  pinMode(PIN_DOUT_CHENILLE_ASCENSEUR_EN2, OUTPUT);
  pinMode(PIN_DOUT_CHENILLE_PWM, OUTPUT);
  pinMode(PIN_DOUT_LASER_ALIM, OUTPUT);

  consignes.servo_popcorn_g = SERVO_OFF;
  consignes.servo_popcorn_d = SERVO_OFF;
  consignes.servo_gobelet_g = SERVO_OFF;
  consignes.servo_gobelet_d = SERVO_OFF;
  consignes.servo_tapi_g = SERVO_OFF;
  consignes.servo_tapi_d = SERVO_OFF;
  consignes.servo_guide_g = SERVO_OFF;
  consignes.servo_guide_d = SERVO_OFF;
  consignes.servo_pince_g = SERVO_OFF;
  consignes.servo_pince_d = SERVO_OFF;

  consignes.dout_ascenseur_pwm = 0;
  consignes.dout_ascenseur_en1 = 0;
  consignes.dout_ascenseur_en2 = 0;
  consignes.dout_ascenseur_position = -1;
  consignes.dout_chenille_ascenseur_pwm = 0;
  consignes.dout_chenille_ascenseur_en1 = 0;
  consignes.dout_chenille_ascenseur_en2 = 0;
  consignes.dout_chenille_pwm = 0;

  writeSensors();

  bufferPosition = -2;

  Serial.begin(115200);
  Wire.begin(); //Start I2C library
  digitalWrite(PIN_DOUT_LASER_ALIM, HIGH);
  if (telemetreAscenseur.VL6180xInit() != 0) {
    Serial.println("VL6180 FAILED TO INITALIZE"); //Initialize device and check for errors
  } else {
    Serial.println("VL6180 OK");
    telemetreAscenseur.VL6180xDefautSettings();
  }
  //telemetreAscenseur.changeAddress(VL6180X_ADDRESS, VL6180X_ADDRESS);
}

// ####################################
// Loop
// ####################################

void loop() {
  readSerial();
  readSensors();
  writeSensors();

  //buzz_state = 1 - buzz_state;
  //digitalWrite(PIN_DOUT_BUZZ, buzz_state);

  if (communicationMetro.check()) {
    writeSerial();
  }

  if (ledMetro.check()) {
    showLEDActivity();
  }
}

// ####################################
// Autres
// ####################################

inline void showLEDActivity() {
  if (Serial.dtr()) {
    led_state = 1 - led_state;
  } else {
    led_state = 1;
  }
  digitalWrite(PIN_LED, led_state);
}

inline void readSerial(void) {
  while (Serial.available()) {
    uint8_t byte = Serial.read();

    if (bufferPosition == -2) {
      if (byte == '@') {
        bufferPosition = -1;
      } else {
        bufferPosition = -2;
      }
    } else if (bufferPosition == -1) {
      if (byte == '@') {
        bufferPosition = 0;
      } else {
        bufferPosition = -2;
      }
    } else if (bufferPosition >= 0) {
      bufferConsignes[bufferPosition] = byte;
      bufferPosition++;

      if (bufferPosition == sizeof(Consignes)) {
        memcpy(&consignes, &bufferConsignes, sizeof(Consignes));
        bufferPosition = -2;
      }
    }
  }
}

inline void writeSerial() {
  Serial.write('@');
  Serial.write((uint8_t*) &output, sizeof(Output));
  //Serial.println(consignes.dout_ascenseur_position);
  Serial.send_now();
}

inline void writeSensors() {
  servo_apply(servo_popcorn_g, PIN_SERVO_POPCORN_G, consignes.servo_popcorn_g);
  servo_apply(servo_popcorn_d, PIN_SERVO_POPCORN_D, consignes.servo_popcorn_d);
  servo_apply(servo_gobelet_g, PIN_SERVO_GOBELET_G, consignes.servo_gobelet_g);
  servo_apply(servo_gobelet_d, PIN_SERVO_GOBELET_D, consignes.servo_gobelet_d);
  servo_apply(servo_tapi_g, PIN_SERVO_TAPI_G, consignes.servo_tapi_g);
  servo_apply(servo_tapi_d, PIN_SERVO_TAPI_D, consignes.servo_tapi_d);
  servo_apply(servo_guide_g, PIN_SERVO_GUIDE_G, consignes.servo_guide_g);
  servo_apply(servo_guide_d, PIN_SERVO_GUIDE_D, consignes.servo_guide_d);
  servo_apply(servo_pince_g, PIN_SERVO_PINCE_G, consignes.servo_pince_g);
  servo_apply(servo_pince_d, PIN_SERVO_PINCE_D, consignes.servo_pince_d);

  if (consignes.dout_ascenseur_position < 0) { // mode manuel
    analogWrite(PIN_DOUT_ASCENSEUR_PWM, consignes.dout_ascenseur_pwm);
    digitalWrite(PIN_DOUT_ASCENSEUR_EN1, consignes.dout_ascenseur_en1);
    digitalWrite(PIN_DOUT_ASCENSEUR_EN2, consignes.dout_ascenseur_en2);
  } else { // mode auto
    const uint16_t erreur_consigne = abs(consignes.dout_ascenseur_position - output.encodeur_ascenseur);
    const uint16_t erreur_marge = 500;

    uint8_t pwm; // = min(pwm_max, max(pwm_min, erreur_consigne * Kp));
    if (erreur_consigne > 8000) {
      pwm = 250;
    } else if (erreur_consigne > 4000) {
      pwm = 200;
    } else {
      pwm = 150;
    }

    if (consignes.dout_ascenseur_position == 0) { // vers le homming
      if (output.din_homming_asc) {
        ascenseur_stop();
        encoderAscenseur.write(0);
      } else {
        ascenseur_descendre(150);
      }
    } else { // vers la position
      if (erreur_consigne < erreur_marge) {
        ascenseur_stop();
      } else if (consignes.dout_ascenseur_position > output.encodeur_ascenseur) {
        ascenseur_monter(pwm);
      } else {
        ascenseur_descendre(pwm);
      }
    }
  }

  analogWrite(PIN_DOUT_CHENILLE_ASCENSEUR_PWM, consignes.dout_chenille_ascenseur_pwm);
  digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN1, consignes.dout_chenille_ascenseur_en1);
  digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN2, consignes.dout_chenille_ascenseur_en2);

  if (consignes.dout_chenille_ascenseur_en1 && !consignes.dout_chenille_ascenseur_en2 && !output.din_chenille_basse) {
    analogWrite(PIN_DOUT_CHENILLE_ASCENSEUR_PWM, consignes.dout_chenille_ascenseur_pwm);
    digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN1, consignes.dout_chenille_ascenseur_en1);
    digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN2, consignes.dout_chenille_ascenseur_en2);
  } else if (!consignes.dout_chenille_ascenseur_en1 && consignes.dout_chenille_ascenseur_en2 && !output.din_chenille_haute) {
    analogWrite(PIN_DOUT_CHENILLE_ASCENSEUR_PWM, consignes.dout_chenille_ascenseur_pwm);
    digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN1, consignes.dout_chenille_ascenseur_en1);
    digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN2, consignes.dout_chenille_ascenseur_en2);
  } else {
    analogWrite(PIN_DOUT_CHENILLE_ASCENSEUR_PWM, 0);
    digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN1, LOW);
    digitalWrite(PIN_DOUT_CHENILLE_ASCENSEUR_EN2, LOW);
  }

  analogWrite(PIN_DOUT_CHENILLE_PWM, consignes.dout_chenille_pwm);
}

inline void readSensors() {
  output.din_couleur = digitalRead(PIN_DIN_COULEUR);
  output.din_test = digitalRead(PIN_DIN_TEST);
  output.din_jack = digitalRead(PIN_DIN_JACK);
  output.din_homming_asc = digitalRead(PIN_DIN_HOMMING_ASC);
  output.din_gobelet_g = digitalRead(PIN_DIN_GOBELET_G);
  output.din_gobelet_d = digitalRead(PIN_DIN_GOBELET_D);
  output.din_chenille_haute = digitalRead(PIN_DIN_CHENILLE_HAUTE);
  output.din_chenille_intermediaire = digitalRead(PIN_DIN_CHENILLE_INTERMEDIAIRE);
  output.din_chenille_basse = digitalRead(PIN_DIN_CHENILLE_BASSE);

  output.ain_courant_moteurs = analogRead(PIN_AIN_COURANT_MOTEURS);
  output.ain_courant_servos = analogRead(PIN_AIN_COURANT_SERVOS);

  output.encodeur_ascenseur = encoderAscenseur.read();

  output.telemetre_ascenseur = telemetreAscenseur.getDistance();
}

// ####################################
// Utilitaires / factorisation
// ####################################

inline void proximity_reset() {
  digitalWrite(PIN_DOUT_LASER_ALIM, LOW);
  delay(1);
  digitalWrite(PIN_DOUT_LASER_ALIM, HIGH);
  delay(1);
  telemetreAscenseur.VL6180xInit();
  telemetreAscenseur.VL6180xDefautSettings();
}

inline void ascenseur_descendre(uint8_t pwm) {
  // en1 = 0, en2 = 1 => descente
  // en1 = 1, en2 = 0 => montée
  analogWrite(PIN_DOUT_ASCENSEUR_PWM, pwm);
  digitalWrite(PIN_DOUT_ASCENSEUR_EN1, LOW);
  digitalWrite(PIN_DOUT_ASCENSEUR_EN2, HIGH);
}

inline void ascenseur_monter(uint8_t pwm) {
  analogWrite(PIN_DOUT_ASCENSEUR_PWM, pwm);
  digitalWrite(PIN_DOUT_ASCENSEUR_EN1, HIGH);
  digitalWrite(PIN_DOUT_ASCENSEUR_EN2, LOW);
}

inline void ascenseur_stop() {
  analogWrite(PIN_DOUT_ASCENSEUR_PWM, 0);
  digitalWrite(PIN_DOUT_ASCENSEUR_EN1, HIGH);
  digitalWrite(PIN_DOUT_ASCENSEUR_EN2, HIGH);
}

inline void servo_apply(Servo servo, const int pin, const uint8_t consigne) {
  if (consigne == SERVO_OFF) {
    servo.detach();
  } else {
    servo.attach(pin);
    servo.write(consigne);
  }
}
