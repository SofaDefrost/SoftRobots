#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* #########################################################
  IMU Sensor info
  ######################################################### */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/* #########################################################
  Motors info
   ######################################################### */

#define TIMER1TICK 0.016f

// SetServoAngle, return 1 if sucess 0 if not
int SetServoAngle(int);
void SetupServos();



void SetupServos() {
  // configure Timer1 in FastPWM Mode, Prescaler = 256, tick = 16us, period = 4096us
  TCCR1A = 0b00000001; // WGM11:0 = 01
  TCCR1B = 0b00001100; // WGM13:2 = 01 CS = 100
  OCR1A = 0;
  OCR1B = 0;
  // configure Timer 2, same as Timer 1
  TCCR2A = 0b00000011; // WGM11:0 = 11
  TCCR2B = 0b00000110; // WGM12 = 1 CS = 110
  OCR2A = 0;
  OCR2B = 0;
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(3, OUTPUT);
}


/*
   Start or update the periodic pulse generation for the corresponding angle
*/
void SetSevoAnglePin9(int angle) {
  int value = map(angle, 0, 180, 34, 150);
  OCR1A = value;
  bitSet(TIMSK1, OCIE1A);
}

void SetSevoAnglePin10(int angle) {
  int value = map(angle, 0, 180, 34, 150);
  OCR1B = value;
  bitSet(TIMSK1, OCIE1B);
}

void SetSevoAnglePin11(int angle) {
  char value = map(angle, 0, 180, 34, 150);
  OCR2A = value;
  bitSet(TIMSK2, OCIE2A);
}
void SetSevoAnglePin3(int angle) {
  char value = map(angle, 0, 180, 34, 150);
  OCR2B = value;
  bitSet(TIMSK2, OCIE2B);
}
/*
   1 period of timer with pulse generation, 4 perdiods without => pulse every 20480us
*/
ISR(TIMER1_COMPA_vect) {
  static char i = 0;
  if (i == 0) {
    bitSet(TCCR1A, COM1A1);
  }
  else
  {
    bitClear(TCCR1A, COM1A1);
  }
  i++;
  if (i > 4)
    i = 0;
}

ISR(TIMER1_COMPB_vect) {
  static char i = 0;
  if (i == 0) {
    bitSet(TCCR1A, COM1B1);
  }
  else
  {
    bitClear(TCCR1A, COM1B1);
  }
  i++;
  if (i > 4)
    i = 0;
}

ISR(TIMER2_COMPA_vect) {
  static char i = 0;
  if (i == 0) {
    bitSet(TCCR2A, COM2A1);
  }
  else
  {
    bitClear(TCCR2A, COM2A1);
  }
  i++;
  if (i > 4)
    i = 0;
}

ISR(TIMER2_COMPB_vect) {
  static char i = 0;
  if (i == 0) {
    bitSet(TCCR2A, COM2B1);
  }
  else
  {
    bitClear(TCCR2A, COM2B1);
  }
  i++;
  if (i > 5)
    i = 0;
}


/* #########################################################
  setup and loop
  ######################################################### */

void setup(void)
{
  Serial.begin(57600);
  Serial.setTimeout(1);
  SetupServos();

  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  Serial.println("bno begin success");
  bno.setExtCrystalUse(true);
  Wire.setClock(400000);
}


void loop(void)
{
  static int value1, value2 = 0;
  int pos1;
  int pos2;
  int pos3;

  if (Serial.available() > 1) {

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    event.orientation.y = -event.orientation.y;
    if (event.orientation.z >= 0) {
      event.orientation.z = event.orientation.z - 180;
    }
    else if (event.orientation.z < 0) {
      event.orientation.z = 180 + event.orientation.z;
    }

    /* Display the floating point data */
    Serial.print(event.orientation.z, 4);
    Serial.print(",");
    Serial.print(event.orientation.y, 4);
    Serial.println(",");

    // read motors command
    pos1 = Serial.parseInt();
    pos2 = Serial.parseInt();
    pos3 = Serial.parseInt();
    SetSevoAnglePin9(pos1);
    SetSevoAnglePin10(pos2);
    SetSevoAnglePin11(pos3);
  }
}
