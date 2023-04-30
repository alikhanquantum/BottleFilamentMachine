//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <thermistor.h>           //thermistor library to enable temperature reading

LiquidCrystal_I2C lcd(0x27,16,2);
thermistor therm1(A0,0);          //Connect thermistor on A0


int PWM_pin = 5;                  //Pin for PWM signal that controls MOSFET for extruder
int speed_pot = A1;               //Potenitometer pin for control menu
int but1 = 7;                     //Button pin for control menu
int EN = 2;                       //Enable pin for stepper motor driver
int STEP = 3;                     //STEP pin for stepper motor driver
int DIR = 4;                      //DIR pin for stepper motor driver
int LED = 13;                     //LED pin for onboard LED

//Variables
float set_temperature = 200;            //Default temperature setpoint. Leave it 0 to control it with rotary encoder
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0;
int max_PWM = 255;

//Stepper Variables
int max_speed = 1000;
int main_speed = 0;
bool but1_state = true;
bool activate_stepper = false;
int rotating_speed = 0;

// Define a stepper motor with pins that it will use
AccelStepper stepper1(1, STEP, DIR);

//PID constants
//////////////////////////////////////////////////////////
int kp = 90;   int ki = 30;   int kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;



void setup() {  
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  stepper1.setMaxSpeed(max_speed);  
  pinMode(but1, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  pinMode(PWM_pin,OUTPUT);
  TCCR0B = TCCR0B & B11111000 | B00000010;    // D6 adn D6 PWM frequency of 7812.50 Hz
  Time = millis();

  TCCR1A = 0;             //Reset entire TCCR1A register
  TCCR1B = 0;             //Reset entire TCCR1B register
  TCCR1A |= B00000010;
  TCNT1 = 0;
  
  lcd.init();
  lcd.backlight();
}

void loop() {
  if(!digitalRead(but1) && but1_state){
    but1_state = false;
    activate_stepper = !activate_stepper;
    delay(10);
  }
  else if(digitalRead(but1) && !but1_state){
    but1_state = true;
  }
  
  if(activate_stepper){
    digitalWrite(LED, HIGH);
    digitalWrite(EN, LOW);
    rotating_speed = map(analogRead(speed_pot),0,1024,main_speed,max_speed);
    stepper1.setSpeed(rotating_speed);
    //stepper1.runSpeed();
  }
  else
  {
    digitalWrite(EN, HIGH);
    digitalWrite(LED, LOW);
    stepper1.setSpeed(0);
    stepper1.runSpeed();
  }
  
  // Temperature control through PID error calculation
  temperature_read = therm1.analog2temp();

  PID_error = set_temperature - temperature_read + 6;
  PID_p = 0.01*kp * PID_error;
  PID_i = 0.01*PID_i + (ki * PID_error);
  

  timePrev = Time;
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000;
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  PID_value = PID_p + PID_i + PID_d;


  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > max_PWM){
    PID_value = max_PWM;
  }

  analogWrite(PWM_pin,PID_value);
  previous_error = PID_error;
  
  delay(250);
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("T: ");  
  lcd.print(temperature_read,1);
  lcd.print(" S: ");  
  lcd.print(rotating_speed);
  
  lcd.setCursor(0,1);
  lcd.print("PID: ");
  lcd.print(PID_value);
 
}


ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;
  stepper1.runSpeed();
}
