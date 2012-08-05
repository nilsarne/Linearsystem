#include <AFMotor.h>
#include <i2cmaster.h>

//================= Variablen =================

#define MOTORMODE DOUBLE

int button_left = 2;
int button_right = 3;
long position_current = 0; //position in steps
long position_left = 0;
long position_right = 0;
int i;
int choose;

int dev = 0x5A<<1;
int data_low = 0;
int data_high = 0;
int pec = 0;
double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614) 
//lsb == least significant bit
double tempData = 0x0000; // zero out the data
int frac; // data past the decimal point

boolean button_left_pressed = false;
boolean button_right_pressed = false;



//init motor
AF_Stepper motor(200, 2);

//================= Subprogramme =================
void drive(int x, char dir){
  int j;
  for(j=1;j<=x;j++){
    if(dir == 'l'){
      motor.step(1, BACKWARD, MOTORMODE );
      position_current--;
    }  
    else if (dir == 'r'){
      motor.step(1, FORWARD, MOTORMODE );
      position_current++;
    }
    else {
      Serial.println("EROOR - Wrong Direction");
      break; 
    }
    if (check_switches()==true)
      break;
  }
}

//-----------------------------
void gotoposition(double x){
  if (x > position_right)
    Serial.println("too much movement to the right");

  else if((position_current - x)<0){
    Serial.print((position_current - x)*-1);
    Serial.println(" steps moving to the right");
    drive((position_current - x)*-1,'r');
  }
  else if((position_current - x)>=0){
    Serial.print(position_current - x);
    Serial.println(" stepsmoving to the left");
    drive((position_current - x),'l');
  }
}

//-----------------------------
boolean check_switches(){
  if (digitalRead(button_left) == HIGH){
    button_left_pressed = true;
    //Serial.println("button left pressed");
    return true;
  }
  else if (digitalRead(button_right) == HIGH){
    button_right_pressed = true;
    //Serial.println("button right pressed");
    return true;
  }
  else {
    button_left_pressed = false;
    button_right_pressed = false;
    return false;
  }
}

//-----------------------------
void check_limit(){
  Serial.println("checking limits");

  do {
    drive(1,'l');
  } 
  while (digitalRead(button_left) == LOW);
  position_current = 0;

  do {
    drive(1,'r');
  }
  while (digitalRead(button_right) == LOW);
  position_right = position_current;

  Serial.print("right limit: ");  
  Serial.print(position_right); 
  Serial.println(" steps");
}

//-----------------------------
float read_temp(){
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);

  // read
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck(); //Read 1 byte and then send ack
  data_high = i2c_readAck(); //Read 1 byte and then send ack
  pec = i2c_readNak();
  i2c_stop();

  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;  //tempData to kelvin conversation

  float celcius = tempData - 273.15;  // kelvin to celsius
  //  Serial.print("Celcius: ");  Serial.println(celcius);
  return celcius;
}

//-----------------------------
void measure(){
  double measure_step = (position_right-2000)/20; //20 Messungen auf der Strecke 

  gotoposition(position_right-1000); //1,5cm rechts vom linken schlater - nullpunkt für messung
  for(i=1;i<=20;i++){
    drive(measure_step,'l');
    Serial.print("Messung: "); 
    Serial.print(i);
    Serial.print("; Temperatur: ");  
    Serial.println(read_temp());
  }

}

//-----------------------------
void about(){

  Serial.println("Welcome to the automatic IR temperatur measuring");
  delay(100);
  Serial.println("by Nils-Arne Pohlanndt / 934053");
  delay(100);
  Serial.println("");
  delay(100);
  Serial.println("This measurement device was developed for my bachelor thesis.");
  delay(100);
  Serial.println("Please make sure that the sledge can move freely.");
  Serial.println("");
  Serial.println("");
  Serial.println("");
}

//================= SETUP =================
void setup(){
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Starting setup");

  //init ir thermometer
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups

  pinMode(button_left, INPUT);
  pinMode(button_right, INPUT);
  motor.setSpeed(200);
  motor.release();
  Serial.println("setup completed");
  delay(100);
  Serial.println("");
  about;
}

//================= LOOP =================
void loop(){
  Serial.println("Please choose working mode:");
  Serial.println("");
  Serial.println("1: Calibrating sledge");
  Serial.println("2: Measure temperature across the whole lenght");
  Serial.println("3: Go to position");
  Serial.println("4: ");
  Serial.println("5: About");


  do {
    if (Serial.available() > 0) {
      choose = int(Serial.read())-48;
    }
    if (choose == 1 || choose == 2 || choose == 3 || choose == 4 || choose == 5 )
      break;

  } 
  while (choose != 1 || choose != 2 || choose != 3 || choose != 4 || choose != 5 );


  switch (choose) {
  case 1:
    check_limit();
    break;
  case 2:
    Serial.println("choose: 2");
    break;
  case 3:
    Serial.println("choose: 3");
    break;
  case 4:
    Serial.println("choose: 4");
    break;
  case 5:
    about();
    break;
  default:
    break;
  }
  choose = 0;  //resetting chooser -> no looping
  /*
  check_limit();
   delay(500);
   measure();
   gotoposition(position_right/2);
   delay(5000);
   */
}













