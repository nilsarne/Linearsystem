#include <AFMotor.h>
#include <i2cmaster.h>
#include <EEPROM.h>

//================= Variablen =================
#define MOTORMODE DOUBLE

// ID of the settings block
#define CONFIG_VERSION "ls3"
// Tell it where to store your config data in EEPROM
#define CONFIG_START 32

//================= settings =================
//settings code via http://arduino.cc/playground/Code/EEPROMLoadAndSaveSettings
struct StoreStruct {
  // The variables of your settings
  long position_current;
  long position_left;
  long position_right;
  int measuresteps;
  // This is for mere detection if they are your settings
  char version_of_program[4]; // it is the last variable of the struct
  // so when settings are saved, they will only be validated if
  // they are stored completely.
} 
settings = {
  // The default values
  0,
  0,
  45000,
  20,
  CONFIG_VERSION
};

int button_left = 2;
int button_right = 3;

long position_mm = settings.position_current/200*3;

int i;
int choose;

char data[8];
unsigned long Zeit;

int dev = 0x5A<<1;
int data_low = 0;
int data_high = 0;
int pec = 0;
double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614) 
//lsb == least significant bit
double tempdata = 0x0000; // zero out the data
int frac; // data past the decimal point

boolean button_left_pressed = false;
boolean button_right_pressed = false;

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (//EEPROM.read(CONFIG_START + sizeof(settings) - 1) == settings.version_of_program[3] // this is '\0'
  EEPROM.read(CONFIG_START + sizeof(settings) - 2) == settings.version_of_program[2] &&
    EEPROM.read(CONFIG_START + sizeof(settings) - 3) == settings.version_of_program[1] &&
    EEPROM.read(CONFIG_START + sizeof(settings) - 4) == settings.version_of_program[0])
  { // reads settings from EEPROM
    for (unsigned int t=0; t<sizeof(settings); t++)
      *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
  } 
  else {
    // settings aren't valid! will overwrite with default settings
    saveConfig();
  }
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(settings); t++)
  { // writes to EEPROM
    EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
    // and verifies the data
    if (EEPROM.read(CONFIG_START + t) != *((char*)&settings + t))
    {
      // error writing to EEPROM
    }
  }
}


//================= init motor =================
AF_Stepper motor(200, 2);

//================= Subprogramme =================
void drive(double x, char dir){
  double j;
  for(j=1;j<=x;j++){
    if(dir == 'l'){
      motor.step(1, BACKWARD, MOTORMODE );
      settings.position_current--;
      position_mm = settings.position_current*0.015;
    }  
    else if (dir == 'r'){
      motor.step(1, FORWARD, MOTORMODE );
      settings.position_current++;
      position_mm = settings.position_current*0.015;
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
  if (x > settings.position_right)
    Serial.println("too much movement to the right");
  else if (x < settings.position_left)
    Serial.println("too much movement to the left");

  else if((settings.position_current - x)<0){
    Serial.print((settings.position_current - x)*-1);
    Serial.println(" steps moving to the right");
    drive((settings.position_current - x)*-1,'r');
  }
  else if((settings.position_current - x)>=0){
    Serial.print(settings.position_current - x);
    Serial.println(" steps moving to the left");
    drive((settings.position_current - x),'l');
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
  settings.position_current = 0;
  motor.step(200, FORWARD, MOTORMODE );
  settings.position_current+=200;  
  do {
    drive(1,'r');
  }
  while (digitalRead(button_right) == LOW);
  settings.position_right = settings.position_current;
  motor.step(200, BACKWARD, MOTORMODE );
  settings.position_current-=200;
  Serial.print("right limit: ");  
  Serial.print(settings.position_right); 
  Serial.println(" steps");
}

//-----------------------------
float read_object_temp(){
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);
  // read
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck(); //Read 1 byte and then send ack
  data_high = i2c_readAck(); //Read 1 byte and then send ack
  pec = i2c_readNak();
  i2c_stop();

  tempdata = (double)(((data_high & 0x007F) << 8) + data_low);
  tempdata = (tempdata * tempFactor)-0.01;  //tempdata to kelvin conversation
  float celcius = tempdata - 273.15;  // kelvin to celsius
  //  Serial.print("Celcius: ");  Serial.println(celcius);
  return celcius;
}

//-----------------------------
float read_ambient_temp(){
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x06);
  // read
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck(); //Read 1 byte and then send ack
  data_high = i2c_readAck(); //Read 1 byte and then send ack
  pec = i2c_readNak();
  i2c_stop();

  tempdata = (double)(((data_high & 0x007F) << 8) + data_low);
  tempdata = (tempdata * tempFactor)-0.01;  //tempdata to kelvin conversation
  float celcius = tempdata - 273.15;  // kelvin to celsius
  //  Serial.print("Celcius: ");  Serial.println(celcius);
  return celcius;
}
//-----------------------------
void measure(){
  double measure_step = (settings.position_right-2000)/settings.measuresteps; //xx Messungen auf der Strecke 

  gotoposition(settings.position_right-1000); //1,5cm rechts vom linken schlater - nullpunkt fuer messung
  for(i=1;i<=settings.measuresteps;i++){
    drive(measure_step,'l');
    Serial.print("Messung: "); 
    Serial.print(i);
    Serial.print("; Position: "); 
    Serial.print(position_mm); 
    Serial.print("mm; Objekttemperatur: ");  
    Serial.print(read_object_temp());
    Serial.print("; Sensortemperatur: ");  
    Serial.println(read_ambient_temp());
  }
}

//-----------------------------
void just_measure(){
  i=1;
  Serial.println("Press '0' for stopping measure.");
  do{
    Serial.print("Messung: "); 
    Serial.print(i);
    Serial.print("; Objekttemperatur: ");  
    Serial.print(read_object_temp());
    Serial.print("; Sensortemperatur: ");  
    Serial.println(read_ambient_temp());
    i++;
    delay(1000);
  } 
  while (Serial.read()!='0');
}

//-----------------------------
void about(){

  Serial.println("");
  Serial.println("");
  Serial.println("Welcome to the automatic IR temperatur measuring");
  Serial.println("by Nils-Arne Pohlanndt / 934053");
  Serial.println("Bergische Universitaet Wuppertal");
  Serial.println("");
  Serial.println("This measurement device was developed for my bachelor thesis.");
  Serial.println("Please make sure that the sledge can move freely.");
  Serial.println("");
  Serial.println("");
  Serial.println("");
}

//================= SETUP =================
void setup(){
  Serial.begin(9600); 
  while (!Serial);  // set up Serial library at 9600 bps
  Serial.println("Starting setup");

  //init ir thermometer
  i2c_init(); //Initialise the i2c bus
  //PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups

  pinMode(button_left, INPUT);
  pinMode(button_right, INPUT);
  loadConfig();
  settings.position_current = settings.position_current;
  Serial.print("Position loaded: ");
  Serial.println(settings.position_current);
  motor.setSpeed(195);
  motor.release();  
  Serial.println("setup completed");
  delay(100);
  Serial.println("");
  about;
}

//================= LOOP =================
void loop(){

  //store position
  saveConfig();
  /*    //Data verification
   loadConfig();
   if (position_current != settings.position_current){
   Serial.println("ERROR writing settings");
   } 
   else Serial.println("Writing settings --> OK");
   Serial.print("Position: ");
   
   */

  Serial.println("");
  Serial.println(" ==================================");
  Serial.println("");
  Serial.println("Please choose working mode:");
  Serial.println("");  
  Serial.println("1: Calibrating sledge");
  Serial.println("2: Measure temperature across the whole lenght");
  Serial.println("3: Go to position");
  Serial.println("4: Measure temperature without moving");
  Serial.println("5: About");
  Serial.println("6: Settings");
  Serial.println("");

  do {
    if (Serial.available() > 0) {
      choose = int(Serial.read())-48;
    }
    if (choose == 1 || choose == 2 || choose == 3 || choose == 4 || choose == 5 || choose == 6)
      break;
  } 
  while (choose != 1 || choose != 2 || choose != 3 || choose != 4 || choose != 5 || choose != 6 );

  switch (choose) {
  case 1:
    check_limit();
    break;
  case 2:
    measure();
    break;
  case 3:
    Serial.println("please enter position. each digit seperat ");
    do { // Wenn Daten verfuegbar Zeichen in data schreiben bis 7 Zeichen erreicht oder 10 Sekunden Warten nach dem ersten uebertragenen byte
      if (Serial.available()) {
        data[i] = Serial.read();
        Serial.print(data[i]);
        i++;
      }    
      if(i<1)Zeit = millis();
    } 
    while (i<7&&(millis()-Zeit) < 3000);
    data[i] = 0;  // Abschliessende Null fuer gueltigen String
    i=0;
    Serial.println("");  
    gotoposition(atof(data));
    break;
  case 4:
    just_measure();
    break;
  case 5:
    about();
    break;
  case 6:
    Serial.println("===== Settings =====");
    Serial.println("measuresteps across lenght: ");
    Serial.println(settings.measuresteps);
    Serial.println("Please enter new value:");
    do { // Wenn Daten verfuegbar Zeichen in data schreiben bis 7 Zeichen erreicht oder 10 Sekunden Warten nach dem ersten uebertragenen byte
      if (Serial.available()) {
        data[i] = Serial.read();
        Serial.print(data[i]);
        i++;
      }    
      if(i<1)Zeit = millis();
    } 
    while (i<7&&(millis()-Zeit) < 3000);
    data[i] = 0;  // Abschliessende Null fuer gueltigen String
    i=0;
    settings.measuresteps=atof(data);
    saveConfig();
    Serial.println("");  
  default:
    break;
  }
  choose = 0;  //resetting chooser -> no looping
}

