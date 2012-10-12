//=========================================================
// Linearsystem zur Messung von Temperaturprofilen
// Entstanden im Rahmen einer Bachelorthesis an der
// Bergischen Universitaet Wuppertal
//
// this program is under public domain!
// https://github.com/nilsarne/Linearsystem
//
// Nils-Arne Pohlandt - 934053
// Wuppertal, 23. September 2012 
//=========================================================

//additional libraries
#include <AFMotor.h>
#include <i2cmaster.h>
#include <EEPROM.h>

//================= Variablen =================
#define MOTORMODE DOUBLE
#define CONFIG_VERSION "ls5"
#define CONFIG_START 32

//================= settings =================
//settings code via http://arduino.cc/playground/Code/EEPROMLoadAndSaveSettings
struct StoreStruct {
  long position_current;
  long position_left;
  long position_right;
  int measuresteps;
  int measure_count;
  float efaktor;

  char version_of_program[4];
} 
settings = {
  0,
  0,
  45000,
  20,
  1,
  1,
  CONFIG_VERSION
};

//buttonpins
int button_left = 2;
int button_right = 3;
boolean button_left_pressed = false;
boolean button_right_pressed = false;
long position_mm = settings.position_current*0.015;
//counters
int i,k;
double j;
//variables for reading data
int choose;
char data[8];
unsigned long Zeit;
//i2c variables
int dev = 0x5A<<1;
int data_low = 0;
int data_high = 0;
int pec = 0;
double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614) 
double tempdata = 0x0000;
int frac;

//================= init motor =================
AF_Stepper motor(200, 2);

//================= Subprogramme =================
/* LoadAndSaveSettings
 * Thijs Elenbaas
 */
//EEPROM lesen
void loadConfig() {
  if (
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

//-------------------------------------------------------------
//EEPROM schreiben
void saveConfig() {
  for (unsigned int t=0; t<sizeof(settings); t++)
  { // writes to EEPROM
    EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
    // and verifies the data
    if (EEPROM.read(CONFIG_START + t) != *((char*)&settings + t))
    {
    }
  }
}
//-------------------------------------------------------------
//Emissionswert senden von Andreas Schultes
//CRC-Checksum berechnen
byte calcCRC8(byte command,byte address,byte lsb,byte msb)
{
  byte t[4];
  t[0]=command;
  t[1]=address;
  t[2]=lsb;
  t[3]=msb;
  unsigned int remainderPolynomial=0;
  int i,j;
  for(i=0;i<4;i++)
  {
    remainderPolynomial=remainderPolynomial^((unsigned int)t[i]*0x0100) ;
    for(j=0;j<8;j++)
    {
      for(int y=0;y<16;y++)
        if(remainderPolynomial&0x8000)
          remainderPolynomial=(remainderPolynomial*0x02)^0x0700;
        else
          remainderPolynomial=(remainderPolynomial*0x02);
    }
  }
  for(int y=0;y<16;y++)
    return remainderPolynomial>>8;
}

//-------------------------------------------------------------
void setMLXemission(int e)
//Emissionwert an mlx90614 uebermitteln
//e = 65535*settings.efaktor
{
  int dlsb,dmsb,pec;
  pec=calcCRC8(dev+I2C_WRITE,0x24,0x00,0x00);
  i2c_init();
  i2c_start_wait(dev+I2C_WRITE); 
  i2c_write(0x24);
  i2c_write(0x00);
  i2c_write(0x00);
  i2c_write(pec);
  i2c_stop();
  dlsb=e&0x00FF;
  dmsb=(e>>8);
  pec=calcCRC8(dev+I2C_WRITE,0x24,dlsb,dmsb);
  delay(1000);
  i2c_init();
  i2c_start_wait(dev+I2C_WRITE); 
  i2c_write(0x24);
  i2c_write(dlsb);
  i2c_write(dmsb);
  i2c_write(pec);
  i2c_stop();
  delay(1000);
}

//-------------------------------------------------------------
void drive(double x, char dir){
  //Motor sicher bewegen + Positionsmanagement
  //void drive(Schritte , Richtung)
  for(j=1;j<=x;j++){
    if (check_switches()==true)
      break;
    else if(dir == 'l'){
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
  }
  delay(10);
}

//-------------------------------------------------------------
void gotoposition(double x){
  //fixe Position zwischen den Schaltern anfahren
  //Position 0 = linker schalter
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

//-------------------------------------------------------------
boolean check_switches(){
  //Abfrage ob einer der Schalter gedrueckt wurde
  if (digitalRead(button_left) == HIGH){
    button_left_pressed = true;
    return true;
  }
  else if (digitalRead(button_right) == HIGH){
    button_right_pressed = true;
    return true;
  }
  else {
    button_left_pressed = false;
    button_right_pressed = false;
    return false;
  }
}

//-------------------------------------------------------------
void check_limit(){
  //manuelles Fahren nach links und rechts
  //wenn Schalter gefunden wurde stoppen und Position merken
  Serial.println("checking limits");
  do {
    motor.step(1, BACKWARD, MOTORMODE );
    settings.position_current--;
  } 
  while (digitalRead(button_left) == LOW);
  settings.position_current = 0;
  motor.step(200, FORWARD, MOTORMODE );
  settings.position_current+=200;  
  do {
    motor.step(1, FORWARD, MOTORMODE );
    settings.position_current++;
  }
  while (digitalRead(button_right) == LOW);
  settings.position_right = settings.position_current;
  motor.step(200, BACKWARD, MOTORMODE );
  settings.position_current-=200;
  Serial.print("right limit: ");  
  Serial.print(settings.position_right); 
  Serial.println(" steps");
}

//-------------------------------------------------------------
float read_object_temp(){
  //Auslesen MLX90614 Objekttemperatur
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck(); //Read 1 byte and then send ack
  data_high = i2c_readAck(); //Read 1 byte and then send ack
  pec = i2c_readNak();
  i2c_stop();
  tempdata = (double)(((data_high & 0x007F) << 8) + data_low);
  tempdata = (tempdata * tempFactor)-0.01;  //tempdata to kelvin conversation
  float celcius = tempdata - 273.15;  // kelvin to celsius
  return celcius;
}

//-------------------------------------------------------------
float read_ambient_temp(){
  //Auslesen MLX90614 Sensortemperatur
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x06);
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck();
  data_high = i2c_readAck();
  pec = i2c_readNak();
  i2c_stop();
  tempdata = (double)(((data_high & 0x007F) << 8) + data_low);
  tempdata = (tempdata * tempFactor)-0.01;
  float celcius = tempdata - 273.15;
  return celcius;
}

//-------------------------------------------------------------
void measure(){
  //Abfahren der Strecke zwischen den Schaltern
  //Messen der Temperaturen in definierten Schritten
  double measure_step = (settings.position_right-2000)/settings.measuresteps;
  //xx Messungen auf der Strecke 
  gotoposition(settings.position_right-1000);
  //1,5cm rechts vom linken schlater - nullpunkt fuer messung
  for(i=1;i<=settings.measuresteps;i++){
    drive(measure_step,'l');
    Serial.print("Messung: "); 
    Serial.print(i);
    Serial.print("; Position [mm]: "); 
    Serial.print(position_mm); 
    Serial.print("; Objekttemperatur: ");  
    Serial.print(read_object_temp()/settings.efaktor);
    Serial.print("; Sensortemperatur: ");  
    Serial.println(read_ambient_temp());
  }
}

//-------------------------------------------------------------
void just_measure(){
  //Temperaturen messen ohne den Schlitten zu bewegen
  i=1;
  Serial.println("Press '0' for stopping measure.");
  do{
    Serial.print("Messung: "); 
    Serial.print(i);
    Serial.print("; Objekttemperatur: ");  
    Serial.print(read_object_temp()/settings.efaktor);
    Serial.print("; Sensortemperatur: ");  
    Serial.println(read_ambient_temp());
    i++;
    delay(1000);
  } 
  while (Serial.read()!='0');
}

//-------------------------------------------------------------
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
  Serial.print("Currently used storage version: ");
  Serial.println(CONFIG_VERSION);
  Serial.println("");
  Serial.println("");
  Serial.println("");
}

//-------------------------------------------------------------
void set_settings(){
  //Einstellen der Parameter
  Serial.println("===== Settings =====");
  Serial.print("1: Set number of measuresteps - ");
  Serial.println(settings.measuresteps);
  //Anzahl der Messungen
  Serial.print("2: number of repeating measurements - ");
  Serial.println(settings.measure_count);
  //Anzahl der Wiederholungen an Messungen
  Serial.print("3: emissionsfaktor - ");
  Serial.println(settings.efaktor);
  //Emissionsfaktor
  Serial.println(" ==================================");
  Serial.println("");

  do {
    if (Serial.available() > 0) {
      choose = int(Serial.read())-48;
    }
    if (choose == 1 || choose == 2 || choose == 3)
      break;
  } 
  while (choose != 1 || choose != 2 || choose != 3);
  switch (choose) {

  case 1:
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
    data[i] = 0;
    // Abschliessende Null fuer gueltigen String
    i=0;
    settings.measuresteps=atof(data);
    Serial.println("");  
    break;

  case 2:
    Serial.println("repeating measurements: ");
    Serial.println(settings.measure_count);
    Serial.println("Please enter new value:");
    do {
      if (Serial.available()) {
        data[i] = Serial.read();
        Serial.print(data[i]);
        i++;
      }    
      if(i<1)Zeit = millis();
    } 
    while (i<7&&(millis()-Zeit) < 3000);
    data[i] = 0;
    i=0;
    settings.measure_count=atof(data);
    Serial.println("");
    break;

  case 3:
    Serial.println("emissionsfaktor ");
    Serial.println(settings.efaktor);
    Serial.println("Please enter new value: (0.01 - 1.00)");
    do {
      if (Serial.available()) {
        //      Serial.flush();
        data[i] = Serial.read();
        Serial.print(data[i]);
        i++;
      }    
      if(i<1)Zeit = millis();
    } 
    while (i<7&&(millis()-Zeit) < 5000);
    data[i] = 0;
    i=0;
    settings.efaktor=atof(data);
    setMLXemission(int(65535*settings.efaktor));
    Serial.println("");
    break;  
  }
}

//================= SETUP =================
void setup(){
  Serial.begin(9600); 
  while (!Serial); 
  Serial.println("Starting setup");
  i2c_init(); //Initialise i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
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
  about();
}

//================= LOOP =================
void loop(){
  Serial.println("");
  Serial.println(" ==================================");
  Serial.println("");
  Serial.println("Please choose working mode:");
  Serial.println("");  
  Serial.println("1: Calibrating sledge");
  Serial.println("2: Measure temperature across the whole lenght");
  Serial.println("3: Go to position");
  Serial.println("4: Measure temperature without moving");
  Serial.println("5: Settings");
  Serial.println("6: About");
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
    for (k=1;k<=settings.measure_count;k++){
      Serial.print("Messung:");
      Serial.println(k);
      measure();
    }
    break;
  case 3:
    Serial.println("please enter position. each digit seperat ");
    do { 
      if (Serial.available()) {
        data[i] = Serial.read();
        Serial.print(data[i]);
        i++;
      }    
      if(i<1)Zeit = millis();
    } 
    while (i<7&&(millis()-Zeit) < 3000);
    data[i] = 0;
    i=0;
    Serial.println("");  
    gotoposition(atof(data));
    break;
  case 4:
    just_measure();
    break;
  case 5:
    set_settings();
    break;
  case 6:
    about();
    break;  
  default:
    break;
  }
  choose = 0;
  //resetting chooser -> no looping
  saveConfig();
  motor.release();
}

