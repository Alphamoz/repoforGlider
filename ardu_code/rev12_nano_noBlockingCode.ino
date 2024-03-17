/* Serial_Motor_Driver_V5_2019
   by Ardian Budi K A
   for GaneshBlue Glider ITB
   uC : Arduino Nano (old bootloader)
   last modified 5 June 2023
  **Added manual parsing data from TS and sending every 1 second BE and MM data.

  Format:
  $#TS bld_val blst_val char_teta rd_val char_pos_e char_pos_n char_strobe (old)
  $#TS bld_val blst_val char_prop char_rudder char_bow char_strobe (new)
  Sample
  $#TS 12.5 13 13 13 13 13 99

  Tested feature:
  BE Pump
  BE Encoder Pin
  BE Valve in
  BE Valve out
  Strobo

  To Do Test:
  BE Encoder Func
  Parsing Data Input
  Ballast
  Bow Depan
  Bow Belakang
  Rudder

  Propeller

  This code is in the private domain.
*/
#include <EEPROM.h>
#include <AccelStepper.h>
//#include <TimerOne.h>
// ======================= pins for the Actuator ==================================
//----------------------------Pin Buoyancy Engine----------------------------------
const int pin_BE_Pump = 10;
const int pin_BE_Valvein = 8;
const int pin_BE_Valveout = 4;
const int pin_BE_Enc = 2;
const int pin_BE_rpm = 3;
//-------------------------------Pin Ballast---------------------------------------
const int pin_Dir = 12;
const int pin_Step = 13;
//-------------------------------Lainnya-------------------------------------------
const int pin_Bow_Dpn = 5;      //---Bow Thruster Depan
const int pin_Bow_Blk = 11;      //---Bow Thruster Depan
const int pin_Prop = 6;         //---Propeller
const int pin_Rudder = 9;       //---Rudder
const int pin_Strobo = 7;       //---Strobo Led Lamp


// ==================================== All Variabel ===============================
//--------------------------------------VAR MOTOR DC--------------------------------
//------------------------------------(Buoyancy Engine)-----------------------------
unsigned long last_timetick;        //---> Timer Debounce Value, only in ISR
unsigned long timetick;             //---> Timer Debounce Value, only in ISR
unsigned long timetick1 = millis();
unsigned long interval = 500;       //--> interval waktu 0.5 second
unsigned long t2;                   //---> not used
long enc_count;   //---> Encoder Value, volatile-> in ISR & Main
bool dir_BE_ref = 0;                //---> BE Flow direction, 0 = Out , 1 = In
bool pos_BE_run = 0;                //---> BE Flow direction, 0 = Off , 1 = On
unsigned long pos_BE_ref;           //---> BE Counter reference, Max = 50000
unsigned long pos_BE_now;                    //---> BE Counter currently, Max = 50000
long target;
//---------------------------------------VAR STEPPER--------------------------------
//----------------------------------------(Ballast)---------------------------------
//==================================STEPPER USING TIMEONE===========================
AccelStepper myStepper(AccelStepper::DRIVER, pin_Step, pin_Dir); //pin 13 = step, pin 7 = direction
int MMPos, per_cur;
unsigned long pos_step, cur_pos;

int BE_pos;
float vol_max_BE = 600;
//----------------------------------------VAR SERVO --------------------------------
//-------------------------(Bow_Depan,Bow_belakang,Propeler,Rudder)-----------------
#include <Servo.h>
Servo srv_BE_pum, srv_bowdpn, srv_bowblk, srv_prop, srv_rud;
int val_BE_pum, val_bowdpn, val_bowblk, val_prop, val_rud;

//----------------------------------------VAR FLOAT --------------------------------
float bld_val, char_teta, rd_val, char_pos_e, char_pos_n, volume_BE, volume_BE_last, volume_BE_max, volume_BE_min;
int char_strobe, rpm_BE_enc, rpm_BE, Status_Cal;

int state = 1;
unsigned long CL_cont = 0;
unsigned long CL_time = 0;
unsigned long CL_time_last = 0;
int cal_BE = 0;
float cal_vol_BE = 0;
Status_Cal = 0;

//----------------------------Setup Program HERE-------------------------
void setup() {
  // initialize serial:
  Serial.begin(9600);
  // make the pins outputs, set digital based control pinout:
  pinMode(pin_BE_Pump, OUTPUT);
  pinMode(pin_BE_Valvein, OUTPUT);
  pinMode(pin_BE_Valveout, OUTPUT);

  pinMode(pin_BE_Enc, INPUT_PULLUP);
  pinMode(pin_BE_rpm, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_BE_Enc), volume, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_BE_rpm), enc_RPM, FALLING);

  pinMode(pin_Rudder, OUTPUT);
  pinMode(pin_Strobo, OUTPUT);
  // attach servo based control pinout:
  srv_BE_pum.attach(pin_BE_Pump);
  srv_bowdpn.attach(pin_Bow_Dpn);
  srv_bowblk.attach(pin_Bow_Blk);
  srv_prop.attach(pin_Prop);
  srv_rud.attach(pin_Rudder);
  // Setup Stepper based control pinout
  myStepper.setMaxSpeed(20000);
  //myStepper.setMaxSpeed(50000);
  myStepper.setSpeed(20000);
  //stepper.setCurrentPosition(500000);
  //EEPROM.update(0, 0);
  BE_pos = EEPROM.read(0);
  enc_count = BE_pos;
  Serial.print(BE_pos);
  Serial.print(" Posisi BE ");
  float q = enc_count * 2.5;
  Serial.print(q);
  Serial.print(" ");

  MMPos = EEPROM.read(1);

  Serial.println(MMPos);


  cur_pos = (MMPos * 2040000) / 100; //untuk 500ml
  Serial.println(cur_pos);
  myStepper.setCurrentPosition(cur_pos);//penempatan posisi mula ballas
  char_teta = MMPos;
  pos_step = (char_teta / 100) * 2040000;
  Serial.println(pos_step);
  //max step 500000
  bld_val = 65;
  volume_BE_last = BE_pos * 2.5;
  volume_BE_max = 0;
  volume_BE_min = 0;
  //  volume_BE = volume_BE_last;
  volume_BE = volume_BE_last - 600;
  CL_cont = 0;
  srv_BE_pum.write(50);
  srv_bowdpn.writeMicroseconds(1500);
  srv_bowblk.writeMicroseconds(1500);
  srv_prop.writeMicroseconds(1200);
  //pos_step = 250000; //penempatan posisi tengah
  //  Timer1.initialize(1000);
  //  Timer1.attachInterrupt(ballas);
  BEmati();
}

//----------------------------MAIN PROGRAM HERE-------------------------
void loop() {


  //-----------------------------Serial Parsing---------------------------
  while (Serial.available() > 0) {
    state = 0;
    String data = Serial.readStringUntil('\n');
    parseSerialData(data);
    if (Status_Cal == 1) {
      state = 1;
      volume_BE = volume_BE_last - 600;
      CL_cont = 0;
      //Serial.println("------Run Calibration-------");
    } else {
      state  = 0;
    }
    //int x = char_teta;

    //Serial.print("update ballas : ");
    //Serial.println(x);

    srv_bowdpn.writeMicroseconds(char_pos_e);
    srv_bowblk.writeMicroseconds(char_pos_e);
    srv_prop.writeMicroseconds(char_pos_n);
    srv_rud.write(val_rud); //max 169 min 15
//    SerPrint();/
  }

  if (state == 0) {
    volume_BE_max = volume_BE + 5;
    volume_BE_min = volume_BE - 5;
    if (volume_BE_last > volume_BE_min && volume_BE_last < volume_BE_max ) {
      //      ballas ();
      BEmati();
      //state=2;
      //Serial.println("State mati");
    } else if (volume_BE_last > volume_BE_max) {
      //      ballas ();
      //BEnaik();
      BEturun();
      
      //Serial.println("State naik");
      //Serial.println(volume_BE_last);
    } else {
      //      ballas ();
      //BEturun();
      BEnaik();
    
      
      //Serial.println("State turun");
      //  //Serial.println(volume_BE_last);
    }
    //    Serial.println(volume_BE/_last);
    volume();
    ballas ();


  } else if (state == 1) {

    BEturun();
    Serial.println(cal_vol_BE);
    if (volume_BE_last == cal_vol_BE) {
      CL_cont = CL_cont + 1;

      if (CL_cont == 1) {
        CL_time = millis();
      }

      CL_time_last = millis();
      if (CL_time_last - CL_time >= 1000 ) {
        enc_count = 0;
        volume_BE_last = 0;
        volume_BE = 0;
        EEPROM.update(0, enc_count);
        state = 0;
        BEmati();
        delay(5000);
        Serial.println("ready");
      }

    } else {
      CL_cont = 0;
      timetick = 0;
    }
    cal_vol_BE = volume_BE_last;

  } else {
    BEmati();
  }
  //Serial.println(volume_BE_last);
  //Serial.println(state);
  //================= Servo based  Actuator Routine ==============================

  if (millis() - timetick1 >= 1000) {
    timetick1 = millis();
    Serial.print("Status ");
    Serial.print(volume_BE_last);
    Serial.print(" ");
    Serial.println(MMPos);
//    Serial.println(volume_BE_last);
  }

  if (myStepper.currentPosition() % 20400 == 0) {
    MMPos = myStepper.currentPosition() / 20400;
    EEPROM.update(1, MMPos);
  }

}
//----------------------------END MAIN PROGRAM------------------------------

void parseSerialData(String data) {
  if (data.startsWith("#TS")) {
    String values = data.substring(4);  // Skip the "#TS " identifier

    int spacePos;
    for (int i = 0; i < 7; i++) {
      spacePos = values.indexOf(' ');
      if (spacePos >= 0) {
        String value = values.substring(0, spacePos);
        values = values.substring(spacePos + 1);
        switch (i) {
          case 0:
            volume_BE = value.toFloat();
            break;
          case 1:
            Status_Cal = value.toInt();
            break;
          case 2:
            char_teta = value.toInt();
            pos_step = (char_teta / 100) * 2040000;
            break;
          case 3:
            rd_val = value.toInt();
            val_rud = map(rd_val, 0, 100, 15, 169);
            break;
          case 4:
            char_pos_e = value.toInt();
            break;
          case 5:
            char_pos_n = value.toInt();
            break;
          case 6:
            char_strobe = value.toInt();
            break;
        }
      }
    }
  }
}

void ballas () {
  //================= Ballast Actuator Routine ==============================
  myStepper.moveTo(pos_step);
  myStepper.setSpeed(5000);
  myStepper.runSpeedToPosition();
  //  Serial.print("funcion ballas jalan bos ");

  //---------------------------------------------------------------------------
}

//------------------------Serial Print----------------------------
void SerPrint() {
  Serial.print(volume_BE);
  Serial.print(' ');
  Serial.print(Status_Cal);
  Serial.print(' ');
  Serial.print(pos_step);
  Serial.print(' ');
  Serial.print(rd_val);
  Serial.print(' ');
  Serial.print(char_pos_e);
  Serial.print(' ');
  Serial.print(char_pos_n);
  Serial.print(' ');
  Serial.print(char_strobe);
  Serial.print(" Encoder : ");
  Serial.println(enc_count);
  EEPROM.update(0, per_cur);
}


//------------------------Naik Routine----------------------------
void BEnaik() {
  digitalWrite(pin_BE_Valvein, LOW);
  digitalWrite(pin_BE_Valveout, HIGH);
  srv_BE_pum.write(bld_val);
  //Serial.println("naik");
}

//------------------------Turun Routine----------------------------
void BEturun() {
  digitalWrite(pin_BE_Valvein, HIGH);
  digitalWrite(pin_BE_Valveout, LOW);
  srv_BE_pum.write(bld_val);
  //Serial.println("turun");
}

void BEmati() {
  digitalWrite(pin_BE_Valvein, LOW);
  digitalWrite(pin_BE_Valveout, LOW);
  srv_BE_pum.write(40);
  //Serial.println("mati");
}

//---------------------ISR (Interrupt Service Routine)----------------------------
void volume() {

  if (volume_BE > volume_BE_last) {
    enc_count++;
  } else if (volume_BE < volume_BE_last) {
    enc_count--;
  }
  hit_vol();

}

void hit_vol() {
  volume_BE_last = enc_count * 2.5; //1 impuls =2.5ml
  BE_pos = enc_count;
  EEPROM.update(0, BE_pos);
}
//------------------------------------------------------------------------------


void enc_RPM() {
  rpm_BE_enc++;
  cont_RPM();
}


void cont_RPM() {

  timetick = millis();
  if (timetick - last_timetick >= interval) {
    rpm_BE = rpm_BE_enc * 120;
    last_timetick = timetick;
    rpm_BE_enc = 0;
    //Serial.println(rpm_BE);
    //    Serial.print(volume_BE_last);
    //    Serial.print("  ");
    //    Serial.print(volume_BE_max);
    //    Serial.print("  ");
    //    Serial.println(volume_BE_min);
  }

}
