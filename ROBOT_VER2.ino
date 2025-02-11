//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| ^_^ |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//       Phat phu ho, qua môn và không bao gio BUG 
//  Buddha bless me to pass Robot In Practice and never BUG
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <WiFi.h>


void home_X();
void home_Y();
void home_Z();
void hut_tha_vat(int tt);
void set_home();
void IRAM_ATTR dem_xung();
void IRAM_ATTR PID();
long di_chuyen_vi_tri_ke_tiep(long xung_qk, long xung_ht);
long chuyen_goc_sang_xung(float goc);
void run_theta23(int t2,int t3);
void ve_box();
void vi_tri_ban_dau();
void run_theta23_first(int t2, int t3);
int sign(long d); 

// cac chan cua Servo
#define ENA 18
#define IN1 19
#define IN2 4

// chan doc encoder
const int encoderPinA = 35;
const int encoderPinB = 34;

// chan cua step Y
#define stepY 21
#define dirY  22
#define enaY  15

// chan cua step Z
#define stepZ 26
#define dirZ  27
#define enaZ  14

// cong tac hanh trinh
#define LMX 25
#define LMY 23
#define LMZ 32

// chan cua nam cham
#define namcham 2

// bien goc box
#define theta1_box  50
#define theta2_box  10
#define theta3_box  0

// goc theta1

// khoi tao cac bien PID
volatile double T,xung;
volatile double setpoint_theta1;
volatile double current_theta1;
volatile double E, E1, E2;
volatile double alpha, beta, gammavalue, Kp, Ki, Kd;
volatile double Output, lastOutput;

// khoi tao timer
hw_timer_t * timer0 = NULL;

// bien trang thai
bool done_home1, done_home2;

// bien cac buoc gap 5 vat
int buoc1 = 0;
int buoc2 = 0;
int buoc3 = 0;
int buoc4 = 0;
int buoc5 = 0;
int buoc6 = 0;
int buoc7 = 0;
int buoc8 = 0;
int buoc9 = 0;
int buoc10 = 0;


// bien step
volatile long xung_phat_the2, xung_dat_the2;
volatile long xung_phat_the3, xung_dat_the3;
volatile long xung_theta;
volatile long xung_theta2_qk, xung_theta3_qk;
volatile long xung_theta2_ht, xung_theta3_ht;
volatile long xung_dat_ke_tiep;

// khai bao struct cho bien vitri
typedef struct 
{
  float theta1;
  float theta2;
  float theta3;
} VITRI;

VITRI pos1;
VITRI pos2;
VITRI pos3;
VITRI pos4;
VITRI pos5;
VITRI box;



AccelStepper Stepy(1,stepY,dirY,enaY);
AccelStepper Stepz(1,stepZ,dirZ,enaZ);



void setup() {

  // khai bao toc do baud
  Serial.begin(9600);

  // khai bao output ngo ra
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(stepY,OUTPUT);
  pinMode(dirY,OUTPUT);
  pinMode(enaY,OUTPUT);

  pinMode(stepZ,OUTPUT);
  pinMode(dirZ,OUTPUT);
  pinMode(enaZ,OUTPUT);

  pinMode(namcham,OUTPUT);

  // khoi tao cac bien INPUT
  // bien LIMIT SWITCH
  pinMode(LMX,INPUT_PULLUP);
  pinMode(LMY,INPUT_PULLUP);
  pinMode(LMZ,INPUT_PULLUP);

  // bien doc encoder
  pinMode(encoderPinA,INPUT);
  pinMode(encoderPinB,INPUT);


  // khoi tao STEP
  Stepy.setEnablePin(enaY);
  Stepy.setPinsInverted(false, false, true);

  Stepz.setEnablePin(enaZ);
  Stepz.setPinsInverted(false,false,true);


  attachInterrupt(digitalPinToInterrupt(encoderPinA),dem_xung,RISING);

  timer0 = timerBegin(1,80,true);
  timerAttachInterrupt(timer0,&PID,true);
  timerAlarmWrite(timer0,10000, true);
  timerAlarmEnable(timer0);


  // khoi tao cac bien PID
  setpoint_theta1 = 0;
  current_theta1 = 0;
  E =  0;
  E1 = 0;
  E2 = 0;
  Output = 0;
  lastOutput = 0; 
  T = 0.01;
  Kp = 80.8;
  Kd = 60.8;
  Ki = 78.0;

  xung_phat_the2 = 0;
  xung_theta2_ht = 0;
  xung_theta3_ht = 0;
  xung_theta3_qk = 0;
  xung_theta2_qk = 0;


}

void loop() {
  if(Serial.available())
  {
    
    String dataread = Serial.readStringUntil('\n');
    StaticJsonDocument<800> doc;
    DeserializationError error = deserializeJson(doc,dataread);
    if(!error)
    {
      const char* type = doc["type"];

      if(strcmp(type,"position") == 0)
      {
        pos1.theta1 = doc["x1"];
        pos1.theta2 = doc["y1"];
        pos1.theta3 = doc["z1"];

        pos2.theta1 = doc["x2"];
        pos2.theta2 = doc["y2"];
        pos2.theta3 = doc["z2"];

        pos3.theta1 = doc["x3"];
        pos3.theta2 = doc["y3"];
        pos3.theta3 = doc["z3"];

        pos4.theta1 = doc["x4"];
        pos4.theta2 = doc["y4"];
        pos4.theta3 = doc["z4"];

        pos5.theta1 = doc["x5"];
        pos5.theta2 = doc["y5"];
        pos5.theta3 = doc["z5"];

        buoc1 = 1;
        if(buoc1 == 1)
        {
          setpoint_theta1 = pos1.theta1;
          xung_phat_the2 = chuyen_goc_sang_xung(pos1.theta2);
          xung_phat_the3 = chuyen_goc_sang_xung(pos1.theta3);
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(1);
          delay(400);
          set_home();
          delay(300);
          buoc2 = 1;
        }
        if(buoc2 == 1)
        {
          ve_box();
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(0);
          set_home();
          delay(8000);
          buoc3 = 1;
        }
        if(buoc3 == 1)
        {
          setpoint_theta1 = pos2.theta1;
          xung_phat_the2=chuyen_goc_sang_xung(pos2.theta2);
          xung_phat_the3=chuyen_goc_sang_xung(pos2.theta3);
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(1);
          delay(400);
          set_home();
          delay(300);
          buoc4 = 1;
        }
        if(buoc4 == 1)
        {
          ve_box();
          run_theta23(xung_phat_the2, xung_phat_the3);
          hut_tha_vat(0);
          delay(200);
          set_home();
          delay(8000);
          buoc5 = 1;
        }
        if(buoc5 == 1)
        {
          setpoint_theta1 = pos3.theta1;
          xung_phat_the2 = chuyen_goc_sang_xung(pos3.theta2);
          xung_phat_the3 = chuyen_goc_sang_xung(pos3.theta3);
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(1);
          delay(400);
          set_home();
          delay(8000);
          buoc6 = 1;
        }
        if(buoc6 == 1)
        {
          ve_box();
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(0);
          delay(200);
          set_home();
          delay(8000);
          buoc7 = 1;
        }
        if(buoc7 == 1)
        {
          setpoint_theta1 = pos4.theta1;
          xung_phat_the2 = chuyen_goc_sang_xung(pos4.theta2);
          xung_phat_the3 = chuyen_goc_sang_xung(pos4.theta3);
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(1);         
          delay(400);
          set_home();
          delay(300);
          buoc8 = 1;
        }
        if(buoc8 == 1)
        {
          ve_box();
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(0);
          delay(200);
          set_home();
          delay(8000);
          buoc9 = 1;
        }
        if(buoc9 == 1)
        {
          setpoint_theta1 = pos5.theta1;
          xung_phat_the2 = chuyen_goc_sang_xung(pos5.theta2);
          xung_phat_the3 = chuyen_goc_sang_xung(pos5.theta3);
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(1);          
          delay(400);
          set_home();
          delay(300);
          buoc10 = 1;
        }
        if(buoc10 == 1)
        {
          ve_box();
          run_theta23(xung_phat_the2,xung_phat_the3);
          hut_tha_vat(0);
          delay(200);
          set_home();
          delay(8000);
        }
        dataread = "";
        buoc1 = 0; buoc2 = 0; buoc3 = 0; buoc4 = 0; buoc5 = 0; buoc6 = 0; buoc7 = 0; buoc8 = 0; buoc9 = 0; buoc10 = 0;
        Serial.println("done save pos");
      }
      else if(strcmp(type,"command") == 0)
      {
        const char* command = doc["command"];
        if(strcmp(command,"sethome") == 0)
        {
          set_home();
          int home = 1;
          if(home == 1)
          {
            Serial.println("home");
          }
          else Serial.println("none home");
          dataread = "";
          home = 0;
        }
        else if(strcmp(command,"CONNECT" )== 0) Serial.println("ESP32 Connected");
        else if(strcmp(command,"CONNECT") != 0) Serial.println("Failed to parse JSON");
      }
      else if(strcmp(type,"FK") == 0)
      {
        int FK = 0;
        setpoint_theta1 = doc["theta1"];
        xung_phat_the2 = chuyen_goc_sang_xung(doc["theta2"]);
        xung_phat_the3 = chuyen_goc_sang_xung(doc["theta3"]); // kiem soat lai kieu du lieu;
        run_theta23(xung_phat_the2,xung_phat_the3);
        dataread = "";
        FK = 1;
        if(FK == 1)
        {
          Serial.println("done fk");
        }
      }
      else if(strcmp(type,"IK") == 0)
      {
        int IK = 0;
        setpoint_theta1 = doc["theta1_ik"];
        xung_phat_the2 = chuyen_goc_sang_xung(doc["theta2_ik"]);
        xung_phat_the3 = chuyen_goc_sang_xung(doc["theta3_ik"]); // kiem soat lai kieu du lieu
        run_theta23(xung_phat_the2,xung_phat_the3); 
        dataread = "";
        IK = 1;
        if(IK == 1)
        {
          Serial.println("done ik");
        }
      }
      else if(strcmp(type,"wificonnect") == 0)
      {
        const char* ssid = "Trong Tin";
        const char* pass = "17022003";
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid,pass);
        int attempt = 0;
        while (WiFi.status() != WL_CONNECTED && attempt < 10)
        {

          Serial.println("connecting");
          attempt++;
        }
        if(WiFi.status() == WL_CONNECTED) Serial.println("Connected to WiFi");
        else Serial.println("Failed to connected Wifi");
        dataread = "";
      }
    }
  }
}

// set home X
void home_X()
{
  // while (digitalRead(LMX) == 1)
  // {
  //   analogWrite(ENA,150);
  //   digitalWrite(IN1,LOW);
  //   digitalWrite(IN2,HIGH);
  //   if(digitalRead(LMX) == 0)
  //   {
  //     digitalWrite(IN1,LOW);
  //     digitalWrite(IN2,LOW);
  //     analogWrite(ENA,0);
  //     break;
  //   }
  // }
  // setpoint_theta1 = -30;
  
}

void home_Y()
{
  int homeY = 0;
  Stepy.setMaxSpeed(100);
  Stepy.setAcceleration(100);
  Stepy.enableOutputs();

  while (digitalRead(LMY) == 1)
  {
    Stepy.moveTo(homeY);
    homeY++;
    Stepy.run();
  }

  Stepy.setCurrentPosition(0);
  homeY = 0;
  
}

// set home Z
void home_Z()
{
  int homeZ = 0;
  Stepz.setMaxSpeed(100);
  Stepz.setAcceleration(100);
  Stepz.enableOutputs();

  while (digitalRead(LMZ) == 1)
  {
    Stepz.moveTo(homeZ);
    homeZ--;
    Stepz.run();
  }

  Stepy.setCurrentPosition(0);
  homeZ = 0;
}


void set_home()
{
  int homeY = 0;
  int homeZ = 0;
  setpoint_theta1 = 0;
  Stepy.setMaxSpeed(100);
  Stepy.setAcceleration(100);
  Stepz.setMaxSpeed(100);
  Stepz.setAcceleration(100);
  Stepy.enableOutputs();
  Stepz.enableOutputs();
  while (digitalRead(LMY) == 1 && digitalRead(LMZ) == 1)
  {
    Stepy.moveTo(homeY);
    Stepz.moveTo(homeZ);
    homeY++;
    homeZ--;
    Stepy.run();
    Stepz.run();
  }

  while (digitalRead(LMY) == 1)
  {
    Stepy.moveTo(homeY);
    homeY++;
    Stepy.run();
  }

  Stepy.setCurrentPosition(0);
  while (digitalRead(LMZ) == 1)
  {
    Stepz.moveTo(homeZ);
    homeZ--;
    Stepz.run();
  }

  Stepz.setCurrentPosition(0);
  
  homeY = 0;
  homeZ = 0;
  
}

long chuyen_goc_sang_xung(float goc)
{
  xung_theta = round((goc*200*4.5)/360.0);
  return xung_theta;
}

// doc xung encoder
void IRAM_ATTR dem_xung()
{
  if(digitalRead(encoderPinB) == LOW)
  {
    xung++;
  }
  else
  {
    xung--;
  }
}

// tinh PID
void IRAM_ATTR PID()
{
  current_theta1 = ((xung * 360.0) / 1848.0); // Calculate position
  E = setpoint_theta1 - current_theta1;          // Error calculation
  alpha = 2 * T * Kp + Ki * T * T + 2 * Kd;
  beta = T * T * Ki - 4 * Kd - 2 * T * Kp;
  gammavalue = 2 * Kd;

  // PID output calculation
  Output = (alpha * E + beta * E1 + gammavalue * E2 + 2 * T * lastOutput) / (2 * T);

  // Update states
  lastOutput = Output;
  E2 = E1;
  E1 = E;

  // Saturate output
  if (Output > 255)
    Output = 255;
  if (Output < -255)
    Output = -255;

  // Motor control
  if (Output > 0)
  {
    analogWrite(18, Output);
    digitalWrite(19, HIGH);
    digitalWrite(4, LOW);
  }
  else if (Output < 0)
  {
    analogWrite(18, abs(Output)); // Use absolute value for PWM
    digitalWrite(19, LOW);
    digitalWrite(4, HIGH);
  }
  else
  {
    analogWrite(18, 0);
    digitalWrite(19, LOW);
    digitalWrite(4, LOW); // Stop motor
  }


}

void hut_tha_vat(int tt)
{
  if(tt == 1) digitalWrite(namcham,HIGH);
  else if(tt == 0) digitalWrite(namcham,LOW);
}

void run_theta23(int t2, int t3)
{
  Stepy.setMaxSpeed(100);
  Stepy.setAcceleration(100);
  Stepz.setMaxSpeed(100);
  Stepz.setAcceleration(100);

  Stepy.moveTo(t2-75);
  Stepz.moveTo(t3-37);

  while (Stepy.distanceToGo() != 0 || Stepz.distanceToGo() != 0)
  {
    Stepy.run();
    Stepz.run();
  }

  while(Stepy.currentPosition() != (t2-75) || Stepz.currentPosition() != (t3-37))
  {
    if(Stepy.currentPosition() != (t2-75))
    {
      Stepy.setSpeed(-10);
      Stepy.runSpeed();
    }
    if(Stepz.currentPosition() != (t3-37))
    {
      Stepz.setSpeed(10);
      Stepz.runSpeed();
    }
  }
  
}

void run_theta23_first(int t2, int t3)
{
  Stepy.setMaxSpeed(100);
  Stepy.setAcceleration(100);
  Stepz.setMaxSpeed(100);
  Stepz.setAcceleration(100);

  Stepy.moveTo(t2);
  Stepz.moveTo(t3);

  while (Stepy.distanceToGo() != 0 || Stepz.distanceToGo() != 0)
  {
    Stepy.run();
    Stepz.run();
    
  }
  while(Stepy.currentPosition() != (t2-75) || Stepz.currentPosition() != t3)
  {
    if(Stepy.currentPosition() != (t2-75))
    {
      Stepy.setSpeed(-10);
      Stepy.runSpeed();
    }
    if(Stepz.currentPosition() != t3)
    {
      Stepz.setSpeed(10);
      Stepz.runSpeed();
    }
  }
  
}

void vi_tri_ban_dau()
{
  setpoint_theta1 = 0;
  xung_phat_the2 =  chuyen_goc_sang_xung(20);
  xung_phat_the3 = chuyen_goc_sang_xung(-30);
  run_theta23(xung_phat_the2,xung_phat_the3);
}

long di_chuyen_vi_tri_ke_tiep(long xung_qk, long xung_ht)
{
  xung_dat_ke_tiep = xung_ht - xung_qk;
  if(xung_dat_ke_tiep >=0)
  {
    xung_dat_ke_tiep = abs(xung_dat_ke_tiep);
  }
  else
  {
    xung_dat_ke_tiep = -abs(xung_dat_ke_tiep);
  }
  return xung_dat_ke_tiep;
}

void ve_box()
{
  box.theta1 = 50;
  box.theta2 = -40;
  box.theta3 = 60;

  setpoint_theta1 = box.theta1;
  xung_phat_the2 = chuyen_goc_sang_xung(box.theta2);
  xung_phat_the3 = chuyen_goc_sang_xung(box.theta3);

}

int sign(double d)
{
  int sign = 1;
  if(d>=0) sign = 1;
  else sign = -1;
  return sign;
}
