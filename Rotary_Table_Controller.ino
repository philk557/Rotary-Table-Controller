/* Rotary Table Controller
 *  
   Runs a stepper motor on Sherline Rotary Table
   SparkFun AutoDriver board drives the steepr motor.
   Uses phi-Panel for LCD display and 4x4 keyboard.
*/
#include <SparkFunAutoDriver.h>
//#include <SPI.h>  Included in the SparkFunADboard library.

// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"

#define print_status false  //Monitor print normal config & status
/*
//  Arduino Uno SPI pins

#define MOSI  11  //Actually ICSP pin 4
#define MISO  12  //Actually ICSP pin 1
#define SCK   13  //Actually ICSP pin 3
#define SS    10  //Also known as CS
*/
//  Ardion Mega SPI pins

#define MOSI  51  //Actually ICSP pin 4
#define MISO  50  //Actually ICSP pin 1 
#define SCK   52  //Actually ICSP pin 3
#define SS    10  //Also known as CS

#define RESET  6  //AD board RESET
#define BUSY   7  //AD board BUSY
#define int_pin 3 //Interrupt pin

#define AD_board_0 0 //First AD board

  const int   Max_Secs          =600; //Max run table time
  const long  steps_div         =130; //microsteps/step
  const float Steps_per_Degree  = 40; //200 steps per 5 degrees divided by 5
  const float Max_Speed         = 20; //Max table speed in degrees per second
  const float Max_Degrees       =361; //Max table move, 1 tsble rev +1

  RTC_DS3231 rtc;
  AutoDriver ADboard(AD_board_0, SS, RESET, BUSY);

  char   daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  long   prev_pos  =0;
  float  degPos    =0;    //Table position in degrees, motor steps *40
  String message   = "                    "; //length is 20

//  Following used by interrupt service routine.
//
  volatile long    steps       =0;      //Steps counted while running
  volatile long    targetSteps =0;      //Move target
  volatile boolean runStop     =false;  //Stop run when true

void setup()
{
  Serial.begin(9600);       //Begin IDE monitor serial communication
  rtc.begin(); 
  Serial.print("\n");
  time_stamp();
  Serial.println("Sketch started");
  Serial1.begin(9600);      //Begin Phi-panel serial communication  
                            // Serial1 on Arduino Mega uses
                            //  Rx pin 19, Tx pin 18
/*
  Serial1._config();
*/  
  DS3231_config();
  SPI_config();
  ADboard_config();
//  attachInterrupt(1,count_Steps,LOW);
  
//  Serial.println("Config done");

  beep(300);
  writeLCD("\f      ROTARY\n");
  writeLCD(  "      TABLE\n"); 
  writeLCD(  "    CONTROLLER\n");
  writeLCD(  "     ver. 0.1"); 
  wait(1000);
}

void loop()
{
  int    maxSpeed    =0;
  char   key         =0;
  String name        = "";

  writeLCD("\f1: Run   4: Jog\n");
  writeLCD(  "2: Move  5: Home\n");
  writeLCD(  "3: Frac  6: Set home");
//  writeLCD("\eP~");         //Buzz
  key=readKeypad();         //Retrieve keypad key press
  writeLCD(" ");
  writeLCD(String(key));
/*
  Serial.print("key: ");
  Serial.println(key);
*/
  
  switch(key)
  {
    case '1':
    {
//      Serial.println("case 1:");
      runTable();     //run table until wait done
      break;
    }
    case '2':
    {
      moveTable();    //move table to new position
      break;
    }
    case '3':
    {
      fracTable();    //move table in fractions of a circle
      break;
    }
    case '4':
    {
      jogTable();     //manually control table movement
      break;
    }
    case '5':
    {
      homeTable();    //move table to home position
      break;
    }
    case '6':
    {
      setHome();      //set home to curent position
      break;
    }
    default: 
    {
      Serial.print("Default: key: ");
      Serial.println(key);
      break;
    }
  }
}  

/* ------------------------------------------------------------------
 *  Run table at nnn deg/sec for for nnn seconds
 */
void  runTable()
{
  
  byte  dir         =FWD;
  int   run_sec     =1;
  float table_speed =0;  //in deg/sec
  
  Serial.println("\nRUN TABLE");
  message ="\fRUN Pos= " +String(degPos, 1) +" deg\n";
  writeLCD(message);
  writeLCD("Run ________ Deg/Sec");
  writeLCD("Run for ____ Secs\n");
  writeLCD("- for CCW");

  table_speed =readNum(2,5,8);
  run_sec     =(int) readNum(3,9,4);
  if(table_speed>0)dir=FWD;
  else
  {
    dir=REV;
    table_speed=-table_speed;
  }
  
  if (table_speed <=0 ||table_speed >Max_Speed)
  {
    message="Max is " +String(Max_Speed) +" deg/sec"; 
    positionCursor(4,1);
    writeLCD(message);
    positionCursor(2,5);
    writeLCD("________"); //blank field
    beep(300);
    table_speed =readNum(2,5,8);
  }
    
  if (run_sec >Max_Secs)
  {
    positionCursor(4,1);
    message="Max time " +String(Max_Secs) +" sec"; 
    writeLCD(message);
    positionCursor(3,9);
    writeLCD("____"); //blank field
    beep(300);
    run_sec =(int) readNum(3,9,4);
  }
  time_stamp();
  if (dir==FWD) Serial.print("Run FWD at ");
  if (dir==REV) Serial.print("Run REV at ");
  Serial.print(table_speed, 2);
  Serial.print(" degrees/sec for ");
  Serial.print(run_sec);
  Serial.println(" secs");
//  ADboard.getPos();
  run_stepper(dir, table_speed, run_sec);    
//  ADboard.getPos();
}   

/* ------------------------------------------------------------------
 *  Move table at nnn deg/sec to xxx degrees
 */
void  moveTable()
{
  byte  dir         =FWD;
  float deg         =0;
  float move_speed  =0;  //in deg/sec

  Serial.println("\nMOVE TABLE");
  message ="\fMOVE Pos=" +String(degPos, 1) +" deg\n";
  writeLCD(message);
  AD_get_pos();
  writeLCD("Move _______ deg\n");
  writeLCD("at _______ deg/sec");
  deg =readNum(2,6,8);
  move_speed   =readNum(3,4,8);

  if (deg >Max_Degrees)
  {
    message="Max is " +String(Max_Degrees) +" deg"; 
    positionCursor(4,1);
    writeLCD(message);
    positionCursor(2,6);
    writeLCD("_______"); //blank field
    beep(300);
    deg =readNum(2,6,8);
  }

  if(move_speed>0) dir=FWD;
  else
  {
    dir=REV;
    move_speed=-move_speed;
  }
  
  if (move_speed >Max_Speed)
  {
    message="Max is " +String(Max_Speed) +" deg/s"; 
    positionCursor(4,1);
    writeLCD(message);
    positionCursor(3,4);
    writeLCD("_______"); //blank field
    beep(300);
    move_speed =readNum(3,4,8);
  }
 
  time_stamp();
  if (dir==FWD) Serial.print("Move FWD ");
  if (dir==REV) Serial.print("Move REV ");
  Serial.print(deg);
  Serial.print(" degrees at ");
  Serial.print(move_speed, 2);
  Serial.println(" degrees/sec");
  move_stepper(dir, deg, move_speed);
  ADboard.softStop();
}

/* ------------------------------------------------------------------
 *  Move table in fractions of a circle.  Repeat until circle completed.
 */
void  fracTable()
{
  Serial.println("\nMOVE TABLE FRACTIONS OF A CIRCLE");
  writeLCD("\fFRAC TABLE\n");
  wait(1000);
}    

/* ------------------------------------------------------------------
 *  Jog table using rotary knob control
 */
void  jogTable()
{
  Serial.println("\nJOG TABLE");
  writeLCD("\fJOG TABLE\n");
  wait(1000);
}    

/* ------------------------------------------------------------------
 *  Move table to home position 
 */
void  homeTable()
{

  Serial.println("\nHOME TABLE");
  writeLCD("\fHOME TABLE\n");
  wait(1000);
}    

/* ------------------------------------------------------------------
 *  Set table home to current position
 */
void  setHome()
{

  Serial.println("\nSET HOME");
  writeLCD("\fSET HOME\n");
  wait(1000);
}    

/* ------------------------------------------------------------------
 *  Run stepper at x deg/sec for y seconds
 *  Positive number moves CW
 *  Negative number moves CCW
 */
void run_stepper(byte dir, float degs_per_sec, int wait_sec)
{
//  long          get_pos       =0;
  float         steps_per_sec =0;
  unsigned long currMs        =0;
  unsigned long elapsedMs     =0;
 
 if (degs_per_sec <=Max_Speed) steps_per_sec =degs_per_sec *Steps_per_Degree;
 else
 {
   steps_per_sec =Max_Speed *Steps_per_Degree;
 }
  time_stamp();
  Serial.print("Run ");
  if (dir==FWD) Serial.print("FWD ");
  if (dir==REV) Serial.print("REV ");
  Serial.print(steps_per_sec);   
  Serial.print(" steps/sec for ");
  Serial.print(wait_sec);   
  Serial.println(" secs");
  
  ADboard.softStop();
  while (ADboard.busyCheck());   //Wait until softStop done
  currMs =millis();
  AD_get_pos();
  ADboard.run(dir, steps_per_sec);
  AD_read_status();
  
  for (int i =0; i <=wait_sec; i++)
  {
    AD_abs_pos();
    AD_read_status();
    wait(1000);
  }    
  
//  wait((unsigned long) wait_sec*1000);
  AD_read_status();
  elapsedMs =millis()-currMs;
  AD_get_pos();
  Serial.print("elapsed ms= ");
  Serial.println(elapsedMs);
  ADboard.softStop();
  while (ADboard.busyCheck());   //Wait until softStop done
  AD_read_status();
}
  
/* ------------------------------------------------------------------
 *  Move stepper x deg
 *  Positive number moves CW
 *  Negative number moves CCW
 */
void move_stepper(byte dir, float move_deg, float degs_per_sec)
{
  const long  decSteps =500;  //Number of decleration steps
  const float decSpeed = 10;  //Deceleration speed in step/sec

  long  currentPos    =0; //Current position in steps
  long  targetPos     =0; //Target position in steps
  long  moveSteps     =0; //Number of steps to move
  float steps_per_sec =0; //Move speed in steps/sec
  
  if ( degs_per_sec <=Max_Speed) steps_per_sec =degs_per_sec *Steps_per_Degree;
  else                           steps_per_sec =Max_Speed    *Steps_per_Degree;
  moveSteps= (long) (move_deg *Steps_per_Degree);
  ADboard.softStop();
  while(ADboard.busyCheck());      //Wait until softStop done
//  targetPos =ADboard.getPos() +moveSteps;
  targetPos =ADboard.getPos() +moveSteps*steps_div;
  Serial.print("target_pos: ");
  Serial.println(targetPos);
  time_stamp();
  Serial.print("move ");
  if (dir==FWD) Serial.print("FWD ");
  if (dir==REV) Serial.print("REV ");
  Serial.print(String(moveSteps));   
  Serial.print(" steps at ");
  Serial.print(steps_per_sec);
  Serial.println(" steps/sec");
    
  if(moveSteps >=decSteps)
  {
    ADboard.run(dir, steps_per_sec);              //Start running
    AD_read_status();
//    while (ADboard.getPos() <=targetPos -decSteps);
    currentPos =AD_get_pos();
    while (currentPos <=targetPos -decSteps)
    {
//      Serial.print("currentPos: ");
//      Serial.println(currentPos);
      wait(1000);
      currentPos =AD_get_pos();
    }
   
    ADboard.run(dir, decSpeed);                   //Decelerate before stopping
    AD_read_status();
    currentPos =AD_get_pos();
    while (currentPos <=targetPos)         //slow down early
    {
//      Serial.print("currentPos: ");
//      Serial.println(currentPos);
      wait(1000);
      currentPos =AD_get_pos();
    }
   }
  else
  {
    ADboard.run(dir, decSpeed);  //Run slowly before stopping
    AD_read_status();
    currentPos =AD_get_pos();
    while (currentPos <=targetPos)         //slow down early
    {      
//      Serial.print("currentPos: ");
//      Serial.println(currentPos);
      wait(1000);
      currentPos =AD_get_pos();
    }
  }
  
  while(currentPos <=targetPos)
  {
//      Serial.print("currentPos: ");
//      Serial.println(currentPos);
    wait(1000);
    currentPos =AD_get_pos();
  }

/*
  runStop =false; 
  while (!runStop)       //Wait until move done
  {
    wait(1000);
    Serial.print("steps= ");
    Serial.println(steps);
  }
*/  
  ADboard.softStop();
  while (ADboard.busyCheck());   //Wait until softStop done
  AD_read_status();
  AD_get_pos();
}

/* ------------------------------------------------------------------
 *  Read stepper motor position from the AutoDriver Board.
*/
 
long  AD_abs_pos()
{
  long abs_pos =0;
  long get_pos =0;
  long twos    =0;

  time_stamp();
  get_pos =ADboard.getPos();
  Serial.print("get_pos: ");
  Serial.print(get_pos, HEX);
  Serial.print(" ");
  Serial.print(get_pos);
  Serial.print(" ");
  Serial.print(get_pos-prev_pos);
  prev_pos=get_pos;
 
  abs_pos =ADboard.getParam(ABS_POS);
  Serial.print("  ABS_POS: ");
  Serial.println(abs_pos, HEX);
//  Serial.print(" ");
//  Serial.print(abs_pos);
//  Serial.print("  &0x00FFFF ");
/*
  abs_pos =abs_pos &0x00FFFF;
  if(abs_pos &0x00008000) twos =abs_pos |0xFFFF8000; //2**15 twos complement
  else twos =abs_pos;    
  Serial.print(abs_pos, HEX);
  Serial.print(" ");
  Serial.print(twos, HEX);
  Serial.print(" ");
  Serial.println(twos);
//  Serial.print(" ");
*/
/*
  if(abs_pos &0x00200000)       //If 21 bit position negative
  {
    twos =abs_pos |0xFFE00000;  //Take 2**21 twos complement
    Serial.print(twos);
  } 
  else Serial.print(abs_pos);
*/
/*
  abs_pos =abs_pos &0x00FFFF;
  Serial.print(abs_pos, HEX);
  Serial.print(" ");
  Serial.println(abs_pos);
*/  
  return (abs_pos);
}
/* ------------------------------------------------------------------
 *  Read stepper motor position from the AutoDriver Board.
*/
 
long  AD_get_pos()
{
  long  microSteps =0;
  long  stepsPos   =0;    //Table position in motor steps
  float movedDeg   =0;
  float movedSteps =0;
  
  microSteps =ADboard.getPos();
  stepsPos =microSteps/steps_div;
  degPos =(float) stepsPos/Steps_per_Degree; //Set global
  movedSteps =(float)(microSteps -prev_pos)/ (float) steps_div;
  movedDeg =(float) movedSteps/Steps_per_Degree; 
  time_stamp();
  Serial.print("microSteps: ");
  Serial.print(microSteps, HEX);
  Serial.print(" Pos: ");
  Serial.print(degPos, 2);
  Serial.print(" deg  ");
  Serial.print(stepsPos);
//  Serial.print(" ");
//  Serial.print(stepsPos, HEX);
  Serial.print(" steps   Plus ");
  Serial.print(movedDeg, 2);
  Serial.print(" deg ");
  Serial.print(movedSteps);
  Serial.println(" steps  ");
  prev_pos =microSteps;
  return (microSteps);
}

/* ------------------------------------------------------------------
 *  Read STATUS register from the AutoDriver Board.
*/
boolean  AD_read_status()
{
  #define HiZ         0x0001 //Briges HiZ state, stepping motor is free
  #define BUSY        0x0002 //AD board busy
  #define SW_F        0x0004 //Switch closed
  #define SW_EVN      0x0008 //Switch turn-on event
  #define DIR         0x0010 //0 = reverse, 1 = forward
  #define MOT_STATUS  0x0060
  #define STOPPED     0 //Motor stopped
  #define M_ACC       1 //Motor accelerating
  #define M_DEC       2 //Motor deccelerating
  #define CONST_SPEED 3 //Motor running at constant speed
  #define NOTPERF_CMD 0x0080 //ommand cannot be performed
  #define WRONG_CMD   0x0100//Command does not exist
  #define UVLO        0x0200 //Undervoltage lockout or board reset (including power-up)
  #define TH_WRN      0x0400 //Thermal warning
  #define TH_SD       0x0080 //Thermal shutdown
  #define OCD         0x1000 //Overcurrent detection
  #define STEP_LOSS_A 0x2000 //Bridge A step loss
  #define STEP_LOSS_B 0x4000 //Bridge B step loss
  #define SCK_MOD     0x8000 //Step clock mode
  
  unsigned int flags =0;  //status register flags
  
  flags = ADboard.getStatus();
  if(print_status)
  {
    Serial.print("Status: ");
    Serial.println(String(flags, HEX));
    if( flags &HiZ)    Serial.println(" HiZ state");
    if(!flags &BUSY)   Serial.println(" Busy");
    if( flags &SW_F)   Serial.println(" Switch closed");
    if( flags &SW_EVN) Serial.println(" Switch turn-on event");
    if( flags &DIR)    Serial.println(" Forward direction");
    else               Serial.println(" Reverse direction");
    mot_stat(flags);
    if( flags &SCK_MOD)      Serial.println(" Step clock mode");
  }
  if( flags &NOTPERF_CMD)
  {
    beep(300);
    Serial.println(" Command cannot be performed");
  }
  if( flags &WRONG_CMD)
  {
      beep(300);
      Serial.println(" Command does not exist");
  }    
  if(!flags &UVLO)
  {
    beep(300);
    Serial.println(" Undervoltage lockout or board reset");
  }    
  if(!flags &TH_WRN)
  {
    beep(300);
    Serial.println(" Thermal warning");
  }
  if(!flags &TH_SD)
  {
    beep(300);
    Serial.println(" Thermal shutdown" );
  }
  if(!flags &OCD)
  {
    beep(300);
    Serial.println(" Overcurrent detection");
  }
  if(!((flags &STEP_LOSS_A) ||(flags &STEP_LOSS_B)))
  {    
    beep(300);
    Serial.println(" Step loss");
  }
}

/* ========================================================
 * Determine motor status 
*/
void mot_stat(unsigned int flags)
{
/*
  Serial.print("flags: ");
  Serial.println(flags, BIN);
*/
  flags <<= 9;  
  flags >>= 14;  
/*    
  Serial.print("flags: ");
  Serial.println(flags, BIN);
*/
  switch(flags)
  {
    case STOPPED:
    {
      Serial.println(" Motor stopped");
      break;
    }
   case  M_ACC:
    {
      Serial.println( " Motor accelerating");
      break;
    }
    case M_DEC:
    {
      Serial.println( " Motor deccelerating");
      break;
    }
    case CONST_SPEED:
    {
      Serial.println(" Motor running at constant speed");
      break;
    }
    default:
    {
      Serial.println(" Invalid motor status " +String(flags, HEX));
      break;
    }
  }
}

/* ------------------------------------------------------------------
 *  Read CONFIG register from the AutoDriver Board.
*/

 int AD_read_config()  
{ 
  #define OSC_SEL   0x0007  //Oscillator selection
  #define EXT_CLK   0x0008  //External clock
  #define SW_MODE   0x0010  //Switch mode
  #define EN_VSCOMP 0x0020  //Motor supply voltage compensation
  #define OC_SD     0x0080  //Overcurrent event bridge shutdown control
  #define POW_SR    0x0300  //Slew rate setting
  #define SR320     0x00    //320 v/micro-sec
  #define SR75      0x01    //75 v/micro-sec
  #define SR110     0x02    //110 v/micro-sec
  #define SR260     0x03    //2600 v/micro-sec
  #define F_PWM_INT 0x1C00  //PWM freq div factor
  #define F_PWM_DEC 0xE000  //PWM freq mult factor

  int configs =0;
  
  configs = ADboard.getParam(CONFIG);
  Serial.print("Config: ");
  Serial.println(String(configs, HEX));

  if( configs &OSC_SEL)  Serial.println(" " +String(CONFIG &OSC_SEL) +" Oscillator selection");
  if( configs &SW_MODE)  Serial.println(" Switch interrupt user disposal");
  else                   Serial.println(" Switch interrupt hard stop");
  if( configs &EN_VSCOMP) Serial.println(" Motor supply voltage compensation enabled");
  if( configs &OC_SD)    Serial.println(" Bridge shutdown on overcurrent event enabled");
/*
  if( configs &POW_SR)   Serial.println(" Slew rate");
  if( configs &F_PWM_INT)Serial.println(" PWM freq div factor");
  if( configs &F_PWM_DEC)Serial.println(" PWM freq mult factor");
*/  
}    

 /* ------------------------------------------------------------------
 *  Read number from Phi-panel LCD screen field using keypad.
 *  Cursor moves to row,col, the start of a field with specified length.
 *  Valid keys are '-', '0-9', BS and ENTER(*)
 *  Number can start with '-' sign and can include decimal point '.'.
 *  Backspace moves cursor back over previous character.
 *  ENTER '*' enters number.
 */

 float readNum (int row, int col, int fieldLength)  
{ 
  char   in_char[9] ={' ',' ',' ',' ',' ',' ',' ',' ','\0'};
  int   num;
  float return_num =0;

/*  Serial.println("readNum started");
  Serial.print(row);
  Serial.print(" ");
  Serial.print(col);
  Serial.print(" ");
  Serial.println(fieldLength);
*/
  message="String(row) +";" +String(col)";
//  Serial.println(message);
  positionCursor(row, col);

  if (Serial1.available() > 0)   // Is a character available?
    
  if (fieldLength>8) fieldLength=8;

  // Read keys and build valid chars until ENTER pushed.

  for (int n = 0; n < fieldLength; n++) 
  {
     in_char[n]=readKeypad();
/*
     Serial.print("in_char[");
     Serial.print(n);
     Serial.print("]: ");
     Serial.print(in_char[n]);
     Serial.print(" ");
     Serial.println(in_char[n],DEC);
*/ 
     if (in_char[n]!='\n')   //if not ENTER
     {
       if(   (in_char[n]>='0')  //Allowable chars
           &&(in_char[n]<='9')
           ||(in_char[n]=='-')
           ||(in_char[n]=='.')
           ||(in_char[n]=='\b'))
       {                 
        
 //         Serial.print(in_char[n]);
 
          Serial1.write(in_char[n]);
          if ((in_char[n]=='\b')&&(n>0)) n-=2; // Back space.
/*        
          Serial.print("inchar:");
          Serial.println(in_char);
*/
       }
       else --n;                //Ignore char
     }
     else
     {
      
//      Serial.print("break for loop\n");
      
      in_char[n]=0;             // Terminate the string
      break;                    // Break out of the for loop
      }
   }
      
        return_num=atof(in_char);  // Get the number from the string
          
//        Serial.println(return_num);

   return return_num;
}

/* ------------------------------------------------------------------
 *  Position Phi-panel LCD cursor at row & col..
 */
void positionCursor(int row, int col)
{
//  Serial.println("row= " +String(row)+"  col= " +String(col));
  writeLCD("\e[" +String(row) +";" +String(col) +"H~");   //Position cursor at row, col
}
  
/* ------------------------------------------------------------------
 *  Read key from Phi-panel keypad.
 */
char readKeypad() 
{
  int  readInt    =0;
  char returnChar ='\0';
/*
  Serial.println("\nreadKeypad");  
  Serial.println("Waiting for Idle");
*/
  while(!(Serial1.available() > 0));  //Wait until idle

//  Serial.println("Idle");

  if (Serial1.available() > 0) 
   {
/*      Serial.print("Serial1.Available: ");
      Serial.println(Serial1.available());
*/      
      readInt = Serial1.read();
      returnChar = char(readInt);
/*      
    Serial.print("ReturnChar: ");
    Serial.println(returnChar);
*/   
       return returnChar;
    }
    else return'\0';
/* 
    Serial.print("ReturnChar: ");
    Serial.println(returnChar);
*/    
}

/* ------------------------------------------------------------------
 *  Wriye msg string to Phi-panel LCD screen at current cursor position..
 */
void writeLCD(String msg) 
{

  char buffer[91];
  int  msgLen  =0;
  int  n       =0;
/*  
  Serial.print("msg: ");
  Serial.print(msg);
  Serial.print(" ");
  Serial.println(msg.length()); 
*/
  if (msg.length()<=90) msgLen=msg.length(); 
  else                  msgLen=90;
/*
  Serial.print("msglen: ");
  Serial.println( msgLen);
*/ 
  for (n = 0; n < msgLen; n++) 
  {
    buffer[n] = msg[n];
/*
    Serial.print(n);
    Serial.print(" ");
    Serial.println(buffer[n]);
*/
  }
  buffer[n]='\0';          //Terminate string
/*
  Serial.print("Buffer: ");
  Serial.print(buffer);
  Serial.print(" ");
  Serial.println(strlen(buffer));
*/ 
  Serial1.write(buffer, msgLen);
  wait(50);                      //Min wait found experimentally
}

/* ------------------------------------------------------------------
 *  Print time stamp
 */
 void time_stamp ()
{
  DateTime now= rtc.now();

  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(' ');
}

/* ------------------------------------------------------------------
 *  Confogure DS3231 Real_Time Clock.
 */
 void DS3231_config ()
{
   if (! rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}
  
/* ------------------------------------------------------------------
 *  Confogure SPI communication to AutoDrive board.
 */
 void SPI_config ()
{
//  Serial.println("SPI_config started");

  // Start by setting up the pins and the SPI peripheral.
  //  The library doesn't do this for you! 
  
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
}
 
/* ------------------------------------------------------------------
 *  Confogure AutoDrive board.
 */
void ADboard_config()
{
  unsigned long param =0;
  
//  Serial.println("ADboard_config started");
    pinMode(int_pin, INPUT_PULLUP);  //Interrupts for each step
    pinMode(RESET, OUTPUT); //Reset pin
    pinMode (BUSY,INPUT);  //Busy pin
    
    digitalWrite(RESET, LOW);
    digitalWrite(RESET, HIGH); //Reset L6470 chip

    // Tell the ADboard object which SPI port you want to use. Some
    //  devices may have more than one.

    ADboard.SPIPortConnect(&SPI);

    ADboard.configSyncPin (BUSY_PIN,0); //Use Busy Pin to know when not busy
    ADboard.configStepMode(STEP_FS);    //Microstepping disabled
    ADboard.setMaxSpeed   (Max_Speed *Steps_per_Degree);
    ADboard.setMinSpeed   (10);
    ADboard.setFullSpeed  (Max_Speed *Steps_per_Degree);
    ADboard.setAcc        (1000);
    ADboard.setDec        (1000);
    ADboard.setOCThreshold(OC_1500mA);
//    ADboard.setPWMFreq    (PWM_DIV_1, PWM_MUL_1_25);
//    ADboard.setSlewRate   (SR_180V_us);
    ADboard.setOCShutdown (OC_SD_ENABLE);
    ADboard.setVoltageComp(VS_COMP_DISABLE);
    ADboard.setSwitchMode (SW_HARD_STOP);        //SW_HARD_STOP or SW_USER
    ADboard.setOscMode    (INT_16MHZ);
    ADboard.setAccKVAL    (0x2C); //44
    ADboard.setDecKVAL    (0x2C); //44
    ADboard.setRunKVAL    (0x2C); //44  
    ADboard.setHoldKVAL   (0x25); //37

    ADboard.resetDev();
//    Serial.println("resetDEV");
    while (ADboard.busyCheck());  //Wait until ADboard board idle

    // first check AD board config register, should be 0x2E88 on bootup
    
    
    // Now check the status of the AD board. Should be 0x7c03

  if(print_status)
  {
    AD_read_config();
    AD_read_status();
   }    
}

/* ------------------------------------------------------------------
 *  Interupt Service Routine.
 */
void count_Steps()
{
//  if(steps++ >=targetSteps) runStop=true;
}

/* ------------------------------------------------------------------
 *  Delay blocks interrupts, use wait instead.
 */
void wait(unsigned long wait_ms)
{
  unsigned long target_ms;
  
  target_ms =millis() +wait_ms;
/*  
  Serial.print("millis()= ");
  Serial.print(millis());
  Serial.print(" wait_ms= ");
  Serial.print(wait_ms);
  Serial.print(" target_ms= ");
  Serial.println(target_ms);
*/  
  while (millis() <target_ms);
/*  
  Serial.print("millis()= ");
  Serial.println(millis());
*/  
}

void beep(int delayms)
{
  analogWrite(9, 255);      // Almost any value can be used except 0 and 255
  wait(delayms);          // wait for a delayms ms
  analogWrite(9, 0);       // 0 turns it off
//  wait(delayms);          // wait for a delayms ms   
}  

