/*
  date: 27/05/2023
  team: BK-AIRBUS
  GV  : Tran Hoang Quan
  subject: Embedded system designs
*/

/*****************************
   version: Example Project for Embedded system designs
   F-CPU: 16MHz
   microcontroller: ATmega328p
 *****************************/

#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Define time remain state of system 
#define TIME_REMAIN_STATE_1 3     //s    1.Normal request
#define TIME_REMAIN_STATE_2 5    //s    2.Allow enter cockpit (Open operation)
#define TIME_REMAIN_STATE_3 10     //s    3.Denied enter cockpiy (Lock operation)
#define TIME_REMAIN_STATE_4 30    //s    4.Emergency request enter cockpit (Enter pin correct)
#define TIME_REMAIN_STATE_5 15   //s     5.Security system failure (Mode 4 success to open door)
#define TIME_DEBOUNCE 20   //ms
//end define time remain state of system


//define function initial hardware 
void initialTimer0();             // Init timer use realtime ios
void initialLcd();                // Init LCD for userinterface
void initialExternalInteruprt();  // Init interrupt for switch lock or unclock
void initialPinConfig();          // Config pin 
void initializationSystem();      // Init system 
void initSPI(unsigned int boudrate);// Init Uart protocol to print report health system
//End define function initial hardware

//global variable of system
unsigned char g_operationCode = 0;
unsigned char g_stateCodeLcd = 0;
bool g_firstInit = 1;
char g_remainState = 0;
char g_remainLcd = 0;
unsigned char g_errorSystem = 0;
unsigned char g_pinInput[4] = {0,0,0,0};
unsigned char g_pinInputInside[4] = {0,0,0,0};
unsigned char g_pinCorrect[4] = {1,1,1,1};
unsigned char g_errorCode[4] = {0,0,0,0};
signed char g_poiterPinOutside = 0;
signed char g_poiterPinInside = 0;
unsigned char g_vPort = 0;
unsigned char g_engineerCode[7] = {0,0,0,0,0,0,0};
unsigned char g_dateTime[10] = {2,5,45,1,0,45,2,0,0,2};
unsigned char g_poiterEC = 0;
//for data text LCD or Uart interface
unsigned char g_uperLineLcd[16];
unsigned char g_lowerLineLcd[16];
char stateMenuLcd = 0;
//for testting
bool testting = 0;
bool g_checkLock = 0;
bool g_checkUnlock = 0;
//for flag
bool g_changeStateLcd = 0;
bool g_changeState = 0;
bool g_priorityState = 0;
bool g_changeSec = 0;
//End define global variable 

//global variable of time in system
struct g_time{
  char mili = 0;
  char sec = 0;
  char minute = 0;
  char hour = 0;
} g_timeSys;
unsigned char g_timeSec = 0;
//end define variable of time in system

//core function operation system
void operationSystem(unsigned char key);    //Core operation, input is key which is get from function scankey
void operationLcd(unsigned char key);       //Core operation of LCD, ...
//End define function core function 

//function software
bool checkCode(); 
char remainTimeTen();
char remainTimeUnit();
void printLcdU();
void printLcdL();
bool checkError();
void USART_TransmitTime();
//for lcd data
void setErrorData();
void setNormalData();
//End define software

//function hardware interface
void setVPort(unsigned char input); //function to write to virtual port 
void setLCDPort(unsigned char input); //function to write data to LCD port
void writeDataLCD(unsigned char data); //function write data to LCD
void writeCommandLCD(unsigned char command); //function write command to LCD
void setBitPB(unsigned char bit, bool logic); //function to write bit to port B
void setBitPD(unsigned char bit, bool logic); //function to write bit to port D
void ledOpen(bool input);
void ledFail(bool input);
void ledAllow(bool input);
void ledDenied(bool input);
void ledLock(bool input);
void ledUnlock(bool input);
void buzzer(bool input);
void soundPushButton();
void USART_Transmit(unsigned char data);
//for scankey
unsigned char scanKeyOutside();
unsigned char scanKeyInside();
//for testting hardware
unsigned char testKeyPad();
unsigned char testDisplay();
void testLight();
void stateTest();
//end define function hardware interface

//function init state
void initState0();  //init stata 0: normal working without any signal
void initState1();  //init state 1: normal request access
void initState2();  //init state 2: unlock
void initState3();  //init state 3: lock 
void initState4();  //init state 4: emergency request
void initState5();  //init state 5: fail system
//for function init lcd
void initStateLcd1();
void initStateLcd2();
//end define function init state

//Function interrupt
ISR(TIMER0_OVF_vect){ //timer 0 use for timer of system with 10ms for interrupt
  TCNT0=0x63;
  g_timeSys.mili ++;
  if(g_timeSys.mili >= 100){
    g_timeSys.mili = 0;
    g_changeSec = 1;
    g_timeSys.sec ++;
    if(g_timeSec > 0) g_timeSec --;
    if(g_remainState > 0) g_remainState --;
    if(g_remainLcd > 0) g_remainLcd --;
    if(g_timeSys.sec >= 60){
      g_timeSys.sec = 0;
      g_timeSys.minute ++;
      if(g_timeSys.minute >= 60){
        g_timeSys.minute = 0;
        g_timeSys.hour ++;
        if(g_timeSys.hour >= 24) g_timeSys.hour = 0;
      }
    }
  }
  if(g_remainLcd == 0); //turnOffLcd();
}

ISR(INT0_vect){   //Unlock function / allow to access cockpit
  if(testting){
    g_checkUnlock = 1;
  }else{
    g_operationCode = 2;  //unlock mode
    initState2();
    g_priorityState = 1;
    g_stateCodeLcd = 0;
    g_uperLineLcd[0]='='; g_uperLineLcd[1]='='; g_uperLineLcd[2]='=';
    g_uperLineLcd[3]='['; g_uperLineLcd[4]='C'; g_uperLineLcd[5]='a';
    g_uperLineLcd[6]='u'; g_uperLineLcd[7]='s'; g_uperLineLcd[8]='s';
    g_uperLineLcd[9]='i'; g_uperLineLcd[10]='o'; g_uperLineLcd[11]='n';
    g_uperLineLcd[12]=']'; g_uperLineLcd[13]='='; g_uperLineLcd[14]='=';
    g_uperLineLcd[15]='=';
    g_lowerLineLcd[0]='D'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='o';
    g_lowerLineLcd[3]='r'; g_lowerLineLcd[4]=' '; g_lowerLineLcd[5]='I';
    g_lowerLineLcd[6]='s'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='O';
    g_lowerLineLcd[9]='p'; g_lowerLineLcd[10]='e'; g_lowerLineLcd[11]='n';
    g_lowerLineLcd[12]='n'; g_lowerLineLcd[13]='i'; g_lowerLineLcd[14]='n';
    g_stateCodeLcd = 0;
    printLcdL();
    printLcdU();
    USART_TransmitTime();
    int counter = 0;
    char tempText[] = "Pilot UNLOCKED cockpit door.\n";
    for(counter = 0; counter < 29; counter ++){
      USART_Transmit(tempText[counter]);
    }
  }
}

ISR(INT1_vect){ //lock function / denied to access cockpit
  if(testting){
    g_checkLock = 1;
  }else{
    g_operationCode = 3;  //lock mode
    initState3();
    g_priorityState = 1;
    g_stateCodeLcd = 0;
    g_uperLineLcd[0]='='; g_uperLineLcd[1]='='; g_uperLineLcd[2]='[';
    g_uperLineLcd[3]='U'; g_uperLineLcd[4]='n'; g_uperLineLcd[5]='n';
    g_uperLineLcd[6]='o'; g_uperLineLcd[7]='r'; g_uperLineLcd[8]='m';
    g_uperLineLcd[9]='a'; g_uperLineLcd[10]='l'; g_uperLineLcd[11]='l';
    g_uperLineLcd[12]='y'; g_uperLineLcd[13]=']'; g_uperLineLcd[14]='=';
    g_uperLineLcd[15]='=';
    g_lowerLineLcd[0]='D'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='o';
    g_lowerLineLcd[3]='r'; g_lowerLineLcd[4]=':'; g_lowerLineLcd[5]='i';
    g_lowerLineLcd[6]='s'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='L';
    g_lowerLineLcd[9]='o'; g_lowerLineLcd[10]='c'; g_lowerLineLcd[11]='k';
    g_lowerLineLcd[12]='k'; g_lowerLineLcd[13]='i'; g_lowerLineLcd[14]='n';
    g_lowerLineLcd[15]='g';  
    printLcdL();
    printLcdU();
    USART_TransmitTime();
    int counter = 0;
    char tempText[] = "Pilot LOCKED cockpit door  .\n";
    for(counter = 0; counter < 29; counter ++){
      USART_Transmit(tempText[counter]);
    }
  }
}
//End define interrupt function

//==========================================MAIN FUNCTION =====================================
int main(){
  L_RESET:  initializationSystem();
  initState0();
  while(1){
    operationSystem(0);
    operationLcd(0);
    if((PIND&(1<<4))==0){
      testLight();
      while((PIND&(1<<4))==0){
        stateTest();
      }
      if(g_stateCodeLcd == 2){
        initStateLcd2();
      }
    }
  }
}
//------------------------------------------END MAIN FUNCTION----------------------------------

//Define core function system
void operationSystem(unsigned char key){
  key = scanKeyOutside();
  signed char counter = 0;
  if(g_operationCode == 0){           //normal working with out any signal 
    if(key >= 0x30 && key <= 0x39){   //press 0->9
      g_pinInput[g_poiterPinOutside] = key - 0x30;
      g_poiterPinOutside ++;
      if(g_poiterPinOutside == 5) g_poiterPinOutside = 0;
    } else if(key == 0x2a){    //press * reset pin inout
      g_poiterPinOutside = 0;
    } else if(key == 0x23){
      if(g_stateCodeLcd != 4){
        if(g_poiterPinOutside == 4){
          g_poiterPinOutside = 0;
          if(checkCode()){
            g_operationCode = 4;
            initState4();
          }else{
            //wrong pin
          }
        }else{
          g_operationCode = 1;
          initState1();   //init mode normal request   
          g_remainState = TIME_REMAIN_STATE_1;
        }
      }
    }
  }else if(g_operationCode == 1){ //normal request access cockpit
    if(g_remainState == 0){
      g_operationCode = 0;
      initState0();
    }else{
      if(g_timeSys.mili < 50){
        buzzer(0);
      }else{
        buzzer(1);
      }
    }
  }else if(g_operationCode == 2){ //Allow access cockpit
    if(g_remainState == 0){
      g_operationCode = 0;
      initState0();
      g_priorityState = 0;
    }
    if(g_priorityState == 0 && g_stateCodeLcd == 0){
      g_stateCodeLcd = 2;
      initStateLcd2();
    }
  }else if(g_operationCode == 3){ //Denied access cockpit
    if(g_remainState == 0){
      g_operationCode = 0;
      initState0();
      g_priorityState = 0;
    }
    if(g_priorityState == 0 && g_stateCodeLcd == 0){
      g_stateCodeLcd = 2;
      initStateLcd2();
    }
  }else if(g_operationCode == 4){
    if(g_remainState == 0){
      g_priorityState = 0;
      g_operationCode = 5;
      g_remainState = TIME_REMAIN_STATE_5;
      initState5();
    }else{
      if((g_timeSys.mili/10)%2){
        buzzer(0);
        ledOpen(1);
        ledAllow(1);
      }else{
        buzzer(1);
        ledAllow(0);
        ledOpen(0);
      }
      if(g_changeSec){
        writeCommandLCD(0x80+12);
        writeDataLCD(0x30+remainTimeTen());
        writeCommandLCD(0x80+13);
        writeDataLCD(0x30+remainTimeUnit());
        g_changeSec = 0;
      }
    }
  }else if(g_operationCode == 5){
    if(g_remainState == 0){
      g_priorityState = 0;
      g_operationCode = 0;
      initState0();
      initStateLcd2();
      g_changeStateLcd = 1;
    }else{
      if((g_timeSys.mili/20)%2){
        //turnOffLcd();
      }else{
        //turnOnLcd();
      }
      if(g_changeSec){
        writeCommandLCD(0xC0+7);
        writeDataLCD(0x30+remainTimeTen());
        writeCommandLCD(0xC0+8);
        writeDataLCD(0x30+remainTimeUnit());
        g_changeSec = 0;
      }
    }
  }else if(g_operationCode == 6){
    //uncomplete init system
  }
}

void operationLcd(unsigned char key){
  key = scanKeyInside();
  if(key != 0xff){
    g_priorityState = 0;
  }
  if(key == 0xff){
    if(g_changeStateLcd){
      key = 0xfe;
      g_changeStateLcd = 0;
    }
    if(g_changeSec){
      key = 0xfe;
      g_changeSec = 0;
    }
  }
  if(g_priorityState != 1){
    if(key != 0xff){
      g_remainLcd = 10;
      //turnonLcd
      if(g_stateCodeLcd == 0){
        g_uperLineLcd[0]=' '; g_uperLineLcd[1]='I'; g_uperLineLcd[2]='n';
        g_uperLineLcd[3]='i'; g_uperLineLcd[4]='t'; g_uperLineLcd[5]='i';
        g_uperLineLcd[6]='a'; g_uperLineLcd[7]='l'; g_uperLineLcd[8]='a';
        g_uperLineLcd[9]='z'; g_uperLineLcd[10]='a'; g_uperLineLcd[11]='t';
        g_uperLineLcd[12]='i'; g_uperLineLcd[13]='o'; g_uperLineLcd[14]='n';
        g_uperLineLcd[15]=' ';
        g_lowerLineLcd[0]='1'; g_lowerLineLcd[1]='.'; g_lowerLineLcd[2]='I';
        g_lowerLineLcd[3]='n'; g_lowerLineLcd[4]='i'; g_lowerLineLcd[5]='t';
        g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]=' ';
        g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]='2';
        g_lowerLineLcd[12]='.'; g_lowerLineLcd[13]='O'; g_lowerLineLcd[14]='f';
        g_lowerLineLcd[15]='f';
        printLcdU();
        printLcdL();
        if(key == 0x31){
          g_stateCodeLcd = 1;
          key = 0xff;
          initStateLcd1();
        }else if(key == 0x32){
          writeCommandLCD(0x08);
        }
      }else if(g_stateCodeLcd == 1){
        unsigned char i = 0;
        if(key >= 0x30 && key <= 0x39){
          if(g_poiterPinInside != 4){
            g_lowerLineLcd[12+g_poiterPinInside] = key;
          }
          g_pinInputInside[g_poiterPinInside] = key - 0x30;
          g_poiterPinInside ++;
          if(g_poiterPinInside == 5){
            g_poiterPinInside = 0;
          }
        }else if(key == 0x23){
          if(g_poiterPinInside == 4){
            g_pinCorrect[0] = g_pinInputInside[0];
            g_pinCorrect[1] = g_pinInputInside[1];
            g_pinCorrect[2] = g_pinInputInside[2];
            g_pinCorrect[3] = g_pinInputInside[3];
            g_uperLineLcd[0]='A'; g_uperLineLcd[1]='c'; g_uperLineLcd[2]='t';
            g_uperLineLcd[3]='i'; g_uperLineLcd[4]='v'; g_uperLineLcd[5]='e';
            g_uperLineLcd[6]='C'; g_uperLineLcd[7]='o'; g_uperLineLcd[8]='d';
            g_uperLineLcd[9]='e'; g_uperLineLcd[10]=':'; g_uperLineLcd[11]=' ';
            g_uperLineLcd[12]=g_pinCorrect[0] + 0x30; g_uperLineLcd[13]=g_pinCorrect[1]+0x30;
            g_uperLineLcd[14]=g_pinCorrect[2] + 0x30; g_uperLineLcd[15]=g_pinCorrect[3]+0x30;
            g_lowerLineLcd[0]='E'; g_lowerLineLcd[1]='n'; g_lowerLineLcd[2]='t';
            g_lowerLineLcd[3]='e'; g_lowerLineLcd[4]='r'; g_lowerLineLcd[5]='C';
            g_lowerLineLcd[6]='o'; g_lowerLineLcd[7]='d'; g_lowerLineLcd[8]='e';
            g_lowerLineLcd[9]=':'; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]=' ';
            g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]=' ';
            g_lowerLineLcd[15]=' ';
            g_poiterPinInside = 0;
          }
        }
        printLcdU();
        printLcdL();
        if(key == 0x2a){
          g_stateCodeLcd = 2;
          g_changeStateLcd = 1;
          key = 0xff;
          initStateLcd2();
        }
      }else if(g_stateCodeLcd == 2){
        g_uperLineLcd[5]=(g_timeSys.hour /10) + 0x30;
        g_uperLineLcd[6]=(g_timeSys.hour%10) + 0x30; g_uperLineLcd[8]=(g_timeSys.minute/10)+0x30;
        g_uperLineLcd[9]=(g_timeSys.minute%10) + 0x30; g_uperLineLcd[11]=(g_timeSys.sec/10)+0x30;
        g_uperLineLcd[12]=(g_timeSys.sec%10) + 0x30;
        if(checkError()){
          g_uperLineLcd[15] = 0xff;
        }
        if(stateMenuLcd == 0){
          g_lowerLineLcd[0]='1'; g_lowerLineLcd[1]='.'; g_lowerLineLcd[2]='E';
          g_lowerLineLcd[3]='d'; g_lowerLineLcd[4]='i'; g_lowerLineLcd[5]='t';
          g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='2';
          g_lowerLineLcd[9]='.'; g_lowerLineLcd[10]='S'; g_lowerLineLcd[11]='t';
          g_lowerLineLcd[12]='r'; g_lowerLineLcd[13]='i'; g_lowerLineLcd[14]='c';
          g_lowerLineLcd[15]='t';
        }else if(stateMenuLcd == 1){
          g_lowerLineLcd[0]=' '; g_lowerLineLcd[1]='3'; g_lowerLineLcd[2]='.';
          g_lowerLineLcd[3]='T'; g_lowerLineLcd[4]='e'; g_lowerLineLcd[5]='s';
          g_lowerLineLcd[6]='t'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='K';
          g_lowerLineLcd[9]='e'; g_lowerLineLcd[10]='y'; g_lowerLineLcd[11]='P';
          g_lowerLineLcd[12]='a'; g_lowerLineLcd[13]='d'; g_lowerLineLcd[14]=' ';
          g_lowerLineLcd[15]=' ';
        }else if(stateMenuLcd == 2){
          g_lowerLineLcd[0]=' '; g_lowerLineLcd[1]='4'; g_lowerLineLcd[2]='.';
          g_lowerLineLcd[3]='T'; g_lowerLineLcd[4]='e'; g_lowerLineLcd[5]='s';
          g_lowerLineLcd[6]='t'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='D';
          g_lowerLineLcd[9]='i'; g_lowerLineLcd[10]='s'; g_lowerLineLcd[11]='p';
          g_lowerLineLcd[12]='l'; g_lowerLineLcd[13]='a'; g_lowerLineLcd[14]='y';
          g_lowerLineLcd[15]=' ';
        }else if(stateMenuLcd == 3){
          g_lowerLineLcd[0]=' '; g_lowerLineLcd[1]='5'; g_lowerLineLcd[2]='.';
          g_lowerLineLcd[3]='I'; g_lowerLineLcd[4]='n'; g_lowerLineLcd[5]='f';
          g_lowerLineLcd[6]='o'; g_lowerLineLcd[7]='r'; g_lowerLineLcd[8]='m';
          g_lowerLineLcd[9]='a'; g_lowerLineLcd[10]='t'; g_lowerLineLcd[11]='i';
          g_lowerLineLcd[12]='o'; g_lowerLineLcd[13]='n'; g_lowerLineLcd[14]=' ';
          g_lowerLineLcd[15]=' ';
        }else if(stateMenuLcd == 4){
          g_lowerLineLcd[0]='6'; g_lowerLineLcd[1]='.'; g_lowerLineLcd[2]=' ';
          g_lowerLineLcd[3]='C'; g_lowerLineLcd[4]='r'; g_lowerLineLcd[5]='e';
          g_lowerLineLcd[6]='a'; g_lowerLineLcd[7]='t'; g_lowerLineLcd[8]='e';
          g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]='R'; g_lowerLineLcd[11]='e';
          g_lowerLineLcd[12]='p'; g_lowerLineLcd[13]='o'; g_lowerLineLcd[14]='r';
          g_lowerLineLcd[15]='t';
        }else if(stateMenuLcd == 5){
          g_lowerLineLcd[0]=' '; g_lowerLineLcd[1]='7'; g_lowerLineLcd[2]='.';
          g_lowerLineLcd[3]='R'; g_lowerLineLcd[4]='e'; g_lowerLineLcd[5]='s';
          g_lowerLineLcd[6]='e'; g_lowerLineLcd[7]='t'; g_lowerLineLcd[8]=' ';
          g_lowerLineLcd[9]='s'; g_lowerLineLcd[10]='y'; g_lowerLineLcd[11]='s';
          g_lowerLineLcd[12]='t'; g_lowerLineLcd[13]='e'; g_lowerLineLcd[14]='m';
          g_lowerLineLcd[15]=' ';
        }
        printLcdU();
        printLcdL();
        if(key == 0x31){
          g_stateCodeLcd = 1;
          key = 0xff;
          initStateLcd1();
        }else if(key == 0x32){
          g_stateCodeLcd = 3;
          g_changeStateLcd = 1;
          key = 0xff;
        }else if(key == 0x2a){
          stateMenuLcd --;
          if(stateMenuLcd < 0){
            stateMenuLcd = 4;
          }
          g_changeStateLcd = 1;
        }else if(key == 0x23){
          stateMenuLcd ++;
          if(stateMenuLcd > 5){
            stateMenuLcd = 0;
          }
          g_changeStateLcd = 1;
        }else if(key == 0x33){
          g_errorCode[0] = 0;
          g_errorCode[1] = 0;
          g_errorCode[2] = 0;
          g_errorSystem += testKeyPad();
          g_stateCodeLcd = 2;
          initStateLcd2();
        }else if(key == 0x34){
          g_errorCode[3] = 0;
          g_errorSystem += testDisplay();
          g_stateCodeLcd = 2;
          initStateLcd2();
        }else if(key == 0x35){
          g_stateCodeLcd = 5;
          stateMenuLcd = 0;
          g_changeStateLcd = 1;
          key = 0xff;
        }else if(key == 0x36){
          g_stateCodeLcd = 6;
          stateMenuLcd = 0;
          g_changeStateLcd = 1;
          g_lowerLineLcd[0]='Y'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='u';
          g_lowerLineLcd[3]='r'; g_lowerLineLcd[4]='C'; g_lowerLineLcd[5]='o';
          g_lowerLineLcd[6]='d'; g_lowerLineLcd[7]='e'; g_lowerLineLcd[8]=':';
          g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]=' ';
          g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]=' ';
          g_lowerLineLcd[15]=' ';
          g_poiterEC = 0;
          key = 0xff;
        }else if(key == 0x37){
          g_stateCodeLcd = 7;
          initStateLcd7();
        }
      }else if(g_stateCodeLcd == 3){        //beforce confirm strict mode
        g_uperLineLcd[0]=' '; g_uperLineLcd[1]=' '; g_uperLineLcd[2]='S';
        g_uperLineLcd[3]='t'; g_uperLineLcd[4]='r'; g_uperLineLcd[5]='i';
        g_uperLineLcd[6]='c'; g_uperLineLcd[7]='t'; g_uperLineLcd[8]=' ';
        g_uperLineLcd[9]='M'; g_uperLineLcd[10]='o'; g_uperLineLcd[11]='d';
        g_uperLineLcd[12]='e'; g_uperLineLcd[13]=' '; g_uperLineLcd[14]=' ';
        g_uperLineLcd[15]=' ';
        g_lowerLineLcd[0]='C'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='n';
        g_lowerLineLcd[3]='f'; g_lowerLineLcd[4]='i'; g_lowerLineLcd[5]='r';
        g_lowerLineLcd[6]='m'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='P';
        g_lowerLineLcd[9]='r'; g_lowerLineLcd[10]='e'; g_lowerLineLcd[11]='s';
        g_lowerLineLcd[12]='s'; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]='#';
        g_lowerLineLcd[15]=' ';
        printLcdU();
        printLcdL();
        if(key == 0x23){
          g_stateCodeLcd = 4;
          g_changeStateLcd = 1;
          key = 0xff;
        }else if(key == 0x2a){
          g_stateCodeLcd = 2;
          key = 0xff;
          initStateLcd2();
        }
      }else if(g_stateCodeLcd == 4){
        g_uperLineLcd[0]=' '; g_uperLineLcd[1]=' '; g_uperLineLcd[2]=' ';
        g_uperLineLcd[3]='S'; g_uperLineLcd[4]='T'; g_uperLineLcd[5]='R';
        g_uperLineLcd[6]='I'; g_uperLineLcd[7]='C'; g_uperLineLcd[8]='T';
        g_uperLineLcd[9]=' '; g_uperLineLcd[10]='M'; g_uperLineLcd[11]='O';
        g_uperLineLcd[12]='D'; g_uperLineLcd[13]='E'; g_uperLineLcd[14]=' ';
        g_uperLineLcd[15]=' ';
        g_lowerLineLcd[0]='='; g_lowerLineLcd[1]='='; g_lowerLineLcd[2]='=';
        g_lowerLineLcd[3]='='; g_lowerLineLcd[4]='='; g_lowerLineLcd[5]='=';
        g_lowerLineLcd[6]='='; g_lowerLineLcd[7]='='; g_lowerLineLcd[8]='=';
        g_lowerLineLcd[9]='='; g_lowerLineLcd[10]='='; g_lowerLineLcd[11]='=';
        g_lowerLineLcd[12]='='; g_lowerLineLcd[13]='='; g_lowerLineLcd[14]='=';
        g_lowerLineLcd[15]='=';
        printLcdU();
        printLcdL();
        if(key == 0x2a){
          g_stateCodeLcd = 2;
          key = 0xff;
          initStateLcd2();
        }
      }else if(g_stateCodeLcd == 5){
        if(key != 0x23 && key != 0x2a){
          if(stateMenuLcd == 0){
            if(checkError()){    //system with error
              g_uperLineLcd[0]='S'; g_uperLineLcd[1]='Y'; g_uperLineLcd[2]='S';
              g_uperLineLcd[3]='T'; g_uperLineLcd[4]='E'; g_uperLineLcd[5]='M';
              g_uperLineLcd[6]=' '; g_uperLineLcd[7]='I'; g_uperLineLcd[8]='N';
              g_uperLineLcd[9]=' '; g_uperLineLcd[10]='E'; g_uperLineLcd[11]='R';
              g_uperLineLcd[12]='R'; g_uperLineLcd[13]='O'; g_uperLineLcd[14]='R';
              g_uperLineLcd[15]=' ';
              g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='r'; g_lowerLineLcd[2]='e';
              g_lowerLineLcd[3]='s'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]=' ';
              g_lowerLineLcd[6]='#'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='t';
              g_lowerLineLcd[9]='o'; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]='v';
              g_lowerLineLcd[12]='i'; g_lowerLineLcd[13]='e'; g_lowerLineLcd[14]='w';
              g_lowerLineLcd[15]=' ';
              printLcdU();
              printLcdL();
              buzzer(1);
              _delay_ms(500);
              buzzer(0);
            }else{        //System normal
              g_uperLineLcd[0]='S'; g_uperLineLcd[1]='Y'; g_uperLineLcd[2]='S';
              g_uperLineLcd[3]='T'; g_uperLineLcd[4]='E'; g_uperLineLcd[5]='M';
              g_uperLineLcd[6]=' '; g_uperLineLcd[7]='I'; g_uperLineLcd[8]='N';
              g_uperLineLcd[9]=' '; g_uperLineLcd[10]='N'; g_uperLineLcd[11]='O';
              g_uperLineLcd[12]='R'; g_uperLineLcd[13]='M'; g_uperLineLcd[14]='A';
              g_uperLineLcd[15]='L';
              g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='r'; g_lowerLineLcd[2]='e';
              g_lowerLineLcd[3]='s'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]=' ';
              g_lowerLineLcd[6]='#'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='t';
              g_lowerLineLcd[9]='o'; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]='v';
              g_lowerLineLcd[12]='i'; g_lowerLineLcd[13]='e'; g_lowerLineLcd[14]='w';
              g_lowerLineLcd[15]=' ';
              printLcdU();
              printLcdL();
            }
          }else if(stateMenuLcd == 1){
            g_uperLineLcd[0]='1'; g_uperLineLcd[1]='.'; g_uperLineLcd[2]='K';
            g_uperLineLcd[3]='e'; g_uperLineLcd[4]='y'; g_uperLineLcd[5]='p';
            g_uperLineLcd[6]='a'; g_uperLineLcd[7]='d'; g_uperLineLcd[8]=' ';
            g_uperLineLcd[9]='I'; g_uperLineLcd[10]='n'; g_uperLineLcd[11]='s';
            g_uperLineLcd[12]='i'; g_uperLineLcd[13]='d'; g_uperLineLcd[14]='e';
            g_uperLineLcd[15]=' ';
            if(g_errorCode[0] == 0){
              setNormalData();
              printLcdU();
              printLcdL();
            }else{
              setErrorData();
              printLcdU();
              printLcdL();
              buzzer(1);
              _delay_ms(500);
              buzzer(0);
            }
          }else if(stateMenuLcd == 2){
            g_uperLineLcd[0]='2'; g_uperLineLcd[1]='.'; g_uperLineLcd[2]='K';
            g_uperLineLcd[3]='e'; g_uperLineLcd[4]='y'; g_uperLineLcd[5]='p';
            g_uperLineLcd[6]='a'; g_uperLineLcd[7]='d'; g_uperLineLcd[8]=' ';
            g_uperLineLcd[9]='O'; g_uperLineLcd[10]='u'; g_uperLineLcd[11]='t';
            g_uperLineLcd[12]='s'; g_uperLineLcd[13]='i'; g_uperLineLcd[14]='d';
            g_uperLineLcd[15]='e';
            if(g_errorCode[1] == 0){
              setNormalData();
              printLcdU();
              printLcdL();
            }else{
              setErrorData();
              printLcdU();
              printLcdL();
              buzzer(1);
              _delay_ms(500);
              buzzer(0);
            }
          }else if(stateMenuLcd == 3){
            g_uperLineLcd[0]='3'; g_uperLineLcd[1]='.'; g_uperLineLcd[2]='S';
            g_uperLineLcd[3]='w'; g_uperLineLcd[4]='i'; g_uperLineLcd[5]='t';
            g_uperLineLcd[6]='c'; g_uperLineLcd[7]='h'; g_uperLineLcd[8]='C';
            g_uperLineLcd[9]='o'; g_uperLineLcd[10]='n'; g_uperLineLcd[11]='t';
            g_uperLineLcd[12]='r'; g_uperLineLcd[13]='o'; g_uperLineLcd[14]='l';
            g_uperLineLcd[15]='l';
            if(g_errorCode[2] == 0){
              setNormalData();
              printLcdU();
              printLcdL();
            }else{
              setErrorData();
              printLcdU();
              printLcdL();
              buzzer(1);
              _delay_ms(500);
              buzzer(0);
            }
          }else if(stateMenuLcd == 4){
            g_uperLineLcd[0]=' '; g_uperLineLcd[1]='4'; g_uperLineLcd[2]='.';
            g_uperLineLcd[3]='D'; g_uperLineLcd[4]='i'; g_uperLineLcd[5]='s';
            g_uperLineLcd[6]='p'; g_uperLineLcd[7]='l'; g_uperLineLcd[8]='a';
            g_uperLineLcd[9]='y'; g_uperLineLcd[10]=' '; g_uperLineLcd[11]='L';
            g_uperLineLcd[12]='c'; g_uperLineLcd[13]='d'; g_uperLineLcd[14]=' ';
            g_uperLineLcd[15]=' ';
            if(g_errorCode[3] == 0){
              setNormalData();
              printLcdU();
              printLcdL();
            }else{
              setErrorData();
              printLcdU();
              printLcdL();
              buzzer(1);
              _delay_ms(500);
              buzzer(0);
            }
          }
        }else{
          if(key == 0x23){
            g_changeStateLcd = 1;
            stateMenuLcd ++;
            if(stateMenuLcd > 4){
              stateMenuLcd = 0;
            }
          }else if(key == 0x2a){
            g_stateCodeLcd = 2;
            initStateLcd2();
          }
        }
      }else if(g_stateCodeLcd == 6){
        if(stateMenuLcd == 0){      //create report enter engineer code
          g_uperLineLcd[0]='1'; g_uperLineLcd[1]='.'; g_uperLineLcd[2]='E';
          g_uperLineLcd[3]='n'; g_uperLineLcd[4]='g'; g_uperLineLcd[5]='i';
          g_uperLineLcd[6]='n'; g_uperLineLcd[7]='e'; g_uperLineLcd[8]='e';
          g_uperLineLcd[9]='r'; g_uperLineLcd[10]=' '; g_uperLineLcd[11]='C';
          g_uperLineLcd[12]='o'; g_uperLineLcd[13]='d'; g_uperLineLcd[14]='e';
          g_uperLineLcd[15]=' ';
          if(key >= 0x30 && key <= 0x39){
            if(g_poiterEC != 7){
              g_lowerLineLcd[9+g_poiterEC] = key;
            }
            g_engineerCode[g_poiterEC] = key;
            g_poiterEC ++;
            if(g_poiterEC == 7){
              g_poiterEC = 0;
            }
            g_changeStateLcd = 1;
            key = 0xff;
          }else if(key == 0x23){
            stateMenuLcd = 1;
            g_lowerLineLcd[0]='D'; g_lowerLineLcd[1]='a'; g_lowerLineLcd[2]='t';
            g_lowerLineLcd[3]='e'; g_lowerLineLcd[4]=':'; g_lowerLineLcd[5]=' ';
            g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='-'; g_lowerLineLcd[8]=' ';
            g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]='-'; g_lowerLineLcd[11]=' ';
            g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]=' ';
            g_lowerLineLcd[15]=' ';
            g_changeStateLcd = 1;
            g_poiterEC = 0;
          }
          printLcdU();
          printLcdL();
          if(key == 0x2a){
            g_stateCodeLcd = 2;
            key = 0xff;
            initStateLcd2();
          }
        }else if(stateMenuLcd == 1){   //Enter date-time
          g_uperLineLcd[0]=' '; g_uperLineLcd[1]=' '; g_uperLineLcd[2]='2';
          g_uperLineLcd[3]='.'; g_uperLineLcd[4]=' '; g_uperLineLcd[5]='D';
          g_uperLineLcd[6]='a'; g_uperLineLcd[7]='t'; g_uperLineLcd[8]='e';
          g_uperLineLcd[9]=' '; g_uperLineLcd[10]='T'; g_uperLineLcd[11]='i';
          g_uperLineLcd[12]='m'; g_uperLineLcd[13]='e'; g_uperLineLcd[14]=' ';
          g_uperLineLcd[15]=' ';
          if(key >= 0x30 && key <= 0x39){
            if(g_poiterEC != 11){
              g_lowerLineLcd[5+g_poiterEC] = key;
            }
            g_dateTime[g_poiterEC] = key;
            g_poiterEC ++;
            if(g_poiterEC == 2){
              g_poiterEC ++;
            }else if(g_poiterEC == 5){
              g_poiterEC ++;
            }
            g_changeStateLcd = 1;
            key = 0xff;
          }else if(key == 0x23){
            stateMenuLcd = 2;
            g_changeStateLcd = 1;
          }
          printLcdU();
          printLcdL();
          if(key == 0x2a){
            g_stateCodeLcd = 2;
            key = 0xff;
            initStateLcd2();
          }
        }else if(stateMenuLcd == 2){ //Setup complete
          g_uperLineLcd[0]=' '; g_uperLineLcd[1]='S'; g_uperLineLcd[2]='e';
          g_uperLineLcd[3]='t'; g_uperLineLcd[4]='u'; g_uperLineLcd[5]='p';
          g_uperLineLcd[6]=' '; g_uperLineLcd[7]='C'; g_uperLineLcd[8]='o';
          g_uperLineLcd[9]='m'; g_uperLineLcd[10]='p'; g_uperLineLcd[11]='l';
          g_uperLineLcd[12]='e'; g_uperLineLcd[13]='t'; g_uperLineLcd[14]='e';
          g_uperLineLcd[15]=' ';
          g_lowerLineLcd[0]=' '; g_lowerLineLcd[1]='P'; g_lowerLineLcd[2]='l';
          g_lowerLineLcd[3]='e'; g_lowerLineLcd[4]='a'; g_lowerLineLcd[5]='s';
          g_lowerLineLcd[6]='e'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='P';
          g_lowerLineLcd[9]='r'; g_lowerLineLcd[10]='e'; g_lowerLineLcd[11]='s';
          g_lowerLineLcd[12]='s'; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]='#';
          g_lowerLineLcd[15]=' ';
          printLcdU();
          printLcdL();
          if(key == 0x2a){
            g_stateCodeLcd = 2;
            key = 0xff;
            initStateLcd2();
          }else if(key == 0x23){
            stateMenuLcd = 3;
            g_changeStateLcd = 1;
          }
        }else if(stateMenuLcd == 3){
          unsigned char i = 0;
          unsigned char string[20];
          string[0]='='; string[1]='='; string[2]='=';
          string[3]='='; string[4]='='; string[5]='=';
          string[6]='='; string[7]='='; string[8]='=';
          string[9]='='; string[10]='='; string[11]='=';
          string[12]='='; string[13]='='; string[14]='=';
          string[15]='='; string[16]='='; string[17]='=';
          string[18]='='; string[19]='=';
          g_uperLineLcd[0]='C'; g_uperLineLcd[1]='r'; g_uperLineLcd[2]='e';
          g_uperLineLcd[3]='a'; g_uperLineLcd[4]='t'; g_uperLineLcd[5]='i';
          g_uperLineLcd[6]='n'; g_uperLineLcd[7]='g'; g_uperLineLcd[8]=' ';
          g_uperLineLcd[9]='R'; g_uperLineLcd[10]='e'; g_uperLineLcd[11]='p';
          g_uperLineLcd[12]='o'; g_uperLineLcd[13]='r'; g_uperLineLcd[14]='t';
          g_uperLineLcd[15]=' ';   //Creating report
          g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='l'; g_lowerLineLcd[2]='e';
          g_lowerLineLcd[3]='a'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]='e';
          g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='W'; g_lowerLineLcd[8]='a';
          g_lowerLineLcd[9]='i'; g_lowerLineLcd[10]='t'; g_lowerLineLcd[11]='t';
          g_lowerLineLcd[12]='i'; g_lowerLineLcd[13]='n'; g_lowerLineLcd[14]='g';
          g_lowerLineLcd[15]=' '; //Please waitting
          printLcdU();
          printLcdL();
          for(i = 0; i<20; i++){
            USART_Transmit(string[i]);
          }
          USART_Transmit('\n');
          _delay_ms(500);
          if(g_engineerCode[0] == 0x32 && g_engineerCode[5] == 0x37 && g_engineerCode[6] == 0x37){
            string[0]='E'; string[1]='n'; string[2]='g';
            string[3]='i'; string[4]='n'; string[5]='e';
            string[6]='e'; string[7]='r'; string[8]=':';
            string[9]='T'; string[10]='h'; string[11]='a';
            string[12]='n'; string[13]='h'; string[14]=' ';
            string[15]='T'; string[16]='o'; string[17]='a';
            string[18]='n'; string[19]=' ';
            i = 0;
            for(i = 0; i<20; i++){
              USART_Transmit(string[i]);
            }
            USART_Transmit(': ');
            for(i=0; i<7;i++){
              USART_Transmit(g_engineerCode[i]);
            }
            USART_Transmit('\n');
          }else{
            string[0]='E'; string[1]='n'; string[2]='g';
            string[3]='i'; string[4]='n'; string[5]='e';
            string[6]='e'; string[7]='r'; string[8]=':';
            string[9]='U'; string[10]='n'; string[11]='k';
            string[12]='n'; string[13]='o'; string[14]='w';
            string[15]=' '; string[16]='D'; string[17]='a';
            string[18]='t'; string[19]='a';
            i = 0;
            for(i = 0; i<20; i++){
              USART_Transmit(string[i]);
            }
            USART_Transmit('\n');
          }
          _delay_ms(700);
          string[0]='D'; string[1]='a'; string[2]='t';
          string[3]='a'; string[4]=' '; string[5]='t';
          string[6]='i'; string[7]='m'; string[8]='e';
          string[9]=':';
          i = 0;
          for(i = 0; i<10; i++){
              USART_Transmit(string[i]);
          }
          for(i=0; i<10; i++){
            USART_Transmit(g_dateTime[i]);
          }
          USART_Transmit('\n');
          _delay_ms(500);
          string[0]='H'; string[1]='e'; string[2]='a';
          string[3]='l'; string[4]='t'; string[5]='h';
          string[6]=' '; string[7]='o'; string[8]='f';
          string[9]=' '; string[10]='s'; string[11]='y';
          string[12]='s'; string[13]='t'; string[14]='e';
          string[15]='m'; string[16]=' '; string[17]=' ';
          string[18]=' '; string[19]=' ';
          i = 0;
          for(i=0; i<20; i++){
            USART_Transmit(string[i]);
          }
          USART_Transmit('\n');
          string[0]='1'; string[1]='.'; string[2]='S';
          string[3]='y'; string[4]='s'; string[5]='t';
          string[6]='e'; string[7]='m'; string[8]=' ';
          string[9]='k'; string[10]='e'; string[11]='y';
          string[12]='p'; string[13]='a'; string[14]='d';
          string[15]=' '; string[16]='i'; string[17]='n';
          string[18]='s'; string[19]='i';
          i = 0;
          for(i=0; i<20; i++){
            USART_Transmit(string[i]);
          }
          USART_Transmit('d');
          USART_Transmit('e');
          USART_Transmit(':');
          if(g_errorCode[0] == 0){
            string[0]='G'; string[1]='O'; string[2]='O';
            string[3]='D'; string[4]=' '; string[5]='H';
            string[6]='e'; string[7]='a'; string[8]='l';
            string[9]='t'; string[10]='h';
            i = 0;
            for(i=0; i<11; i++){
              USART_Transmit(string[i]);
            }
          }else{
            string[0]='E'; string[1]='R'; string[2]='R';
            string[3]='O'; string[4]='R'; string[5]=' ';
            string[6]='M'; string[7]='o'; string[8]='d';
            string[9]='u'; string[10]='l'; string[11]='e';
            string[12]=' '; string[13]='f'; string[14]='a';
            string[15]='i'; string[16]='l'; string[17]='u';
            string[18]='r'; string[19]='e';
            i = 0;
            for(i=0; i<20; i++){
              USART_Transmit(string[i]);
            }
          }
          USART_Transmit('\n');
          string[0]='2'; string[1]='.'; string[2]='S';
          string[3]='y'; string[4]='s'; string[5]='t';
          string[6]='e'; string[7]='m'; string[8]=' ';
          string[9]='k'; string[10]='e'; string[11]='y';
          string[12]='p'; string[13]='a'; string[14]='d';
          string[15]=' '; string[16]='o'; string[17]='u';
          string[18]='t'; string[19]='s';
          i = 0;
          for(i=0; i<20; i++){
              USART_Transmit(string[i]);
          }
          USART_Transmit('i');
          USART_Transmit('d');
          USART_Transmit('e');
          USART_Transmit(':');
          if(g_errorCode[1] == 0){
            string[0]='G'; string[1]='O'; string[2]='O';
            string[3]='D'; string[4]=' '; string[5]='H';
            string[6]='e'; string[7]='a'; string[8]='l';
            string[9]='t'; string[10]='h';
            i = 0;
            for(i=0; i<11; i++){
              USART_Transmit(string[i]);
            }
          }else{
            string[0]='E'; string[1]='R'; string[2]='R';
            string[3]='O'; string[4]='R'; string[5]=' ';
            string[6]='M'; string[7]='o'; string[8]='d';
            string[9]='u'; string[10]='l'; string[11]='e';
            string[12]=' '; string[13]='f'; string[14]='a';
            string[15]='i'; string[16]='l'; string[17]='u';
            string[18]='r'; string[19]='e';
            i = 0;
            for(i=0; i<20; i++){
              USART_Transmit(string[i]);
            }
          }
          USART_Transmit('\n');
          string[0]='3'; string[1]='.'; string[2]='S';
          string[3]='y'; string[4]='s'; string[5]='t';
          string[6]='e'; string[7]='m'; string[8]=' ';
          string[9]='S'; string[10]='w'; string[11]='i';
          string[12]='t'; string[13]='c'; string[14]='h';
          string[15]=':'; string[16]=' ';
          i = 0;
          for(i=0; i<17; i++){
              USART_Transmit(string[i]);
          }
          if(g_errorCode[2] == 0){
            string[0]='G'; string[1]='O'; string[2]='O';
            string[3]='D'; string[4]=' '; string[5]='H';
            string[6]='e'; string[7]='a'; string[8]='l';
            string[9]='t'; string[10]='h';
            i = 0;
            for(i=0; i<11; i++){
              USART_Transmit(string[i]);
            }
          }else{
            string[0]='E'; string[1]='R'; string[2]='R';
            string[3]='O'; string[4]='R'; string[5]=' ';
            string[6]='M'; string[7]='o'; string[8]='d';
            string[9]='u'; string[10]='l'; string[11]='e';
            string[12]=' '; string[13]='f'; string[14]='a';
            string[15]='i'; string[16]='l'; string[17]='u';
            string[18]='r'; string[19]='e';
            i = 0;
            for(i=0; i<20; i++){
              USART_Transmit(string[i]);
            }
          }
          USART_Transmit('\n');
          string[0]='4'; string[1]='.'; string[2]='S';
          string[3]='y'; string[4]='s'; string[5]='t';
          string[6]='e'; string[7]='m'; string[8]=' ';
          string[9]='L'; string[10]='c'; string[11]='d';
          string[12]=' '; string[13]='D'; string[14]='i';
          string[15]='s'; string[16]='p'; string[17]='l';
          string[18]='a'; string[19]='y';
          for(i=0; i<20; i++){
              USART_Transmit(string[i]);
          }
          if(g_errorCode[3] == 0){
            string[0]='G'; string[1]='O'; string[2]='O';
            string[3]='D'; string[4]=' '; string[5]='H';
            string[6]='e'; string[7]='a'; string[8]='l';
            string[9]='t'; string[10]='h';
            i = 0;
            for(i=0; i<11; i++){
              USART_Transmit(string[i]);
            }
          }else{
            string[0]='E'; string[1]='R'; string[2]='R';
            string[3]='O'; string[4]='R'; string[5]=' ';
            string[6]='M'; string[7]='o'; string[8]='d';
            string[9]='u'; string[10]='l'; string[11]='e';
            string[12]=' '; string[13]='f'; string[14]='a';
            string[15]='i'; string[16]='l'; string[17]='u';
            string[18]='r'; string[19]='e';
            i = 0;
            for(i=0; i<20; i++){
              USART_Transmit(string[i]);
            }
          }
          USART_Transmit('\n');
          _delay_ms(1200);
          g_uperLineLcd[0]='R'; g_uperLineLcd[1]='e'; g_uperLineLcd[2]='p';
          g_uperLineLcd[3]='o'; g_uperLineLcd[4]='r'; g_uperLineLcd[5]='t';
          g_uperLineLcd[6]=' '; g_uperLineLcd[7]='C'; g_uperLineLcd[8]='o';
          g_uperLineLcd[9]='m'; g_uperLineLcd[10]='p'; g_uperLineLcd[11]='l';
          g_uperLineLcd[12]='e'; g_uperLineLcd[13]='t'; g_uperLineLcd[14]='e';
          g_uperLineLcd[15]='d';
          g_lowerLineLcd[0]='R'; g_lowerLineLcd[1]='e'; g_lowerLineLcd[2]='t';
          g_lowerLineLcd[3]='u'; g_lowerLineLcd[4]='r'; g_lowerLineLcd[5]='n';
          g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='i'; g_lowerLineLcd[8]='n';
          g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]='f'; g_lowerLineLcd[11]='e';
          g_lowerLineLcd[12]='w'; g_lowerLineLcd[13]='S'; g_lowerLineLcd[14]='e';
          g_lowerLineLcd[15]='c';
          printLcdU();
          printLcdL();
          soundPushButton();
          _delay_ms(2000);
          g_stateCodeLcd = 2;
          key = 0xff;
          initStateLcd2();
        }
      }else if(g_stateCodeLcd == 7){
        if(key == 0x23){
          g_uperLineLcd[0]='R'; g_uperLineLcd[1]='e'; g_uperLineLcd[2]='s';
          g_uperLineLcd[3]='e'; g_uperLineLcd[4]='t'; g_uperLineLcd[5]='t';
          g_uperLineLcd[6]='i'; g_uperLineLcd[7]='n'; g_uperLineLcd[8]='g';
          g_uperLineLcd[9]=' '; g_uperLineLcd[10]='S'; g_uperLineLcd[11]='y';
          g_uperLineLcd[12]='s'; g_uperLineLcd[13]='t'; g_uperLineLcd[14]='e';
          g_uperLineLcd[15]='m';
          g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='l'; g_lowerLineLcd[2]='e';
          g_lowerLineLcd[3]='a'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]='e';
          g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='W'; g_lowerLineLcd[8]='a';
          g_lowerLineLcd[9]='i'; g_lowerLineLcd[10]='t'; g_lowerLineLcd[11]=' ';
          g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]=' ';
          g_lowerLineLcd[15]=' ';
          printLcdL();
          printLcdU();
          unsigned char tempCounter = 0;
          USART_TransmitTime();
          writeCommandLCD(0xC0 + 11);
          writeDataLCD('.');
          _delay_ms(300);
          char tempText2[] = "[Causion] System has been reseted.\n";
          writeDataLCD('.');
          _delay_ms(100);
          writeDataLCD('.');
          for(tempCounter = 0; tempCounter < 35; tempCounter ++){
            USART_Transmit(tempText2[tempCounter]);
          }
          writeDataLCD('.');
          soundPushButton();
          _delay_ms(200);
          MCUSR = MCUSR & 0b11111101; 
        }else if(key == 0x2a){
          g_stateCodeLcd = 2;
          initStateLcd2();
        }
      }
    }
  }
}

//End define core function

//define software function
bool checkCode(){
  signed char counter = 0;
  for(counter = 0; counter < 4; counter ++){
    if(g_pinInput[counter] != g_pinCorrect[counter]) return false;
  }
  return true;
}
char remainTimeTen(){
  return g_remainState/10;
}
char remainTimeUnit(){
  return g_remainState%10;
}
void printLcdU(){
  unsigned char i = 0;
  writeCommandLCD(0x80);
  for(i = 0; i<16 ; i++){
    writeDataLCD(g_uperLineLcd[i]);
  }
}
void printLcdL(){
  unsigned char i = 0;
  writeCommandLCD(0xc0);
  for(i = 0; i<16 ; i++){
    writeDataLCD(g_lowerLineLcd[i]);
  }
}
void setErrorData(){
  g_lowerLineLcd[0]='M'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='d';
  g_lowerLineLcd[3]='u'; g_lowerLineLcd[4]='l'; g_lowerLineLcd[5]='e';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='i'; g_lowerLineLcd[8]='s';
  g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]='E'; g_lowerLineLcd[11]='R';
  g_lowerLineLcd[12]='R'; g_lowerLineLcd[13]='O'; g_lowerLineLcd[14]='R';
  g_lowerLineLcd[15]=' ';
}
void setNormalData(){
  g_lowerLineLcd[0]='M'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='d';
  g_lowerLineLcd[3]='u'; g_lowerLineLcd[4]='l'; g_lowerLineLcd[5]='e';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='i'; g_lowerLineLcd[8]='s';
  g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]='n'; g_lowerLineLcd[11]='o';
  g_lowerLineLcd[12]='r'; g_lowerLineLcd[13]='m'; g_lowerLineLcd[14]='a';
  g_lowerLineLcd[15]='l';
}

bool checkError(){
  unsigned char i = 0;
  unsigned error = 0;
  for(i = 0; i < 4; i++){
    error += g_errorCode[i];
  }
  if(error){
    return 1;
  }else{
    return 0;
  }
}
void USART_TransmitTime(){
  unsigned char counter = 0;
  char tempText[] = "<>Time: 00h00m00s - ";
  tempText[8] = g_timeSys.hour/10 + 0x30;
  tempText[9] = g_timeSys.hour%10 + 0x30;
  tempText[11] = g_timeSys.minute/10 + 0x30;
  tempText[12] = g_timeSys.minute%10 + 0x30;
  tempText[14] = g_timeSys.sec/10 + 0x30;
  tempText[15] = g_timeSys.sec%10 + 0x30;
  for(counter = 0; counter < 20; counter ++){
    USART_Transmit(tempText[counter]);
  }
}
//end define software function

//define function hardware interface
unsigned char scanKeyOutside(){                                 //scankey outside
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xfe);
  _delay_us(1);
  if((PINC&(1<<0))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<0))==0);
    soundPushButton();
    return 0x31;
  }else if((PINC&(1<<1))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<1))==0);
    soundPushButton();
    return 0x32;
  }else if((PINC&(1<<2))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<2))==0);
    soundPushButton();
    return 0x33;
  }
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xfd);
  _delay_us(1);
  if((PINC&(1<<0))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<0))==0);
    soundPushButton();
    return 0x34;
  }else if((PINC&(1<<1))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<1))==0);
    soundPushButton();
    return 0x35;
  }else if((PINC&(1<<2))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<2))==0);
    soundPushButton();
    return 0x36;
  }
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xfb);
  _delay_us(1);
  if((PINC&(1<<0))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<0))==0);
    soundPushButton();
    return 0x37;
  }else if((PINC&(1<<1))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<1))==0);
    soundPushButton();
    return 0x38;
  }else if((PINC&(1<<2))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<2))==0);
    soundPushButton();
    return 0x39;
  }
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xf7);
  _delay_us(1);
  if((PINC&(1<<0))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<0))==0);
    soundPushButton();
    return 0x2a;
  }else if((PINC&(1<<1))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<1))==0);
    soundPushButton();
    return 0x30;
  }else if((PINC&(1<<2))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<2))==0);
    soundPushButton();
    return 0x23;
  }
  return 0xff;
}

unsigned char scanKeyInside(){                            //ScanKey inside
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xfe);
  _delay_us(1);
  if((PINC&(1<<5))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<5))==0);
    soundPushButton();
    return 0x31;
  }else if((PINC&(1<<4))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<4))==0);
    soundPushButton();
    return 0x32;
  }else if((PINC&(1<<3))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<3))==0);
    soundPushButton();
    return 0x33;
  }
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xfd);
  _delay_us(1);
  if((PINC&(1<<5))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<5))==0);
    soundPushButton();
    return 0x34;
  }else if((PINC&(1<<4))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<4))==0);
    soundPushButton();
    return 0x35;
  }else if((PINC&(1<<3))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<3))==0);
    soundPushButton();
    return 0x36;
  }
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xfb);
  _delay_us(1);
  if((PINC&(1<<5))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<5))==0);
    soundPushButton();
    return 0x37;
  }else if((PINC&(1<<4))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<4))==0);
    soundPushButton();
    return 0x38;
  }else if((PINC&(1<<3))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<3))==0);
    soundPushButton();
    return 0x39;
  }
  g_vPort |= 0x0f;
  setVPort(g_vPort & 0xf7);
  _delay_us(1);
  if((PINC&(1<<5))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<5))==0);
    soundPushButton();
    return 0x2a;
  }else if((PINC&(1<<4))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<4))==0);
    soundPushButton();
    return 0x30;
  }else if((PINC&(1<<3))==0){
    _delay_ms(TIME_DEBOUNCE);
    while((PINC&(1<<3))==0);
    soundPushButton();
    return 0x23;
  }
  return 0xff;
}

void setVPort(unsigned char input){
  g_vPort = input;
  char i = 0;
  setBitPB(2,0);
  setBitPB(0,0);
  for(i=7;i>=0;i--){
    setBitPB(0,0);
    if((input >> i)&0x01){
        setBitPB(1,1);
    }else{
        setBitPB(1,0);
    }
      setBitPB(0,1);
  }
  setBitPB(2,1);
}

void setLCDPort(unsigned char input){
    char i = 0;
    setBitPB(3,0);
    setBitPB(0,0);
    for(i=7;i>=0;i--){
      setBitPB(0,0);
      if((input >> i)&0x01){
          setBitPB(1,1);
      }else{
          setBitPB(1,0);
      }
      setBitPB(0,1);
    }
    setBitPB(3,1);
}

void writeDataLCD(unsigned char data){
    setBitPD(5,1);
    setBitPD(6,0);
    setLCDPort(data);
    setBitPD(6,1);
    _delay_ms(2);
    setBitPD(6,0);
}

void writeCommandLCD(unsigned char command){
    setBitPD(5,0);
    setBitPD(6,0);
    setLCDPort(command);
    setBitPD(6,1);
    _delay_ms(2);
    setBitPD(6,0);
}

void setBitPB(unsigned char bit, bool logic){
  if(logic){
    PORTB |= 0x01 << bit;
  }else{
    PORTB &= ~(0x01<<bit);
  }
}

void setBitPD(unsigned char bit, bool logic){
  if(logic){
    PORTD |= 0x01 << bit;
  }else{
    PORTD &= ~(0x01<<bit);
  }
}

void ledOpen(bool input){
  if(input){
    setVPort(g_vPort|(1<<6));
  }else{
    setVPort(g_vPort&(~(1<<6)));
  }
}
void ledFail(bool input){
  if(input){
    setVPort(g_vPort|(1<<7));
  }else{
    setVPort(g_vPort&(~(1<<7)));
  }
}
void ledAllow(bool input){
  if(input){
    setVPort(g_vPort|(1<<4));
  }else{
    setVPort(g_vPort&(~(1<<4)));
  }
}
void ledDenied(bool input){
  if(input){
    setVPort(g_vPort|(1<<5));
  }else{
    setVPort(g_vPort&(~(1<<5)));
  }
}
void ledLock(bool input){
  if(input){
    PORTB = PORTB|(1<<5);
  }else{
    PORTB = PORTB&(~(1<<5));
  }
}
void ledUnlock(bool input){
  if(input){
    PORTB = PORTB|(1<<4);
  }else{
    PORTB = PORTB&(~(1<<4));
  }
}
void buzzer(bool input){
  if(input){
    setBitPD(7,1);
  }else{
    setBitPD(7,0);
  }
}

void soundPushButton(){
  setBitPD(7,1);
  _delay_ms(50);
  setBitPD(7,0);
}
void USART_Transmit( unsigned char data )
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<5)) );
  /* Put data into buffer, sends the data */
  UDR0 = data;
  while ( !( UCSR0A & (1<<6)) );
}
//end define function hardware interface

//function init state
void initState0(){
  g_changeState = 1;
  ledLock(1);
  ledUnlock(0);
  ledDenied(0);
  ledOpen(0);
  ledFail(0);
  ledAllow(0);
  buzzer(0);
} 

void initState1(){
  g_changeState = 1;
  ledLock(1);
  ledUnlock(0);
  ledDenied(0);
  ledOpen(0);
  ledFail(0);
  ledAllow(0);
  buzzer(0);
  g_remainState = TIME_REMAIN_STATE_1;
}
void initState2(){
  g_changeState = 1;
  ledLock(0);
  ledUnlock(1);
  ledDenied(0);
  ledOpen(1);
  ledFail(0);
  ledAllow(1);
  buzzer(0);
  g_remainState = TIME_REMAIN_STATE_2;
}
void initState3(){
  g_changeState = 1;
  ledLock(1);
  ledUnlock(0);
  ledDenied(1);
  ledOpen(0);
  ledFail(0);
  ledAllow(0);
  buzzer(0);
  g_remainState = TIME_REMAIN_STATE_3;
}
void initState4(){
  g_changeState = 1;
  g_priorityState = 1;
  ledLock(1);
  ledUnlock(0);
  ledDenied(0);
  ledOpen(0);
  ledFail(0);
  ledAllow(0);
  buzzer(0);
  g_remainState = TIME_REMAIN_STATE_4;
  g_priorityState = 1;
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]='E'; g_uperLineLcd[2]='M';
  g_uperLineLcd[3]='E'; g_uperLineLcd[4]='R'; g_uperLineLcd[5]='G';
  g_uperLineLcd[6]='E'; g_uperLineLcd[7]='N'; g_uperLineLcd[8]='C';
  g_uperLineLcd[9]='Y'; g_uperLineLcd[10]=' '; g_uperLineLcd[11]=' ';
  g_uperLineLcd[12]=' '; g_uperLineLcd[13]=' '; g_uperLineLcd[14]='s';
  g_uperLineLcd[15]=' ';
  printLcdU();
  g_lowerLineLcd[0]='S'; g_lowerLineLcd[1]='w'; g_lowerLineLcd[2]='i';
  g_lowerLineLcd[3]='t'; g_lowerLineLcd[4]='c'; g_lowerLineLcd[5]='h';
  g_lowerLineLcd[6]='T'; g_lowerLineLcd[7]='o'; g_lowerLineLcd[8]='L';
  g_lowerLineLcd[9]='o'; g_lowerLineLcd[10]='c'; g_lowerLineLcd[11]='k';
  g_lowerLineLcd[12]='N'; g_lowerLineLcd[13]='o'; g_lowerLineLcd[14]='w';
  g_lowerLineLcd[15]='!';
  printLcdL();
  USART_TransmitTime();
  int counter = 0;
  char tempText[] = "Emergency pin access has been used.\n";
  for(counter = 0; counter < 36; counter ++){
    USART_Transmit(tempText[counter]);
  }
}
void initState5(){
  g_changeState = 1;
  g_priorityState = 1;
  ledLock(0);
  ledUnlock(1);
  ledDenied(0);
  ledOpen(1);
  ledFail(1);
  ledAllow(1);
  buzzer(0);
  g_remainState = TIME_REMAIN_STATE_5;
  g_priorityState = 1;
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]='S'; g_uperLineLcd[2]='Y';
  g_uperLineLcd[3]='S'; g_uperLineLcd[4]='T'; g_uperLineLcd[5]='E';
  g_uperLineLcd[6]='M'; g_uperLineLcd[7]=' '; g_uperLineLcd[8]='F';
  g_uperLineLcd[9]='A'; g_uperLineLcd[10]='I'; g_uperLineLcd[11]='L';
  g_uperLineLcd[12]='U'; g_uperLineLcd[13]='R'; g_uperLineLcd[14]='E';
  g_uperLineLcd[15]=' ';
  printLcdU();
  g_lowerLineLcd[0]='='; g_lowerLineLcd[1]='='; g_lowerLineLcd[2]='=';
  g_lowerLineLcd[3]='='; g_lowerLineLcd[4]='='; g_lowerLineLcd[5]='=';
  g_lowerLineLcd[6]='='; g_lowerLineLcd[7]='='; g_lowerLineLcd[8]='=';
  g_lowerLineLcd[9]='='; g_lowerLineLcd[10]='='; g_lowerLineLcd[11]='=';
  g_lowerLineLcd[12]='='; g_lowerLineLcd[13]='='; g_lowerLineLcd[14]='=';
  g_lowerLineLcd[15]='=';
  printLcdL();
  int counter = 0;
  char tempText[] = "[Warnning] System has been failed. \n";
  for(counter = 0; counter < 36; counter ++){
    USART_Transmit(tempText[counter]);
  }
}
void initStateLcd1(){
  g_changeStateLcd = 1;
  g_uperLineLcd[0]='A'; g_uperLineLcd[1]='c'; g_uperLineLcd[2]='t';
  g_uperLineLcd[3]='i'; g_uperLineLcd[4]='v'; g_uperLineLcd[5]='e';
  g_uperLineLcd[6]='C'; g_uperLineLcd[7]='o'; g_uperLineLcd[8]='d';
  g_uperLineLcd[9]='e'; g_uperLineLcd[10]=':'; g_uperLineLcd[11]=' ';
  g_uperLineLcd[12]=g_pinCorrect[0] + 0x30; g_uperLineLcd[13]=g_pinCorrect[1]+0x30;
  g_uperLineLcd[14]=g_pinCorrect[2] + 0x30; g_uperLineLcd[15]=g_pinCorrect[3]+0x30;
  printLcdU();
  g_lowerLineLcd[0]='E'; g_lowerLineLcd[1]='n'; g_lowerLineLcd[2]='t';
  g_lowerLineLcd[3]='e'; g_lowerLineLcd[4]='r'; g_lowerLineLcd[5]='C';
  g_lowerLineLcd[6]='o'; g_lowerLineLcd[7]='d'; g_lowerLineLcd[8]='e';
  g_lowerLineLcd[9]=':'; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]=' ';
  g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]=' ';
  g_lowerLineLcd[15]=' ';
  printLcdL();
}
void initStateLcd2(){
  g_changeStateLcd = 1;
  g_uperLineLcd[0]='M'; g_uperLineLcd[1]='e'; g_uperLineLcd[2]='n';
  g_uperLineLcd[3]='u'; g_uperLineLcd[4]=' '; g_uperLineLcd[5]=(g_timeSys.hour /10) + 0x30;
  g_uperLineLcd[6]=(g_timeSys.hour%10) + 0x30; g_uperLineLcd[7]='h'; g_uperLineLcd[8]=(g_timeSys.minute/10)+0x30;
  g_uperLineLcd[9]=(g_timeSys.minute%10) + 0x30; g_uperLineLcd[10]='m'; g_uperLineLcd[11]=(g_timeSys.sec/10)+0x30;
  g_uperLineLcd[12]=(g_timeSys.sec%10) + 0x30; g_uperLineLcd[13]='s'; g_uperLineLcd[14]=' ';
  g_uperLineLcd[15]=' ';
  stateMenuLcd = 0;
  printLcdU();
}
void initStateLcd7(){
  g_changeStateLcd = 1;
  g_uperLineLcd[0]='R'; g_uperLineLcd[1]='e'; g_uperLineLcd[2]='s';
  g_uperLineLcd[3]='e'; g_uperLineLcd[4]='t'; g_uperLineLcd[5]=' ';
  g_uperLineLcd[6]='S'; g_uperLineLcd[7]='y'; g_uperLineLcd[8]='s';
  g_uperLineLcd[9]='t'; g_uperLineLcd[10]='e'; g_uperLineLcd[11]='m';
  g_uperLineLcd[12]=' '; g_uperLineLcd[13]='?'; g_uperLineLcd[14]=' ';
  g_uperLineLcd[15]=' ';
  g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='r'; g_lowerLineLcd[2]='e';
  g_lowerLineLcd[3]='s'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]=' ';
  g_lowerLineLcd[6]='#'; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='c';
  g_lowerLineLcd[9]='o'; g_lowerLineLcd[10]='n'; g_lowerLineLcd[11]='f';
  g_lowerLineLcd[12]='i'; g_lowerLineLcd[13]='r'; g_lowerLineLcd[14]='m';
  g_lowerLineLcd[15]=' ';
  printLcdL();
  printLcdU();
}
//end define function init state
//Define function testting  
unsigned char testKeyPad(){
  testting = 1;
  _delay_ms(500);
  g_uperLineLcd[0]='T'; g_uperLineLcd[1]='e'; g_uperLineLcd[2]='s';
  g_uperLineLcd[3]='t'; g_uperLineLcd[4]=' '; g_uperLineLcd[5]='K';
  g_uperLineLcd[6]='e'; g_uperLineLcd[7]='y'; g_uperLineLcd[8]='P';
  g_uperLineLcd[9]=' '; g_uperLineLcd[10]='I'; g_uperLineLcd[11]='n';
  g_uperLineLcd[12]='s'; g_uperLineLcd[13]='i'; g_uperLineLcd[14]='d';
  g_uperLineLcd[15]='e';
  unsigned char i = 0;
  unsigned char parameter = 0;
  unsigned char result = 0;
  for(i = 0; i < 12; i++){
    g_timeSec = 4;
    g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='r'; g_lowerLineLcd[2]='e';
    g_lowerLineLcd[3]='s'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]=' ';
    g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='.'; g_lowerLineLcd[8]='.';
    g_lowerLineLcd[9]='.'; g_lowerLineLcd[10]='.'; g_lowerLineLcd[11]=' ';
    g_lowerLineLcd[12]='s'; g_lowerLineLcd[13]='e'; g_lowerLineLcd[14]='c';
    g_lowerLineLcd[15]=' ';
    if(i == 10){
      parameter = 0x2a;
      g_lowerLineLcd[6] = 0x2a;
    }else if(i == 11){
      g_lowerLineLcd[6] = 0x23;
      parameter = 0x23;
    }else{
      g_lowerLineLcd[6] = i + 0x30;
      parameter = i + 0x30;
    }
    printLcdU();
    printLcdL();
    while(scanKeyInside() != parameter){
      if(g_changeSec == 1){
        g_changeSec == 0;
        writeCommandLCD(0xcb);
        writeDataLCD(0x30 + g_timeSec);
      }
      if(g_timeSec == 0){
        result ++;
        g_errorCode[0] = 1;
        soundPushButton();
        soundPushButton();
        break;
      }
    }
  }
  g_uperLineLcd[0]='T'; g_uperLineLcd[1]='e'; g_uperLineLcd[2]='s';
  g_uperLineLcd[3]='t'; g_uperLineLcd[4]=' '; g_uperLineLcd[5]='k';
  g_uperLineLcd[6]='e'; g_uperLineLcd[7]='y'; g_uperLineLcd[8]=' ';
  g_uperLineLcd[9]='O'; g_uperLineLcd[10]='u'; g_uperLineLcd[11]='t';
  g_uperLineLcd[12]='s'; g_uperLineLcd[13]='i'; g_uperLineLcd[14]='d';
  g_uperLineLcd[15]='e';
  for(i = 0; i < 12; i++){
    g_timeSec = 4;
    g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='r'; g_lowerLineLcd[2]='e';
    g_lowerLineLcd[3]='s'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]=' ';
    g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='.'; g_lowerLineLcd[8]='.';
    g_lowerLineLcd[9]='.'; g_lowerLineLcd[10]='.'; g_lowerLineLcd[11]=' ';
    g_lowerLineLcd[12]='s'; g_lowerLineLcd[13]='e'; g_lowerLineLcd[14]='c';
    g_lowerLineLcd[15]=' ';
    if(i == 10){
      parameter = 0x2a;
      g_lowerLineLcd[6] = 0x2a;
    }else if(i == 11){
      g_lowerLineLcd[6] = 0x23;
      parameter = 0x23;
    }else{
      g_lowerLineLcd[6] = i + 0x30;
      parameter = i + 0x30;
    }
    printLcdU();
    printLcdL();
    while(scanKeyOutside() != parameter){
      if(g_changeSec == 1){
        g_changeSec == 0;
        writeCommandLCD(0xcb);
        writeDataLCD(0x30 + g_timeSec);
      }
      if(g_timeSec==0){
        result ++;
        g_errorCode[1] = 1;
        soundPushButton();
        soundPushButton();
        break;
      }
    }
  }
  g_checkLock = 0;
  g_checkUnlock = 0;
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]=' '; g_uperLineLcd[2]='T';
  g_uperLineLcd[3]='E'; g_uperLineLcd[4]='S'; g_uperLineLcd[5]='T';
  g_uperLineLcd[6]=' '; g_uperLineLcd[7]='S'; g_uperLineLcd[8]='W';
  g_uperLineLcd[9]='I'; g_uperLineLcd[10]='T'; g_uperLineLcd[11]='C';
  g_uperLineLcd[12]='H'; g_uperLineLcd[13]=' '; g_uperLineLcd[14]=' ';
  g_uperLineLcd[15]=' ';
  g_lowerLineLcd[0]='U'; g_lowerLineLcd[1]='n'; g_lowerLineLcd[2]='l';
  g_lowerLineLcd[3]='o'; g_lowerLineLcd[4]='c'; g_lowerLineLcd[5]='k';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='t'; g_lowerLineLcd[8]='e';
  g_lowerLineLcd[9]='s'; g_lowerLineLcd[10]='t'; g_lowerLineLcd[11]=' ';
  g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]='s'; g_lowerLineLcd[14]='e';
  g_lowerLineLcd[15]='c';
  printLcdU();
  printLcdL();
  g_timeSec = 5;
  while(g_checkUnlock == 0){
    if(g_changeSec == 1){
      g_changeSec = 0;
      writeCommandLCD(0xcb);
      writeDataLCD(0x30 + g_timeSec);
    }
    if(g_timeSec == 0){
      result ++;
      g_errorCode[2] = 1;
      soundPushButton();
      soundPushButton();
      break;
    }
  }
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]=' '; g_uperLineLcd[2]='T';
  g_uperLineLcd[3]='E'; g_uperLineLcd[4]='S'; g_uperLineLcd[5]='T';
  g_uperLineLcd[6]=' '; g_uperLineLcd[7]='S'; g_uperLineLcd[8]='W';
  g_uperLineLcd[9]='I'; g_uperLineLcd[10]='T'; g_uperLineLcd[11]='C';
  g_uperLineLcd[12]='H'; g_uperLineLcd[13]=' '; g_uperLineLcd[14]=' ';
  g_uperLineLcd[15]=' ';
  g_lowerLineLcd[0]='L'; g_lowerLineLcd[1]='o'; g_lowerLineLcd[2]='c';
  g_lowerLineLcd[3]='k'; g_lowerLineLcd[4]=' '; g_lowerLineLcd[5]='T';
  g_lowerLineLcd[6]='e'; g_lowerLineLcd[7]='s'; g_lowerLineLcd[8]='t';
  g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]=' ';
  g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]='s'; g_lowerLineLcd[14]='e';
  g_lowerLineLcd[15]='c';
  printLcdU();
  printLcdL();
  g_timeSec = 5;
  while(g_checkLock == 0){
    if(g_changeSec == 1){
      g_changeSec = 0;
      writeCommandLCD(0xcb);
      writeDataLCD(0x30 + g_timeSec);
    }
    if(g_timeSec == 0){
      result ++;
      g_errorCode[2] = 1;
      soundPushButton();
      soundPushButton();
      break;
    }
  }
  testting = 0;
  g_changeStateLcd = 1;
  return result;
}
//for testting
unsigned char testDisplay(){
  testting = 1;
  unsigned char result = 0;
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]=' '; g_uperLineLcd[2]='T';
  g_uperLineLcd[3]='e'; g_uperLineLcd[4]='s'; g_uperLineLcd[5]='t';
  g_uperLineLcd[6]=' '; g_uperLineLcd[7]='D'; g_uperLineLcd[8]='i';
  g_uperLineLcd[9]='s'; g_uperLineLcd[10]='p'; g_uperLineLcd[11]='l';
  g_uperLineLcd[12]='a'; g_uperLineLcd[13]='y'; g_uperLineLcd[14]=' ';
  g_uperLineLcd[15]=' ';
  g_lowerLineLcd[0]=' '; g_lowerLineLcd[1]='S'; g_lowerLineLcd[2]='t';
  g_lowerLineLcd[3]='a'; g_lowerLineLcd[4]='r'; g_lowerLineLcd[5]='t';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='T'; g_lowerLineLcd[8]='e';
  g_lowerLineLcd[9]='s'; g_lowerLineLcd[10]='t'; g_lowerLineLcd[11]='t';
  g_lowerLineLcd[12]='i'; g_lowerLineLcd[13]='n'; g_lowerLineLcd[14]='g';
  g_lowerLineLcd[15]=' ';
  printLcdU();
  printLcdL();
  _delay_ms(100);
  writeCommandLCD(0xcd);
  writeDataLCD('.');
  _delay_ms(20);
  writeDataLCD('.');
  _delay_ms(50);
  writeDataLCD('.');
  _delay_ms(300);
  writeCommandLCD(0x01);
  unsigned char i = 0;
  writeCommandLCD(0x80);
  for(i = 0; i < 16; i++){
    writeDataLCD(0xff);
    _delay_ms(50);
  }
  writeCommandLCD(0xc0);
  for(i = 0; i < 16; i++){
    writeDataLCD(0xff);
    _delay_ms(50);
  }
  _delay_ms(500);
  g_uperLineLcd[0]='P'; g_uperLineLcd[1]='r'; g_uperLineLcd[2]='e';
  g_uperLineLcd[3]='s'; g_uperLineLcd[4]='s'; g_uperLineLcd[5]=' ';
  g_uperLineLcd[6]='*'; g_uperLineLcd[7]=' '; g_uperLineLcd[8]='i';
  g_uperLineLcd[9]='f'; g_uperLineLcd[10]=' '; g_uperLineLcd[11]='E';
  g_uperLineLcd[12]='R'; g_uperLineLcd[13]='R'; g_uperLineLcd[14]='O';
  g_uperLineLcd[15]='R';
  g_lowerLineLcd[0]='P'; g_lowerLineLcd[1]='r'; g_lowerLineLcd[2]='e';
  g_lowerLineLcd[3]='s'; g_lowerLineLcd[4]='s'; g_lowerLineLcd[5]='#';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]='I'; g_lowerLineLcd[8]='f';
  g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]='N'; g_lowerLineLcd[11]='o';
  g_lowerLineLcd[12]='r'; g_lowerLineLcd[13]='m'; g_lowerLineLcd[14]='a';
  g_lowerLineLcd[15]='l';
  soundPushButton();
  printLcdL();
  printLcdU();
  g_timeSec = 5;
  bool run = 1;
  unsigned char keyTemp = 0xff;
  do{
    keyTemp = scanKeyInside();
    if(g_timeSec == 0){
      result ++;
      g_errorCode[0] = 1;
      g_errorCode[3] = 1;
      soundPushButton();
      soundPushButton();
      g_changeStateLcd = 1;
      run = 0; 
    }
    if(keyTemp == 0x2a){
      result ++;
      g_errorCode[3] = 1;
      g_changeStateLcd = 1;
      run = 0;
    }
    if(keyTemp == 0x23){
      g_errorCode[3] = 0;
      g_changeStateLcd = 1;
      run = 0;
    }
  }while(run);
  testting = 0;
  return result;
}
void testLight(){
  _delay_ms(200);
  writeCommandLCD(0x0c);
  g_remainLcd = 10;
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]='T'; g_uperLineLcd[2]='E';
  g_uperLineLcd[3]='S'; g_uperLineLcd[4]='T'; g_uperLineLcd[5]=' ';
  g_uperLineLcd[6]='O'; g_uperLineLcd[7]='P'; g_uperLineLcd[8]='E';
  g_uperLineLcd[9]='R'; g_uperLineLcd[10]='A'; g_uperLineLcd[11]='T';
  g_uperLineLcd[12]='I'; g_uperLineLcd[13]='O'; g_uperLineLcd[14]='N';
  g_uperLineLcd[15]=' ';
  g_lowerLineLcd[0]='O'; g_lowerLineLcd[1]='n'; g_lowerLineLcd[2]='l';
  g_lowerLineLcd[3]='y'; g_lowerLineLcd[4]=' '; g_lowerLineLcd[5]='f';
  g_lowerLineLcd[6]='o'; g_lowerLineLcd[7]='r'; g_lowerLineLcd[8]='T';
  g_lowerLineLcd[9]='e'; g_lowerLineLcd[10]='s'; g_lowerLineLcd[11]='t';
  g_lowerLineLcd[12]='t'; g_lowerLineLcd[13]='i'; g_lowerLineLcd[14]='n';
  g_lowerLineLcd[15]='g';
  printLcdU();
  printLcdL();
  ledOpen(1);
  ledFail(1);
  ledAllow(1);
  ledDenied(1);
  ledLock(1);
  _delay_ms(500);
  ledUnlock(1);
  _delay_ms(500);
  ledUnlock(1);
  _delay_ms(1000);
}
void stateTest(){
  buzzer(1);
  _delay_ms(25);
  buzzer(0);
  _delay_ms(175);
  g_remainLcd = 2;
}
//end define test hardware


//DEfine function Init SYSTEM
void initializationSystem(){
  initialPinConfig();
  ledOpen(1);
  ledAllow(1);
  ledDenied(0);
  ledUnlock(1);
  ledLock(0);
  ledFail(0);
  initialLcd();
  g_uperLineLcd[0]=' '; g_uperLineLcd[1]='I'; g_uperLineLcd[2]='n';
  g_uperLineLcd[3]='i'; g_uperLineLcd[4]='t'; g_uperLineLcd[5]='i';
  g_uperLineLcd[6]='a'; g_uperLineLcd[7]='l'; g_uperLineLcd[8]='i';
  g_uperLineLcd[9]='z'; g_uperLineLcd[10]='a'; g_uperLineLcd[11]='t';
  g_uperLineLcd[12]='i'; g_uperLineLcd[13]='o'; g_uperLineLcd[14]='n';
  g_uperLineLcd[15]=' ';
  g_lowerLineLcd[0]='['; g_lowerLineLcd[1]='='; g_lowerLineLcd[2]='=';
  g_lowerLineLcd[3]=' '; g_lowerLineLcd[4]=' '; g_lowerLineLcd[5]=' ';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]=' ';
  g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]=' ';
  g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]=' ';
  g_lowerLineLcd[15]=']';
  printLcdU();
  printLcdL();
  _delay_ms(1000);
  initialExternalInteruprt();
  g_lowerLineLcd[3]='='; g_lowerLineLcd[4]='='; g_lowerLineLcd[5]='=';
  printLcdL();
  _delay_ms(700);
  initialTimer0();
  g_lowerLineLcd[6]='='; g_lowerLineLcd[7]='='; g_lowerLineLcd[8]='=';
  printLcdL();
  g_lowerLineLcd[9]='='; g_lowerLineLcd[10]='='; g_lowerLineLcd[11]='=';
  _delay_ms(200);
  printLcdL();
  _delay_ms(1000);
  initSPI(9600);
  g_lowerLineLcd[12]='='; g_lowerLineLcd[13]='='; g_lowerLineLcd[14]='=';
  printLcdL();
  _delay_ms(700);
  g_uperLineLcd[0]='P'; g_uperLineLcd[1]='l'; g_uperLineLcd[2]='e';
  g_uperLineLcd[3]='a'; g_uperLineLcd[4]='s'; g_uperLineLcd[5]='e';
  g_uperLineLcd[6]=' '; g_uperLineLcd[7]='I'; g_uperLineLcd[8]='n';
  g_uperLineLcd[9]='i'; g_uperLineLcd[10]='t'; g_uperLineLcd[11]=' ';
  g_uperLineLcd[12]='T'; g_uperLineLcd[13]='i'; g_uperLineLcd[14]='m';
  g_uperLineLcd[15]='e';
  g_lowerLineLcd[0]='T'; g_lowerLineLcd[1]='i'; g_lowerLineLcd[2]='m';
  g_lowerLineLcd[3]='e'; g_lowerLineLcd[4]=':'; g_lowerLineLcd[5]=' ';
  g_lowerLineLcd[6]=' '; g_lowerLineLcd[7]=' '; g_lowerLineLcd[8]='h';
  g_lowerLineLcd[9]=' '; g_lowerLineLcd[10]=' '; g_lowerLineLcd[11]='m';
  g_lowerLineLcd[12]=' '; g_lowerLineLcd[13]=' '; g_lowerLineLcd[14]='s';
  g_lowerLineLcd[15]=' ';
  printLcdU();
  printLcdL();
  SREG |= 0x80;  //Global interrupt enable
  buzzer(1);
  _delay_ms(50);
  buzzer(0);
  bool runLocal = 1;
  unsigned char keyTempLocal = 0;
  unsigned char tempHour = 0;
  unsigned char tempMinute = 0;
  unsigned char tempSec = 0;
  unsigned counterTemp = 0;
  do{
    keyTempLocal = scanKeyInside();
    if(keyTempLocal == 0x23){
      runLocal = 0;
      g_timeSys.hour = tempHour;
      g_timeSys.minute = tempMinute;
      g_timeSys.sec = tempSec;
    }else if(keyTempLocal >= 0x30 && keyTempLocal <= 0x39){
      if(counterTemp == 0){
        g_lowerLineLcd[6] = keyTempLocal;
        tempHour = (keyTempLocal-0x30)*10;
        counterTemp ++;
      }else if(counterTemp == 1){
        g_lowerLineLcd[7] = keyTempLocal;
        tempHour += keyTempLocal-0x30;
        counterTemp ++;
      }else if(counterTemp == 2){
        g_lowerLineLcd[9] = keyTempLocal;
        tempMinute = (keyTempLocal-0x30)*10;
        counterTemp ++;
      }else if(counterTemp == 3){
        g_lowerLineLcd[10] = keyTempLocal;
        tempMinute += keyTempLocal - 0x30;
        counterTemp ++;
      }else if(counterTemp == 4){
        g_lowerLineLcd[12] = keyTempLocal;
        tempSec = (keyTempLocal-0x30)*10;
        counterTemp ++;
      }else if(counterTemp == 5){
        g_lowerLineLcd[13] = keyTempLocal;
        tempSec += keyTempLocal - 0x30;
        counterTemp ++;
      }
      if(counterTemp == 6) counterTemp = 0;
      printLcdL();
    }else if(keyTempLocal != 0xff){
      printLcdL();
    }
  }while(runLocal);
}

void initialLcd(){
  g_remainLcd = 20;
  //turnonoLcd
  writeCommandLCD(0x0c);
  writeCommandLCD(0x38);
  _delay_ms(2000);
}
void initialTimer0(){
  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 7.813 kHz
  // Mode: Normal top=0xFF
  // OC0A output: Disconnected
  // OC0B output: Disconnected
  // Timer Period: 9.984 ms
  TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
  TCCR0B=(0<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);    //source clock / 1024
  TCNT0=0x63;       //10ms
  OCR0A=0x00;
  OCR0B=0x00;
  // Timer/Counter 0 Interrupt(s) initialization
  TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);  
}
void initialExternalInteruprt(){
  // External Interrupt(s) initialization
  // INT0: On
  // INT0 Mode: Falling Edge
  // INT1: On
  // INT1 Mode: Falling Edge
  // Interrupt on any change on pins PCINT0-7: Off
  // Interrupt on any change on pins PCINT8-14: Off
  // Interrupt on any change on pins PCINT16-23: Off
  EICRA=(1<<ISC11) | (0<<ISC10) | (1<<ISC01) | (0<<ISC00);
  EIMSK=(1<<INT1) | (1<<INT0);
  EIFR=(1<<INTF1) | (1<<INTF0);
  PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);
}
void initialPinConfig(){
  DDRB |= 0x3f; //bit 0 -> 5 is output
  DDRC &= 0xc0; //bit 0 -> 5 is input
  PORTC = 0xFF; //all 1 to pull up
  DDRD |= 0xe0; //bit 7 -> 5 is output else is input 
  PORTD |= 0x11;
}
void initSPI(unsigned int boudrate){
  UBRR0H = (unsigned char)((F_CPU/16/boudrate - 1)>>8);
  UBRR0L = (unsigned char)(F_CPU/16/boudrate -1);
  UCSR0B |= (1<<4); //enable RX bit 4
  UCSR0B |= (1<<3); //enable TX bit 3
  UCSR0A |= (1<<6);
  UCSR0C = 0b00000110;  //Asynchronous USART //disable parity //1 bit stop //8bit data // polo raide

}
//end define init system
