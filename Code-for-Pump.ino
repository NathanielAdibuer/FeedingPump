#include <DHT.h>
#define Type DHT11
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
int sensorpin = A3;
DHT HT(sensorpin, Type);
//LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 or 0x20
LiquidCrystal_I2C lcd(0x27,20,4);

//Pin selection
int motor1pin = 7; //choose pin for motor
int enablepin = 5; // choose enable pin, for controlling speed
int pushIncrease = 8; //choose pushbutton pin for increasing value of flowrate/volume
int pushDecrease = 9; //choose pushbutton pin for decreasing value of flowrate/volume
int pushVolume = 10; //choose pin for entering volume
int pushFlowrate = 11; //choose pin for entering flowrate
int pushOK = 12; //choose pin for entering okay
int buzzer = A1; //choose pin for buzzer
int ledGreen = 2; //choose pin for ledGreen
int ledRed= 3; //choose pin for ledRed
int ledBlue= 4; //choose pin for ledBlue
int pushStart = 13;
float tempC;

//Storing values in decleared variables
int pushIncrease_state;       //State of pushIncrease button (HIGH/LOW)
int pushDecrease_state;       //State of pushDecrease button (HIGH/LOW)
int pushVolume_state;         //State of pushVolume button (HIGH/LOW)
int pushFlowrate_state;       //State of push Flowrate button (HIGH/LOW)
int pushOK_state;             //State of pushOK button (HIGH/LOW)
int pushStart_state;
unsigned long stoptime;
int pumpState = LOW;


//Values stored in variables
int volume = 0;       //current volume of pump
int Time = 0;     //current flowrate of pump
int readButtons = 0;  //variable assigned to buttons


long previousMillis = 0;  
unsigned long currentMillis ;
long interval = 1000;
unsigned long multiplier = 3000; //value to change based on calibration
//multiplier = milliseconds needed to fill 1ml liquid
const int currentPin = A0;

void setup() {
  
  Serial.begin(9600);
  //Motorpin setup
  pinMode (motor1pin, OUTPUT); //select motor pin1 as output
  pinMode (enablepin, OUTPUT); //select pushbutton as input

  //Push mode setup
  pinMode (pushIncrease, INPUT);        //select increase pushbutton as input
  pinMode (pushDecrease, INPUT);        //select decrease pushbutton as input
  pinMode (pushVolume, INPUT);          //select volume pushbutton as input
  pinMode (pushFlowrate, INPUT);        //select flowrate pushbutton as input
  pinMode (pushOK, INPUT);              //select OK pushbutton as input
  pinMode (buzzer, OUTPUT);             //select pushbutton as input
  pinMode (ledGreen, OUTPUT);           //select green led as output
  pinMode (ledRed, OUTPUT);             //select red led as output
  pinMode (ledBlue, OUTPUT);            //select blue led as output
  
        //Start up
        lcd.init();
        lcd.backlight();
        lcd.setCursor(0,0);
        lcd.print("WELCOME");
        delay(3000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Enter volume and");
        lcd.setCursor(0,1);
        lcd.print("Time...");
        delay(3000);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Waiting...");

        
}

void enterVolume() {
        lcd.setCursor(0,0);
        lcd.print("Enter volume to ");
        lcd.setCursor(0,1);
        lcd.print("be fed: ");
        lcd.setCursor(9,1);
        lcd.print("  ");
        lcd.setCursor(8,1);
        lcd.print(volume);
        lcd.setCursor(11,1);
        lcd.print("ml");
        lcd.setCursor(13,1);
        lcd.print("    ");
        controlVolume();

   }

void controlVolume(){
        pushIncrease_state = digitalRead(pushIncrease);
        pushDecrease_state = digitalRead(pushDecrease);

 
    if(pushIncrease_state == HIGH && volume <100) 
    {
      
       volume = volume+=1;
       lcd.setCursor(8,1);
       lcd.print(volume);
       delay(125);
    }

    if(pushDecrease_state == HIGH && volume >0) 
    {
      
       volume = volume-=1;
       lcd.setCursor(9,1);
       lcd.print("  ");
       lcd.setCursor(8,1);
       lcd.print(volume);
       delay(125);
    }
  }

void enterFlowrate(){
        lcd.setCursor(0,0);
        lcd.print("Enter Time: ");
        lcd.setCursor(0,1);
        lcd.print("   ");
        lcd.setCursor(0,1);
        lcd.print(Time);
        lcd.setCursor(3,1);
        lcd.print("min");
        lcd.setCursor(9,1);
        lcd.print("       ");
        lcd.print("  ");
        controlFlowrate();
  }

void controlFlowrate(){
        pushIncrease_state = digitalRead(pushIncrease);
        pushDecrease_state = digitalRead(pushDecrease);

 
    if(pushIncrease_state == HIGH && Time <20) 
    {
      
     Time = Time+=1;
       lcd.setCursor(0,1);
       lcd.print("   ");
       lcd.setCursor(0,1);
       lcd.print(Time);
       delay(125);
    }

    if(pushDecrease_state == HIGH && Time >0) 
    {
      
       Time = Time-=1;
       lcd.setCursor(0,1);
       lcd.print("   ");
       lcd.setCursor(0,1);
       lcd.print(Time);
       delay(125);
    }
  }


void loop (){

  
  Serial.println(volume);
  Serial.println(Time);
  pushVolume_state = digitalRead(pushVolume);
  pushFlowrate_state = digitalRead(pushFlowrate);
  pushOK_state = digitalRead(pushOK);
  pushIncrease_state = digitalRead(pushIncrease);
  pushDecrease_state = digitalRead(pushDecrease);
  pushStart_state = digitalRead(pushStart);

        if(pushStart_state == HIGH && volume !=0 && Time!=0){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Feeding in");
          lcd.setCursor(0,1);
          lcd.print("progress...");
          tempC = HT.readTemperature();
          lcd.setCursor(0, 3);
         lcd.print("T=");
         lcd.setCursor(3, 3);
         lcd.print(tempC);
          lcd.setCursor(10,3);
         lcd.print("C=");
         lcd.setCursor(13, 3);
         
  unsigned int x=0;
float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++){ //Get 150 samples
  AcsValue = analogRead(currentPin);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
AvgAcs=Samples/150.0;//Taking Average of Samples

//((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
//2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
//out to be 2.5 which is out offset. If your arduino is working on different voltage than 
//you must change the offset according to the input voltage)
//0.185v(185mV) is rise in output voltage when 1A current flows at input
AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.185;
        lcd.print(AcsValueF);
        float Area = 0.13;
float velocity = (volume/(Area*Time));
Serial.print(velocity);
//        if(volume >= 35)
//          stoptime = ((volume-5) * 460000)/100;
//        if(volume >= 20 && volume < 35)
//          stoptime = ((volume-2) * 460000)/100;
//        if(volume < 20 && volume > 14)
//          stoptime = ((volume-1) * 460000)/100;
//        if(volume < 14)
//          stoptime = ((volume-1) * 460000)/100;
          
          
          //digitalWrite(ledGreen, HIGH);
          //delay(1000);
         digitalWrite(enablepin, velocity);
          digitalWrite(motor1pin, HIGH);
          delay(Time*60000);
          digitalWrite(motor1pin, LOW);
          digitalWrite(ledGreen, LOW);
          
          
          
          }
        
    if(pushVolume_state == HIGH)
      readButtons = 1;
    if(pushFlowrate_state == HIGH)
      readButtons = 2;
    if(pushOK_state == HIGH)
      readButtons = 3;
  
      switch(readButtons){
        
        case 1:
        enterVolume();
        break;

        case 2:
        enterFlowrate();
        break;

        case 3:
    if(volume != 0 && Time != 0){
    lcd.setCursor(0,0);
    lcd.print("Vol:   Time:");
    lcd.setCursor(0,1);
    lcd.print("   ");
    lcd.setCursor(0,1);
    lcd.print(volume);
    lcd.setCursor(3,1);
    lcd.print("ml  ");
    lcd.setCursor(7,1);
    lcd.print("   ");
    lcd.setCursor(7,1);
    lcd.print(Time);
    lcd.setCursor(10,1);
    lcd.print("min");
    delay(3000);
    lcd.setCursor(0,0);
    lcd.print("Press increase &");
    lcd.setCursor(0,1);
    lcd.display();
    lcd.print("done to start.  ");
    delay(2000);
    break;

    }
   if(volume == 0 && Time == 0){
    lcd.setCursor(0,0);
    lcd.print("Enter Time");
    lcd.setCursor(14,0);
    lcd.print("  ");
    lcd.setCursor(0,1);
    lcd.print("& volume!");
    lcd.setCursor(11,1);
    lcd.print("    ");
    break;
    
    }
   if(volume == 0 && Time > 0){
    lcd.setCursor(0,0);
    lcd.print("Enter volume!");
    lcd.setCursor(13,0);
    lcd.print("   ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    break;
    
    }
   if(volume > 0 && Time == 0){
    lcd.setCursor(0,0);
    lcd.print("Enter Time!");
    lcd.setCursor(0,1);
    lcd.print("                ");
    break;
    
    }
    
    }
    }
