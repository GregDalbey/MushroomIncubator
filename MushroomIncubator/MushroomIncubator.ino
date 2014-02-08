/*

Pin Mapping


const int ButtonUp = 4;
const int ButtonDown = 7;
const int ButtonMenu = 2;
const int ButtonNone = -1;

#define REDLITE 3
#define GREENLITE 5
#define BLUELITE 6

int RelayPinTemp = 10; 
int RelayPinHumidity = 11;
#define DHTPIN 12     // what pin we're connected to
int RelayPinCO2Exhaust = 13;

*/


#include <PID_AutoTune_v0.h>
#include <DHT.h>
#include <stdint.h>
#include <avr/io.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>


#define REDLITE 3
#define GREENLITE 5
#define BLUELITE 6

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7


// These are the values that are read from the instrument
double currentHumidity;
double currentTemperature;


double setpointHumidity;
double setpointTemperature;

// TODO - remove
double setpointCo2;

// Set point values.  These are changeable from with the Menu System
int setpointCO2Numerator;
int setpointCO2Denominator;


// TEMP SETTINGS
int RelayPinTemp = 10;
double PIDOutputTemp;
double  KpTemp;
double  KiTemp;
double  KdTemp;
volatile long onTimeTemp = 0;

PID PIDTemp(&currentTemperature, &PIDOutputTemp, &setpointTemperature, KpTemp, KiTemp, KdTemp, DIRECT);
int windowSizeTemp = 10000; 
unsigned long windowStartTimeTemp;


// HUMIDITY SETTINGS
int RelayPinHumidity = 11;
double PIDOutputHumidity;
double  KpHumidity;
double  KiHumidity;
double  KdHumidity;
volatile long onTimeHumidity = 0;


PID PIDHumidity(&currentHumidity, &PIDOutputHumidity, &setpointHumidity, KpHumidity, KiHumidity, KdHumidity, DIRECT);
int windowSizeHumidity = 10000; 
unsigned long windowStartTimeHumidity;

// CO2 Exhaust
int RelayPinCO2Exhaust = 13;
volatile int CO2CycleClicks = 0;


byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 


#define DHTPIN 12     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

const int ButtonUp = 4;
const int ButtonDown = 7;
const int ButtonMenu = 2;
const int ButtonNone = -1;
 
 
// you can change the overall brightness by range 0 -> 255
int brightness = 255;
 
// ************************************************
// DiSplay Variables and constants
// ************************************************

LiquidCrystal lcd(0);

DHT dht(DHTPIN, DHTTYPE);

// These #defines make it easy to set the backlight color

enum operatingState { OFF = 0, MAIN, TUNE_TEMP, TUNE_HUMIDITY, TUNE_CO2, TUNE_CO2_NUM, TUNE_CO2_DEN };
operatingState opState = OFF;

int buttonValue;

void setup() {
   
      Serial.begin(9600);

	setpointHumidity = 40;
	setpointTemperature = 20;
	setpointCo2 = 400;
               
        setpointCO2Numerator = 0;
        setpointCO2Denominator = 0;
        	 
	KpTemp = 850;
	KiTemp = 0.5;
	KdTemp = 0.1;

	KpHumidity = 850;
	KiHumidity = 0.5;
	KdHumidity = 0.1;

	lcd.begin(16, 2);
	lcd.clear();

	setBacklight(GREEN);

	pinMode(REDLITE, OUTPUT);
	pinMode(GREENLITE, OUTPUT);
	pinMode(BLUELITE, OUTPUT);

	pinMode(ButtonUp, INPUT); 
	pinMode(ButtonDown, INPUT);
	pinMode(ButtonMenu, INPUT);

	pinMode(RelayPinTemp, OUTPUT);
	pinMode(RelayPinHumidity, OUTPUT);
	pinMode(RelayPinCO2Exhaust, OUTPUT);



	dht.begin();

	PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);
	PIDTemp.SetSampleTime(1000);
	PIDTemp.SetOutputLimits(0, windowSizeTemp);
	PIDTemp.SetMode(AUTOMATIC);

        windowStartTimeTemp = millis();

	PIDHumidity.SetTunings(KpHumidity,KiHumidity,KdHumidity);
	PIDHumidity.SetSampleTime(1000);
	PIDHumidity.SetOutputLimits(0, windowSizeHumidity);
	PIDHumidity.SetMode(AUTOMATIC);

        windowStartTimeHumidity = millis();

	// Run timer2 interrupt every 15 ms 
	TCCR2A = 0;
	TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

	//Timer2 Overflow Interrupt Enable
	TIMSK2 |= 1<<TOIE2;
  
 }
 
// ************************************************
// Timer Interrupt Handler
// ************************************************
ISR(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPinTemp, LOW);  // make sure relay is off
    digitalWrite(RelayPinHumidity, LOW);  // make sure relay is off
    digitalWrite(RelayPinCO2Exhaust, LOW);
  }
  else
  {
    DrivePIDOutputTemp();
    DrivePIDOutputHumidity();
    DriveOutputCO2Exhaust();
  }
}


 void loop() {
	 

	switch (opState)
	{
		case OFF:
		Off();
		break;
	 
		case MAIN:
		Main();
		break;
	 
		case TUNE_TEMP:
		TuneTemp();
		break;

		case TUNE_HUMIDITY:
		TuneHumidity();
		break;

		case TUNE_CO2_NUM:
		TuneCO2Numerator();
		break;

		case TUNE_CO2_DEN:
		TuneCO2Denominator();
		break;

	}
 } 
 
  
 int ReadButtons()
 {
	 int returnVal = ButtonNone;

	 int resultButtonMenu;
	 int resultButtonUp;
	 int resultButtonDown;
	 
	 resultButtonMenu = digitalRead(ButtonMenu);
	 resultButtonUp = digitalRead(ButtonUp);
	 resultButtonDown = digitalRead(ButtonDown);

	 if (resultButtonMenu == HIGH)
          {
            returnVal = ButtonMenu;
          }

	 if (resultButtonUp == HIGH)
          {
            returnVal = ButtonUp;
          }

	 if (resultButtonDown == HIGH)
          {
            returnVal = ButtonDown;
          }

	return returnVal;

 }
 
 
 void Off()
 {

    int buttons = ButtonNone;
    buttons = ReadButtons();

    setBacklight(GREEN);
    lcd.setCursor(0,0);
    lcd.print("Mushroom  ");
    lcd.setCursor(0,1);
    lcd.print(" Incubator! =D ");

    while(buttons == ButtonNone)
    {
      buttons = ReadButtons();
            
      if (buttons == ButtonMenu)  // Should be at steady-state
      {
        opState = MAIN;
	return;
    		}
    	}
 }


 void Main()
 {
        setBacklight(BLUE);
        lcd.clear();

	int buttons = ButtonNone;
        buttons = ReadButtons();

        PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);
        PIDHumidity.SetTunings(KpHumidity,KiHumidity,KdHumidity);
        

	while(buttons == ButtonNone)
	{
		DoControl();
 
		if (isnan(currentTemperature)  || isnan(currentHumidity)) {
		} else {
			lcd.setCursor(0,0);
			lcd.print("T: ");
			lcd.print(currentTemperature);

			lcd.print("   H: ");
			lcd.print(currentHumidity);

			lcd.setCursor(0,1);

			lcd.print("Ex: ");
                        lcd.print(setpointCO2Numerator);
			lcd.print("/");
			lcd.print(setpointCO2Denominator);
		}

		buttons = ReadButtons();

		if (buttons == ButtonMenu) 
		{
			opState = TUNE_TEMP;
			return;
		}
	
	}
 }

void DoControl()
{
  // turn off interupts for a bit while we do some things that cannot be interupted
  
    noInterrupts();

    currentHumidity = dht.readHumidity();
    currentTemperature = dht.readTemperature();

    // tell each PID object to compute

      /*
        if (tuning) // run the auto-tuner
        {
           if (aTune.Runtime()) // returns 'true' when done
           {
              FinishAutoTune();
           }
        }
        else // Execute control algorithm
        {
          
      */

     PIDTemp.Compute();
     PIDHumidity.Compute();

    interrupts();

  // Time Proportional relay state is updated regularly via timer interrupt.

    onTimeTemp = PIDOutputTemp; 
    onTimeHumidity = PIDOutputHumidity; 

    if (windowSizeTemp == onTimeTemp && currentTemperature == 0)
    {
      onTimeTemp = 0; 
      onTimeHumidity = 0;   
    }
  
  Serial.print(onTimeHumidity);
  Serial.println();
  
}


// ************************************************
// Called by ISR every 15ms to drive the PIDOutputTemp
// ************************************************

void DrivePIDOutputTemp()
{  
  long now = millis();
  // Set the PIDOutputTemp
  // "on time" is proportional to the PID PIDOutputTemp

  if (now - windowStartTimeTemp > windowSizeTemp)
  { 
     //time to shift the Relay Window
     windowStartTimeTemp += windowSizeTemp;
  }
  
  if((onTimeTemp > 100) && (onTimeTemp > (now - windowStartTimeTemp)))
  {
     digitalWrite(RelayPinTemp,HIGH);
  }
  else
  {
     digitalWrite(RelayPinTemp,LOW);  
  }
}


void DriveOutputCO2Exhaust()
{
    CO2CycleClicks += 1;
    int numMinutes = (CO2CycleClicks / 3960) + 1;
    
    // if we surpassed the total amount of minutes, reset

    if (numMinutes > setpointCO2Denominator)
      CO2CycleClicks = 0;
      
    if (numMinutes <= setpointCO2Numerator)
    {
     digitalWrite(RelayPinCO2Exhaust,HIGH);     
    }
    else
    {
     digitalWrite(RelayPinCO2Exhaust,LOW);      
    }
}


// ************************************************
// Called by ISR every 15ms to drive the PIDOutputHumidity
// ************************************************
void DrivePIDOutputHumidity()
{  
  long now = millis();
  // Set the PIDOutputTemp
  // "on time" is proportional to the PID PIDOutputTemp


  if(now - windowStartTimeHumidity > windowSizeHumidity)
  { //time to shift the Relay Window
     windowStartTimeHumidity += windowSizeHumidity;
  }
  if((onTimeHumidity > 100) && (onTimeHumidity > (now - windowStartTimeHumidity)))
  {
     digitalWrite(RelayPinHumidity,HIGH);
  }
  else
  {
     digitalWrite(RelayPinHumidity,LOW);
  }
}



void setBacklight(int color)
{
	
  uint8_t r;
  uint8_t g;
  uint8_t b;
  
  switch (color)
	 {
		 case RED:
		 r = 255;
		 g = 10;
		 b = 10;
		 break;

		 case YELLOW:
		 r = 255;
		 g = 255;
		 b = 0;
		 break;
		 
		 case BLUE:
		 r = 0;
		 g = 0;
		 b = 255;
		 break;
		 
		 case GREEN:
		 r = 0;
		 g = 255;
		 b = 0;
		 break;
		 
		 case TEAL:
		 r = 20;
		 g = 255;
		 b = 200;
		 break;
		 
		case VIOLET:
		r = 255;
		g=10;
		b= 215;
		break;

		case WHITE:
		r = 255;
		g=255;
		b= 255;
		break;
	 }

	// normalize the red LED - its brighter than the rest!
	r = map(r, 0, 255, 0, 130);
	g = map(g, 0, 255, 0, 150);

	r = map(r, 0, 255, 0, brightness);
	g = map(g, 0, 255, 0, brightness);
	b = map(b, 0, 255, 0, brightness);

	// common anode so invert!
	r = map(r, 0, 255, 255, 0);
	g = map(g, 0, 255, 255, 0);
	b = map(b, 0, 255, 255, 0);

	analogWrite(REDLITE, r);
	analogWrite(GREENLITE, g);
	analogWrite(BLUELITE, b);
}

void DisplayTuneTemp()
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set Temperature ");
    lcd.setCursor(0,1);
    lcd.print(setpointTemperature);

}

 void TuneTemp()
 {
	 // lcd.clear();
	 // lcd.print("temp");

	 // tell PIDs to set their tunings - which are hard const for now
	 // PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);

	setBacklight(RED);

	DisplayTuneTemp();
	
	// tell PIDs to set their tunings - which are hard const for now
	// PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);
	
	int buttons = ButtonNone;
	buttons = ReadButtons();

	while(buttons == ButtonNone)
	{
		buttons = ReadButtons();
		if (buttons == ButtonMenu)  // Should be at steady-state
		{
			opState = TUNE_HUMIDITY;
			return;
		}

		if (buttons == ButtonUp)  // Should be at steady-state
		{
			if (setpointTemperature >= 50)
				setpointTemperature = 50;
			else
			{
				setpointTemperature += 1;
				DisplayTuneTemp();
			}
		}

		if (buttons == ButtonDown)  // Should be at steady-state
		{
			if (setpointTemperature <= 10)
				setpointTemperature = 10;
			else
			{
				setpointTemperature -= 1;
				DisplayTuneTemp();
			}
		}
		
		// have // lcd report PID PIDOutputTemps / values of temp, humidity, co2
	} 
}

void DisplayTuneCO2()
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set CO2         ");
    lcd.setCursor(0,1);
    lcd.print(setpointCo2);
}

 void TuneCO2()
 {

        setBacklight(TEAL);
	DisplayTuneCO2();

	int buttons = ButtonNone;
	buttons = ReadButtons();

	while(buttons == ButtonNone)
	{
		buttons = ReadButtons();
		if (buttons == ButtonMenu)
		{
			opState = MAIN;
			return;
		}

		if (buttons == ButtonUp)  
		{
			if (setpointCo2 >= 2000)
				setpointCo2 = 2000;
			else
			{
				setpointCo2 += 100;
				DisplayTuneCO2();
			}
		}

		if (buttons == ButtonDown)  
		{
			if (setpointCo2 <= 100)
				setpointCo2 = 100;
			else
			{
				setpointCo2 -= 100;
				DisplayTuneCO2();
			}
		}

		// have // lcd report PID PIDOutputTemps / values of temp, humidity, co2
	}
 }


void DisplayTuneCO2Numerator()
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set CO2 Numer   ");
    lcd.setCursor(0,1);
    lcd.print(setpointCO2Numerator);
}

 void TuneCO2Numerator()
 {

      setBacklight(YELLOW);
	DisplayTuneCO2Numerator();

	int buttons = ButtonNone;
	buttons = ReadButtons();

	while(buttons == ButtonNone)
	{
	
		buttons = ReadButtons();
		if (buttons == ButtonMenu)  // Should be at steady-state
		{
			opState = TUNE_CO2_DEN;
			return;
		}

		if (buttons == ButtonUp)  
		{
			if (setpointCO2Numerator >= 30)
				setpointCO2Numerator = 30;
			else
			{
				setpointCO2Numerator += 1;
				DisplayTuneCO2Numerator();
			}
		}

		if (buttons == ButtonDown)  
		{
			if (setpointCO2Numerator <= 1)
				setpointCO2Numerator = 1;
			else
			{
				setpointCO2Numerator -= 1;
				DisplayTuneCO2Numerator();
			}
		}

		// have // lcd report PID PIDOutputTemps / values of temp, humidity, co2
	}
 }



void DisplayTuneCO2Denominator()
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set CO2 Denom   ");
    lcd.setCursor(0,1);
    lcd.print(setpointCO2Denominator);
}

 void TuneCO2Denominator()
 {
	 // lcd.clear();
	 // lcd.print("CO2");

	 // tell PIDs to set their tunings - which are hard const for now
	 // PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);

    setBacklight(TEAL);
	DisplayTuneCO2Denominator();

	// tell PIDs to set their tunings - which are hard const for now
	// PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);

	int buttons = ButtonNone;
	buttons = ReadButtons();

	while(buttons == ButtonNone)
	{
		//setBacklight(PURPLE);  // set backlight based on state
	
		buttons = ReadButtons();
		if (buttons == ButtonMenu)  // Should be at steady-state
		{
			opState = MAIN;
			return;
		}

		if (buttons == ButtonUp)  
		{
			if (setpointCO2Denominator >= 30)
				setpointCO2Denominator = 30;
			else
			{
				setpointCO2Denominator += 1;
				DisplayTuneCO2Denominator();
			}
		}

		if (buttons == ButtonDown)  
		{
			if (setpointCO2Denominator <= 1)
				setpointCO2Denominator = 1;
			else
			{
				setpointCO2Denominator -= 1;
				DisplayTuneCO2Denominator();
			}
		}

		// have // lcd report PID PIDOutputTemps / values of temp, humidity, co2
	}
 }








 void DisplayTuneHumidity()
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set Humidity    ");
    lcd.setCursor(0,1);
    lcd.print(setpointHumidity);
}

 void TuneHumidity()
 {
	 // lcd.clear();
	 // lcd.print("Humidity");

	 // tell PIDs to set their tunings - which are hard const for now
	 // PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);

    setBacklight(GREEN);

	DisplayTuneHumidity();

	// tell PIDs to set their tunings - which are hard const for now
	// PIDTemp.SetTunings(KpTemp,KiTemp,KdTemp);

    int buttons = ButtonNone;
    buttons = ReadButtons();

	while(buttons == ButtonNone)
	{
		buttons = ReadButtons();
		if (buttons == ButtonMenu)  // Should be at steady-state
		{
			opState = TUNE_CO2_NUM;
			return;
		}

		if (buttons == ButtonUp)  // Should be at steady-state
		{
			if (setpointHumidity >= 100)
				setpointHumidity = 100;
			else
			{
				setpointHumidity += 1;
				DisplayTuneHumidity();
			}
		}

		if (buttons == ButtonDown)  // Should be at steady-state
		{
			if (setpointHumidity <= 0)
				setpointHumidity = 0;
			else
			{
				setpointHumidity -= 1;
				DisplayTuneHumidity();
			}
		}
		
		// have // lcd report PID PIDOutputTemps / values of temp, humidity, co2
	}
 }
