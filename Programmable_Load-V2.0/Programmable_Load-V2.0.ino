 // Author: Abhishek Tiwari 
// Date: 03-April-2021
// work in progress
#include <Arduino.h>
#include <LiquidCrystal.h>

/* define the LCD 16-bit interface,those pin connect to Microcontroller(12,11,5,4,3,7) */
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 7;  
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int amp                             =   A0;                // define analog read from pin A0      
int volt                            =   A1;                // define anlog read from pin A1
int sw                              =   A2;                // define anlog read from pin A2 and copy this values in sel
int dac                             =    9;                // pin is 9 ,this is can output pwm signal whichi is filtered by low pass filter to control mosfet to maintain cc,cp and cr
int loadled                         =   10;                // load indicator and fan
const byte pinA                     =    2;                // digital pin (also interrupt pin) for the A pin of the Rotary Encoder (changed to digital pin 2)
const byte pinB                     =    8;                // digital pin for the B pin of the Rotary Encoder
int bright                          =    6;                // brightness pwm controlling pin defined which connect to transisitor
int brightsw                        =    0;    
bool yes                            =  true;               // to check if load on or off/ switch is pressed or not
int setbright                       =    0;                // check brightness button is presses or not
int sensVal                         =    0;             
int sel;                                                   // analoge read the input values from switch to be pressed for selection
int AVG_NUM                         =   60;                // average number for precision in reading adc values from sensor
float amps;                                                // stored calculated ampere in amps variable
float volts;                                               // stored calculated voltage in volts variable   
float power;                                               // stored calculated power in power variable
float setCurrent                    =    0;                // setcurrent is initialy zero,it get the values from encoder
float setPower                      =    0;                // setpower is initialy zero,it get the values from encoder
float setResistance                 =    0;                // setresistance is initialy zero,it get the values from encoder
unsigned long controlVoltage        =    0;                // used for DAC to control MOSFET
float setControlCurrent             =    0;                // final calculated values sent to dac to write
float PowerCutOff                   =   60;                // maximum Power allowed in Watts - then limited to this level CAN BE CHANGED AS REQUIRED
float CurrentCutOff                 =    9;                // maximum Current setting allowed in Amps - then limited to this level
String Mode                         =   "  ";              // used to identify which mode
boolean toggle                      =  false;              // to check if load on or off/ switch is pressed or not
int Load                            =    0;                // initialy load set to zero
int lastCount                       =   50;                // end is 50
volatile float encoderPosition      =    1;                // starting position is set by this
volatile unsigned long factor       =    1;                // number of steps to jump
volatile unsigned long encoderMax   =   60;                // sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED
int reading                         =    0;
int CP                              =   14;                //cursor start position

/* define character in lcd example library
heart for introduction screen to make it more attaractive */
byte heart[8]    = { 0b00000,0b01010,0b11111,0b11111,0b11111,0b01110,0b00100,0b00000 };

/* smiley for introduction screen to make it more attaractive */
byte smiley[8]   = { 0b00000,0b00000,0b01010,0b00000,0b00000,0b10001,0b01110,0b00000 };

/* a guy standing for introduction screen to make it more attaractive */
byte frownie[8]  = { 0b00000,0b00000,0b01010,0b00000,0b00000,0b00000,0b01110,0b10001 };

/* arms moving down for introduction screen to make it more attaractive */
byte armsDown[8] = { 0b00100,0b01010,0b00100,0b00100,0b01110,0b10101,0b00100,0b01010 };

/* arms moving up for introduction screen to make it more attaractive */
byte armsUp[8]   = { 0b00100,0b01010,0b00100,0b10101,0b01110,0b00100,0b00100,0b01010 };

/* isr helps the rotatory enocder to never miss a clock and data in any condition */
void isr() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();

    if (interruptTime - lastInterruptTime > 4) {  
        if (digitalRead(pinB) == LOW) {
        encoderPosition = encoderPosition - factor;          //to decrease the numbers
        } else {
            encoderPosition = encoderPosition + factor;           // to increase the numbers
            }
    
    encoderPosition = min(encoderMax, max(0, encoderPosition));  // sets maximum range of rotary encoder
    lastInterruptTime = interruptTime;
    }
}

void lcdSetup() {
    lcd.begin(16,2);                                           // to begin the 16X2 lcd
    lcd.createChar(0, heart);                                  // create a new character
    lcd.createChar(1, smiley);                                 
    lcd.createChar(2, frownie);    
    lcd.createChar(3, armsDown); 
    lcd.createChar(4, armsUp);
    lcd.clear();                                               // clear the lcd
    lcd.setCursor(0,0);                                         // the cursor is set to start the character from 0 row and 0 column
    lcd.print(" PROGRAMMABLE ");                               // pinting the character    
    lcd.write((byte)1);                                        // smiley
    lcd.setCursor(0,1);                                        // the cursor is set to start the character from 1 row and 0 column
    lcd.print("ELECTRONIC LOAD");                              // pinting the character
    lcd.write(byte(0));                                        // heart
    delay(3000);                                               // wait to see by users
    lcd.clear();                                               // clear everything for new character
    /* this for fun if you want this too then uncomment it */
    /* lcd.setCursor(0,0);                                   
    lcd.print("I Feel The Need");                        
    lcd.write(byte(0));
    lcd.setCursor(0,1);
    lcd.print("Need For Speed");
    lcd.write(byte(0));
    delay(3000);
    lcd.clear(); */
    lcd.setCursor(0, 0);                                       // set the cursor
    lcd.print("Max. POWER 100W");
    lcd.write(byte(0));                                        // heart
    lcd.setCursor(0, 1);
    lcd.print("Max. CURRENT 9A");               
    /* loop to give graphics in which man is waving hands up and down*/
    for(int i=0;i<3;i++) {  
    delay(50);                                            // wait
    lcd.setCursor(15, 1);                                 // column 15 row 1
    lcd.write(3);                                         // down hand
    delay(500);                                           // to maintain synchronous hand moment
    lcd.setCursor(15, 1);                                 // column 15 row 1
    lcd.write(4);                                         // up hand
    delay(500);                                           // to maintain synchronous hand moment
}
    lcd.clear();                                  
    }

void gpioModeSetup() {
    pinMode (pinA, INPUT);                                    // make sure to define which pin is use for inputs and outputs
    pinMode (pinB, INPUT);                                    // make sure to define which pin is use for inputs and outputs
    pinMode (bright, OUTPUT);                                 // make sure to define which pin is use for inputs and outputs
    pinMode (loadled, OUTPUT);                                // make sure to define which pin is use for inputs and outputs
    pinMode (sw, INPUT_PULLUP);                               // pull up means sw pin become high at 5v,no need to use external pulllup resistor to get input from switch
    pinMode (dac,OUTPUT);                                     // make sure to define which pin is use for inputs and outputs
}

void gpioInitialState() {
    digitalWrite(dac,LOW);                                    // intialy mosfet don,t get any values it means dac is low or mosfet is resting
} 
/*----------------------modes selection------------------------------------*/
void modesdacreading() {
/*----------------------Select Constant current------------------------------------*/
if ( Mode == "CC" ) {                                         // if mode is constant current
    setCurrent = reading;                                     // reading take through rotory encoder is copied to set current
    setControlCurrent = 44+((setCurrent-1)*18);               // let me give you simple explanation on this, i did this by roughly giving values to dac then i conclude is formula
    /* when i set 44 value to controlvoltage(dac) load draw 1A ampere through supply
    which means 44 is starting then i make relation between encoder and dac by this formula
    if user set 1 value vua encoder, setcontrolvoltage(44+(1-1*18)) make it 44.
    for 2 ampere, setcontrolvoltage(44+(2-1*18)) make it 62. */ 
    controlVoltage = setControlCurrent ;                      // for dac to opertae mosfet
}
/* ----------------------Select Constant Power------------------------------------ */
if ( Mode == "CP" ) {                                         // if mode is constant power
    setPower = reading;                                       // reading take through rotory encoder is copied to set power
    setCurrent = setPower/volts;                              // to maintain power p=v*i,voltage is constant,power has to constant and only one variable is used to maintain power is current.
    setControlCurrent = 44+((setCurrent-1)*18);
    controlVoltage = setControlCurrent;                       // for dac to opertae mosfet
}
/* ----------------------Select Constant current------------------------------------ */
if ( Mode == "CR" ) {                                         // if mode is constant resistance
    setResistance = reading;                                  // reading take through rotory encoder is copied to set resistance
    setCurrent = (volts/setResistance);                       // i=v/r(ohms law)
    setControlCurrent = 44+((setCurrent-1)*18);
    controlVoltage = setControlCurrent;                       // for dac to opertae mosfet
}
/* ----------------------selection of brightness button------------------------------------ */
if ( Mode == "BR" ) {
setbright = reading;
    if( brightsw == 1 ) {
    sensVal= map(setbright, 0, 50, 150, 0);                  // 0 to 50 values fro encoder is forcefully mapped in 150 to 0
    analogWrite(bright,sensVal);                             // write is for brightness of lcd is to manintain selected sensval value to pin bright
    }
}
}

/* ----------------------Select dac values to control output load------------------------------------ */
void daccontrol() {
if ( !toggle ) {
    digitalWrite(dac,LOW);                                   // set DAC output voltage to 0 if Load Off selected
} else {
    analogWrite(dac,controlVoltage);                         // set DAC output voltage for Range selected
    }
}

/* --------------------cursor position is change(0 to 50)------------------------------------------------ */
void encoderdisplay() {
    lcd.setCursor(13,1);                                    // start position of setting cusor position (indicates which digit will change)
if ( reading < 10 ) {
    lcd.print("0");
    }
lcd.print (reading);
lcd.setCursor(CP,1);
lcd.cursor();  
}

/* ----------------------Limit Maximum Current Setting----------------------------------------- */
void maxcc() {
if ( Mode == "CC" && reading > CurrentCutOff ) {           // Limit maximum Current Setting
    reading = CurrentCutOff;
    encoderPosition = (CurrentCutOff * 1);                 // keep encoder position value at maximum Current Limit
    lcd.setCursor(12,1);
    lcd.print("   ");                                      // 20 spaces to clear last line of LCD 
}
}

/* ----------------------Power Level Cutoff Routine------------------------------------------ */
void maxpow() {
if ( power  > PowerCutOff ) {                                // Check if Power Limit has been exceed
    reading = 1;
    encoderPosition = 1; 
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(3,1);
    lcd.print("LIMITED P");
    //lcd.setCursor(13,1);
    lcd.setCursor(10,1);
    lcd.print("      ");
    toggle = false;                                         // switch Load Off
}
}
/* ----------------------Select Cursor-position------------------------------------ */
void cursorpos(void)  {
    delay(50);                                               //simple key bounce delay  
    CP = CP + 1;
    if ( CP > 14 ) {
    CP = 13;                                                 
    }
    if ( CP == 14 ) {
    factor = 1;
    }
    if ( CP == 13 ) {
    factor = 10;
    }                                          
}

/* ----------------------Select Constant Current------------------------------------ */
void Current(void) {
    Mode = ("CC");
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(3,1);
    lcd.print("Set I = ");
    lcd.setCursor(15,1);
    lcd.print("A");
    CP = 14;
}

/* ----------------------Select Constant Power------------------------------------ */
void Power(void) {
    Mode = ("CP");
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(3,1);
    lcd.print("Set W = ");
    lcd.setCursor(15,1);
    lcd.print("W");
    CP = 13;
}

/* ----------------------- Select Constant Resistance--------------------------------------- */
void Resistance(void) {
    Mode = ("CR");
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(3,1);
    lcd.print("Set R = ");
    lcd.setCursor(15,1);
    lcd.print((char)0xF4);
    lcd.print(" ");
    CP = 14;
}

/* ----------------------load switch toggle------------------------------------ */
void loadsw(void) {
if( toggle ) {
    lcd.setCursor(3,1);
    lcd.print("LOAD Off ");
    digitalWrite(loadled,LOW);
    toggle = !toggle;
} else {
    lcd.setCursor(3,1);
    lcd.print("LOAD On ");
    digitalWrite(loadled,HIGH);
    toggle = !toggle;
    }
delay(200);
}

/* ----------------------Select brightness------------------------------------ */
void brightness(void) {
    if( yes ) {
    Mode = ("BR");
    lcd.setCursor(0,1);
    lcd.print("                ");                              //to clear previous data
    lcd.setCursor(3,1);
    lcd.print("Set BR ");
    lcd.setCursor(15,1);
    lcd.print("%");
    brightsw = 1;
    yes= !yes;
} else {
    Mode = ("  ");
    lcd.setCursor(3,1);
    lcd.print("            ");
    lcd.setCursor(15,1);
    lcd.print(" ");
    brightsw = 0;
    yes = !yes;
    }
}

/* ----------------------Select switch values with funtion------------------------------------ */
void readswitch() {
    sel = analogRead(sw);                                   // read the switch values  

if ( sel < 100 && sel > 0 ) {
    delay(100);
    digitalWrite(loadled,LOW);
    toggle = false;                                         // switch Load OFF
    encoderPosition =0;
    Current();                                              // call current funtion to print on display
} else
if ( sel > 100 && sel < 199 ) {
    delay(200);
    digitalWrite(loadled,LOW);
    toggle = false;                                        // switch Load OFF
    encoderPosition = 10;
    Power();                                               // call power funtion to print on display
} else
if( sel > 200 && sel < 250 ) { 
    delay(200);
    digitalWrite(loadled,LOW);
    toggle = false;                                        // switch Load OFF
    encoderPosition = 0;
    Resistance();                                          // call resistance funtion to print on display
} else
if( sel > 300 && sel < 340 ) {
    delay(300);
    cursorpos();                                           // call cursorposition funtion to change position of cursor on display
} else
if( sel > 400 && sel < 699 ) {
    delay(100);
    loadsw();                                             // call load switch funtion to on/off the load and print on display
} else
if( sel >= 700 && sel < 900 ) {
    delay(100);
    digitalWrite(loadled,LOW);
    toggle = false;                                       // switch Load OFF
    encoderPosition = 50;
    brightness();                                         // call brightness funtion to control pwm signal by which brightness means to control
}
}

/* ----------------------Select average values------------------------------------ */
int read_adc(int channel) {
    float sum = 0;
    float temp=0;
    int i;
    for (i=0; i<AVG_NUM; i++) {                         // loop through reading raw adc values AVG_NUM number of times  
    temp = analogRead(channel);                         // read the input pin  
    sum += temp;                                        // store sum for averaging
    delay(5);                                           // pauses for microseconds  
    }
    return(sum / AVG_NUM);                              // divide sum by AVG_NUM to get average and return it
}

/* ----------------------Select adc values of current power voltage------------------------------------ */
void read_data(void) {
    amps =    (read_adc(amp) * 0.0049 * 1.45);          // input of  amps
    volts =  (read_adc(volt) * 0.004887585 * 10.42);    // input of  volts  
    //power = amps * volts ;                            // calculations of  watts                  
}

/* ----------------------Select display voltage,power and current------------------------------------ */
void lcd_display() {
    lcd.setCursor(0,0);
    /* completely ignore this part because of my current sensor wire get soo hot. i will improve current sensor */
    if ( amps > 1.52 && amps < 1.65 ) {
        amps = 1.99;
        } else if ( amps > 2.01 && amps < 2.30 ) {
            amps = 3.07;
            } else if ( amps >= 2.40 && amps <= 2.52 ) {
                amps = 4.13; 
                } else if ( amps >= 2.89 && amps <= 2.94 ) {
                    amps = 5.12;
                    } else if ( amps >= 3.31 && amps <= 3.35 ) {
                        amps = 6.02;
                        } else if ( amps >= 3.89 && amps <= 3.94 ) {
                            amps = 6.89;
                            } else if ( amps > 4.00 && amps <= 4.35 ) {
                                amps = 7.89; 
                                } else if ( amps > 4.35 && amps <= 5.00 ) {
                                    amps = 8.21; 
                                    }

    lcd.print(amps,2);
    lcd.print("A ");
    lcd.setCursor(6,0);
    lcd.print(volts,2);
    lcd.print("V ");
    lcd.setCursor(11,0);
    lcd.print(" ");
    power = amps * volts ;
    power = round(power);
    lcd.print(power,1);
    lcd.print("W");
    // lcd.noCursor();
}

void setup() { 
    gpioModeSetup();
    gpioInitialState();
    attachInterrupt(digitalPinToInterrupt(pinA), isr, LOW );
    lcdSetup();  
}

void loop() { 
    analogWrite(bright,sensVal);                                       // initial setup for brightness of lcd to maintain in loop , bright is pin number,sensval is pwm value                 
    /*----------------------read and display data------------------------------------*/
    read_data();                                                       // read data from adc volateg and current inputs
    lcd_display();                                                     // display voltage current and power
    readswitch();
    /*----------------------to print present mode------------------------------------*/
    lcd.setCursor(0,1);                                                // col 0 row 1
    lcd.print(Mode);                                                   // just printing the mode is selected by user cc,cp and cr
    /*----------------------to read modes and analogwrite------------------------------------*/
    daccontrol();                                                      // analogwrite dac values to on/off load
    modesdacreading();                                                 // select modes-cc,cp,cr,br
    /*----------------------for encoder------------------------------------*/
    reading = encoderPosition;                                         // read input from rotary encoder
    lastCount = encoderPosition;                                       // store encoder values
    maxcc();                                                           // set maxiumum Current allowed in Constant Current Mode (CC)
    maxpow();                                                          // Check if Power Limit has been exceed
    encoderdisplay();                                                  // display encoder reading on display
}   //loop end

