#include "mbed.h"
#include "SHA256.h"
#include "string"
#include "sstream"
#include "inttypes.h"
#include "locale"

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards

//motor offset
int8_t orState = 0;    //Rotot offset at motor state 0

//Hash count
int16_t hashCount = 0;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

Ticker hash_report;

SHA256 h;

//Hasing setup   
uint8_t sequence[]={0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,0x20,0x53,0x79,
0x73,0x74,0x65,0x6D,0x73,0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,0x20,0x61,
0x6E,0x64,0x20,0x64,0x6F,0x20,0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,0x74,
0x68,0x69,0x6E,0x67,0x73,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint64_t* key =(uint64_t*)((int)sequence + 48);
uint64_t* nonce =(uint64_t*)((int)sequence + 56);
uint8_t hash[32];    

//Serial Setup
RawSerial pc(SERIAL_TX, SERIAL_RX);
volatile char last_recieved = '\0';

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}
 
void positionChange(){
    
    int8_t intState = 0;
    int8_t intStateOld = 0;
    
    intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
            //pc.printf("%d\n\r",intState);
        }
} 

void runRotate(){}
void maxSpeed(){
    //Maximum speed: V\d{1,3}(\.\d{1,3})?
    
}
void setKey(char arr_key[50]){
    
    string str_key = "";
    str_key.push_back(arr_key[1]);
    str_key.push_back(arr_key[2]);
    str_key.push_back(arr_key[3]);
    str_key.push_back(arr_key[4]);
    str_key.push_back(arr_key[5]);
    str_key.push_back(arr_key[6]);
    str_key.push_back(arr_key[7]);
    str_key.push_back(arr_key[8]);
    str_key.push_back(arr_key[9]);
    str_key.push_back(arr_key[10]);
    str_key.push_back(arr_key[11]);
    str_key.push_back(arr_key[12]);
    str_key.push_back(arr_key[13]);
    str_key.push_back(arr_key[14]);
    str_key.push_back(arr_key[15]);
    str_key.push_back(arr_key[16]);
    
    pc.printf("%s", str_key);
    
    istringstream iss(str_key);
    iss >> *key;   
}

void playTune(char tune[]){
    /*
     * UNTESTED 
     * T([A-G][#\^]?[1-8]){1,16}
     * Parser replaces repeated notes with the same note again.
     * Simplifies by denoting flats as lower case, naturals as upper
     * Replaces sharps with flats.
     * Longer note is denoted by '-' for as many repitions is needed
     * All 'empty' items in the tune array are shown by a '.'.
     */
    char parsed_tune[128];
    int parsed_pointer = 0;
    int parsing_pointer = 0;
    
/*    while(parsing_pointer < tune.size()){
        char temp = tune[parsing_pointer++];
        if(tune[parsing_pointer] == '#'){
            switch(temp){
                case 'A' :
                    temp = 'b';
                    break;
                case 'B' :
                    temp = 'C';
                    break;
                case 'C' :
                    temp = 'd';
                    break;
                case 'D' :
                    temp = 'e';
                    break;
                case 'E' :
                    temp = 'F';
                    break;
                case 'F' :
                    temp = 'g';
                    break;
                case 'G' :
                    temp = 'a';
                    break;
            }
            parsing_pointer++;
        }else if(tune[parsing_pointer] == '^'){
            switch(temp){
                case 'A' :
                    temp = 'a';
                    break;
                case 'B' :
                    temp = 'b';
                    break;
                case 'C' :
                    temp = 'B';
                    break;
                case 'D' :
                    temp = 'd';
                    break;
                case 'E' :
                    temp = 'e';
                    break;
                case 'F' :
                    temp = 'E';
                    break;
                case 'G' :
                    temp = 'g';
                    break;
            }
            parsing_pointer++;
        }
        parsed_tune[parsed_pointer++] = temp;
        int repititions = tune[parsing_pointer++] - '0';
        
        for(int i = 0; i < repititions - 1; i++){
            parsed_tune[parsed_pointer++] = '-';
        }
    }
    //parsed_tune now has the parsed music, do something with it here*/
}

void parseCommand(){
    /*
     * Required commands:
     * Rotation: R-?\d{1,3}(\.\d{1,2})?
     * Maximum speed: V\d{1,3}(\.\d{1,3})?
     * Bitcoin key: K[0-9a-f]{16}
     * Tune: T([A-G][#\^]?[1-8]){1,16}
     *
     */
    int index = 0;
    char c = pc.getc();
    char command[50];
    while(c != '\n'){
        command[index] = c;
        index++;
        c = pc.getc();
    }
    
    /********** UNTESTED *********
    switch(command[0]) {
        case 'R' : 
            setKey(runRotate);
            break;
        case 'V' : 
            maxSpeed(command);
            break;
        case 'K' : 
            setKey(command);
            break;
        case 'T' : 
            playTune(command);
            break;
        default : 
            break;
    }
    /********** UNTESTED **********/
     
}
 
    
//Main
int main() {
    pc.attach(&parseCommand);
    
    pc.printf("\n\rHello");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("\n\rRotor origin: %x",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    I1.fall(&positionChange);
    I2.fall(&positionChange);
    I3.fall(&positionChange);
    
    Timer t;

    t.start();
    while (1) {
        h.computeHash(hash, sequence, 64);
        hashCount++;
        if( hash[0] == 0 && hash[1] == 0){
            pc.printf("\n\rNonce: ");
            for(int i = 63; i >= 56; i--){
                pc.printf("%02x", sequence[i]);
            }
        }
        ++*nonce;        
        if(t.read_ms() >= 1000){
            //pc.printf("\n\rHash rate: ");
            //pc.printf("%i", hashCount);
            hashCount = 0;   
            t.reset();
            
            string str = "";
            ostringstream o;
            o << key;
            str += o.str();
            pc.printf("\n\rKey: ");
            /* Printing of key doesn't currently work */
            pc.printf("%PRIu64 ", *key );
        }
    }
}























