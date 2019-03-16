#include "mbed.h"
#include "SHA256.h"
#include "string"
#include "sstream"

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
/************************************************************************/
//IO
RawSerial pc(SERIAL_TX, SERIAL_RX);
typedef struct {
  uint32_t type;           /* Tells printing function what type of message this is */
  uint32_t int_package;    /* int message when that type is required */
  float    float_package;  /* float message when that type is required */
  float    float_package_2;
} mail_t;

Mail<mail_t, 16> mail_box;
Queue<void, 8> inCharQ;
/************************************************************************/
//Threads
Thread serial_out_thread(osPriorityNormal);
Thread serial_in_thread(osPriorityNormal);
Thread hashing_thread(osPriorityNormal);
Thread motor_cntrl_thread(osPriorityAboveNormal, 1024);
/************************************************************************/
//Mutexes
Mutex testString_mut;
Mutex key_mut;
/************************************************************************/
//Hashing setup   
SHA256 h;
uint8_t sequence[]={0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,0x20,0x53,0x79,
0x73,0x74,0x65,0x6D,0x73,0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,0x20,0x61,
0x6E,0x64,0x20,0x64,0x6F,0x20,0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,0x74,
0x68,0x69,0x6E,0x67,0x73,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key =(uint64_t*)((int)sequence + 48);
uint64_t new_key = 0;
uint64_t* nonce =(uint64_t*)((int)sequence + 56);
uint8_t hash[32];    
int16_t hash_count = 0;
/************************************************************************/
//Timers & Tickers
Timer hash_rate_print;
/************************************************************************/
//Motor
PwmOut motor_pwm(PWMpin);
float max_speed = 0;
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards
//motor offset
int8_t orState = 0;    //Rotot offset at motor state 0
//motor state track
int16_t stateCount = 0;
/************************************************************************/
//Test Items
/************************************************************************/

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
 
void mailbox_add(uint32_t type, uint32_t int_package = 0, float float_package = 0.0, float float_package_2 = 0.0){
    mail_t *mail = mail_box.alloc();
    
    mail->type = type;           
    mail->int_package = int_package;       
    mail->float_package = float_package;  
    mail->float_package_2 = float_package_2;
    
    mail_box.put(mail);

}
int8_t intStateOld = 0;
int8_t intState = 0;
void positionChange(){
    intState = readRotorState();
    if (intState != intStateOld) {
        if((intState > intStateOld && !(intState == 5 && intStateOld ==0)) || (intState == 0 && intStateOld == 5)){
            stateCount++;
        }else{
            stateCount--;
        }   
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        intStateOld = intState;
    }
    
    
} 



void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}



void set_key(string input){
    input.erase(0,1);
    
    if(input.length() != 16){
        mailbox_add(4);
    }else{
        stringstream ss;
        
        ss << hex << input;
        
        key_mut.lock();
        ss >> new_key;
        key_mut.unlock();
        
        mailbox_add(3);   
    }
}

void set_speed(string input){
    input.erase(0,1);
    stringstream ss;
    float speed;
    ss << input;
    ss >> speed;
    
    if(speed > -1000 && speed < 1000){
        max_speed = speed;
    }else{
        mailbox_add(4);
    }
}

void parse_input(string input){
    switch(input[0]){
        case 'R' :
            mailbox_add(2);
            break;
        case 'V' :
            set_speed(input);
            break;
        case 'K' :
            set_key(input);
            break;
        case 'T' :
            mailbox_add(2);
            break;
        default :
            mailbox_add(2);
    }
}


void message_input_thread(){
    pc.attach(&serialISR);
    string temp = "";
    while(1){
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if(newChar != '\r'){
            temp.push_back(newChar);
        }else{
            parse_input(temp);
            temp = "";
        }
    }
} 

void message_print_thread(){
    //Runs in its own thread, and is the only part of the program that can write to serial.
    while (true) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            
            switch(mail->type){
                case 0: //Test message
                    pc.printf("%i\r\n", mail->int_package);
                    break;
                case 1: //Startup message
                    pc.printf("--------------- Starting Up ---------------\r\n");
                    break;
                case 2: //Input command error
                    pc.printf("Unknown Command\r\n");
                    break;
                case 3: //Prints the current key being used
                    pc.printf("New Key Set: %llx\r\n", new_key);
                    break;
                case 4: //Input command format error
                    pc.printf("Command Formatting Error\r\n");
                    break;
                case 5: //Prints the current nonce
                    pc.printf("Nonce: %llx\r\n", *nonce);
                    break;
                case 6: //Prints current hash count
                    pc.printf("Hash Rate: %i\r\n", mail->int_package);
                    break;
                case 7:
                    pc.printf("Velocity: %.3f Torque: %.3f\r\n", mail->float_package_2, mail->float_package);
                    //pc.printf("Velocity: %.3f\r\n", mail->float_package);
                    break;
                default:
                    pc.printf("Unknown type message added to print queue. Please ensure corresponding case is in the message_print_thread function.\r\n");
            }
            mail_box.free(mail);
        }
    } 
}
void perform_hashing_thread(){
    while(1){
        key_mut.lock();
        *key = new_key;
        key_mut.unlock();
        
        h.computeHash(hash, sequence, 64);
        hash_count++;
        
        if(hash[0] == 0 && hash[1] == 0){
            mailbox_add(5);
        }
        
        ++*nonce;
        if(hash_rate_print.read_ms() >= 1000){
            //mailbox_add(6, hash_count);
            hash_count = 0;
            hash_rate_print.reset();
        }
    }
    
    
}
void motorControlTick(){
    motor_cntrl_thread.signal_set(0x1);
}

void motor_controller_thread(){
    Ticker motorControlTicker;
    motorControlTicker.attach_us(&motorControlTick,100000);
    int iteration = 0;
    int last_pos = 0;
    int current_pos;
    float velocity = 0.0;
    float proportion = 25;
    while(1){
        motor_cntrl_thread.signal_wait(0x1);
        
        core_util_critical_section_enter();
        current_pos = stateCount;
        core_util_critical_section_exit();
        
        velocity = (current_pos - last_pos) * 10 / 6;
        
        
        /**** Controller Here ****/
        float diff = max_speed - velocity;
        float T = diff * proportion;
        /************************/
        
        float pwm;
        if(T < 0){
            lead = -2;
            pwm *= -T;
        }else{
            lead = 2;
            pwm = T;
        }
        if(pwm > 1){
            pwm = 1;
        }
        
        motor_pwm.write(pwm);
        
        if(iteration++ == 9){
            iteration = 0;
            mailbox_add(7, 0, T, velocity);
        }
        last_pos = current_pos;
    }
}

int main() {    
    /**** Motor Setup ****/
    motor_pwm.period(2e-3);
    motor_pwm.write(1);
    //Run the motor synchronisation
    orState = motorHome();
    I1.fall(&positionChange);
    I2.fall(&positionChange);
    I3.fall(&positionChange);
    I1.rise(&positionChange);
    I2.rise(&positionChange);
    I3.rise(&positionChange);
    
    /**** Initialise Threads ****/
    serial_out_thread.start(message_print_thread);
    serial_in_thread.start(message_input_thread);
    hashing_thread.start(perform_hashing_thread);
    motor_cntrl_thread.start(motor_controller_thread);
    
        
    /**** Initialise Timers ****/        
    hash_rate_print.start();

    /**** Startup Message ****/
    mailbox_add(1);



    while(1){};
}
















    






