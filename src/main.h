#define MPU_ARDUINO 1
#define ARDUINO_TESTBENCH 1
//#define CUSTOM_MILLIS
#define TIMER0_PRESCALER 0x2 // 8
#define TIMER2_PRESCALER 0x1 // 1

#define BUZZER_LOW 80
#define BUZZER_MED 150
#define BUZZER_HIGH 220


#if MPU_ARDUINO == 1
  #define MPU_CPU 16000000L
#else
  #define MPU_CPU 14745600L
#endif 

uint8_t dynamicBuzzState();

void disableBuzz(); //Disconnects OC0A

void enableBuzz(); //Connects OC0A


