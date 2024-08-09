#include <pcint.h>
#include <ppm.h>

// six channel

// control drone  
int throttle = 0;
int roll = 0;
int pitch = 0;
int yaw = 0;

//auxiliary control
int aux1 = 0;
int aux2 = 0;
unsigned long previousMillis = 0;

class Control {
  private:
    #define PPM A0
    #define THROTTLE        3
    #define ROLL            1
    #define PITCH           2
    #define YAW             4
    #define INTERVAL        50
    #define POT1            5
    #define POT2            6
  public:
    void startPPM(void){
      ppm.begin(PPM, false);
    };
    void readingPPM(void){
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= INTERVAL){
        previousMillis = currentMillis;
        throttle      =   ppm.read_channel(THROTTLE);
        roll          =   ppm.read_channel(ROLL);
        pitch         =   ppm.read_channel(PITCH);
        yaw           =   ppm.read_channel(YAW);
      }
    };
    void AuxChannel(void){
      aux1         =   ppm.read_channel(POT1);
      aux2         =   ppm.read_channel(POT2);
    }

};