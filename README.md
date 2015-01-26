# cloaked-nemesis
#khep-III C
/* k3_circle_test: application that lets K3 move on a circle with a given radius
 * and speed for an amount of time to be entered in milliseconds. Entering 0
 * will terminate the program.
 * 
 * Default value for speed: 20
 * Default value for radius: 20 cm
 *
 * Alternative values for speed and radius can be specified as command line
 * parameters:
 * ./k3_circle_test radius motspeed
 *
 * i.e. the first parameter will be interpreted as radius, the 
 * second parameter - if present - as speed value. 
 */

/* korebot.h contains korebot-specific definitions */
#include <korebot/korebot.h>
#include <stdlib.h>
/* macro definitions for constants needed below */
#define K3_DIAMETER 13.0
#define K3_IR_THRESHOLD 200
#define K3_TURN_SPEED 30
#define K3_TURN_DUR 250000

/* pointers for interaction with Khepera III devices, initialized in 
 * function main()
 */
static knet_dev_t * mot1;
static knet_dev_t * mot2;
static knet_dev_t * dsPic;

/* function turn() makes K3 turn on the spot:
 *
 * argument = 0: turn left
 * argument != 0: turn right
 */

void turn( int dir){
  switch(dir){
  case 0:  kmot_SetPoint( mot1, kMotRegSpeed, -K3_TURN_SPEED);
           kmot_SetPoint( mot2, kMotRegSpeed, K3_TURN_SPEED);	   
	   break;

  default: kmot_SetPoint( mot1, kMotRegSpeed, K3_TURN_SPEED);
           kmot_SetPoint( mot2, kMotRegSpeed, -K3_TURN_SPEED);
  }

  usleep(K3_TURN_DUR);
  kmot_SetPoint( mot1, kMotRegSpeed, 0);
  kmot_SetPoint( mot2, kMotRegSpeed, 0);
}

  

/* function k3_move_dur()  lets K3 move for time dur specified in ms and 
 * motor speeds m1 1nd m2.
 *
 * A simple form of obstacle avoidance for K3 is implemented:
 * if any of the ir sensors 3, 4, 5, or 6 returns a proximity value
 * exceeding the value defined in macro K3_IR_THRESHOLD, K3 moves 
 * backward 300 ms, turns on the spot away from the obstacle, and 
 * resumes the original motor speed values.
 */

void k3_move_dur(int dur, int m1, int m2){
  double irsensor3 = 0;
  double irsensor4 = 0;
  double irsensor5 = 0; 
  double irsensor6 = 0;
  char Buffer[512];

  /* tell motor controllers to move K3 forward */    
  kmot_SetPoint( mot1, kMotRegSpeed, m1);
  kmot_SetPoint( mot2, kMotRegSpeed, m2);

  dur = 1000*dur;

  while(dur > 0){
    dur -= 50000;
      
    if(kh3_proximity_ir((char *)Buffer, dsPic)){
      irsensor3 = (double)(Buffer[5]|Buffer[6]<<8);
      irsensor4 = (double)(Buffer[7]|Buffer[8]<<8);
      irsensor5 = (double)(Buffer[9]|Buffer[10]<<8);
      irsensor6 = (double)(Buffer[11]|Buffer[12]<<8);
      printf("Sensor3: %4.1f, Sensor 4: %4.1f, Sensor 5: %4.1f, \
             Sensor 6: %4.1f \n", irsensor3, irsensor4, irsensor5, irsensor6);
      
      if(irsensor3 > K3_IR_THRESHOLD || irsensor4 > K3_IR_THRESHOLD ||\
	 irsensor5 > K3_IR_THRESHOLD || irsensor6 > K3_IR_THRESHOLD){
	 system("cp boing1.wav /dev/sound/dsp");
	 kmot_SetPoint( mot1, kMotRegSpeed, -60);
         kmot_SetPoint( mot2, kMotRegSpeed, -60);
	 usleep(500000);
	 turn(irsensor3 > irsensor6);
	 kmot_SetPoint( mot1, kMotRegSpeed, m1);
         kmot_SetPoint( mot2, kMotRegSpeed, m2);
      }
      else{
	usleep(50000);
      }     
    }  
    else{
      printf("error reading proximity sensors!\n");
    }
  }
     
  kmot_SetPoint( mot1, kMotRegSpeed, 0);
  kmot_SetPoint( mot2, kMotRegSpeed, 0);
}


int main (int argc, char *argv[])
{
  /* definitions of default values for variables */
  int motspeed1 = 20;
  int motspeed2 = 20;
  int duration = 1000;
  double radius = 20.0;

  /* the kh3_init() routine is required */
  kh3_init();

  if(argc > 1)
    radius = atof(argv[1]);
  if(argc > 2)
    motspeed1 = atoi(argv[2]);

  printf("radius: %5.1f \n", radius);

/* open various sockets and store the handles in their respective pointers:
 * the motor controllers and the dsPic processor are addressed as independent
 * I2C devices.
 * 
 * mot1 points to the left motor, mot1 to the right.
 */
dsPic = knet_open("Khepera3:dsPic", KNET_BUS_I2C, 0, NULL);
mot1  = knet_open("Khepera3:mot1",  KNET_BUS_I2C, 0, NULL);
mot2  = knet_open("Khepera3:mot2",  KNET_BUS_I2C, 0, NULL);



/* initialize motor controller 1 */
kmot_SetMode( mot1, kMotModeIdle);
kmot_SetSampleTime( mot1, 1550);
kmot_SetMargin( mot1, 6);
kmot_SetOptions( mot1, 0x0, kMotSWOptWindup | kMotSWOptStopMotorBlk |\
		 kMotSWOptDirectionInv);
kmot_ResetError( mot1);
kmot_SetBlockedTime( mot1, 10);
kmot_ConfigurePID( mot1, kMotRegSpeed, 400, 0, 10);
kmot_ConfigurePID( mot1, kMotRegPos, 620, 3, 10);
kmot_SetSpeedProfile( mot1, 30, 3);


/* initialize motor controller 2 */
kmot_SetMode( mot2, kMotModeIdle);
kmot_SetSampleTime( mot2, 1550);
kmot_SetMargin( mot2, 6);
kmot_SetOptions( mot2, 0x0, kMotSWOptWindup | kMotSWOptStopMotorBlk);
kmot_ResetError( mot2);
kmot_SetBlockedTime( mot2, 10);
kmot_ConfigurePID( mot2, kMotRegSpeed, 400, 0, 10);
kmot_ConfigurePID( mot2, kMotRegPos, 620, 3, 10);
kmot_SetSpeedProfile( mot2, 30, 3);


 /* make sure that no division by 0 will occur */
 if (radius < 0.001){
   printf("Radius too small!!\n");
   exit(1);
 }

 /* calculate motspeed2 based on the choice of motspeed1:
  * K3_DIAMETER represents the approximate diameter of K3 (13.0 cm)
  * 'radius' stands for the inner radius of the circle in cm
 */ 
      
   motspeed2 = (int)(motspeed1*(1+K3_DIAMETER/radius));


     /* print momentary speed for motors */
   printf("motspeed1 %u \n", motspeed1);
   printf("motspeed2 %u \n", motspeed2);



   /* infinite loop, interrupted by entering 0 */ 
while(1){
    

  printf("enter duration in ms; entering 0 will terminate program:");
  scanf("%i", &duration);
  printf("\n \n");

  if(duration){  
    printf("duration: %u \n\n", duration);

 
     
    k3_move_dur(duration, motspeed1, motspeed2);
  }
  else{
    printf("bye");
    exit(0);
  }
 }


}
