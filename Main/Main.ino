#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define ROWS 2
#define COLS 40

const double L1 = 6.5;
const double L2 = 6.5;
const double pi = 3.14159;
const double rad_to_degree = 180.0/pi;
double alpha,beta,gamma,c1,s1,c2,s2,q2,q1,x_foot,y_foot;
double path_1[ROWS][COLS];
unsigned long working_time = 0;

void backward_motion(void);
void direct_kinematic(double alpha, double gamma, double beta, double* x, double* y, double* z );
void inverse_kinematic(double x, double y, double z, double* alpha, double* beta, double* gamma );

void front_right(double x,double y);
void front_left(double x,double y);
void rear_right(double x,double y);
void rear_left(double x,double y);

void forward_motion(void);
void backward_motion(void);
void straight_shoulders(void);
void throtting_CCW (void);
void throtting_CW (void);
void up(double H);

void LinearCartesianPath(double x1, double y1, double x2, double y2,double x3, double y3, double x4, double y4, double traj[ROWS][COLS]);


void setup() {

  delay(3000);

  pwm.begin();
  pwm.setPWMFreq(60);  
  straight_shoulders();
  //LinearCartesianPath(1, 11, 0, 10,-1, 9, -1, 11 , path_1); 
  LinearCartesianPath(2, 11, 0, 10,-1, 9, -1, 11 , path_1); 
}

void loop() {
  working_time = millis();



if(working_time < 4000){throtting_CCW();
  }else if(working_time < 7000){straight_shoulders();up(11);}
  else if(working_time < 10000){throtting_CW();}
  else if(working_time < 13000){straight_shoulders();up(11);}
  else if(working_time < 18000){straight_shoulders();forward_motion();}

}

