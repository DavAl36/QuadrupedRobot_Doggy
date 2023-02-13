
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void up(double H);

void backward_motion(void);
void direct_kinematic(double alpha, double gamma, double beta, double* x, double* y, double* z );
void inverse_kinematic(double x, double y, double z, double* alpha, double* beta, double* gamma );

void front_right(double x,double y);
void front_left(double x,double y);
void rear_right(double x,double y);
void rear_left(double x,double y);

void forward_motion(void);
void backward_motion(void);

#define ROWS 2
#define COLS 40
void LinearCartesianPath(double x1, double y1, double x2, double y2,double x3, double y3, double x4, double y4, double traj[ROWS][COLS]);
const int L1 = 6.5;
const int L2 = 6.5;
const double pi = 3.14159;
const double rad_to_degree = 180.0/pi;

double alpha;
double beta;
double gamma;
double c1,s1,c2,s2,q2,q1,x_foot,y_foot;
int t = 250;
int ang_2=45;

double path_1[ROWS][COLS];


unsigned long working_time = 0;

void setup() {


  pwm.begin();
  pwm.setPWMFreq(60);  

  delay(3000);
  
  pwm.setPWM(12, 0, angleToPulse(110,90,480) );
  pwm.setPWM(8, 0, angleToPulse(110,90,480) );
  pwm.setPWM(4, 0, angleToPulse(90,90,480) );
  pwm.setPWM(0, 0, angleToPulse(90,90,480) );

  /*
  //tilted to the left
  pwm.setPWM(12, 0, angleToPulse(90,90,480) );
  pwm.setPWM(8, 0, angleToPulse(110,90,480) );
  pwm.setPWM(4, 0, angleToPulse(110,90,480) );
  pwm.setPWM(0, 0, angleToPulse(90,90,480) );
  //tilted to the left
  pwm.setPWM(12, 0, angleToPulse(110,90,480) );
  pwm.setPWM(8, 0, angleToPulse(130,90,480) );
  pwm.setPWM(4, 0, angleToPulse(90,90,480) );
  pwm.setPWM(0, 0, angleToPulse(70,90,480) );
  */
//LinearCartesianPath(2, 7, 2, 5,-1, 4, -1, 7, path_1); 
//LinearCartesianPath(1, 7, 0, 6,-1, 5, -1, 7, path_1); 
LinearCartesianPath(1, 11, 0, 10,-1, 9, -1, 11 , path_1); 

up(5);
delay(1000);
up(6);
delay(1000);
up(7);
delay(1000);
up(8);
delay(1000);
up(9);
delay(1000);
up(10);
delay(1000);
up(11);
delay(1000);

}

void loop() {
  working_time = millis();

  pwm.setPWM(12, 0, angleToPulse(110,90,480) );
  pwm.setPWM(8, 0, angleToPulse(110,90,480) );
  pwm.setPWM(4, 0, angleToPulse(90,90,480) );
  pwm.setPWM(0, 0, angleToPulse(90,90,480) );

  
  if(working_time < 16000){
    forward_motion();
  }
  

}



int angleToPulse(int ang, int SERVOMIN, int SERVOMAX){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);
   return pulse;
}
 void direct_kinematic(double alpha, double gamma, double beta, double* x, double* y, double* z ){
     double diff = gamma-beta;
     *x = L1*sin(beta)+L2*sin(diff);
     *y = sin(alpha)*(L1*cos(beta)+L2*cos(diff));
     *z = cos(alpha)*(L1*cos(beta)+L2*cos(diff));
     return;
  }
  
 void inverse_kinematic(double x, double y, double z, double* alpha, double* beta, double* gamma ){

     double h = -sqrt(pow(z,2)+pow(y,2));
     double c = sqrt(pow(x,2)+pow(z,2));
     double D1 = atan2(x,z);
     double D2 = acos((pow(c,2)+pow(L1,2)-pow(L2,2))/(2*c*L1));
     
     *alpha = -atan2(y,z) + pi;
     *beta = D1 + D2;
     *gamma = acos((pow(L1,2)+pow(L2,2)-pow(c,2))/(2*L1*L2));
     return;
  }
  
void up(double H){

   front_right(0,H);
   front_left( 0,H);
   rear_right( 0,H);
   rear_left(  0,H);
  }

void forward_motion(void){
    int c2 = 20;
    for(int c = 0;c<40;c++){
      if(c2 == 39){c2 =0;}
      else{c2++;}
      front_left( path_1[0][c],path_1[1][c]);
      rear_right( path_1[0][c],path_1[1][c]);
      front_right( path_1[0][c2],path_1[1][c2]);
      rear_left( path_1[0][c2],path_1[1][c2]);
      }
} 
void backward_motion(){
    int c2 = 20;
    for(int c = 40;c>0;c--){
      if(c2 == 0){c2 = 39;}
      else{c2--;}
      front_left( path_1[0][c],path_1[1][c]);
      rear_right( path_1[0][c],path_1[1][c]);
      front_right( path_1[0][c2],path_1[1][c2]);
      rear_left( path_1[0][c2],path_1[1][c2]);
      }
  }
    
void front_right(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 = sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*rad_to_degree;
   q2 = q2*rad_to_degree;
   beta  = map(  q1 ,0, 180, 35,170);
   gamma = map(  q2 ,130, 0, 30,170);  
   pwm.setPWM(11, 0, angleToPulse(gamma+15,0,420) ); // offset
   pwm.setPWM(10, 0, angleToPulse(beta,0,675) );  
   }
   
void rear_right(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 = sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*rad_to_degree;
   q2 = q2*rad_to_degree;
   beta  = map(  q1 ,0, 180, 35,170);
   gamma = map(  q2 ,130, 0, 30,170);  
   pwm.setPWM(3, 0, angleToPulse(gamma,0,420) );  
   pwm.setPWM(2, 0, angleToPulse(beta,0,675) );
   } 
    
void front_left(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 = sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*rad_to_degree;
   q2 = q2*rad_to_degree;
   beta  = map(  q1 ,180, 0, 30,170);
   gamma = map(  q2 ,0, 130, 30,170); 
   pwm.setPWM(15, 0, angleToPulse(gamma+15,0,420) );  // offset          
  pwm.setPWM(14, 0, angleToPulse(beta-10,0,675) );  // offset           
   }
   
void rear_left(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 = sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*rad_to_degree;
   q2 = q2*rad_to_degree;
   beta  = map(  q1 ,180, 0, 30,170);
   gamma = map(  q2 ,0, 130, 30,170); 
   pwm.setPWM(7, 0, angleToPulse(gamma,0,420) );             
   pwm.setPWM(6, 0, angleToPulse(beta,0,675) );             
   }

void LinearCartesianPath(double x1, double y1, double x2, double y2,double x3, double y3, double x4, double y4, double traj[ROWS][COLS])
{
  double diff_x = abs(x2-x1)/10;
  double diff_y = abs(y2-y1)/10;
  for(int counter = 0; counter <=9; counter++){
     traj[0][counter] = x1 - diff_x*counter;  
     traj[1][counter] = y1 - diff_y*counter;  
  }
  diff_x = abs(x3-x2)/10;
  diff_y = abs(y3-y2)/10;
  for(int counter = 10; counter <=19; counter++){
     traj[0][counter] = x2 - diff_x*(counter-10);  
     traj[1][counter] = y2 - diff_y*(counter-10);  
  }
  diff_x = abs(x4-x3)/10;
  diff_y = abs(y4-y3)/10;
  for(int counter = 20; counter <=29; counter++){
     traj[0][counter] = x3 + diff_x*(counter-20);  
     traj[1][counter] = y3 + diff_y*(counter-20);  
  }  
  diff_x = abs(x1-x4)/10;
  diff_y = abs(y1-y4)/10;
  for(int counter = 30; counter <=39; counter++){
     traj[0][counter] = x4 + diff_x*(counter-30);  
     traj[1][counter] = y4 + diff_y*(counter-30);  
  }
  
}
