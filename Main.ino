
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void up(void);
void down(void);
void backward_motion(void);
 void direct_kinematic(double alpha, double gamma, double beta, double* x, double* y, double* z );
void inverse_kinematic(double x, double y, double z, double* alpha, double* beta, double* gamma );

void front_right(double x,double y);
void front_left(double x,double y);
void rear_right(double x,double y);
void rear_left(double x,double y);
void test_leg_1(void);
void test_leg_2(byte timing);
void forward_motion(void);
void backward_motion(void);
void turn_clockwise_motion(void);
void turn_counterclockwise_motion(void);
#define ROWS 2
#define COLS 40
void LinearCartesianPath(double x1, double y1, double x2, double y2,double x3, double y3, double x4, double y4, double traj[ROWS][COLS]);
const int L1 = 6.5;
const int L2 = 6.5;
const double pi = 3.14159;

double alpha;
double beta;
double gamma;
double c1,s1,c2,s2,q2,q1,x_foot,y_foot;
int t = 250;
int ang_2=45;

double path_1[ROWS][COLS];


  
void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);  

  delay(1000);

   pwm.setPWM(8, 0, angleToPulse(140,0,420) ); 
   pwm.setPWM(12, 0, angleToPulse(140,0,420) );
   pwm.setPWM(0, 0, angleToPulse(115,0,420) );
   pwm.setPWM(4, 0, angleToPulse(115,0,420) );   
   front_right(0,6);
   front_left( 0,6);
   rear_right( 0,11);
   rear_left(  0,11);
   delay(1000);
   up();
   delay(1000);
   front_right(0,11);
   front_left( 0,11);
   rear_right( 0,6);
   rear_left(  0,6);
   delay(1000);
   up();
   delay(1000);
   pwm.setPWM(8, 0, angleToPulse(120,0,420) ); 
   pwm.setPWM(12, 0, angleToPulse(110,0,420) );
   pwm.setPWM(0, 0, angleToPulse(150,0,420) );
   pwm.setPWM(4, 0, angleToPulse(140,0,420) );
   delay(1000);
   pwm.setPWM(8, 0, angleToPulse(160,0,420) ); 
   pwm.setPWM(12, 0, angleToPulse(160,0,420) );
   pwm.setPWM(0, 0, angleToPulse(100,0,420) );
   pwm.setPWM(4, 0, angleToPulse(90,0,420) );
   delay(1000);
   pwm.setPWM(8, 0, angleToPulse(140,0,420) ); 
   pwm.setPWM(12, 0, angleToPulse(140,0,420) );
   pwm.setPWM(0, 0, angleToPulse(125,0,420) );
   pwm.setPWM(4, 0, angleToPulse(120,0,420) );  
   delay(1000);

LinearCartesianPath(2, 7, 2, 5,-1, 4, -1, 7, path_1); 

}

void loop() {
   pwm.setPWM(8, 0, angleToPulse(140,0,420) ); 
   pwm.setPWM(12, 0, angleToPulse(140,0,420) );
   pwm.setPWM(0, 0, angleToPulse(125,0,420) );
   pwm.setPWM(4, 0, angleToPulse(120,0,420) );  
   forward_motion();
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
  
void up(){
   front_right(0,8);
   front_left( 0,8);
   rear_right( 0,8);
   rear_left(  0,8);
  }

void forward_motion(void){
    int c2 = 20;
    for(int c = 0;c<40;c++){
      if(c2 == 39){c2 =0;}
      else{c2++;}
      front_left( path_1[0][c],path_1[1][c]+0.5);
      rear_right( path_1[0][c],path_1[1][c]);
      front_right( path_1[0][c2],path_1[1][c2]+0.5);
      rear_left( path_1[0][c2],path_1[1][c2]);
      }
} 
void backward_motion(){
    int c2 = 20;
    for(int c = 40;c>0;c--){
      if(c2 == 0){c2 = 39;}
      else{c2--;}
      front_left( path_1[0][c],path_1[1][c]+0.5);
      rear_right( path_1[0][c],path_1[1][c]);
      front_right( path_1[0][c2],path_1[1][c2]+0.5);
      rear_left( path_1[0][c2],path_1[1][c2]);
      }
  }
    
void front_right(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 =  sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*180/pi;
   q2 = q2*180/pi;
   beta  = map(  q1 ,0, 180, 35,170);
   gamma = map(  q2 ,130, 0, 30,170);  
   pwm.setPWM(11, 0, angleToPulse(gamma,0,420) ); 
   pwm.setPWM(10, 0, angleToPulse(beta,0,675) );  
   }
   
void rear_right(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 =  sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   beta  = map(  q1 ,0, 180, 35,170);
   gamma = map(  q2 ,130, 0, 30,170);  
   pwm.setPWM(3, 0, angleToPulse(gamma,0,420) );  
   pwm.setPWM(2, 0, angleToPulse(beta,0,675) );
   } 
    
void front_left(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 =  sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*180/pi+30;//la costante serve a tarare
   q2 = q2*180/pi+20;//la costante serve a tarare
   beta  = map(  q1 ,180, 30, 35,170);
   gamma = map(  q2 ,0, 130, 30,170); 
   pwm.setPWM(15, 0, angleToPulse(gamma,0,420) );            
   pwm.setPWM(14, 0, angleToPulse(beta,0,675) );             
   }
   
void rear_left(double x,double y)
   { 
   c2 =(pow(x,2)+pow(y,2)-(pow(L1,2)+pow(L2,2)))/(2*L1*L2);
   s2 =  sqrt(1-pow(c2,2));
   q2 = atan2(s2,c2);
   q1 = atan2(y,x)-atan2(L2*s2,L1+(L2*c2));
   q1 = q1*180/pi;
   q2 = q2*180/pi;
   beta  = map(  q1 ,180, 0, 30,170);
   gamma = map(  q2 ,0, 130, 30,170); 
   pwm.setPWM(7, 0, angleToPulse(gamma,0,420) );             
   pwm.setPWM(6, 0, angleToPulse(beta,0,675) );             
   }

void test_leg_1(void){
  //spalle centrate
   pwm.setPWM(8, 0, angleToPulse(140,0,420) ); 
   pwm.setPWM(12, 0, angleToPulse(140,0,420) );
   pwm.setPWM(0, 0, angleToPulse(115,0,420) );
   pwm.setPWM(4, 0, angleToPulse(115,0,420) );   
  front_right(-6.5,6.5);
  front_left( -6.5,6.5);
  rear_right( -6.5,6.5);
  rear_left(-6.5,6.5);
  delay(1000);
  front_right(6.5,6.5);
  front_left( 6.5,6.5);
  rear_right( 6.5,6.5);
  rear_left(  6.5,6.5);
  delay(1000);
  return;
  }    

void test_leg_2(int timing)
{
   front_right(0,7);
   front_left( 0,7);
   rear_right( 0,9);
   rear_left(  0,9);
   delay(timing);
   front_right(0,3);
   front_left( 0,7);
   rear_right( 0,9);
   rear_left(  0,9);
   delay(timing);
   front_right(0,7);
   front_left( 0,7);
   rear_right( 0,9);
   rear_left(  0,9);
   delay(timing);   
   front_right(0,7);
   front_left( 0,3);
   rear_right( 0,9);
   rear_left(  0,9);
   delay(timing);
   front_right(0,7);
   front_left( 0,7);
   rear_right( 0,9);
   rear_left(  0,9);
   delay(timing);
   front_right(0,7);
   front_left( 0,7);
   rear_right( 0,5);
   rear_left(  0,9);
   delay(timing);
   front_right(0,7);
   front_left( 0,7);
   rear_right( 0,9);
   rear_left(  0,9);
   delay(timing);
   front_right(0,7);
   front_left( 0,7);
   rear_right( 0,9);
   rear_left(  0,5);
   delay(timing);
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
