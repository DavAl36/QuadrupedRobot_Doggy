void up(double H){

   front_right(0,H);
   front_left( 0,H);
   rear_right( 0,H);
   rear_left(  0,H);
  }

void straight_shoulders(void){
  /*
  12 front left
  8 front right
  4 rear left
  0 rear right
  */
  pwm.setPWM(12, 0, angleToPulse(110,90,480) ); // UP > 110 | DOWN < 110
  pwm.setPWM(8, 0, angleToPulse(110,90,480) );  // UP < 110 | DOWN > 110
  pwm.setPWM(4, 0, angleToPulse(90,90,480) );   // UP <  90 | DOWN >  90
  pwm.setPWM(0, 0, angleToPulse(90,90,480) );   // UP >  90 | DOWN <  90  

}
  void throtting_CCW (void)
  {
  
  pwm.setPWM(12, 0, angleToPulse(120,90,480) ); // UP > 110 | DOWN < 110
  pwm.setPWM(8, 0, angleToPulse(100,90,480) );  // UP < 110 | DOWN > 110
  pwm.setPWM(4, 0, angleToPulse(80,90,480) );   // UP <  90 | DOWN >  90
  pwm.setPWM(0, 0, angleToPulse(100,90,480) );   // UP >  90 | DOWN <  90 
  front_right(0,12);
  front_left( 0,11);
  rear_right( 0,11);
  rear_left(  0,12);
  delay(80);
  pwm.setPWM(12, 0, angleToPulse(100,90,480) ); // UP > 110 | DOWN < 110
  pwm.setPWM(8, 0, angleToPulse(120,90,480) );  // UP < 110 | DOWN > 110
  pwm.setPWM(4, 0, angleToPulse(100,90,480) );   // UP <  90 | DOWN >  90
  pwm.setPWM(0, 0, angleToPulse(80,90,480) );   // UP >  90 | DOWN <  90
  front_right(0,11);
  front_left( 0,12);
  rear_right( 0,12);
  rear_left(  0,11);
  delay(80);
  
  }
void throtting_CW (void)
  {
  pwm.setPWM(12, 0, angleToPulse(100,90,480) ); // UP > 110 | DOWN < 110
  pwm.setPWM(8, 0, angleToPulse(120,90,480) );  // UP < 110 | DOWN > 110
  pwm.setPWM(4, 0, angleToPulse(100,90,480) );   // UP <  90 | DOWN >  90
  pwm.setPWM(0, 0, angleToPulse(80,90,480) );   // UP >  90 | DOWN <  90
  front_right(0,12);
  front_left( 0,11);
  rear_right( 0,11);
  rear_left(  0,12);
  delay(80);
  pwm.setPWM(12, 0, angleToPulse(120,90,480) ); // UP > 110 | DOWN < 110
  pwm.setPWM(8, 0, angleToPulse(100,90,480) );  // UP < 110 | DOWN > 110
  pwm.setPWM(4, 0, angleToPulse(80,90,480) );   // UP <  90 | DOWN >  90
  pwm.setPWM(0, 0, angleToPulse(100,90,480) );   // UP >  90 | DOWN <  90 
  front_right(0,11);
  front_left( 0,12);
  rear_right( 0,12);
  rear_left(  0,11);
  delay(80);
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
   