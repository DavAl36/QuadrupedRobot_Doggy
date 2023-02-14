 int angleToPulse(int ang, int SERVOMIN, int SERVOMAX){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);
   return pulse;
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


