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