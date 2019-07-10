#ifndef STATISTICS_h               
#define STATISTICS_h              
 
#include "mbed.h"

class Statistics 
{ 
public: 
    //Defining default constructor
    Statistics(int* Input, double dataLength, int firstDataPointIn = 0); 
    //Defining methods
    double stdev();
    double mean();
    
  private:
   int* data;                                       //pointer to data
   double dataLength;
   int firstDataPoint;

    
};

#endif   //STATISTICS_h