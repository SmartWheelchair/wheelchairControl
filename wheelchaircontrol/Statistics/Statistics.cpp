
#include "Statistics.h"
#include "mbed.h"

    /**************************************************************************
     * This function initializes the private members of the class Statistics  *
     **************************************************************************/
    Statistics::Statistics(int* Input, double dataLengthIn, int firstDataPointIn){
        data = Input;
        dataLength = dataLengthIn;
        firstDataPoint = firstDataPointIn;
        }
        
    /**************************************************************************
     * This function returns the mean(average) of an array.                   *
     **************************************************************************/
    double Statistics::mean(){
        double sum;
        for(int i = firstDataPoint; i < (firstDataPoint + dataLength); ++i)
        {
            sum += data[i];
        }
        //printf("sum %f, data length %f\r\n", sum, dataLength);
        wait(.001);
        double average = sum/dataLength;
        return average;
    }
    
    /**************************************************************************
     * This function returns the standard deviation of an array. This is      *
     * used to determine whether the data is valid or not.                    *
     **************************************************************************/
    double Statistics::stdev(){
        double sum = 0.0, mean, standardDeviation = 0.0;
        //printf("The length of the array is %d\n", dataLength);
        //double num = (pow(2.0,2.0));
        //printf("2^2 is %f\n", num);
        //int i;

        for(int i = firstDataPoint; i < (firstDataPoint + dataLength); ++i)
        {
            sum += data[i];
            //printf("%d\n", data[i]);
        }
    
        mean = sum/dataLength;
    
        for(int i = firstDataPoint; i < (firstDataPoint + dataLength); ++i) {
            //standardDeviation += pow(data[i] - mean, 2);
            standardDeviation += (data[i] - mean)*(data[i] - mean);
            }
    
        return sqrt(standardDeviation / dataLength);
    }