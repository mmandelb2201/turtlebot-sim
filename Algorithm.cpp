#include <iostream>
#include <math.h>




using namespace std;

int xRule1, xRule2,xRule3, xRule4,yRule1,yRule2,yRule3,yRule4,xindexControl,yindexControl, weightSum, weight,indexControl, bot1x, bot1y, bot2x, bot2y


// void spider(int _rule)
// {
//     for (int i = 0; i<)
// }
void rule_1() {
    //spider reset
    //rule coords reset
    
    xRule1 = 0;
    yRule1 = 0;
    
    //void spider (1); // I think it would be a void function
   
    if (weightSum =! 0) {
        xRule1 = (0 - xRuleCoord) / weightSum;
        yRule1 = (0 - yRuleCoord) / weightSum;
        
    }

    else {
        xRule1 = 0;
        yRule1 = 0;
        
    }
}   
int rule_2() {
    //spider reset
    
    xRule2 = 0;
    yRule2 = 0;
    
    //void spider (2); // I think it would be a void function
   
    if (weightSum =! 0) {
        xRule2 = (xRule2 / weightSum) - (velocity * cos(radDirection));
        yRule2 = (yRule2 / weightSum) - (velocity * sin(radDirection));
    }

    else {
        xRule2 = 0;
        yRule2 = 0;
    }
}   
void rule_3() {
    //spider reset
    //rule coords reset
    
    xRule3 = 0;
    yRule3 = 0;
    
   // void spider (3); // I think it would be a void function
   
    if (weightSum =! 0) {
        xRule3 = xRuleCoord / weightSum;
        yRule3 = yRuleCoord / weightSum;
    }

    else {
        xRule3 = 0;
        yRule3 = 0;
    }
}   
void rule_4() {
    
    xRule4 = 0;
    yRule4 = 0;
    xindexControl = 2;
    yindexControl = 2;

    //might need to do an additiional calculation to find the indexControl of radDirection (xindexControl and yindexControl)

    xRule4 = (((cos(xindexControl + 180) * (LOID_OFFSET * UNIT_LENGTH)) + xindexControl) - xposition) / UNIT_LENGTH;
    yRule4 = (((cos(yindexControl + 180) * (LOID_OFFSET * UNIT_LENGTH)) + yindexControl) - yposition) / UNIT_LENGTH;
}   

int interpret()
{
    float accelX =0;
    float accelY =0;
    float sumMagnitude = 0;

    if (rule_1() == 1)
    {
        float ruleMagnitude = sqrt((xRule1*xRule1)+(yRule1*yRule1));
        if(!(ruleMagnitude ==0))
        {
             if (collison_limit < ruleMagnitude)
             {
                 accelX += MaxAcceleration*(collison_limit/ruleMagnitude)*(xRule1/ruleMagnitude);
                 accelY += MaxAcceleration*(collison_limit/ruleMagnitude)*(yRule1/ruleMagnitude);
                 sumMagnitude += MaxAcceleration*(colison_limit/ruleMagnitude);
             }
             else
             {
                 accelX += MaxAcceleration*(xRule1/ruleMagnitude);
                 accelY += MaxAcceleration*(yRule1/ruleMagnitude);
                 sumMagnitude += MaxAcceleration;
             }
        }
    }
    if(sumMagnitude < MaxAcceleration)
    {
        if(rule_2() ==1 )
        {
           float ruleMagnitude = sqrt((xRule2*xRule2)+(yRule2*yRule2)); 
           if(!(ruleMagnitude ==0))
        { float interpretRule2 = MaxAcceleration/(log(1+(2*MaxVelocity));
             if (!((sumMagnitude+interpretRule2)>MaxAcceleration))
             {
                 accelX += (interpretRule2*(xRule2/ruleMagnitude));
                 accelY += (interpretRule2*(yRule2/ruleMagnitude));
                 sumMagnitude += interpretRule2;
             }
             else
             {
                  accelX += ((MaxAcceleration-sumMagnitude)*(xRule2/ruleMagnitude));
                 accelY += ((MaxAcceleration-sumMagnitude)*(yRule2/ruleMagnitude));
                 sumMagnitude += MaxAcceleration;
             }
        } 
        }
        if(sumMagnitude < MaxAcceleration)
    {
        if(rule_3() ==1 )
        {
           float ruleMagnitude = sqrt((xRule3*xRule3)+(yRule3*yRule3)); 
           if(!(ruleMagnitude ==0))
        {
             if ((ruleMagnitude < CentoridLimit)&&((sumMagnitude+(MaxAcceleration*(ruleMagnitude/CentoridLimit))<MaxAcceleration))
             {
                 accelX += MaxAcceleration*(ruleMagnitude/CentoridLimit)*(xRule3/ruleMagnitude);
                 accelY += MaxAcceleration*(ruleMagnitude/CentoridLimit)*(yRule3/ruleMagnitude);
                 sumMagnitude += MaxAcceleration*(ruleMagnitude/CentoridLimit);
             }
             else
             {
                 accelX += (MaxAcceleration-sumMagnitude)*(xRule3/ruleMagnitude);
                 accelY += (MaxAcceleration-sumMagnitude)*(yRule3/ruleMagnitude);
                 sumMagnitude += MaxAcceleration;
             }
        } 
        }
        if(sumMagnitude < MaxAcceleration)
    {
        if(rule_4() ==1 )
        {
           float ruleMagnitude = sqrt((xRule4*xRule4)+(yRule4*yRule4)); 
           
        
             if ((ruleMagnitude < loidLeash)&&((sumMagnitude+(MaxAcceleration*(ruleMagnitude/loidLeash))<MaxAcceleration))
             {
                 accelX += MaxAcceleration*(ruleMagnitude/loidLeash)*(xRule4/ruleMagnitude);
                 accelY += MaxAcceleration*(ruleMagnitude/loidLeash)*(yRule4/ruleMagnitude);
                 sumMagnitude += MaxAcceleration*(ruleMagnitude/loidLeash);
             }
             else
             {
                 accelX += MaxAcceleration*(xRule4/ruleMagnitude);
                 accelY += MaxAcceleration*(yRule4/ruleMagnitude);
                 sumMagnitude += MaxAcceleration;
             }
        } 
        
    }
    }
    }

}

int main (int x1, int x2,int y1, int y2)
{
    bot1x = x1;
    bot1y = y1;
    bot2x = x2;
    bot2y = y2;
}