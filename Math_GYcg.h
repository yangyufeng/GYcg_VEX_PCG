#ifndef __Math_GYcg__
#define __Math_GYcg__

#include <math.h>

namespace Math_GYcg {
    
    //基础波动
    float BaseNoise( float seed ; float mul )
    {   
        float x;

        x = frac( sin(seed) * mul ); 

        return x;
    }

}


#endif
