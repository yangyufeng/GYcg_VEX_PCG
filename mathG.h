#ifndef __mathG_h__
#define __mathG_h__

    #include <math.h>

    //基础噪波//BaseNoise
    float BaseNoise_G( float seed ; float mul ; float fre ; float bias )
    {   
        float x;

        x = abs( frac( sin( seed * fre + bias ) * mul ) ) - 0.5; 

        return x;

    }

#endif
