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

    //求多边形重心,线段中点 //Finding the center of gravity of a polygon
    vector Barycenter_G( vector Poss[] )
    {   
        vector V;
        
        int PossLen;

        float Sum_V_X = 0;
        float Sum_V_Y = 0;
        float Sum_V_Z = 0;
        
        PossLen = len(Poss);

        foreach( vector v ;  Poss ){

            Sum_V_X += v.x;
            Sum_V_Y += v.y;
            Sum_V_Z += v.z;

        }

        V.x = Sum_V_X / PossLen;
        V.y = Sum_V_Y / PossLen;
        V.z = Sum_V_Z / PossLen;

        return V;

    }

    

#endif
