#ifndef __curvefitting__
#define __curvefitting__

#include <math.h>

    float PaperGames_X3_RoadSystem_Curvature( float Width ; float angle )
    {   

        float Bias;

        Bias = ( Width * 4.193 - Width * 0.91325 ) / ( 1 + pow( angle / 0.8667 , 2.9124 ) ) + Width * 0.91325;

        return Bias;
        
    }

#endif

