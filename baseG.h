#ifndef __base_h__
#define __base_h__

    #include <math.h>
    #include <GYcg/mathG.h>
    
    //对于一个点，基于当前位置随机新的位置( 根据一个向量指定方向 )//For a point, random new position based on current position（ in a direction ）
    vector PosRandByVector( float Seed ; float Range ;  vector Pos ; vector Vec  )
    {   
        Pos.x = Pos.x + BaseNoise_G( Seed , 3000.0 , 100 , -500 ) * Range * Vec.x;
        Pos.y = Pos.y + BaseNoise_G( Seed , 3000.0 , 100 , 170  ) * Range * Vec.y;
        Pos.z = Pos.z + BaseNoise_G( Seed , 3000.0 , 100 , -240 ) * Range * Vec.z;

        return Pos;
    }

#endif