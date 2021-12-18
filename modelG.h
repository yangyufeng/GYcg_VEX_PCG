#ifndef __modelG_h__
#define __modelG_h__

    #include <math.h>
    #include "mathG.h"  

    //基础噪波 //BaseNoise
    float BaseNoise_Cus( float seed ; float mul ; float fre ; float bias )
    {        
        float x;

        x = BaseNoise_G( seed ,  mul , fre , bias ); 

        return x;
    }
 
    //对于一个点，基于当前位置随机新的位置//For a point, random new position based on current position
    vector PosRand3D( float seed ; vector Range ;  vector Pos )
    {   
        Pos.x = Pos.x + BaseNoise_G( seed , 3000.0 , 100 , -500 ) * Range.x;
        Pos.y = Pos.y + BaseNoise_G( seed , 3000.0 , 100 , 170  ) * Range.y;
        Pos.z = Pos.z + BaseNoise_G( seed , 3000.0 , 100 , -240 ) * Range.z;

        return Pos; 
    }

    //对于一个点，基于当前位置随机新的位置( 不影响Y轴 )//For a point, random new position based on current position（in X-Z plan ）
    vector PosRand2D( float seed ; vector Range ;  vector Pos )
    {   
        Pos.x = Pos.x + BaseNoise_G( seed , 3000.0 , 100 , -500 ) * Range.x;
        Pos.z = Pos.z + BaseNoise_G( seed , 3000.0 , 100 , -240 ) * Range.z;

        return Pos;
    }
    
    //对于一个点，基于当前位置随机新的位置( 根据一个向量指定方向 )//For a point, random new position based on current position（ in a direction ）
    vector PosRandByVector( float seed ; float Range ;  vector Pos ; vector vec  )
    {   
        Pos.x = Pos.x + BaseNoise_G( seed , 3000.0 , 100 , -500 ) * Range * vec.x;
        Pos.y = Pos.y + BaseNoise_G( seed , 3000.0 , 100 , 170  ) * Range * vec.y;
        Pos.z = Pos.z + BaseNoise_G( seed , 3000.0 , 100 , -240 ) * Range * vec.z;

        return Pos;
    }
    
    //根据点序号和点的法线随机重置一个点的位置，点的新位置会在法线方向上。//Randomly resets the position of a point based on its ptnum and its N. The new position of the point will be in the normal direction.  
    void PosRandResetByN( float seed ; float Range ; int ptnum )
    {
        vector Pos = point(0,"P", ptnum );
        vector N = point(0,"N", ptnum );
        Pos = PosRandByVector( seed , Range , Pos , N );
        setpointattrib(0,"P", ptnum , Pos , "set" );
    }

    //旋转功能 //rotate
    matrix3 rotate_cus(float amount)
    {
        matrix3 ident = maketransform({0,0,1},{0,1,0});
        float angle = amount*360;
        rotate(ident,radians( angle ),{0,1,0});

        return ident;
    }

    //设置点的旋转值 //setpointattrib orient
    void rotate_cus_setOrient(float amount ; int ptnum)
    {
        matrix3 ident = maketransform({0,0,1},{0,1,0});
        float angle = amount*360;
        rotate(ident,radians( angle ),{0,1,0});

        setpointattrib(0, "orient", ptnum , ident , "set");
    }

    
#endif