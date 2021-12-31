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
    matrix3 Rotate_Cus(float amount)
    {
        matrix3 ident = maketransform({0,0,1},{0,1,0});
        float angle = amount*360;
        rotate(ident,radians( angle ),{0,1,0});

        return ident;
    }

    //设置点的旋转值 //setpointattrib orient
    void RotateCusSetOrient(float amount ; int ptnum)
    {   
        //模型Y轴竖直向上时，使用{1,0,0},{0,0,1}作为正交基
        matrix3 ident = maketransform({1,0,0},{0,0,1});
        float angle = amount*360;
        rotate(ident,radians( angle ),{0,1,0});

        setpointattrib(0, "orient", ptnum , ident , "set");  
    }
    
    //获取多边形重心,线段中点的坐标 //Finding the center of gravity of a polygon
    vector Barycenter_Cus( vector Poss[] )
    {
        vector V;

        V = Barycenter_G( Poss );

        return V;

    }

    //从法线生成随机贝塞尔23控制点。Seed小于0的时候（例如-1）不进行随机干扰。
    vector[] CreateBezierCentrolPoint( int ptnum1 ; int ptnum4 ; float Seed )
    {
        //23点生成。
        vector N1,N4,P1,P2,P3,P4;
        vector ReV[];
        int Ptnum2,Ptnum3;
        float ifNoise = 1;
        float Dis = 0;
        
        P1 = point(0,"P",ptnum1);
        P4 = point(0,"P",ptnum4);

        N1 = point(0,"N",ptnum1);
        N4 = point(0,"N",ptnum4);

        float noise1 = BaseNoise_Cus(Seed , 3000 , 2 , 1);
        float noise4 = BaseNoise_Cus(Seed , 3000 , 2 , 27);
        
        //当Seed小于0的时候设置控制点1和控制点4之间的距离为1-2，2-3之间的距离
        if( Seed < 0 )
        {
            ifNoise = 0;
            Dis = distance(P1,P4);
        }

        P2 = P1 + ( N1 * ( noise1 * 0.5 + 0.5 ) ) * ifNoise + N1 * Dis * 2000; 
        P3 = P4 - ( N4 * ( noise4 * 0.5 + 0.5 ) ) * ifNoise + N4 * Dis * 2000; 

        Ptnum2 = addpoint(0,P2);
        Ptnum3 = addpoint(0,P3);

        append(ReV,P1); 
        append(ReV,P2); 
        append(ReV,P3); 
        append(ReV,P4); 

        return ReV;

    }


#endif