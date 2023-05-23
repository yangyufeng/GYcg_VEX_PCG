#ifndef __modelG_h__
#define __modelG_h__

    #include <math.h>
    #include <GYcg/mathG.h> 
    #include <GYcg/baseG.h>  
    #include <file.h>   

    //基础噪波 //BaseNoise Range(-0.5,0.5)
    float BaseNoise_Cus( float Seed ; float Mul ; float Fre ; float Bias )
    {        
        float x;

        x = BaseNoise_G( Seed ,  Mul , Fre , Bias ); 

        return x;
    }
 
    //对于一个点，基于当前位置随机新的位置//For a point, random new position based on current position
    vector PosRand3D( float Seed ; vector Range ;  vector Pos )
    {   
        Pos.x = Pos.x + BaseNoise_G( Seed * 0.2   , 3000 , 100 , -500 ) * Range.x;
        Pos.y = Pos.y + BaseNoise_G( Seed * 2     , 3000 , 100 , 170  ) * Range.y;
        Pos.z = Pos.z + BaseNoise_G( -Seed * Seed , 3000 , 100 , -240 ) * Range.z;

        return Pos;
    }

    //（未完成）范围和方向限制。
    vector PosRand3D( float Seed ; vector Range ;  vector Pos ; float Radius )
    {   
        //Rotate_Cus
        //Dir1 Dir2 Dir3
        vector Dir1 , Dir2 , Dir3 ;

        Dir1.x = ( Pos.x + BaseNoise_G( Seed , 3000 , 100 , -500 ) * Range.x ) - Pos.x;
        Dir1.y = ( Pos.y + BaseNoise_G( Seed , 3000 , 100 , -500 ) * Range.y ) - Pos.y;   
        Dir1.z = ( Pos.z + BaseNoise_G( Seed , 3000 , 100 , -500 ) * Range.z ) - Pos.z;

        Pos = PosRandByVector( Seed , Radius , Pos , Dir1 ) ;

        return Pos; 
    }

    // //对于一个点，基于当前位置随机新的位置( 不影响Y轴 )//For a point, random new position based on current position（in X-Z plan ）
    // vector PosRand2D( float Seed ; vector Range ;  vector Pos )
    // {   
    //     Pos.x = Pos.x + BaseNoise_G( Seed , 3000.0 , 100 , -500 ) * Range.x;
    //     Pos.z = Pos.z + BaseNoise_G( Seed , 3000.0 , 100 , -240 ) * Range.z;

    //     return Pos;
    // }
    
    //根据点序号和点的法线随机重置一个点的位置，点的新位置会在法线方向上。//Randomly resets the position of a point based on its ptnum and its N. The new position of the point will be in the normal direction.  
    void PosRandResetByN( float Seed ; float Range ; int ptnum )
    {
        vector Pos = point(0,"P", ptnum );
        vector N = point(0,"N", ptnum );
        Pos = PosRandByVector( Seed , Range , Pos , N );
        setpointattrib(0,"P", ptnum , Pos , "set" );
    }

    //旋转功能,沿着Y轴 //rotate
    matrix3 Rotate_Cus(float Amount)
    {
        matrix3 Ident = maketransform({1,0,0},{0,0,1});
        float Angle = Amount*360;
        rotate(Ident,radians( Angle ),{0,1,0});

        return Ident;
    }

    //旋转功能,随机轴 //rotate
    matrix3 Rotate_Cus( float Seed ; float Amount )
    {   
        vector Up = set(0,1,0);
        vector Axis1 , Axis2 ;
        Axis1.x = BaseNoise_G( Seed * 2.5 , 3000 , 100 , -180 );
        Axis1.y = BaseNoise_G( -Seed + 23 , 3000 , 100 , 350  );
        Axis1.z = BaseNoise_G( Seed * Seed, 3000 , 100 , 190  );
        Axis1 = normalize(Axis1);
        Axis2 = cross( Up , Axis1);

        matrix3 Ident = maketransform( Axis1 , Axis2 );
        float Angle = Amount*360;
        rotate(Ident,radians( Angle ),{0,1,0});

        return Ident;
    }    

    //设置点的旋转值,沿着Y轴 //setpointattrib orient
    void RotateCusSetOrient(float Amount ; int ptnum)
    {   
        //模型Y轴竖直向上时，使用{1,0,0},{0,0,1}作为正交基
        matrix3 Ident = maketransform({1,0,0},{0,0,1});
        float Angle = Amount*360;
        rotate(Ident,radians( Angle ),{0,1,0});

        setpointattrib(0, "orient", ptnum , Ident , "set");  
    }

    //设置点的旋转值,随机轴 //setpointattrib orient
    void RotateCusSetOrient( float Seed ; float Amount ; int ptnum)
    {   
        //随机轴
        vector Up = set(0,1,0);
        vector Axis1 , Axis2 ;
        Axis1.x = BaseNoise_G( Seed * 2.5 , 3000 , 100 , -180 );
        Axis1.y = BaseNoise_G( -Seed + 23 , 3000 , 100 , 350  );
        Axis1.z = BaseNoise_G( Seed * Seed, 3000 , 100 , 190  );
        Axis1 = normalize(Axis1);
        Axis2 = cross( Up , Axis1);

        matrix3 Ident = maketransform( Axis1 , Axis2 );
        float Angle = Amount*360;
        rotate(Ident,radians( Angle ),{0,1,0});

        setpointattrib(0, "orient", ptnum , Ident , "set");  
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
        float IfNoise = 1;
        float Dis = 0;
        
        P1 = point(0,"P",ptnum1);
        P4 = point(0,"P",ptnum4);

        N1 = point(0,"N",ptnum1);
        N4 = point(0,"N",ptnum4);

        float Noise1 = BaseNoise_Cus(Seed , 3000 , 2 , 1);
        float Noise4 = BaseNoise_Cus(Seed , 3000 , 2 , 27);
        
        //当Seed小于0的时候设置控制点1和控制点4之间的距离为1-2，2-3之间的距离
        if( Seed < 0 )
        {
            IfNoise = 0;
            Dis = distance(P1,P4);
        }

        P2 = P1 + ( N1 * ( Noise1 * 0.5 + 0.5 ) ) * IfNoise + N1 * Dis * 2000; 
        P3 = P4 - ( N4 * ( Noise4 * 0.5 + 0.5 ) ) * IfNoise + N4 * Dis * 2000; 

        Ptnum2 = addpoint(0,P2);
        Ptnum3 = addpoint(0,P3);

        append(ReV,P1); 
        append(ReV,P2); 
        append(ReV,P3); 
        append(ReV,P4); 

        return ReV;

    }


    int adjustPrimLength_rePtnum(const int geo, prim; const float currentlength, targetlength)
    {

        float diff = targetlength - currentlength;

        int points[] = primpoints(geo, prim);

        int ptnum = 0;

        if(diff > 0)
        {
            vector posA = point(geo, "P", points[-2]);
            vector posB = point(geo, "P", points[-1]);

            vector posC = diff * normalize(posB-posA) + posB;

            ptnum = addpoint(geo, posC);

            addvertex(geo, prim, ptnum);

        }
        else if(diff < 0)
        {
            vector lastpos = 0;
            float length = 0;
            int cut = 0;

            foreach(int idx; int pt; points)
            {
                vector pos = point(geo, "P", pt);
                if(idx > 0 && !cut)
                {
                    float seglength = distance(pos, lastpos);

                    if(length+seglength > targetlength)
                    {
                        float frac = (targetlength-length)/seglength;

                        vector newpos = lerp(lastpos, pos, frac);

                        ptnum = addpoint(geo, newpos);

                        for(int i=idx; i<len(points); i++)
                            removepoint(geo, points[i]);

                        addvertex(geo, prim, ptnum);

                        break;
                    }

                    length += seglength;
                }

                lastpos = pos;
            }

            
        }

        return ptnum;

    }

    int[] test( int a ; int b){
        
        int c[];
        
        append( c , a );
        append( c , b );

        return c;
    
    }

int IsEntityPointInPosReturnPtnum( int Geometry ; vector Pos  )
{   
    int Findptnum , re ;
    vector FindPointPos;
    float Dis;
    
    Findptnum = nearpoint(Geometry , Pos );
    FindPointPos = point( Geometry , "P" , Findptnum );
    Dis = distance( Pos , FindPointPos );

    if( Dis < 0.01 ){
    
        re = Findptnum;
    
    }else{
    
        re = -1;
    
    }
    
    return re;
    
}

struct TestStruct{

    int IsTrueSelected = 1 ;

    int CollectPtnum[] = {3,2,1} ;

    string Struct1[] = { "Mid2Side0" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" };

    //不能用 array( "Mid2Side0" , "Mid2Side0_" ,...)'

    int getIsTrueSelected(){
    
        return IsTrueSelected;
    
    }

    int[] getCollectPtnum(){
    
        return CollectPtnum;
    
    }

    void setCollectPtnum( int array[] ){
    
        CollectPtnum = array;
    
    }

}

struct OverLapData{

    int IsTrueSelected = 1 ;

    int CollectPtnum[] = {} ;

    int getIsTrueSelected(){
    
        return IsTrueSelected;
    
    }

    int[] getCollectPtnum(){
    
        return CollectPtnum;
    
    }

    void setCollectPtnum( int array[] ){
    
        CollectPtnum = array;
    
    }

}

//OverLap re 0 , not OverLap re 1;
OverLapData IsOverLap( int CheckPtnum[] ; int CollectPtnum[] ){
        
    OverLapData overLapData ;
    int IsInCollect[] = {};
    int MinCollect , IsTrueSelected = 1 ;



    foreach( int v ; CheckPtnum ){
    
        if( find( CollectPtnum , v ) >= 0 ){
    
            IsTrueSelected = 0 ;
            append( IsInCollect , -1 );

        }else{
        
            append( IsInCollect , 1 ); 
        }

    }

    MinCollect = min( IsInCollect );

    if( MinCollect != -1){
    
        foreach( int n ; CheckPtnum ){
        
            append(CollectPtnum , n ); 
        
        }
        
    }
    
    overLapData = OverLapData( IsTrueSelected , CollectPtnum );

    return overLapData ;

}

struct CliffPathData{

    string Path = "Assets/Art/prefab/TerrainBlock/";
    string Postfix = ".prefab" ;

    string Struct1[] = { "Mid2Side0" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" , "Mid2Side0_" };
    string Struct2[] = { "MidL0R1" , "MidL0R1_TR1" , "MidL0R1_TR0" , "MidL0R1_TR0" , "MidL0R1_TR1" , "MidL0R1_TR0" , "MidL0R1_TR1" , "MidL0R1_TR1_" };
    string Struct3[] = { "MidL0R2" , "MidL0R2_TR1" , "MidL0R2_" , "MidL0R2_" , "MidL0R2_" , "MidL0R2_" , "MidL0R2_" , "MidL0R2_" };
    string Struct4[] = { "MidL1R2" , "MidL1R2_" , "MidL1R2_" , "MidL1R2_" , "MidL1R2_" , "MidL1R2" , "MidL1R2_" , "MidL1R2" };
    string Struct5[] = { "MidL1R0" , "MidL1R0_TR0" , "MidL1R0_TL1" , "MidL1R0_TR0" , "MidL1R0_TR0" , "MidL1R0_TL1" , "MidL1R0_TR1_" , "MidL1R0_TL1" };
    string Struct6[] = { "MidL2R0" , "MidL2R0_TL1" , "MidL2R0_" , "MidL2R0_" , "MidL2R0_" , "MidL2R0_" , "MidL2R0_" , "MidL2R0_" };
    string Struct7[] = { "MidL2R1" , "MidL2R1_" , "MidL2R1_" , "MidL2R1_" , "MidL2R1" , "MidL2R1_" , "MidL2R1" , "MidL2R1_" };
    string Struct8[] = { "MidL1R1" , "MidL1R1_TL1TR0" , "MidL1R1_TL0TR1" , "MidL1R1_TL1TR1" , "MidL1R1_TL1TR0" , "MidL1R1_TL0TR1" , "MidL1R1_TL1TR0" , "MidL1R1_TL0TR1" };
    string Struct9[] = { "MidL2R2" , "MidL2R2" , "MidL2R2" , "MidL2R2" , "MidL2R2" , "MidL2R2" , "MidL2R2" , "MidL2R2" };

    string getPath( int StructNum ; int CaseNum ){
        
        string restr , StructArray[] ;
        
        if( StructNum == 1  ){
        
            StructArray = Struct1;
        
        }else if( StructNum == 2 ){
        
            StructArray = Struct2;
        
        }else if( StructNum == 3 ){
        
            StructArray = Struct3;
        
        }else if( StructNum == 4 ){
        
            StructArray = Struct4;
        
        }else if( StructNum == 5 ){
        
            StructArray = Struct5;
        
        }else if( StructNum == 6 ){
        
            StructArray = Struct6;
        
        }else if( StructNum == 7 ){
        
            StructArray = Struct7;
        
        }else if( StructNum == 8 ){
        
            StructArray = Struct8;
        
        }else if( StructNum == 9 ){
        
            StructArray = Struct9;
        
        }

        restr = Path + StructArray[CaseNum] + Postfix;

        return restr;
    
    }

}

void SetOrientToPoint( int PtNum ; float Angle){
    
    vector4 Orient = eulertoquaternion(radians(set( 0 , Angle , 0)), 0) ;
    setpointattrib( 0 , "orient" , PtNum , Orient );

}

//Rock SAA


//创建单个随机矩阵
function matrix CreateRandMat( float Iteration  ; int FractalSize ; int style )
{
    matrix ReMat = ident();

    float FractalTypeArray[] ;

    if( style == 0 ){
        //长条形石头
        FractalTypeArray = { 0.5 , 0.6 , 0.6 , 0.9 , 0.6 , 0.8 , 0.3 , 0.5 , 0.3 , 0.65 , 0.4 , 0.6 , 0.4 , 0.5 , 0.3 , 0.65 , 0.4 , 0.6 , 3 , 2 , 20 };

    }else{
        //方形石头
        FractalTypeArray = { 0.5 , 0.6 , 0.6 , 0.9 , 0.6 , 0.8 , 0.3 , 0.5 , 0.3 , 0.65 , 0.4 , 0.6 , 0.4 , 0.5 , 0.3 , 0.65 , 0.4 , 0.6 , 1 , 1 , 1 };
    
    }
    
    //创建位移
    /*
    vector TransPos;
    int FeasibleRegionNum = int( random_brj( Iteration , Iteration ) * NumFeasibleRegion ) ;
    TransPos = point(1 , "P" , FeasibleRegionNum);
    ReMat.xw = TransPos.x ;
    ReMat.yw = TransPos.y ;
    ReMat.zw = TransPos.z ;
    */

    //创建缩放
    int FractalSizeOut ;
    if( FractalSize == 1 ){
        FractalSizeOut = FractalSize;
        ReMat.xx = fit01( random_brj( Iteration+71.1 , Iteration ) , FractalTypeArray[0] , FractalTypeArray[1] );//0.2~0.6
        ReMat.yy = fit01( random_brj( Iteration+53.1 , Iteration ) , FractalTypeArray[2] , FractalTypeArray[3] );//0.6~0.9
        ReMat.zz = fit01( random_brj( Iteration+17.1 , Iteration ) , FractalTypeArray[4] , FractalTypeArray[5] );//0.6~0.8        
    
    }else if( FractalSize == 2 ){
        FractalSizeOut = FractalSize;
        ReMat.xx = fit01( random_brj(Iteration+45.1 , Iteration ) , FractalTypeArray[6] , FractalTypeArray[7] );//0.2~0.6
        ReMat.yy = fit01( random_brj(Iteration+57.1 , Iteration ) , FractalTypeArray[8] , FractalTypeArray[9] );//0.6~0.9
        ReMat.zz = fit01( random_brj(Iteration+19.1 , Iteration ) , FractalTypeArray[10] , FractalTypeArray[11] );//0.6~0.8      
    
    }else{
        FractalSizeOut = 3;
        ReMat.xx = fit01( random_brj(Iteration+6.1 , Iteration ) , FractalTypeArray[12] , FractalTypeArray[13] );//0.2 , 0.3
        ReMat.yy = fit01( random_brj(Iteration+81.1 , Iteration ) , FractalTypeArray[14] , FractalTypeArray[15] );//0.2 , 0.3
        ReMat.zz = fit01( random_brj(Iteration+3.1 , Iteration ) , FractalTypeArray[16] , FractalTypeArray[17] );//0.2 , 0.3          
    
    }
    ReMat.ww = FractalSizeOut;

    //创建旋转
    float angle1 = radians( (2 * random_brj( Iteration + 97.1 , Iteration ) - 1) * FractalTypeArray[18]  ) ;
    float angle2 = radians( (2 * random_brj( Iteration + 116.1 , Iteration ) - 1) * FractalTypeArray[19] ) ;
    float angle3 = radians( (2 * random_brj( Iteration + 55.1 , Iteration ) - 1) * FractalTypeArray[20] ) ;
    
    ReMat *= dihedral( set( 1 , 0 , 0 ) , set( cos(angle1), sin(angle1) , 0 ) );//Z轴旋转
    ReMat *= dihedral( set( 0 , 1 , 0 ) , set( 0 , cos(angle2), sin(angle2) ) );//X轴旋转
    ReMat *= dihedral( set( 0 , 0 , 1 ) , set( sin(angle3), 0 , cos(angle3) ) );//Y轴旋转
    
    return ReMat;
    
}

//创建分型矩阵方法
function matrix[] CreateFractal(float seed ; int NumFractal ; int style )
{

    matrix reMats[] , reMat;    
    
    //分型大小分布
    float NumGigRate = 0.3;
    float NumMidRate = 0.3;
    float NumSmallRate = 0.4;
    
    int NumGig = int(NumFractal * NumGigRate) ;
    int NumMid = int(NumFractal * NumMidRate);
    int NumSmall = NumFractal - ( NumGig + NumMid );
    
    //大
    for(int i = 0 ; i < NumGig ; i++ ){
    
        reMat = CreateRandMat( float( i + seed ) , 1 , style );
    
        append(reMats ,reMat);
    
    }
    
    //中
    for(int i = NumGig ; i < ( NumGig + NumMid ) ; i++ ){
    
        reMat = CreateRandMat( float( i + seed ) , 2 , style );
    
        append(reMats ,reMat);    
    
    }
    
    //小
    for(int i = ( NumGig + NumMid ) ; i < NumFractal ; i++ ){
        
        reMat = CreateRandMat( float( i + seed ) , 3 , style );
        append(reMats ,reMat);
    
    }

    return reMats;
    
}


//mat to mat3

function matrix3 Mat4ToMat3( matrix Mat )
{
    matrix3 ReMat;
    
    ReMat = set( 
                    Mat.xx , Mat.xy , Mat.xz,
                    Mat.yx , Mat.yy , Mat.yz,
                    Mat.zx , Mat.zy , Mat.zz
                    
                ); 
    
    return ReMat;
}

//计算出一个三角面的三个角的度数


//terrainMesh Reduce
function void GetAcuteTriangPrim( int i ; float Angle ; float AreaPerimeterRateLimit )
{

    //标记出AcuteTriang，用“AcutePrim”属性 ，i 是面序号
    int TriangPtnum[] = primpoints(0,i);

    vector Pos0 , Pos1 , Pos2 , Dri0_1 , Dri1_2 , Dri2_0 ,Dri1_0 , Dri2_1 , Dri0_2 ;
    float cosAngle0 , cosAngle1 , cosAngle2 , Angle0 , Angle1 , Angle2 , Dis0 , Dis1 , Dis2 , DisArray[] ;
    int argArray[] ;

    Pos0 = point( 0 , "P" , TriangPtnum[0] );
    Pos1 = point( 0 , "P" , TriangPtnum[1] );
    Pos2 = point( 0 , "P" , TriangPtnum[2] );

    //将三角边长最长的边所对应的角标记出来
    Dis0 = distance( Pos1 , Pos2 );
    Dis1 = distance( Pos0 , Pos2 );
    Dis2 = distance( Pos0 , Pos1 );

    DisArray = {};
    append( DisArray , Dis0 );
    append( DisArray , Dis1 );
    append( DisArray , Dis2 );

    argArray = argsort(DisArray);

    //printf( "%g\n" , TriangPtnum );

    TriangPtnum = reorder(TriangPtnum,argArray);

    //printf( "%g\n" , TriangPtnum );
    /*
    Dri0_1 = normalize( Pos1 - Pos0 );
    Dri1_0 = normalize( Pos0 - Pos1 );

    Dri1_2 = normalize( Pos1 - Pos2 );
    Dri2_1 = normalize( Pos2 - Pos1 );

    Dri2_0 = normalize( Pos2 - Pos0 );
    Dri0_2 = normalize( Pos0 - Pos2 );

    cosAngle0 = dot(Dri0_1 ,Dri0_2 );
    cosAngle1 = dot(Dri1_2 ,Dri1_0 );
    cosAngle2 = dot(Dri2_0 ,Dri2_1 );

    Angle0 = acos( cosAngle0 );
    Angle1 = acos( cosAngle1 );
    Angle2 = acos( cosAngle2 );

    Angle0 = degrees(Angle0);
    Angle1 = degrees(Angle1);
    Angle2 = degrees(Angle2);
    */

    Pos2 = point( 0 , "P" , TriangPtnum[2] );
    Pos0 = point( 0 , "P" , TriangPtnum[0] );
    Pos1 = point( 0 , "P" , TriangPtnum[1] );

    Dri2_0 = normalize( Pos0 - Pos2 );
    Dri2_1 = normalize( Pos1 - Pos2 );

    cosAngle2 = dot(Dri2_0 ,Dri2_1 );
    Angle2 = acos( cosAngle2 );
    Angle2 = degrees(Angle2);

    if( Angle2 < 90 ){
        
        Angle2 = 180 - Angle2;
    
    }

    //求出面积/周长
    float area = prim( 0 , "area" , i );
    float perimeter = prim( 0 , "perimeter" , i );

    float APRate = area / perimeter ;

    //printf("%g\n",Angle2);

    if( Angle2 > Angle ){
        
        if( APRate < AreaPerimeterRateLimit ){
            
            setprimattrib( 0 ,"AcutePrim", i , 1 );     
        
        }

    }

}

function int GetAcuteTrianglesObtusePtnum( int PrimNum )
{
    vector Pos0 , Pos1 , Pos2 ;
    float  Dis0 , Dis1 , Dis2 , DisArray[] ;
    int argArray[] ;

    int TriangPtnum[] = primpoints(0,PrimNum);

    Pos0 = point( 0 , "P" , TriangPtnum[0] );
    Pos1 = point( 0 , "P" , TriangPtnum[1] );
    Pos2 = point( 0 , "P" , TriangPtnum[2] );

    Dis0 = distance( Pos1 , Pos2 );
    Dis1 = distance( Pos0 , Pos2 );
    Dis2 = distance( Pos0 , Pos1 );

    DisArray = {};
    append( DisArray , Dis0 );
    append( DisArray , Dis1 );
    append( DisArray , Dis2 );

    argArray = argsort(DisArray);
    TriangPtnum = reorder(TriangPtnum,argArray);

    return TriangPtnum[2];

}

//四舍五入取整
function int Round( float f )
{
    int rounded_value ;

    if( f >= 0 ){
    
        rounded_value = int(floor( f + 0.5 ));
    
    }else{
    
        rounded_value = int(floor( f - 0.5 ));
    
    }

    return rounded_value;

}

//array排重
function int[] uniqueValuesInt( int array[] )
{
    int outputArray[] ;

    // Iterate over the input array
    foreach (int value; array)
    {
        // Check if the value already exists in the output array
        int index = find(outputArray, value);

        // If the value does not exist, add it to the output array
        if (index < 0)
        {
            push(outputArray, value);
        }
    }

    return outputArray;

}

#endif