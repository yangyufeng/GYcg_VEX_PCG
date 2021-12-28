/*
    Original Code:
    https://github.com/gupon/Houdini-Gulib/blob/master/vex/include/bezier.vfl
*/
vector[] primPointsPositions(int geo; int primnum)
{
    int points[] = primpoints(geo, primnum);
    vector pos[];
    foreach (int pt; points)
        append(pos, vector(point(geo, "P", pt)));
    return pos;
}

/*
    Utility to handle lines and cubic bezier curves.
*/
struct Curve {
    vector p0, p1, p2, p3;

    vector[] points() {
        return array(p0, p1, p2, p3);
    }

    void set(vector curve[]) {
        this.p0 = curve[0];
        this.p1 = curve[1];
        this.p2 = curve[2];
        this.p3 = curve[3];
    }
}

struct CurveInfo {
    Curve curve;
    int index = 0;
    int divCount = 0;
    float tStart = 0.0;
    float tEnd = 1.0;

    void setSubRange(CurveInfo baseinfo; float start; float end) {
        float t0 = baseinfo.tStart;
        float t1 = baseinfo.tEnd;
        this.tStart = t0 + (t1 - t0) * start;
        this.tEnd = t0 + (t1 - t0) * end;
    }
}

/*
    Get point position on bezier curve from 4 point positions
*/
vector getPointOnBezier (float t; vector p0; vector p1; vector p2; vector p3) 
{
    return (1-t)*(1-t)*(1-t)*p0 + 3*t*(1-t)*(1-t)*p1 + 3*t*t*(1-t)*p2 + t*t*t*p3;
}

vector getPointOnBezier (float t; vector p[]) 
{
    return getPointOnBezier (t, p[0], p[1], p[2], p[3]);
}

/*
    Get point position on bezeir curve from 4 ptnums and t
*/      
vector getPointOnBezier (int geo; float t; int curve_pts[])
{
    vector p0 = point(geo, "P", curve_pts[0]);
    vector p1 = point(geo, "P", curve_pts[1]);
    vector p2 = point(geo, "P", curve_pts[2]);
    vector p3 = point(geo, "P", curve_pts[3]);
    return getPointOnBezier(t, p0, p1, p2, p3);
}

/*
    split bezier curve at T and returns first half
*/
vector[] splitBezierFirst (float t; vector p0; vector p1; vector p2; vector p3; int reverse)
{
    vector c0 = lerp(p0, p1, t);
    vector c1 = lerp(p1, p2, t);
    vector c2 = lerp(p2, p3, t);

    vector d0 = lerp(c0, c1, t);
    vector d1 = lerp(c1, c2, t);
    
    vector e0 = lerp(d0, d1, t);
    
    return reverse ? array(e0, d0, c0, p0) : array(p0, c0, d0, e0) ;
}

vector[] splitBezierFirst (float t; vector p[]; int reverse)
{
    return splitBezierFirst(t, p[0], p[1], p[2], p[3], reverse);
}
/*
    split bezier curve at T and returns the last half
*/
vector[] splitBezierLast (float t; vector p0; vector p1; vector p2; vector p3; int reverse)
{
    return splitBezierFirst(1.0 - t, p3, p2, p1, p0, 1 - reverse);
}

vector[] splitBezierLast (float t; vector p[]; int reverse)
{
    return splitBezierLast(t, p[0], p[1], p[2], p[3], reverse);
}


/*
    subcurve

    returns part of the curve between t0 and t1
*/

vector[] subcurve(vector curve_pos[]; float t0; float t1)
{
    vector _curve_pos[] = set(curve_pos);
    if (t0 > 0.0) {
        _curve_pos = splitBezierLast(t0, _curve_pos, 0);
    }
    if (t1 < 1.0) {
        _curve_pos = splitBezierFirst((t1 - t0) / (1.0 - t0), _curve_pos, 0);
    }
    return _curve_pos;
}

vector[] subcurve(vector p0; vector p1; vector p2; vector p3; float t0; float t1)
{
    vector curve_pos[] = array(p0, p1, p2, p3);
    return subcurve(curve_pos, t0, t1);
}

vector[] subcurve(int geo; int p0; int p1; int p2; int p3; float t0; float t1)
{
    return subcurve(point(geo, "P", p0),
                    point(geo, "P", p1),
                    point(geo, "P", p2),
                    point(geo, "P", p3),
                    t0, t1);
}

vector[] subcurve(int geo; int curve_pts[]; float t0; float t1)
{
    vector curve_pos[] = array(
            vector(point(geo, "P", curve_pts[0])),
            vector(point(geo, "P", curve_pts[1])),
            vector(point(geo, "P", curve_pts[2])),
            vector(point(geo, "P", curve_pts[3])));
    return subcurve(curve_pos, t0, t1);
}

/*
    returns a subline from two points
*/  
vector[] subline(vector line[]; float t0; float t1)
{
    vector p0 = line[0];
    vector p1 = line[1];
    if (t0 > 0) p0 = line[0] + (line[1] - line[0]) * t0;
    if (t1 < 1) p1 = line[1] + (line[0] - line[1]) * (1 - t1);
    return array(p0, p1);
}

/*
    returns a subline from polyline
    TODO: support loop (when first and end points are shared)
*/  
vector[] sublineMulti(vector curve[]; float t0; float t1)
{
    int NUMSEG = len(curve) - 1;
    float SEGLEN = 1 / float(NUMSEG);

    // swap if opposite
    float _t0 = min(t0, t1);
    float _t1 = max(t0, t1);

    // shift range
    _t1 = _t1 - floor(_t0);
    _t0 = frac(_t0);

    // init variables
    vector _curve[];
    int i = 0; float lt0, lt1;

    // find t0 segment index
    while (i++ < NUMSEG) if (_t0 <= i / float(NUMSEG)) break;
    lt0 = (_t0 - (i - 1) * SEGLEN) / SEGLEN;

    if (_t1 <= i / float(NUMSEG)) {
        // case _t0 and _t1 are in same segment
        lt1 = (_t1 - (i - 1) * SEGLEN) / SEGLEN;
        push(_curve, subline(curve[i-1: i+1], lt0, lt1));
    } else {
        // from midpoint to the end
        push(_curve, subline(curve[i-1: i+1], lt0, 1.0));
        
        // find _t1 segment index
        while (i++ < NUMSEG * ceil(_t1)) {
            if (_t1 <= i / float(NUMSEG)) {
                lt1 = (_t1 - (i - 1) * SEGLEN) / SEGLEN;
                i = i % NUMSEG ? i % NUMSEG : NUMSEG;
                push(_curve, subline(curve[i-1 : i+1], 0, lt1)[1]);
                break;
            } else {
                push(_curve, curve[i % NUMSEG]);
            }
        }
    }
    return _curve;
}

// WIP!!!!!!!!!
/*
vector[] subcurveMulti(vector pos[]; int idx0; float t0; int idx1; float t1)
{
    // offset index to ensure: 0 <= idx0 < numseg
    if (idx0 < 0 || idx0 > numseg - 1) {
        idx1 += (idx0 % numseg) - idx0;
        idx0 %= numseg;
    }

    curve = pos[idx0*3 : idx0*3+4];

    if (idx0 == idx1) {  // inside same segment
        curve = subcurve(curve, t0, t1);
    } else {
        curve = subcurve(curve, t0, 1.0);
        if ((idx0 + 1) < idx1) {
            for (int j=idx0+1; j<idx1; j++) {
                int _j = j % numseg; // wrap closed curve segment
                vector c[] = pos[_j*3 : _j*3+4];
                append(curve, c[1:]);
            }
        }
        idx1 %= numseg;
        vector curve1[] = pos[idx1*3 : idx1*3+4];
        append(curve, subcurve(curve1, 0.0, t1)[1:]);
    }

    int nprm = addprim(0, "polyline");
    setprimattrib(0, "bz_id_orig", nprm, i@bz_id);
    foreach (vector p; curve) {
        int npt = addpoint(0, p);
        addvertex(0, nprm, npt);
    }
}
*/

/*
    Find Cubic Roots and return t value.
    
    Original Code:
    https://pomax.github.io/bezierinfo/#extremities
*/
float[] getCubicRoots(vector p0; vector p1; vector p2; vector p3;) {
    float d = (-p0.z + 3*p1.z - 3*p2.z + p3.z);
    float a = (3*p0.z - 6*p1.z + 3*p2.z) / d;
    float b = (-3*p0.z + 3*p1.z) / d;
    float c = p0.z / d;

    float p = (3*b - a*a) / 3.0;
    float pc = p/3;
    float q = (2*a*a*a - 9*a*b + 27*c) / 27.0;
    float q2 = q/2;
    float D = q2*q2 + pc*pc*pc;
    
    float u1,v1,root1,root2,root3;
    float result[] = {};
        
    
    if (D < 0)
    {
        float mp3  = pow(-p/3, 3);
        float r = sqrt(mp3);
        float t = -q / (2*r);
        float cosphi = t < -1 ? -1 : t > 1 ? 1 : t;
        float phi  = acos(cosphi);
        float t1   = 2*cbrt(r);
        root1 = t1 * cos(phi/3) - a/3;
        root2 = t1 * cos((phi+2*PI)/3) - a/3;
        root3 = t1 * cos((phi+4*PI)/3) - a/3;
        push(result, root1);
        push(result, root2);
        push(result, root3);
    }
    else if (D == 0)
    {
        u1 = q2 < 0 ? cbrt(-q2) : -cbrt(q2);
        root1 = 2*u1 - a/3;
        root2 = -u1 - a/3;
        push(result, root1);
        push(result, root2);
    }
    else
    {
        float sd = sqrt(D);
        u1 = cbrt(sd - q2);
        v1 = cbrt(sd + q2);
        root1 = u1 - v1 - a/3;
        push(result, root1);
    }
    
    return result;
}


/*
    Get intersection point with 2 segmented line.
    returns 0 if there's no intersection.
*/
vector intersectionLine(vector a0; vector a1; vector b0; vector b1; int succeed)
{
    if (
        max(a0.x, a1.x) <= min(b0.x, b1.x) ||
        min(a0.x, a1.x) >= max(b0.x, b1.x) ||
        max(a0.z, a1.z) <= min(b0.z, b1.z) ||
        min(a0.z, a1.z) >= max(b0.z, b1.z)
        ) {
            succeed = 0;
            return set(0);
    } 

    vector a = a1 - a0;
    vector b = b1 - b0;
    float d = cross(a, b).y;
    
    if (d == 0) {
        succeed = 0;
        return set(0);
    }
    
    vector c = b0 - a0;
    float u = cross(c, b).y / d;
    float v = cross(c, a).y / d;
    
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) {
        succeed = 0;
        return set(0);
    }
    
    succeed = 1;
    return a0 + a * u;
}

/*
    Get intersection point with 2 segmented line.
    returns 0 if there's no intersection.
*/
float[] intersectionLine(vector a0; vector a1; vector b0; vector b1)
{
    if (
        max(a0.x, a1.x) <= min(b0.x, b1.x) ||
        min(a0.x, a1.x) >= max(b0.x, b1.x) ||
        max(a0.z, a1.z) <= min(b0.z, b1.z) ||
        min(a0.z, a1.z) >= max(b0.z, b1.z)
        ) {
            return {};
    } 

    vector a = a1 - a0;
    vector b = b1 - b0;
    float d = cross(a, b).y;
    
    if (d == 0) return {};
    
    vector c = b0 - a0;
    float u = cross(c, b).y / d;
    float v = cross(c, a).y / d;
    
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) return {};
    
    return array(u, v);
}


/*
    Get intersection point between horizontal line and a segment
*/
vector horizontalLineIntersection(vector p0; vector p1; float y; int succeed)
{
    vector _p0 = set(p0.x, p0.z, 1);
    vector _p1 = set(p1.x, p1.z, 1);
    
    vector lineA = cross(_p0, _p1);
    vector x = set(-(lineA.y * y + lineA.z) / lineA.x, y, 0);
    
    vector pmin = min(_p0, _p1);
    vector pmax = max(_p0, _p1);
    
    if (x.x <= pmax.x && x.x >= pmin.x && x.y <= pmax.y && x.y >= pmin.y) {
        succeed = 1;
        return set(x.x, 0, x.y);
    } else {
        succeed = 0;
        return set(0);
    }
}


/*
    check two lines' intersection
*/
int intersects(vector a0; vector a1; vector b0; vector b1)
{
    vector a = a1 - a0;
    vector b = b1 - b0;
    float d = cross(a, b).y;
    
    if (d == 0) return 0;
    
    vector c = b0 - a0;
    float u = cross(c, b).y / d;
    float v = cross(c, a).y / d;
    
    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) return 0;
    
    return 1;
}


/*
    Get intersection points with line and bezier curve.
*/
float[] intersectionLineBezierT(vector curve[]; vector line[]) {
    /*
        check if the point is inside line segment area
    */
    function int pointInLineSegment(vector p; vector l0; vector l1)
    {
        if (l0.x == l1.x) return p.z >= min(l0.z, l1.z) && p.z <= max(l0.z, l1.z);
        if (l0.z == l1.z) return p.x >= min(l0.x, l1.x) && p.x <= max(l0.x, l1.x);
        return (
            p.z >= min(l0.z, l1.z)
            && p.z <= max(l0.z, l1.z)
            && p.x >= min(l0.x, l1.x)
            && p.x <= max(l0.x, l1.x)
        );
    }

    float result[] = {};

    matrix m = ident();
    float angle = atan2(line[1].z - line[0].z, line[1].x - line[0].x);
    translate(m, -line[0]);
    rotate(m, angle, {0, 1, 0});

    float roots[] = getCubicRoots(curve[0] * m, curve[1] * m, curve[2] * m, curve[3] * m);

    foreach (float t; roots) {
        if (t >= 0.0 && t <= 1.0) {
            vector p = getPointOnBezier(t, curve);
            if (pointInLineSegment(p, line[0], line[1])) {
                push(result, t);
            }
        }
    }
    return result;
}

/*
    Get intersection points with line and bezier curve.
    each result is a list of normalized u.
*/
void intersectionLineBezier(vector curve[]; vector line[]; float resultA[]; float resultB[]) {
    /*
        check if the point is inside line segment area
    */
    function int pointInLineSegment(vector p; vector l0; vector l1)
    {
        if (l0.x == l1.x) return p.z >= min(l0.z, l1.z) && p.z <= max(l0.z, l1.z);
        if (l0.z == l1.z) return p.x >= min(l0.x, l1.x) && p.x <= max(l0.x, l1.x);
        return (
            p.z >= min(l0.z, l1.z)
            && p.z <= max(l0.z, l1.z)
            && p.x >= min(l0.x, l1.x)
            && p.x <= max(l0.x, l1.x)
        );
    }

    matrix m = ident();
    float angle = atan2(line[1].z - line[0].z, line[1].x - line[0].x);
    translate(m, -line[0]);
    rotate(m, angle, {0, 1, 0});

    float roots[] = getCubicRoots(curve[0] * m, curve[1] * m, curve[2] * m, curve[3] * m);
    float lineLen = distance(line[0], line[1]);

    // edge case when D==0
    if (len(roots) == 2 && roots[0] == 0 && roots[1] == 0) return;

    foreach (float t; roots) {
        if (t >= 0.0 && t <= 1.0) {
            vector p = getPointOnBezier(t, curve);
            if (pointInLineSegment(p, line[0], line[1])) {
                push(resultA, t);
                push(resultB, distance(p, line[0]) / lineLen);
            }
        }
    }
}

float bezierLength(vector p0; vector p1; vector p2; vector p3; int divide)
{
    float t = 0;
    vector curr, next;
    curr = getPointOnBezier(0, p0, p1, p2, p3);
    float sum = 0;
    float step = 1.0 / float(divide);
    while (t < 1.0)
    {
        t += step;
        next = getPointOnBezier(t, p0, p1, p2, p3);
        sum += length(next - curr);
        curr = next;
    }
    return sum;
}


float bezierLength(vector curve[]; int divide)
{
    return bezierLength(curve[0], curve[1], curve[2], curve[3], divide);
}

/* experimental */
float[] bezierLengthSegment(vector p0; vector p1; vector p2; vector p3; int divnum)
{
    float seglen[];
    vector pA, pB;
    pA = getPointOnBezier(0, p0, p1, p2, p3);
    float step = 1.0 / float(divnum);
    float t;
    for (int i=1; i<=divnum; i++) {
        t = i * step;
        pB = getPointOnBezier(t, p0, p1, p2, p3);
        append(seglen, length(pB - pA));
        pA = pB;
    }
    return seglen;
}
float[] bezierLengthSegment(vector curve[]; int divide)
{
    return bezierLengthSegment(curve[0], curve[1], curve[2], curve[3], divide);
}

vector[] convexHull(vector pos[])
{
    vector hull[];
    vector A, B, C;
    float X;
    
    int start = 0;
    foreach(int i; vector p; pos) {
        if (pos[start].x > p.x) start = i;
    }
    
    A = pos[start];
    int count = 0;
    do {
        push(hull, A);
        B = pos[0];
        for (int i=1; i<len(pos); i++)
        {
            C = pos[i];
            if (B == A) {
                B = C;
            } else {
                X = cross(B - A, C - A).y;
                if (X > 0 || (X == 0 && length(C-A) > length(B-A))) {
                    B = C;
                }
            }
        }
        A = B;
    } while (A != hull[0] && len(hull) <= len(pos));
    
    return hull;
}


int doOverlap(vector curveA[]; vector curveB[])
{
    vector amin = min(curveA);
    vector amax = max(curveA);
    vector bmin = min(curveB);
    vector bmax = max(curveB);
    
    if (amin.x > bmax.x || bmin.x > amax.x) {
        return 0;
    }
    if (amin.z > bmax.z || bmin.z > amax.z) {
        return 0;
    }
    
    return 1;
}


float signedLinePointDist(vector line; vector pt) {
    float a = line.x;
    float b = line.y;
    float c = line.z;
    float x = pt.x;
    float y = pt.z;
    return (a*x + b*y + c) / sqrt(a*a + b*b);
}


float[] bezierClipping(vector curveA[]; vector curveB[]) {
    // lineB as homogenious coord
    vector lineB = cross(
        set(curveB[0].x, curveB[0].z, 1),
        set(curveB[3].x, curveB[3].z, 1)
    );
    float d1 = signedLinePointDist(lineB, curveB[1]);
    float d2 = signedLinePointDist(lineB, curveB[2]);

    // get fat line for line B
    float coeff = (d1 * d2 > 0) ? (3 / 4.) : (4 / 9.);
    float dmin = min(0, d1, d2) * coeff;
    float dmax = max(0, d1, d2) * coeff;
    float dRange[] = array(dmin, dmax);

    // get distance to lineB for each curveA's control points
    // printf("bc:: tdist \n");
    vector tDist[];
    float tCandidates[];
    foreach (int i; vector p; curveA) {
        float t = i / 3.0;
        float dist = signedLinePointDist(lineB, p);
        append(tDist, set(t, 0, dist));
        
        if (dist >= dmin && dist <= dmax) 
            append(tCandidates, t);
    }

    // case all in range
    if (len(tCandidates) == 4) return {0.0, 1.0};

    // convexhull of dist curve
    vector hull[] = convexHull(tDist);

    // intersection between D convexhull and dmin/dmax
    int total = len(hull);
    int succeed;
    vector x, p0, p1;
    for (int i=0; i<total; i++) {
        p0 = hull[i];
        p1 = hull[(i + 1) % total];

        foreach (float y; dRange) {
            x = horizontalLineIntersection(p0, p1, y, succeed);
            if (succeed) append(tCandidates, x.x);
        }
    }
    // printf("drange: %g\n", drange);
    // printf("drange fat: %g\n", dRange);
    // printf("tDist: %g\n", tDist);
    // printf("hull: %g\n", hull);
    // printf("tCandidates: %g\n", tCandidates);

    // case no intersection
    if (len(tCandidates) == 0) {
        return {0, 0};
    } else {
        return array(min(tCandidates), max(tCandidates));
    }
}

void _addPairs (CurveInfo pairs[]; CurveInfo infoA; CurveInfo infoB; float tRange[]; int resetDivCount) {
    /* only used in findbezierIntersection() */
    CurveInfo info;
    info.curve->set(subcurve(infoB.curve->points(), tRange[0], tRange[1]));
    info->setSubRange(infoB, tRange[0], tRange[1]);
    info.index = infoB.index;
    if (resetDivCount) {
        infoA.divCount = 0;
    } else {
        info.divCount = infoB.divCount + 1;
        infoA.divCount += 1;
    }
    pairs[len(pairs)] = infoA;
    pairs[len(pairs)] = info;
}

void findBezierIntersection (vector curveA[]; vector curveB[]; float resultsA[]; float resultsB[])
{
    float T_STOP_THRES = 0.002;  // threshold for both curve to stop clipping iteration
    float T_LINEAR_THRES = 0.001;  // threshold for one of the curve to operate line-curve intersection
    float SUBDIV_THRES = 0.3;
    int MAX_CLIP_NUM = 50;

    if (length2(curveA[0] - curveA[3]) == 0 || length2(curveB[0] - curveB[3]) == 0) {
        // printf("no length \n");
        return;
    }

    if (!doOverlap(curveA, curveB)) {
        // printf("no intersectioin! skip! \n");
        return;
    }

    CurveInfo infoA = CurveInfo();
    infoA.curve->set(curveA);
    CurveInfo infoB = CurveInfo();
    infoB.curve->set(curveB);
    infoB.index = 1;

    CurveInfo pairs[], nextPairs[], results[];
    pairs[0] = infoA;
    pairs[1] = infoB;

    float tRangeA[], tRangeB[];
    float diffA, diffB, removeA, removeB;
    vector pointsA[], pointsB[];
    int count = 0;
    while (count < MAX_CLIP_NUM && len(pairs)) {
        for (int i=0; i<len(pairs)/2; i++) {
            infoA = pairs[i * 2 + 0];
            infoB = pairs[i * 2 + 1];
            pointsA = infoA.curve->points();
            pointsB = infoB.curve->points();
            diffA = infoA.tEnd - infoA.tStart;
            diffB = infoB.tEnd - infoB.tStart;

            if (diffA < T_STOP_THRES && diffB < T_STOP_THRES) {
                results[len(results)] = infoA;
                results[len(results)] = infoB;
                // printf("CONTINUE: Short Enough\n");
                continue;
            } else if (!doOverlap(pointsA, pointsB)) {
                // printf("CONTINUE: No Overlap\n");
                continue;
            }

             
            if (diffA < T_LINEAR_THRES || diffB < T_LINEAR_THRES) {
                vector line[] = diffA < diffB ? array(pointsA[0], pointsA[3]) : array(pointsB[0], pointsB[3]);
                vector curve[] = diffA < diffB ? pointsB : pointsA;
                float ts[] = intersectionLineBezierT(curve, line);
                float t = sum(ts) / float(len(ts)); //average
                if (diffA < diffB) {
                    infoB->setSubRange(infoB, t, t);
                } else {
                    infoA->setSubRange(infoA, t, t);
                }
                results[len(results)] = infoA;
                results[len(results)] = infoB;
                // printf("CONTINUE: short enough half\n");
                continue;
            }

            tRangeA = bezierClipping(pointsA, pointsB);
            removeA = 1 - (tRangeA[1] - tRangeA[0]);

            
            // compare only for the first time + after subdiv
            if (infoA.divCount == 0) {
                tRangeB = bezierClipping(pointsB, pointsA);
                removeB = 1 - (tRangeB[1] - tRangeB[0]);
            } else {
                removeB = 0;
            }

            if (removeA == 1 || removeB == 1)  continue;

            if (max(removeA, removeB) < SUBDIV_THRES) {
                if (infoA.tEnd - infoA.tStart > infoB.tEnd - infoB.tStart) {
                    _addPairs(nextPairs, infoB, infoA, {0.0, 0.5}, 1);
                    _addPairs(nextPairs, infoB, infoA, {0.5, 1.0}, 1);
                } else {
                    _addPairs(nextPairs, infoA, infoB, {0.0, 0.5}, 1);
                    _addPairs(nextPairs, infoA, infoB, {0.5, 1.0}, 1);
                }
            } else if (removeA > removeB) {
                _addPairs(nextPairs, infoB, infoA, tRangeA, 0);
            } else {
                _addPairs(nextPairs, infoA, infoB, tRangeB, 0);
            }
        }
        pairs = {};
        pairs = set(nextPairs);
        nextPairs = {};
        count++;
        // printf("count: %i\n", count);
        // printf("pairs: %i\n", len(pairs));
        // printf("results: %g\n", results);
        // printf("points: %g\n %g\n", pointsA, pointsB);
    }

    foreach (CurveInfo info; results) {
        float value = (info.tStart + info.tEnd) / 2;
        if (info.index) append(resultsB, value);
        else append(resultsA, value);
    }
}