#ifndef COMMON_H_
#define COMMON_H_

#include <list>
#include <vector>
using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "simobject.h"
#include "arguments.h"

typedef struct {
    double m_fX;
    double m_fY;
} TPosition;

extern double GetDistanceBetweenPositions(const TPosition* pt_pos1, const TPosition* pt_pos2);
extern double GetSquaredDistanceBetweenPositions(const TPosition* pt_pos1, const TPosition* pt_pos2);

#define PI 3.14159265
#define EPSILON 1e-10

#define ERRENDL fprintf(stderr, "\n");
#define PRINTVEC2(label, vec) printf("%s, x: %2.6f, y: %2.6f\n", label, vec.x, vec.y);  
#define FILEANDLINE                                          { fprintf(stderr, "In %s:%d: ", __FILE__, __LINE__); }
#define ERROR(s)                                             { FILEANDLINE; fprintf(stderr, s); ERRENDL; }
#define ERROR1(s, p1)                                        { FILEANDLINE; fprintf(stderr, s, p1); ERRENDL; }
#define ERROR2(s, p1, p2)                                    { FILEANDLINE; fprintf(stderr, s, p1, p2); ERRENDL; }
#define ERROR3(s, p1, p2, p3)                                { FILEANDLINE; fprintf(stderr, s, p1, p2, p3); ERRENDL; }
#define ERROR4(s, p1, p2, p3, p4)                            { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4 ); ERRENDL; }
#define ERROR5(s, p1, p2, p3, p4, p5)                        { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5 ); ERRENDL; }
#define ERROR6(s, p1, p2, p3, p4, p5, p6)                    { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6 ); ERRENDL; }
#define ERROR7(s, p1, p2, p3, p4, p5, p6, p7 )               { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7 ); ERRENDL; }
#define ERROR8(s, p1, p2, p3, p4, p5, p6, p7, p8 )           { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7, p8 ); ERRENDL; }
#define ERROR9(s, p1, p2, p3, p4, p5, p6, p7, p8, p9 )       { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7, p8, p9 ); ERRENDL; }
#define ERROR10(s, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10 ) { FILEANDLINE; fprintf(stderr, s, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10 ); ERRENDL; }

// If debugging is defined then:
#ifdef  DEBUG

#include <stdio.h>

#define DEBUGOUT(s)                  { fprintf(stderr, s); }
#define DEBUGOUT1(s, p1)             { fprintf(stderr, s, p1); }
#define DEBUGOUT2(s, p1, p2)         { fprintf(stderr, s, p1, p2); }
#define DEBUGOUT3(s, p1, p2, p3)     { fprintf(stderr, s, p1, p2, p3); }
#define DEBUGOUT4(s, p1, p2, p3, p4) { fprintf(stderr, s, p1, p2, p3, p4 ); }
#else
// Otherwise simply define the macros as being empty:
#define DEBUGOUT(s)                  
#define DEBUGOUT1(s, p1)             
#define DEBUGOUT2(s, p1, p2)         
#define DEBUGOUT3(s, p1, p2, p3)     
#define DEBUGOUT4(s, p1, p2, p3, p4) 
#endif


#endif
