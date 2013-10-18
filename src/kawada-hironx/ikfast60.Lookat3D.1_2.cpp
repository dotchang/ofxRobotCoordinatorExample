/// autogenerated analytical inverse kinematics code from ikfast program part of OpenRAVE
/// \author Rosen Diankov
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///     http://www.apache.org/licenses/LICENSE-2.0
/// 
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// ikfast version 60 generated on 2012-09-02 09:43:07.031783
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==60);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#define IKFAST_STRINGIZE2(s) #s
#define IKFAST_STRINGIZE(s) IKFAST_STRINGIZE2(s)

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#endif // _MSC_VER

// lapack routines
extern "C" {
  void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
  void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
  void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
  void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
  void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
  void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_LOOKAT3D {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)0.000001)
#endif

// used to check input to atan2 for degenerate cases
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)2e-6)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

/// solves the forward kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {
IkReal x0,x1,x2,x3,x4;
x0=IKcos(j[0]);
x1=IKsin(j[0]);
x2=IKsin(j[1]);
x3=IKcos(j[1]);
x4=((IkReal(0.0850000000000000))*(x2));
eetrans[0]=((((x0)*(x4)))+(((IkReal(-0.0700000000000000))*(x1))));
eetrans[1]=((((x1)*(x4)))+(((IkReal(0.0700000000000000))*(x0))));
eetrans[2]=((IkReal(0.569500000000000))+(((IkReal(0.0850000000000000))*(x3))));
eerot[0]=((x0)*(x3));
eerot[1]=((x1)*(x3));
eerot[2]=((IkReal(-1.00000000000000))*(x2));
}

IKFAST_API int GetNumFreeParameters() { return 0; }
IKFAST_API int* GetFreeParameters() { return NULL; }
IKFAST_API int GetNumJoints() { return 2; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

IKFAST_API int GetIkType() { return 0x23000006; }

class IKSolver {
public:
IkReal j1,cj1,sj1,htj1,j2,cj2,sj2,htj2,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij1[2], _nj1,_ij2[2], _nj2;

bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
j1=numeric_limits<IkReal>::quiet_NaN(); _ij1[0] = -1; _ij1[1] = -1; _nj1 = -1; j2=numeric_limits<IkReal>::quiet_NaN(); _ij2[0] = -1; _ij2[1] = -1; _nj2 = -1; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    solutions.Clear();
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_px=px;
new_py=py;
new_pz=((IkReal(-0.569500000000000))+(pz));
px = new_px; py = new_py; pz = new_pz;
pp=(((px)*(px))+((py)*(py))+((pz)*(pz)));
{
IkReal dummyeval[1];
dummyeval[0]=((IkReal(-1.00000000000000))+(((IkReal(-14.2857142857143))*(py))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
continue;

} else
{
{
IkReal j1array[2], cj1array[2], sj1array[2];
bool j1valid[2]={false};
_nj1 = 2;
IkReal x5=((IKabs(((IkReal(-0.140000000000000))+(((IkReal(-2.00000000000000))*(py))))) != 0)?((IkReal)1/(((IkReal(-0.140000000000000))+(((IkReal(-2.00000000000000))*(py)))))):(IkReal)1.0e30);
IkReal x6=((IkReal(2.00000000000000))*(px)*(x5));
if( (((IkReal(-0.0196000000000000))+(((IkReal(4.00000000000000))*((py)*(py))))+(((IkReal(4.00000000000000))*((px)*(px)))))) < (IkReal)-0.00001 )
    continue;
IkReal x7=((x5)*(IKsqrt(((IkReal(-0.0196000000000000))+(((IkReal(4.00000000000000))*((py)*(py))))+(((IkReal(4.00000000000000))*((px)*(px))))))));
j1array[0]=((IkReal(2.00000000000000))*(atan(((x6)+(((IkReal(-1.00000000000000))*(x7)))))));
sj1array[0]=IKsin(j1array[0]);
cj1array[0]=IKcos(j1array[0]);
j1array[1]=((IkReal(2.00000000000000))*(atan(((x7)+(x6)))));
sj1array[1]=IKsin(j1array[1]);
cj1array[1]=IKcos(j1array[1]);
if( j1array[0] > IKPI )
{
    j1array[0]-=IK2PI;
}
else if( j1array[0] < -IKPI )
{    j1array[0]+=IK2PI;
}
j1valid[0] = true;
if( j1array[1] > IKPI )
{
    j1array[1]-=IK2PI;
}
else if( j1array[1] < -IKPI )
{    j1array[1]+=IK2PI;
}
j1valid[1] = true;
for(int ij1 = 0; ij1 < 2; ++ij1)
{
if( !j1valid[ij1] )
{
    continue;
}
_ij1[0] = ij1; _ij1[1] = -1;
for(int iij1 = ij1+1; iij1 < 2; ++iij1)
{
if( j1valid[iij1] && IKabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
{
    j1valid[iij1]=false; _ij1[1] = iij1; break; 
}
}
j1 = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];

{
IkReal dummyeval[1];
dummyeval[0]=((((IkReal(204.081632653061))*((pz)*(pz))*((sj1)*(sj1))))+(((IkReal(-28.5714285714286))*(cj1)*(py)))+(((IkReal(204.081632653061))*((py)*(py))))+((cj1)*(cj1)));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
{
IkReal dummyeval[1];
IkReal x8=(cj1)*(cj1);
IkReal x9=(px)*(px);
IkReal x10=((IkReal(204.081632653061))*(x8));
dummyeval[0]=((IkReal(-1.00000000000000))+(((IkReal(-1.00000000000000))*(x10)*(x9)))+(((IkReal(-408.163265306122))*(cj1)*(px)*(py)*(sj1)))+(((IkReal(204.081632653061))*(x9)))+(((x10)*((py)*(py)))));
if( IKabs(dummyeval[0]) < 0.0000010000000000  )
{
continue;

} else
{
{
IkReal j2array[4], cj2array[4], sj2array[4];
bool j2valid[4]={false};
_nj2 = 4;
IkReal x11=(cj1)*(cj1);
IkReal x12=(py)*(py);
IkReal x13=(px)*(px);
IkReal x14=((x11)*(x12));
IkReal x15=((IkReal(2.00000000000000))*(cj1)*(px)*(py)*(sj1));
IkReal x16=((IkReal(1.00000000000000))*(x11)*(x13));
IkReal x17=((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(((IkReal(-1.00000000000000))*(x15)))+(x13)+(x14))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(((IkReal(-1.00000000000000))*(x15)))+(x13)+(x14)))):(IkReal)1.0e30);
IkReal x18=((x13)*(x17));
if( (((x18)+(((x14)*(((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(((IkReal(-1.00000000000000))*(x15)))+(x13)+(((x11)*(x12))))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(((IkReal(-1.00000000000000))*(x15)))+(x13)+(((x11)*(x12)))))):(IkReal)1.0e30))))+(((IkReal(-1.00000000000000))*(x16)*(((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x15)))+(((IkReal(-1.00000000000000))*(x11)*(x13)))+(x13)+(x14))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x15)))+(((IkReal(-1.00000000000000))*(x11)*(x13)))+(x13)+(x14)))):(IkReal)1.0e30))))+(((IkReal(-0.00490000000000000))*(x17)))+(((IkReal(-1.00000000000000))*(x15)*(((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(x13)+(x14)+(((IkReal(-2.00000000000000))*(cj1)*(px)*(py)*(sj1))))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(x13)+(x14)+(((IkReal(-2.00000000000000))*(cj1)*(px)*(py)*(sj1)))))):(IkReal)1.0e30)))))) < (IkReal)-0.00001 )
    continue;
IkReal x19=IKsqrt(((x18)+(((x14)*(((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(((IkReal(-1.00000000000000))*(x15)))+(x13)+(((x11)*(x12))))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(((IkReal(-1.00000000000000))*(x15)))+(x13)+(((x11)*(x12)))))):(IkReal)1.0e30))))+(((IkReal(-1.00000000000000))*(x16)*(((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x15)))+(((IkReal(-1.00000000000000))*(x11)*(x13)))+(x13)+(x14))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x15)))+(((IkReal(-1.00000000000000))*(x11)*(x13)))+(x13)+(x14)))):(IkReal)1.0e30))))+(((IkReal(-0.00490000000000000))*(x17)))+(((IkReal(-1.00000000000000))*(x15)*(((IKabs(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(x13)+(x14)+(((IkReal(-2.00000000000000))*(cj1)*(px)*(py)*(sj1))))) != 0)?((IkReal)1/(((IkReal(-0.00490000000000000))+(((IkReal(-1.00000000000000))*(x16)))+(x13)+(x14)+(((IkReal(-2.00000000000000))*(cj1)*(px)*(py)*(sj1)))))):(IkReal)1.0e30))))));
cj2array[0]=x19;
cj2array[2]=((IkReal(-1.00000000000000))*(x19));
if( cj2array[0] >= -1-IKFAST_SINCOS_THRESH && cj2array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j2valid[0] = j2valid[1] = true;
    j2array[0] = IKacos(cj2array[0]);
    sj2array[0] = IKsin(j2array[0]);
    cj2array[1] = cj2array[0];
    j2array[1] = -j2array[0];
    sj2array[1] = -sj2array[0];
}
else if( isnan(cj2array[0]) )
{
    // probably any value will work
    j2valid[0] = true;
    cj2array[0] = 1; sj2array[0] = 0; j2array[0] = 0;
}
if( cj2array[2] >= -1-IKFAST_SINCOS_THRESH && cj2array[2] <= 1+IKFAST_SINCOS_THRESH )
{
    j2valid[2] = j2valid[3] = true;
    j2array[2] = IKacos(cj2array[2]);
    sj2array[2] = IKsin(j2array[2]);
    cj2array[3] = cj2array[2];
    j2array[3] = -j2array[2];
    sj2array[3] = -sj2array[2];
}
else if( isnan(cj2array[2]) )
{
    // probably any value will work
    j2valid[2] = true;
    cj2array[2] = 1; sj2array[2] = 0; j2array[2] = 0;
}
for(int ij2 = 0; ij2 < 4; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 4; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[3];
IkReal x20=IKsin(j2);
IkReal x21=IKcos(j2);
IkReal x22=(cj1)*(cj1);
IkReal x23=(py)*(py);
IkReal x24=(pz)*(pz);
IkReal x25=(x21)*(x21);
IkReal x26=((IkReal(0.0119000000000000))*(cj1)*(sj1));
IkReal x27=((IkReal(0.0700000000000000))*(x20));
IkReal x28=((py)*(x20));
IkReal x29=((pz)*(sj1)*(x21));
IkReal x30=((x24)*(x25));
evalcond[0]=((((IkReal(-1.00000000000000))*(x27)))+(((cj1)*(x28)))+(((IkReal(-1.00000000000000))*(px)*(sj1)*(x20))));
evalcond[1]=((((IkReal(-0.0850000000000000))*(sj1)))+(((IkReal(-1.00000000000000))*(cj1)*(x27)))+(x28)+(x29));
evalcond[2]=((IkReal(0.00722500000000000))+(((IkReal(-1.00000000000000))*(x30)))+(((x20)*(x25)*(x26)))+(((IkReal(-1.00000000000000))*(x23)))+(((x23)*(x25)))+(((x22)*(x30)))+(((x26)*((x20)*(x20)*(x20))))+(((IkReal(-0.00490000000000000))*(x22)*(x25)))+(((IkReal(-0.00232500000000000))*(x22)))+(((IkReal(-2.00000000000000))*(x28)*(x29))));
if( IKabs(evalcond[0]) > 0.000001  || IKabs(evalcond[1]) > 0.000001  || IKabs(evalcond[2]) > 0.000001  )
{
continue;
}
}

IkReal soleval[1];
soleval[0]=((((IkReal(-1.00000000000000))*(pz)*(sj2)))+(((cj2)*(((((cj1)*(px)))+(((py)*(sj1))))))));
if( soleval[0] > 0.0000000000000000  )
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(2);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j1;
vinfos[0].indices[0] = _ij1[0];
vinfos[0].indices[1] = _ij1[1];
vinfos[0].maxsolutions = _nj1;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j2;
vinfos[1].indices[0] = _ij2[0];
vinfos[1].indices[1] = _ij2[1];
vinfos[1].maxsolutions = _nj2;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[2], cj2array[2], sj2array[2];
bool j2valid[2]={false};
_nj2 = 2;
IkReal x31=((((IkReal(-0.0700000000000000))*(cj1)))+(py));
if( IKabs(((pz)*(sj1))) < IKFAST_ATAN2_MAGTHRESH && IKabs(x31) < IKFAST_ATAN2_MAGTHRESH )
    continue;
IkReal x32=((IkReal(1.00000000000000))*(IKatan2(((pz)*(sj1)), x31)));
if( ((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1)))))) < (IkReal)-0.00001 )
    continue;
if( (((IkReal(0.0850000000000000))*(sj1)*(((IKabs(IKabs(IKsqrt((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1)))))))) != 0)?((IkReal)1/(IKabs(IKsqrt((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1))))))))):(IkReal)1.0e30)))) < -1-IKFAST_SINCOS_THRESH || (((IkReal(0.0850000000000000))*(sj1)*(((IKabs(IKabs(IKsqrt((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1)))))))) != 0)?((IkReal)1/(IKabs(IKsqrt((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1))))))))):(IkReal)1.0e30)))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x33=IKasin(((IkReal(0.0850000000000000))*(sj1)*(((IKabs(IKabs(IKsqrt((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1)))))))) != 0)?((IkReal)1/(IKabs(IKsqrt((((x31)*(x31))+((((pz)*(pz))*((sj1)*(sj1))))))))):(IkReal)1.0e30))));
j2array[0]=((((IkReal(-1.00000000000000))*(x32)))+(x33));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
j2array[1]=((IkReal(3.14159265358979))+(((IkReal(-1.00000000000000))*(x33)))+(((IkReal(-1.00000000000000))*(x32))));
sj2array[1]=IKsin(j2array[1]);
cj2array[1]=IKcos(j2array[1]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
if( j2array[1] > IKPI )
{
    j2array[1]-=IK2PI;
}
else if( j2array[1] < -IKPI )
{    j2array[1]+=IK2PI;
}
j2valid[1] = true;
for(int ij2 = 0; ij2 < 2; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 2; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[3];
IkReal x34=IKsin(j2);
IkReal x35=(cj1)*(cj1);
IkReal x36=(px)*(px);
IkReal x37=(py)*(py);
IkReal x38=IKcos(j2);
IkReal x39=(pz)*(pz);
IkReal x40=((cj1)*(sj1));
IkReal x41=(x38)*(x38);
IkReal x42=((IkReal(1.00000000000000))*(x37));
IkReal x43=((IkReal(1.00000000000000))*(x36));
IkReal x44=((IkReal(2.00000000000000))*(px)*(py));
IkReal x45=((py)*(x34));
IkReal x46=((IkReal(0.00490000000000000))*(x41));
IkReal x47=((x39)*(x41));
IkReal x48=((x35)*(x41));
evalcond[0]=((((IkReal(-0.0700000000000000))*(x34)))+(((cj1)*(x45)))+(((IkReal(-1.00000000000000))*(px)*(sj1)*(x34))));
evalcond[1]=((IkReal(0.00490000000000000))+(((x35)*(x36)))+(((x36)*(x41)))+(((x40)*(x44)))+(((IkReal(-1.00000000000000))*(x46)))+(((IkReal(-1.00000000000000))*(x43)))+(((x37)*(x48)))+(((IkReal(-1.00000000000000))*(x40)*(x41)*(x44)))+(((IkReal(-1.00000000000000))*(x43)*(x48)))+(((IkReal(-1.00000000000000))*(x35)*(x42))));
evalcond[2]=((IkReal(0.00722500000000000))+(((IkReal(0.0119000000000000))*(x34)*(x40)*(x41)))+(((IkReal(-1.00000000000000))*(x47)))+(((x35)*(x47)))+(((IkReal(-1.00000000000000))*(x42)))+(((x37)*(x41)))+(((IkReal(0.0119000000000000))*(x40)*((x34)*(x34)*(x34))))+(((IkReal(-0.00232500000000000))*(x35)))+(((IkReal(-2.00000000000000))*(pz)*(sj1)*(x38)*(x45)))+(((IkReal(-1.00000000000000))*(x35)*(x46))));
if( IKabs(evalcond[0]) > 0.000001  || IKabs(evalcond[1]) > 0.000001  || IKabs(evalcond[2]) > 0.000001  )
{
continue;
}
}

IkReal soleval[1];
soleval[0]=((((IkReal(-1.00000000000000))*(pz)*(sj2)))+(((cj2)*(((((cj1)*(px)))+(((py)*(sj1))))))));
if( soleval[0] > 0.0000000000000000  )
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(2);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j1;
vinfos[0].indices[0] = _ij1[0];
vinfos[0].indices[1] = _ij1[1];
vinfos[0].maxsolutions = _nj1;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j2;
vinfos[1].indices[0] = _ij2[0];
vinfos[1].indices[1] = _ij2[1];
vinfos[1].maxsolutions = _nj2;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}
}
return solutions.GetNumSolutions()>0;
}

};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "a9d5a8c4aae099ac57da051863bf79c7"; }

IKFAST_API const char* GetIkFastVersion() { return IKFAST_STRINGIZE(IKFAST_VERSION); }

#ifdef IKFAST_NAMESPACE

int ik_solve(IkReal eerot[9], IkReal eetrans[3], std::vector<std::vector<IkReal> >& sol_joint, std::vector<IkReal> free_joint)
{
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
	for(std::size_t i = 0; i < vfree.size(); ++i)
		vfree[i] = free_joint[i];
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        return -1;
    }

	sol_joint.resize(solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
		sol_joint[i].resize(solvalues.size());
        for( std::size_t j = 0; j < solvalues.size(); ++j)
			sol_joint[i][j] = solvalues[j];
    }
    return 0;
}

} // end namespace
#endif

#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_LOOKAT3D;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
    return 0;
}

#endif