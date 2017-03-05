#ifndef _QUATERNION_H
#define _QUATERNION_H

//This is a (static-ized) port of the libgdx Quaternion & Vector3 Classes made by vesuvio and xoppa, ported by VaTTeRGeR.

#define Q_FP_ERR                0.000001

#define Q_PI                    3.141592
#define Q_PI2                   6.283185

#define Q_RADDEG                57.29577
#define Q_DEGRAD                0.01745329

typedef struct {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  float w = 1.0;
} Quaternion;

typedef struct {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
} Vector3;

Vector3 tmp0;

Quaternion tmp1;
Quaternion tmp2;

void qidt(Quaternion& q0) {
  q0.x = 0.0;
  q0.y = 0.0;
  q0.z = 0.0;
  q0.w = 1.0;
}

void qconjugate(Quaternion& q0) {
  q0.x = -q0.x;
  q0.y = -q0.y;
  q0.z = -q0.z;
}

void qset(Vector3& v0, float x, float y, float z) {
  v0.x = x;
  v0.y = y;
  v0.z = z;
}

void qset(Vector3& v0, Vector3& v1) {
  v0.x = v1.x;
  v0.y = v1.y;
  v0.z = v1.z;
}

void qset(Quaternion& q0, Quaternion& q1) {
  q0.x = q1.x;
  q0.y = q1.y;
  q0.z = q1.z;
  q0.w = q1.w;
}

void qset(Quaternion& q0, float x, float y, float z, float w) {
  q0.x = x;
  q0.y = y;
  q0.z = z;
  q0.w = w;
}

float qscl(Vector3& v0, float scl) {
  v0.x *= scl;
  v0.y *= scl;
  v0.z *= scl;
}

float qlen(Vector3& v0) {
  return sqrt(v0.x*v0.x + v0.y*v0.y + v0.z*v0.z);
}

float qlen2(Vector3& v0) {
  return v0.x*v0.x + v0.y*v0.y + v0.z*v0.z;
}

float qlen(Quaternion& q0) {
  return sqrt(q0.x*q0.x + q0.y*q0.y + q0.z*q0.z + q0.w*q0.w);
}

float qlen2(Quaternion& q0) {
  return q0.x*q0.x + q0.y*q0.y + q0.z*q0.z + q0.w*q0.w;
}

float qdot(Vector3& v0, Vector3& v1) {
  return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

void qnor(Quaternion& q0) {
  float len = qlen2(q0);
  if(len != 0.0 &! abs(len - 1.0) <= Q_FP_ERR) {
    len = sqrt(len);
    q0.x /= len;
    q0.y /= len;
    q0.z /= len;
    q0.w /= len;
  }
}

void qnor(Vector3& v0) {
  float len = qlen2(v0);
  if(len != 0.0 &! abs(len - 1.0) <= Q_FP_ERR) {
    len = sqrt(len);
    v0.x /= len;
    v0.y /= len;
    v0.z /= len;
  }
}

void qmul(Quaternion& q0, Quaternion& q1) {
  float nX = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y;
  float nY = q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z;
  float nZ = q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x;
  float nW = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z;
  q0.x = nX;
  q0.y = nY;
  q0.z = nZ;
  q0.w = nW;
}

void qmulLeft(Quaternion& q0, Quaternion& q1) {
  float nX = q1.w * q0.x + q1.x * q0.w + q1.y * q0.z - q1.z * q0.y;
  float nY = q1.w * q0.y + q1.y * q0.w + q1.z * q0.x - q1.x * q0.z;
  float nZ = q1.w * q0.z + q1.z * q0.w + q1.x * q0.y - q1.y * q0.x;
  float nW = q1.w * q0.w - q1.x * q0.x - q1.y * q0.y - q1.z * q0.z;
  q0.x = nX;
  q0.y = nY;
  q0.z = nZ;
  q0.w = nW;
}

void qtransform(Quaternion& q0, Vector3& v0) {
  //tmp2 <- q0 * ( (tmp1<-[v0,0]) * (tmp2<-q0') )
  
  qset(tmp2,q0);
  qconjugate(tmp2);

  qset(tmp1, v0.x, v0.y, v0.z, 0.0);

  qmulLeft(tmp2, tmp1);
  qmulLeft(tmp2, q0);
  
  v0.x = tmp2.x;
  v0.y = tmp2.y;
  v0.z = tmp2.z;
}

void qsetFromAxis(Quaternion& q0, Vector3& v0, float angle) {
  angle *= Q_DEGRAD;
  float d = qlen(v0);
  
  if(d == 0.0) {
    qidt(q0);
  } else {
    d = 1.0/d;
    
    float l_ang = angle < 0.0 ? (Q_PI2 - fmod(-angle, Q_PI2)) : fmod(angle, Q_PI2);
    
    float l_sin = sin(l_ang / 2.0);
    
    float l_cos = cos(l_ang / 2.0);
    
    qset(q0, d * v0.x * l_sin, d * v0.y * l_sin, d * v0.z * l_sin, l_cos);
    qnor(q0);
  }
}

void qsetFromCross(Quaternion& q0, Vector3& v0, Vector3& v1) {
  float dot = qdot(v0, v1);
  dot = min(dot, 1.0);
  dot = max(dot, -1.0);

  float angle = acos(dot);

  tmp0.x = v0.y * v1.z - v0.z * v1.y;
  tmp0.y = v0.z * v1.x - v0.x * v1.z;
  tmp0.z = v0.x * v1.y - v0.y * v1.x;
  
  qsetFromAxis(q0, tmp0, angle * Q_RADDEG);
}

void qlerp(Vector3& v0, Vector3& v1, float alpha) {
  v0.x += alpha * (v1.x - v0.x);
  v0.y += alpha * (v1.y - v0.y);
  v0.z += alpha * (v1.z - v0.z);
}

void qslerp(Vector3& v0, Vector3& v1, float alpha) {
  float dot = qdot(v0,v1);
  if(dot > 0.9995 || dot < -0.9995) {
    qlerp(v0, v1, alpha);
  } else {
    float theta0 = acos(dot);
    float theta = theta0 * alpha;

    float st = sin(theta);
    float tx = v1.x - v0.x * dot;
    float ty = v1.y - v0.y * dot;
    float tz = v1.z - v0.z * dot;
    float l2 = tx * tx + ty * ty + tz * tz;
    float dl = st * ((l2 < 0.0001) ? (1.0) : (1.0/sqrt(l2)));

    qscl(v0, cos(theta));
    
    v0.x += tx * dl;
    v0.y += ty * dl;
    v0.z += tz * dl;
    
    qnor(v0);
  }
}

#endif
