#pragma once

#include <Arduino.h>
#include <math.h>    // Required for: sinf(), cosf(), tan(), atan2f(), sqrtf(), floor(), fminf(), fmaxf(), fabsf()
#include <cmath>     // std:: float overloads of the above
#include <algorithm> // std::clamp

// ----------------------------------------------------------------------------------------

constexpr float cPI = 3.14159265358979323846f;
constexpr float cTwoPI = 2.0f * cPI;

constexpr float cEPSILON = 0.000001f;

constexpr float deg2rad = cPI/180.0f;
constexpr float rad2deg = 180.0f/cPI;

template<typename T>
constexpr T sqr(T a) { return a * a; }

// Canonical wrap to [−π, π]
inline float wrapPi(float angleRad)
{
  angleRad = fmodf(angleRad + cPI, cTwoPI);
  if (angleRad < 0)
    angleRad += cTwoPI;
  return angleRad - cPI;
}

// An Arduino style map function supporting float
// If fromLow == fromHigh (degenerate/zero-width input range), returns toLow rather than
// dividing by zero - callers passing a fixed input range should ensure it's non-degenerate.
inline float mapf(float x, float fromLow, float fromHigh, float toLow, float toHigh)
{
  if (fromHigh == fromLow)
    return toLow;

  return (x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

// Check whether two given floats are almost equal
inline bool floatEquals(float x, float y)
{
  return (fabsf(x - y)) <= (cEPSILON*fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))));
}

// Calculate linear interpolation between two floats.
inline float lerpf(float start, float end, float amount)
{
  amount = std::clamp(amount, 0.0f, 1.0f);   // matches the Vector lerp() overloads
  return start + amount * (end - start);
}

// Normalize input value to a 0.0 - 1.0 range
// If start == end (degenerate/zero-width range), returns 0.0f rather than dividing by zero.
inline float normalize(float value, float start, float end)
{
  if (end == start)
    return 0.0f;

  return (value - start)/(end - start);
}

// Wrap input value from min to max
// If min == max (degenerate/zero-width range), returns min rather than dividing by zero.
inline float wrap(float value, float min, float max)
{
  if (max == min)
    return min;

  return value - (max - min)*floorf((value - min)/(max - min));
}

inline float smoothstep(float value)
{
  // Clamp t to the [0, 1] range to prevent undefined behavior
  float t = std::clamp(value, 0.0f, 1.0f);
  
  // The cubic formula: 3t^2 - 2t^3
  return t * t * (3.0f - 2.0f * t);
}

inline float smoothstep(float value, float start, float end)
{
  return smoothstep(normalize(value, start, end));
}

inline float smootherstep(float value)
{
  // Clamp t to the [0, 1] range to prevent undefined behavior
  float t = std::clamp(value, 0.0f, 1.0f);
  
  // smootherstep f(x) = 6*x^5 - 15*x^4 + 10*x^3
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

// A First-Order Low-Pass Filter (Time-Constant based).
// This function "softens" sudden changes in a signal (like joystick input) 
// to simulate physical inertia or momentum. It is frame-rate independent.
// After ~5×tau the output reaches ~99% of the target value
// (e.g. tau = 100 ms → ~500 ms to effectively settle)
inline float lowPassFilter(float current, // The current smoothed value (from the previous frame).
                           float target,  // The new raw input value we are trying to reach
                           float tau_ms,  // The Time Constant (in ms). This is the "weight."
                           float dt_ms)   // The time elapsed since the last update (Delta Time in ms).
{
  if (tau_ms <= 0.0f)
      return target;

  float alpha = dt_ms / (tau_ms + dt_ms);
  float result = current + (target - current) * alpha;

  return result;
}

// ---- Vector2 ---------------------------------------------------------------------------
// 2d vector math. Code is a C++ port from the raylib implementation
// Refer to https://github.com/raysan5/raylib

class Vector2
{
  public: 
    float x, y;

  public:
   Vector2()                   : x(0),  y(0)    {}
    Vector2(float vx, float vy) : x(vx), y(vy)   {}

    // explicit: a bare float is not a vector, so it must not convert to one implicitly.
    // Spell out (vx, 0), or use Vector2() where a zero vector is meant.
    explicit Vector2(float vx)  : x(vx), y(0)    {}

    Vector2(const Vector2&)            = default;
    Vector2& operator=(const Vector2&) = default;

    // Check whether two given vectors are almost equal
    bool operator==(const Vector2& v) const;
    bool operator!=(const Vector2& v) const;

    // Convert to string
    [[nodiscard]] String toString() const;
    operator String() const;

    // Add two vectors (v1 - v2)
    Vector2 operator+(const Vector2& v) const;
    Vector2& operator+=(const Vector2& v);

    // Subtract two vectors (v1 - v2)
    Vector2 operator-(const Vector2& v) const;
    Vector2& operator-=(const Vector2& v);

    // Negate vector
    Vector2 operator-() const; 

    // Multiply vector by vector
    Vector2 operator*(const Vector2& v) const;
    Vector2& operator*=(const Vector2& v); 

    // Scale vector (multiply by value)
    Vector2 operator*(const float scale) const;
    Vector2& operator*=(const float scale);

    // Divide vector by vector
    Vector2 operator/(const Vector2& v) const;
    Vector2& operator/=(const Vector2& v);
    
    // Scale vector (divide by value)
    Vector2 operator/(const float div) const;
    Vector2& operator/=(const float div);

    // Clamp the components of the vector between
    [[nodiscard]] Vector2 clamp(float min, float max) const;

    // Normalize provided vector
    [[nodiscard]] Vector2 normalize() const;

    // Calculate vector length
    [[nodiscard]] float length() const;

    // Calculate vector square length
    [[nodiscard]] float lengthSqr() const;

  protected:
    void set(const Vector2& v) 
    {
        x = v.x;
        y = v.y;
    }
};

// Calculate linear interpolation between two vectors
inline Vector2 lerp(const Vector2& v1, const Vector2& v2, float u)
{
  // Ensure u in [0, 1]
  u = std::clamp(u, 0.0f, 1.0f);

  return v1 + (v2 - v1) * u;
}

// Quadratic Bezier for 2D Vectors
inline Vector2 bezier2(const Vector2& p0, const Vector2& p1, const Vector2& p2, float u) 
{
  u = std::clamp(u, 0.0f, 1.0f);
  float v = 1.0f - u;

  return (p0 * (v * v))        + 
         (p1 * (2.0f * v * u)) + 
         (p2 * (u * u));
}

// Maps a coordinate from a square input [-1, 1] to a unit disk.
// This ensures the length (magnitude) never exceeds 1.0 even in the corners.
inline Vector2 circularNormalization(float x, float y) 
{
  // Clamped defensively (rather than merely assumed) - an out-of-range input would
  // otherwise make (1 - y2/2) or (1 - x2/2) go negative, producing NaN from sqrtf().
  x = std::clamp(x, -1.0f, 1.0f);
  y = std::clamp(y, -1.0f, 1.0f);

  float x2 = x * x;
  float y2 = y * y;

  // Standard analytical mapping: 
  float nx = x * sqrtf(1.0f - (y2 / 2.0f));
  float ny = y * sqrtf(1.0f - (x2 / 2.0f));

  return Vector2(nx, ny);
}


// ---- Vector3 ---------------------------------------------------------------------------
// 3d vector math. Code is a C++ port from the raylib implementation
// Refer to https://github.com/raysan5/raylib

class Vector3
{
  public: 
    float x, y, z;

  public:
    Vector3()                             : x(0),  y(0),  z(0)  {}
    Vector3(float vx, float vy, float vz) : x(vx), y(vy), z(vz) {}
    explicit Vector3(float vx, float vy)  : x(vx), y(vy), z(0)  {}

    // explicit: a bare float is not a vector, so it must not convert to one implicitly.
    // Spell out (vx, 0, 0), or use Vector3() where a zero vector is meant.
    explicit Vector3(float vx)            : x(vx), y(0),  z(0)  {}

    Vector3(const Vector3&)            = default;
    Vector3& operator=(const Vector3&) = default;

    // Check whether two given vectors are almost equal
    bool operator==(const Vector3& v) const;
    bool operator!=(const Vector3& v) const;

    // Convert to string
    [[nodiscard]] String toString() const;
    operator String() const;

    // Add two vectors (v1 + v2)
    Vector3 operator+(const Vector3& v) const;
    Vector3& operator+=(const Vector3& v);
    Vector3 operator+(const Vector2& v) const;

    // Subtract two vectors (v1 - v2)
    Vector3 operator-(const Vector3& v) const;
    Vector3& operator-=(const Vector3& v);
    Vector3 operator-(const Vector2& v) const;

    // Negate vector
    Vector3 operator-() const; 

    // Multiply vector by vector
    Vector3 operator*(const Vector3& v) const;
    Vector3& operator*=(const Vector3& v); 
    
    // Scale vector (multiply by value)
    Vector3 operator*(const float scale) const;
    Vector3& operator*=(const float scale);

    // Divide vector by vector
    Vector3 operator/(const Vector3& v) const;
    Vector3& operator/=(const Vector3& v);

    // Scale vector (divide by value)
    Vector3 operator/(const float div) const;
    Vector3& operator/=(const float div);
    
    // Normalize provided vector
    [[nodiscard]] Vector3 normalize() const;

    // Calculate vector length
    [[nodiscard]] float length() const;

    // Calculate vector square length
    [[nodiscard]] float lengthSqr() const;

    // Calculate distance between two vectors
    [[nodiscard]] float distance(const Vector3& v) const;

    // Calculate square distance between two vectors
    [[nodiscard]] float distanceSqr(const Vector3& v) const; 

    // Clamp the components of the vector between
    [[nodiscard]] Vector3 clamp(float min, float max) const;

  protected:
    void set(const Vector3& v) 
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }
};

// Rotate a vector around the X-axis
inline Vector3 rotateAroundX(const Vector3& v, float angleRad)
{
  float cosTheta = cosf(angleRad);
  float sinTheta = sinf(angleRad);

  Vector3 rotated;

  rotated.x = v.x; // X remains unchanged
  rotated.y = v.y * cosTheta - v.z * sinTheta;
  rotated.z = v.y * sinTheta + v.z * cosTheta;

  return rotated;
}

// Rotate a vector around the Y-axis
inline Vector3 rotateAroundY(const Vector3& v, float angleRad)
{
  float cosTheta = cosf(angleRad);
  float sinTheta = sinf(angleRad);

  Vector3 rotated;

  rotated.x = v.x * cosTheta + v.z * sinTheta;
  rotated.y = v.y; // Y remains unchanged
  rotated.z = -v.x * sinTheta + v.z * cosTheta;

  return rotated;
}

// Rotate a vector around the Z-axis (2D rotation)
inline Vector3 rotateAroundZ(const Vector3& v, float angleRad) 
{
  float cosTheta = cosf(angleRad);
  float sinTheta = sinf(angleRad);

  Vector3 rotated;
  // Standard 2D rotation formula applied to X and Y
  rotated.x = v.x * cosTheta - v.y * sinTheta;
  rotated.y = v.x * sinTheta + v.y * cosTheta;
  rotated.z = v.z; // Z remains unchanged 

  return rotated;
}

// Rotate vector by Euler angles (X = roll, Y = pitch, Z = yaw)
// Rotation order: X -> Y -> Z
inline Vector3 rotateXYZ(const Vector3& v, const Vector3& angleRad)
{
  // Precompute trig once
  const float cx = cosf(angleRad.x);
  const float sx = sinf(angleRad.x);

  const float cy = cosf(angleRad.y);
  const float sy = sinf(angleRad.y);

  const float cz = cosf(angleRad.z);
  const float sz = sinf(angleRad.z);

  // ----- Rotate around X -----
  const float x1 = v.x;
  const float y1 = v.y * cx - v.z * sx;
  const float z1 = v.y * sx + v.z * cx;

  // ----- Rotate around Y -----
  const float x2 = x1 * cy + z1 * sy;
  const float y2 = y1;
  const float z2 = -x1 * sy + z1 * cy;

  // ----- Rotate around Z -----
  Vector3 result;
  result.x = x2 * cz - y2 * sz;
  result.y = x2 * sz + y2 * cz;
  result.z = z2;

  return result;
}

// Calculate linear interpolation between two vectors
inline Vector3 lerp(const Vector3& v1, const Vector3& v2, float u)
{
  // Ensure u in [0, 1]
  u = std::clamp(u, 0.0f, 1.0f);

  return v1 + (v2 - v1) * u;
}

// Calculate smoothstep interpolation between two vectors
inline Vector3 smoothstep(const Vector3& v1, const Vector3& v2, float u)
{
  // Ensure u in [0, 1]
  u = std::clamp(u, 0.0f, 1.0f);

  // smoothstep f(x) = 3*x^2 - 2*x^3 
  float s = u * u * (3.0f - 2.0f * u); 
    
  return v1 + (v2 - v1) * s;
}

// Calculate minjerk interpolation between two vectors
inline Vector3 minjerk(const Vector3& v1, const Vector3& v2, float u)
{
  // Ensure u in [0, 1]
  u = std::clamp(u, 0.0f, 1.0f);

  // minjerk f(x) = 10x^3 - 15x^4 + 6x^5 
  float s = u*u*u*(10 - 15*u + 6*u*u);

  return v1 + (v2 - v1) * s;
}

// Cubic Bezier Curve interpolation
inline Vector3 bezier3(
  const Vector3& p0, // Start: Where the path begins.
  const Vector3& p1, // Control Point 1: Pulls the curve away from the start.
  const Vector3& p2, // Control Point 2: Pulls the curve toward the end.
  const Vector3& p3, // End: Where the path finishes.
  float u)           // [0, 1]
{
  u = std::clamp(u, 0.0f, 1.0f);

  // The formula: (1-u)^3*P0 + 3*(1-u)^2*u*P1 + 3*(1-u)*u^2*P2 + u^3*P3

  float v = 1.0f - u;
  float b0 = v*v*v;
  float b1 = 3*v*v*u;
  float b2 = 3*v*u*u;
  float b3 = u*u*u;

  return p0*b0 + p1*b1 + p2*b2 + p3*b3;
}

