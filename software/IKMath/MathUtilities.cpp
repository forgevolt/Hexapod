#include <algorithm>
#include "MathUtilities.h"


// ---- Vector2 ---------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
bool Vector2::operator==(const Vector2& v) const
{
  return floatEquals(x, v.x) && floatEquals(y, v.y);
}

// ----------------------------------------------------------------------------------------
bool Vector2::operator!=(const Vector2& v) const
{
  return !floatEquals(x, v.x) || !floatEquals(y, v.y);
}

// ----------------------------------------------------------------------------------------
String Vector2::toString() const
{
  // "%.2f" of a float can need ~44 characters (FLT_MAX), so no fixed buffer covers every
  // possible value. Size for the ranges actually used here, and fall back to the compact
  // "%g" form instead of truncating: a runaway coordinate should be visible in the log,
  // not quietly cut in half.
  char buffer[40];

  const int n = snprintf(buffer, sizeof(buffer), "%.2f, %.2f", x, y);
  if (n < 0 || n >= static_cast<int>(sizeof(buffer)))
    snprintf(buffer, sizeof(buffer), "%g, %g", x, y);

  return buffer;
}
  
// ----------------------------------------------------------------------------------------
Vector2::operator String() const
{
  return toString();
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator+(const Vector2& v) const 
{ 
  return Vector2(x + v.x, y + v.y);
}

// ----------------------------------------------------------------------------------------
Vector2& Vector2::operator+=(const Vector2& v)
{
  set(*this + v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator-(const Vector2& v) const
{ 

  return Vector2(x - v.x, y - v.y);
}

// ----------------------------------------------------------------------------------------
Vector2& Vector2::operator-=(const Vector2& v) 
{
  set(*this - v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator-() const
{
  return Vector2(-x, -y);
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator*(const Vector2& v) const
{
  return Vector2(x*v.x, y*v.y);
}

// ----------------------------------------------------------------------------------------
Vector2& Vector2::operator*=(const Vector2& v)
{
  set(*this * v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator*(const float scale) const
{
  return Vector2(x * scale, y * scale);
}

// ----------------------------------------------------------------------------------------
Vector2& Vector2::operator*=(const float scale) 
{
   x *= scale;
   y *= scale;
   return *this;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator/(const Vector2& v) const
{
  return Vector2(v.x == 0.0f ? 0.0f : x/v.x,
                 v.y == 0.0f ? 0.0f : y/v.y);
}

// ----------------------------------------------------------------------------------------
Vector2& Vector2::operator/=(const Vector2& v)
{
  set(*this / v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::operator/(const float div) const
{
  if (div == 0.0f)
    return Vector2(0.0f, 0.0f);

  return Vector2(x / div, y / div);
}
    
// ----------------------------------------------------------------------------------------
Vector2& Vector2::operator/=(const float div)
{
  if (div == 0.0f)
  {
    x = 0.0f;
    y = 0.0f;
    return *this;
  }

  x /= div;
  y /= div;
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::clamp(float min, float max) const
{
  Vector2 result;

  result.x = std::clamp(x, min, max);
  result.y = std::clamp(y, min, max);

  return result;
}

// ----------------------------------------------------------------------------------------
Vector2 Vector2::normalize() const
{
  Vector2 result;
  float length = sqrtf((x*x) + (y*y));

  if (length > 0)
  {
    float ilength = 1.0f/length;
    result.x = x*ilength;
    result.y = y*ilength;
  }

  return result;  
}

// ----------------------------------------------------------------------------------------
float Vector2::length() const
{
  return sqrtf((x*x) + (y*y));
}

// ----------------------------------------------------------------------------------------
float Vector2::lengthSqr() const
{
  return (x*x) + (y*y);
}


// ---- Vector3 ---------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
bool Vector3::operator==(const Vector3& v) const
{
  return floatEquals(x, v.x) && floatEquals(y, v.y) && floatEquals(z, v.z);
}

// ----------------------------------------------------------------------------------------
bool Vector3::operator!=(const Vector3& v) const
{
  return !floatEquals(x, v.x) || !floatEquals(y, v.y) || !floatEquals(z, v.z);
}

// ----------------------------------------------------------------------------------------
String Vector3::toString() const
{
  // See Vector2::toString() for why this is sized the way it is.
  char buffer[60];

  const int n = snprintf(buffer, sizeof(buffer), "%.2f, %.2f, %.2f", x, y, z);
  if (n < 0 || n >= static_cast<int>(sizeof(buffer)))
    snprintf(buffer, sizeof(buffer), "%g, %g, %g", x, y, z);

  return buffer;
}
  
// ----------------------------------------------------------------------------------------
Vector3::operator String() const
{
  return toString();
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator+(const Vector3& v) const 
{ 
  return Vector3(x + v.x, y + v.y, z + v.z);
}

// ----------------------------------------------------------------------------------------
Vector3& Vector3::operator+=(const Vector3& v)
{
  set(*this + v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator+(const Vector2& v) const 
{ 
  return Vector3(x + v.x, y + v.y, z);
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator-(const Vector3& v) const
{ 
  return Vector3(x - v.x, y - v.y, z - v.z);
}

// ----------------------------------------------------------------------------------------
Vector3& Vector3::operator-=(const Vector3& v) 
{
  set(*this - v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator-(const Vector2& v) const
{ 
  return Vector3(x - v.x, y - v.y, z);
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator-() const
{
  return Vector3(-x, -y, -z);
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator*(const Vector3& v) const
{
  return Vector3(x*v.x, y*v.y, z*v.z);
}

// ----------------------------------------------------------------------------------------
Vector3& Vector3::operator*=(const Vector3& v)
{
  set(*this * v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator*(const float scale) const
{
  return Vector3(x * scale, y * scale, z * scale);
}

// ----------------------------------------------------------------------------------------
Vector3& Vector3::operator*=(const float scale) 
{
   x *= scale;
   y *= scale;
   z *= scale;
   return *this;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator/(const Vector3& v) const
{
  return Vector3(v.x == 0.0f ? 0.0f : x/v.x,
                 v.y == 0.0f ? 0.0f : y/v.y,
                 v.z == 0.0f ? 0.0f : z/v.z);
}

// ----------------------------------------------------------------------------------------
Vector3& Vector3::operator/=(const Vector3& v)
{
  set(*this / v);
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::operator/(const float div) const
{
  if (div == 0.0f)
    return Vector3(0.0f, 0.0f, 0.0f);

  return Vector3(x / div, y / div, z / div);
}
    
// ----------------------------------------------------------------------------------------
Vector3& Vector3::operator/=(const float div)
{
  if (div == 0.0f)
  {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    return *this;
  }

  x /= div;
  y /= div;
  z /= div;
  return *this;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::clamp(float min, float max) const
{
  Vector3 result;

  result.x = std::clamp(x, min, max);
  result.y = std::clamp(y, min, max);
  result.z = std::clamp(z, min, max);

  return result;
}

// ----------------------------------------------------------------------------------------
Vector3 Vector3::normalize() const
{
  Vector3 result(*this);

  float length = sqrtf(x*x + y*y + z*z);
  if (length != 0.0f)
  {
      float ilength = 1.0f/length;

      result.x *= ilength;
      result.y *= ilength;
      result.z *= ilength;
  }

  return result;  
}

// ----------------------------------------------------------------------------------------
float Vector3::length() const
{
  return sqrtf((x*x) + (y*y) + (z*z));
}

// ----------------------------------------------------------------------------------------
float Vector3::lengthSqr() const
{
  return (x*x) + (y*y) + (z*z);
}

// ----------------------------------------------------------------------------------------
float Vector3::distance(const Vector3& v) const
{
  float dx = v.x - x;
  float dy = v.y - y;
  float dz = v.z - z;

  return sqrtf(dx*dx + dy*dy + dz*dz);
}

// ----------------------------------------------------------------------------------------
float Vector3::distanceSqr(const Vector3& v) const
{
  float dx = v.x - x;
  float dy = v.y - y;
  float dz = v.z - z;

  return dx*dx + dy*dy + dz*dz;
}

