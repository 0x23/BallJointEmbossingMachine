// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once
#include <cmath>

//--- Vec3F -----------------------------------------------------------------------------

class Vec3F {
  public:
    Vec3F() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3F(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3F operator+(const Vec3F& v) const { return Vec3F(x + v.x, y + v.y, z + v.z); }
    Vec3F operator-(const Vec3F& v) const { return Vec3F(x - v.x, y - v.y, z - v.z); }
    Vec3F operator*(float s) const { return Vec3F(x * s, y * s, z * s); }
    Vec3F operator/(float s) const { float t = 1.0f/s; return Vec3F(x*t, y*t, z*t); }

    float dot(const Vec3F& v) const { return x * v.x + y * v.y + z * v.z; }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    float sqr_length() const { return x * x + y * y + z * z; }
    Vec3F cross(const Vec3F& v) const {
        return Vec3F(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    Vec3F normalized() const {
        float len = length();
        return len > 0.0f ? (*this / len) : Vec3F(0, 0, 0);
    }

  public:
    float x, y, z;
};

//--- QuaternionF -----------------------------------------------------------------------

class QuaternionF {
  public:
    QuaternionF() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    QuaternionF(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static QuaternionF from_axis_angle(const Vec3F& axis, float angle_rad) {
        Vec3F naxis = axis.normalized();
        float half_angle = 0.5f * angle_rad;
        float s = std::sin(half_angle);
        return QuaternionF(std::cos(half_angle), naxis.x * s, naxis.y * s, naxis.z * s);
    }

    static QuaternionF from_rot_vec(const Vec3F& rot_vec) {
        float length = rot_vec.length();
        Vec3F naxis = rot_vec/length;
        float half_angle = 0.5f * length;
        float s = std::sin(half_angle);
        return QuaternionF(std::cos(half_angle), naxis.x * s, naxis.y * s, naxis.z * s);
    }

    void to_axis_angle(Vec3F& axis, float& angle) const {
        angle = 2.0f * std::acos(std::fmax(-1.0f, std::fmin(1.0f, w)));

        float sin_half_angle = std::sqrt(1.0f - w * w);

        if (sin_half_angle < 1e-6f)
          axis = Vec3F(1.0f, 0.0f, 0.0f);
        else
          axis = Vec3F(x, y, z) / sin_half_angle;
    }

    float angle() {
      // Clamp to avoid domain errors due to floating point
      float angle = 2.0f * std::acos(std::fmax(-1.0f, std::fmin(1.0f, w)));
      return angle;  // In radians, range: [0, π]
    }

    // return inverse of a normalized quaternion
    QuaternionF normalized_inverse() const {
      return QuaternionF(w, -x, -y, -z);
    }

    QuaternionF operator*(const QuaternionF& q) const {
        return QuaternionF(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    Vec3F rotate(const Vec3F& v) const {
      Vec3F qvec(x, y, z);
      Vec3F t = qvec.cross(v) * 2.0f;
      return v + t * w + qvec.cross(t);
    }

    QuaternionF normalized() const {
        float norm = std::sqrt(w * w + x * x + y * y + z * z);
        return norm > 0.0f ? QuaternionF(w / norm, x / norm, y / norm, z / norm) : QuaternionF();
    }

    // spherical linear interpolation, both input quaternions must be normalized
    QuaternionF slerp(const QuaternionF& other, float t) const {
      float dot = w * other.w + x * other.x + y * other.y + z * other.z;
      QuaternionF q2 = other;

      if (dot < 0.0f) {
          dot = -dot;
          q2 = QuaternionF(-q2.w, -q2.x, -q2.y, -q2.z);
      }

      const float DOT_THRESHOLD = 0.9995f;
      if (dot > DOT_THRESHOLD) {
          // LERP + normalize for nearly identical quaternions
          QuaternionF result(
              w + t * (q2.w - w),
              x + t * (q2.x - x),
              y + t * (q2.y - y),
              z + t * (q2.z - z)
          );
          return result.normalized();  // still necessary for lerp path
      }

      float theta_0 = std::acos(dot);
      float theta = theta_0 * t;
      float sin_theta = std::sin(theta);
      float sin_theta_0 = std::sin(theta_0);

      float s1 = std::cos(theta) - dot * sin_theta / sin_theta_0;
      float s2 = sin_theta / sin_theta_0;

      return QuaternionF(
          s1 * w + s2 * q2.w,
          s1 * x + s2 * q2.x,
          s1 * y + s2 * q2.y,
          s1 * z + s2 * q2.z
      );
    }

  public:
    float w, x, y, z;
};

//--- Pose6DF ---------------------------------------------------------------------------

class Pose6DF {
  public:
    Pose6DF() : translation(), rotation() {}
    Pose6DF(const Vec3F& t, const QuaternionF& r) : translation(t), rotation(r.normalized()) {}

    /// Transform a point from local to world space
    Vec3F transformPoint(const Vec3F& localPoint) const {
        return rotation.rotate(localPoint) + translation;
    }

    /// Combine with another pose (this * other)
    Pose6DF operator*(const Pose6DF& other) const {
        Vec3F newTranslation = transformPoint(other.translation);
        QuaternionF newRotation = (rotation * other.rotation).normalized();
        return Pose6DF(newTranslation, newRotation);
    }

    /// Invert this pose
    Pose6DF inverse() const {
        QuaternionF inv_rot = rotation.normalized_inverse();
        Vec3F inv_trans = inv_rot.rotate(translation * -1.0f);
        return Pose6DF(inv_trans, inv_rot);
    }

    // Linearly interpolate between two poses
    static Pose6DF lerp(const Pose6DF& a, const Pose6DF& b, float t) {
        // Linear interpolation for translation
        Vec3F translation = a.translation * (1.0f - t) + b.translation * t;

        // Spherical linear interpolation for rotation
        QuaternionF rotation = a.rotation.slerp(b.rotation, t).normalized();

        return Pose6DF(translation, rotation);
    }
    
  public:
    Vec3F translation;
    QuaternionF rotation;
};

//--- LinearAngular ---------------------------------------------------------------------

class LinearAngular {
  public:
    LinearAngular(float l = 1.0f, float a = 1.0f) : linear(l), angular(a) {};

    LinearAngular operator+(const LinearAngular& other) const {
      return { linear + other.linear, angular + other.angular };
    }
    LinearAngular operator-(const LinearAngular& other) const {
      return { linear - other.linear, angular - other.angular };
    }
    LinearAngular operator*(float scalar) const {
      return { linear * scalar, angular * scalar };
    }
    LinearAngular operator*(const LinearAngular& other) const {
      return { linear * other.linear, angular * other.angular };
    }

  public:
    float linear = 1.0f;     // mm/s
    float angular = 1.0f;    // rad/s
};


