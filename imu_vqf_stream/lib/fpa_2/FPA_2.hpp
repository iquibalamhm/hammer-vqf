#pragma once
/**
 * FPA.hpp
 * Streaming Foot Progression Angle (FPA) estimator for foot‑mounted IMUs.
 *
 * This implements a practical, online variant of Laidig & Jocham et al.'s method:
 * - Detect foot‑flat (rest) vs motion using combined accel/gyro norms with hysteresis
 * - Define per‑stride rest instants t_rest at the midpoint of foot‑flat
 * - Between consecutive t_rest, strapdown integrate gravity‑compensated acceleration
 *   in an earth frame (E) using the current orientation quaternion (S->E)
 *   with linear drift correction so v(t_rest_next)=0
 * - The heading of the line of progression δ_progression is atan2(Δy, Δx) from the
 *   displacement between consecutive rest instants
 * - Sensor‑to‑foot alignment (F<-S) is estimated from average gravity during foot‑flat
 *   to derive foot yaw; FPA is foot yaw minus δ_progression (sign-inverted on right foot)
 *
 * NOTE: This version is fully self‑contained C++ and uses no dynamic allocations by default.
 *       It is designed to compile on Arduino/ESP32 (no STL required).
 */
#include <stdint.h>
#include <math.h>

#ifndef ENABLE_FPA_TRACE
#define ENABLE_FPA_TRACE 1
#endif

#ifndef FPA_TRACE_SAMPLE
#define FPA_TRACE_SAMPLE(stride_idx, sample_time_s, dt_s, v_raw_ptr, v_df_ptr, pos_ptr) \
  do { (void)(stride_idx); (void)(sample_time_s); (void)(dt_s); (void)(v_raw_ptr); (void)(v_df_ptr); (void)(pos_ptr); } while (0)
#endif

struct FPA_Result {
  bool valid;          // true when a new FPA was computed for the most recent stride
  float fpa_deg;       // foot progression angle in degrees (positive = out‑toeing)
  float stride_duration_s;
  float delta_xy_m[2]; // stride horizontal displacement (x,y) in meters in E frame
  uint32_t stride_index;
};

struct FPA_Debug {
  float time_s;
  float last_dt;
  int state;
  int state_raw;
  bool have_thr;
  bool have_tfc;
  bool have_trest;
  uint32_t stride_index;
  uint32_t grav_count;
  int sample_count;
  float acc_thr;
  float gyr_thr;
  bool last_result_valid;
  float last_result_fpa_deg;
  float last_result_stride_duration_s;
  float last_result_dx;
  float last_result_dy;
  uint32_t last_result_stride_index;
  int last_reject_reason; // 0=ok, 1=no samples buffered, 2=stride too short
  bool last_foot_valid;
  float last_foot_roll_deg;
  float last_foot_pitch_deg;
  float last_foot_yaw_deg;
};

class FPA {
public:
  // MAX_SAMPLES: capacity for buffered samples between consecutive rest instants.
  // For ~1 s stride at 200 Hz effective feeding -> 200; we pick 1024 to be safe.
  static const int MAX_SAMPLES = 512;

  // Construct with whether this is the LEFT foot (true) or RIGHT foot (false).
  // Sample period can be variable; pass dt per sample in feed().
  FPA(bool is_left_foot=true);

  // Configure thresholds (units: accel m/s^2, gyro rad/s); hysteresis fraction in [0,1).
  // Set to <=0 to keep defaults.
  void configure(float acc_diff_th, float gyr_norm_th, float hysteresis_frac,
                 float min_rest_s, float min_motion_s);

  // Feed one synchronized sample (acc, gyr, q_SE) at time step dt (seconds).
  // q_SE is quaternion (w,x,y,z) rotating a vector from Sensor S to Earth E: v_E = q * v_S * q^-1.
  // Units: acc (m/s^2), gyr (rad/s). dt must be > 0.
  void feed(const float acc[3], const float gyr[3], const float q_SE[4], float dt);

  // Returns true if a NEW FPA was computed since last call; fills out result.
  bool poll(FPA_Result* out);

  // Reset internal state (buffers, indices, thresholds keep current values)
  void reset();

  // Debug helper to inspect internal state (non-thread-safe)
  void getDebug(FPA_Debug* out) const;

private:
  enum RejectReason {
    REJECT_NONE = 0,
    REJECT_NO_SAMPLES = 1,
    REJECT_TOO_SHORT = 2
  };

  // --- internal math ----
  static inline float dot3(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  }
  static inline void cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
  }
  static inline float norm3(const float a[3]) {
    return sqrtf(dot3(a,a));
  }
  static inline void normalize3(float v[3]) {
    float n = norm3(v);
    if (n > 0) { v[0]/=n; v[1]/=n; v[2]/=n; }
  }
  static inline void quat_conj(const float q[4], float qc[4]) {
    qc[0] = q[0]; qc[1] = -q[1]; qc[2] = -q[2]; qc[3] = -q[3];
  }
  static inline void quat_mul(const float a[4], const float b[4], float out[4]) {
    // (w, x, y, z)
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
  }
  static inline void quat_rotate_vec(const float q[4], const float v[3], float out[3]) {
    // v_out = q * (0,v) * q^-1
    float vq[4] = {0.0f, v[0], v[1], v[2]};
    float qi[4]; quat_conj(q, qi);
    float tmp[4]; quat_mul(q, vq, tmp);
    float res[4]; quat_mul(tmp, qi, res);
    out[0] = res[1]; out[1] = res[2]; out[2] = res[3];
  }
  static inline float wrap_pi(float a) {
    while (a > 3.14159265358979323846f) a -= 6.2831853071795864769f;
    while (a < -3.14159265358979323846f) a += 6.2831853071795864769f;
    return a;
  }
  static inline float rad2deg(float r) { return r * 57.29577951308232f; }
  static inline void quat_to_euler_zyx(const float q[4], float* roll, float* pitch, float* yaw) {
    // q = (w, x, y, z)
    float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
    float roll_val = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
    float pitch_val;
    if (fabsf(sinp) >= 1.0f) {
      pitch_val = copysignf(1.57079632679f, sinp); // pi/2
    } else {
      pitch_val = asinf(sinp);
    }

    float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
    float yaw_val = atan2f(siny_cosp, cosy_cosp);

    if (roll) *roll = roll_val;
    if (pitch) *pitch = pitch_val;
    if (yaw) *yaw = yaw_val;
  }

  // Create foot-to-sensor quaternion q_FS from estimated axes (columns are foot axes in S)
  static void rotm_to_quat_FS(const float yF_S[3], const float zF_S[3], float q_FS[4]);

  // --- detection / buffering ----
  float t_;                   // accumulated time [s]
  float last_dt_;
  bool is_left_;

  // thresholds / hysteresis
  float acc_th_;   // threshold on | |a|-g |
  float gyr_th_;   // threshold on ||ω||
  float hys_;      // hysteresis fraction
  float min_rest_s_, min_motion_s_;

  // debounce
  int min_rest_count_, min_motion_count_;
  int rest_cnt_, motion_cnt_;

  // combined rest/motion state (0=rest/foot-flat, 1=motion)
  int state_;      // debounced state
  int state_raw_;  // instant state before debounce

  // latest transition times
  float last_thr_time_;   // heel rise (rest->motion)
  float last_tfc_time_;   // full contact (motion->rest)
  float last_trest_time_; // midpoint of foot-flat (defined once both sides known)
  bool  have_tfc_, have_thr_, have_trest_;

  // buffers between consecutive trest
  int   n_;                      // number of buffered samples
  float buf_t_[MAX_SAMPLES];     // absolute time of sample
  float buf_dt_[MAX_SAMPLES];
  float buf_acc_[MAX_SAMPLES][3];
  float buf_gyr_[MAX_SAMPLES][3];
  float buf_qSE_[MAX_SAMPLES][4];

  // index of the first sample at/after current stride start (trest_k)
  int   idx_stride_begin_;

  // foot alignment running estimate (average gravity during foot-flat)
  float grav_accum_[3];
  uint32_t grav_count_;

  // last result
  FPA_Result last_{false, NAN, 0.0f, {0.0f,0.0f}, 0};
  uint32_t stride_idx_;

  FPA_Result last_debug_result_;
  bool last_debug_valid_;
  int last_reject_reason_;
  bool last_foot_valid_;
  float last_foot_roll_deg_;
  float last_foot_pitch_deg_;
  float last_foot_yaw_deg_;
  
  // helpers
  void update_state_(float accDiff, float gyrNorm);
  void maybe_finalize_trest_();
  void integrate_and_compute_();
};

#ifndef FPA_NO_IMPLEMENTATION
// ------------------- Implementation -------------------
#include <math.h>
#include <string.h>

FPA::FPA(bool is_left_foot)
: t_(0.0f), last_dt_(0.0f), is_left_(is_left_foot),
  acc_th_(0.6f), gyr_th_(1.2f), hys_(0.2f),
  min_rest_s_(0.08f), min_motion_s_(0.08f),
  min_rest_count_(8), min_motion_count_(8),
  rest_cnt_(0), motion_cnt_(0),
  state_(0), state_raw_(0),
  last_thr_time_(NAN), last_tfc_time_(0.0f), last_trest_time_(NAN),
  have_tfc_(false), have_thr_(false), have_trest_(false),
  n_(0), idx_stride_begin_(0), grav_accum_{0,0,0}, grav_count_(0),
  stride_idx_(0),
  last_debug_result_{false, NAN, 0.0f, {0.0f,0.0f}, 0},
  last_debug_valid_(false),
  last_reject_reason_(REJECT_NONE),
  last_foot_valid_(false),
  last_foot_roll_deg_(NAN),
  last_foot_pitch_deg_(NAN),
  last_foot_yaw_deg_(NAN)
{
  memset(buf_t_, 0, sizeof(buf_t_));
  memset(buf_dt_, 0, sizeof(buf_dt_));
  memset(buf_acc_, 0, sizeof(buf_acc_));
  memset(buf_gyr_, 0, sizeof(buf_gyr_));
  memset(buf_qSE_, 0, sizeof(buf_qSE_));
}

void FPA::configure(float acc_diff_th, float gyr_norm_th, float hysteresis_frac,
                    float min_rest_s, float min_motion_s) {
  if (acc_diff_th > 0) acc_th_ = acc_diff_th;
  if (gyr_norm_th > 0) gyr_th_ = gyr_norm_th;
  if (hysteresis_frac >= 0 && hysteresis_frac < 1.0f) hys_ = hysteresis_frac;
  if (min_rest_s > 0) min_rest_s_ = min_rest_s;
  if (min_motion_s > 0) min_motion_s_ = min_motion_s;
  // translate to counts using the last seen dt or assume 200 Hz
  float fs = (last_dt_ > 0) ? (1.0f/last_dt_) : 200.0f;
  min_rest_count_   = (int)fmaxf(1.0f, roundf(min_rest_s_ * fs));
  min_motion_count_ = (int)fmaxf(1.0f, roundf(min_motion_s_ * fs));
}

void FPA::reset() {
  t_ = 0.0f; last_dt_ = 0.0f;
  rest_cnt_=motion_cnt_=0; state_=state_raw_=0;
  last_thr_time_=NAN; last_tfc_time_=0.0f; last_trest_time_=NAN;
  have_tfc_=have_thr_=have_trest_=false;
  n_=0; idx_stride_begin_=0; grav_accum_[0]=grav_accum_[1]=grav_accum_[2]=0; grav_count_=0;
  last_ = {false, NAN, 0.0f, {0.0f,0.0f}, 0};
  last_debug_result_ = last_;
  last_debug_valid_ = false;
  last_reject_reason_ = REJECT_NONE;
  last_foot_valid_ = false;
  last_foot_roll_deg_ = NAN;
  last_foot_pitch_deg_ = NAN;
  last_foot_yaw_deg_ = NAN;
  stride_idx_=0;
}

void FPA::rotm_to_quat_FS(const float yF_S[3], const float zF_S[3], float q_FS[4]) {
  // Columns of R_FS: xF_S = yF_S x zF_S, yF_S, zF_S
  float xF_S[3]; cross3(yF_S, zF_S, xF_S);
  normalize3(xF_S);
  float yN[3] = { yF_S[0], yF_S[1], yF_S[2] };
  float zN[3] = { zF_S[0], zF_S[1], zF_S[2] };
  normalize3(yN); normalize3(zN);

  float R[9] = {
    xF_S[0], yN[0], zN[0],
    xF_S[1], yN[1], zN[1],
    xF_S[2], yN[2], zN[2]
  };
  // Convert rotation matrix (column-major) to quaternion (w,x,y,z)
  float trace = R[0] + R[4] + R[8];
  if (trace > 0.0f) {
    float s = sqrtf(trace + 1.0f) * 2.0f;
    q_FS[0] = 0.25f * s;
    q_FS[1] = (R[7] - R[5]) / s;
    q_FS[2] = (R[2] - R[6]) / s;
    q_FS[3] = (R[3] - R[1]) / s;
  } else if ((R[0] > R[4]) && (R[0] > R[8])) {
    float s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
    q_FS[0] = (R[7] - R[5]) / s;
    q_FS[1] = 0.25f * s;
    q_FS[2] = (R[1] + R[3]) / s;
    q_FS[3] = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    float s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
    q_FS[0] = (R[2] - R[6]) / s;
    q_FS[1] = (R[1] + R[3]) / s;
    q_FS[2] = 0.25f * s;
    q_FS[3] = (R[5] + R[7]) / s;
  } else {
    float s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
    q_FS[0] = (R[3] - R[1]) / s;
    q_FS[1] = (R[2] + R[6]) / s;
    q_FS[2] = (R[5] + R[7]) / s;
    q_FS[3] = 0.25f * s;
  }
  // normalize
  float n = sqrtf(q_FS[0]*q_FS[0] + q_FS[1]*q_FS[1] + q_FS[2]*q_FS[2] + q_FS[3]*q_FS[3]);
  if (n > 0) { q_FS[0]/=n; q_FS[1]/=n; q_FS[2]/=n; q_FS[3]/=n; }
}

void FPA::update_state_(float accDiff, float gyrNorm) {
  // raw: 1 if motion detected by either accel or gyro
  float ath_on  = acc_th_ * (1.0f + hys_);
  float ath_off = acc_th_ * (1.0f - hys_);
  float gth_on  = gyr_th_ * (1.0f + hys_);
  float gth_off = gyr_th_ * (1.0f - hys_);
  int ra = (accDiff > ath_on) ? 1 : ((accDiff < ath_off) ? 0 : state_raw_);
  int rg = (gyrNorm > gth_on) ? 1 : ((gyrNorm < gth_off) ? 0 : state_raw_);
  int r = (ra || rg) ? 1 : 0;  // OR-combine
  state_raw_ = r;

  // debounce to enforce minimum durations
  if (r == 0) { // rest
    rest_cnt_++; motion_cnt_=0;
    if (state_==1 && rest_cnt_ >= min_rest_count_) {
      state_ = 0;
      last_tfc_time_ = t_; have_tfc_ = true; // motion->rest
      maybe_finalize_trest_();  // t_rest = (t_fc + next t_hr)/2; here we only know t_fc
    }
  } else { // motion
    motion_cnt_++; rest_cnt_=0;
    if (state_==0 && motion_cnt_ >= min_motion_count_) {
      state_ = 1;
      last_thr_time_ = t_; have_thr_ = true; // rest->motion
      maybe_finalize_trest_(); // now we may have both ends of foot-flat to define t_rest
    }
  }
}

void FPA::maybe_finalize_trest_() {
  // We define t_rest = (t_fc + t_hr)/2 of the SAME foot-flat phase
  if (have_thr_ && have_tfc_) {
    float trest = 0.5f * (last_tfc_time_ + last_thr_time_);
    last_trest_time_ = trest;
    have_trest_ = true;

    // When a NEW t_rest appears, finalize the previous stride [t_rest_prev, t_rest_curr]
    // The first t_rest only sets the stride start index
    if (stride_idx_ == 0) {
      // First stride: drop any pre-run samples (often idle) to avoid index overflow
      n_ = 0;
      idx_stride_begin_ = 0;
      grav_accum_[0] = grav_accum_[1] = grav_accum_[2] = 0.0f;
      grav_count_ = 0;
    } else {
      // integrate previous stride
      integrate_and_compute_();
      // mark new stride begin index
      idx_stride_begin_ = n_;
    }
    stride_idx_++;
    // reset for next phase
    have_thr_=have_tfc_=false;
  }
}

void FPA::integrate_and_compute_() {
  if (!have_trest_ || stride_idx_ < 1) return;
  last_foot_valid_ = false;
  // Find sample index closest to previous and current t_rest
  float t0 = last_trest_time_; // this is the "current" t_rest (end); we need previous end
  // To get previous t_rest time, we must store it. We'll approximate using the earliest time in buffer.
  // We recorded idx_stride_begin_ at previous t_rest, so t_prev ~= buf_t_[idx_stride_begin_]
  if (idx_stride_begin_ >= n_) {
    last_reject_reason_ = REJECT_NO_SAMPLES;
    return; // nothing to integrate
  }
  float t_prev = buf_t_[idx_stride_begin_];
  float duration = t0 - t_prev;
  if (!(duration > 0.02f)) {
    last_reject_reason_ = REJECT_TOO_SHORT;
    return; // too short
  }

#if ENABLE_FPA_TRACE
  uint32_t stride_index_trace = (stride_idx_ > 0) ? (stride_idx_ - 1u) : 0u;
#endif

  // 1) integrate velocity from t_prev..t0 in E frame
  float v[3] = {0,0,0};
  // Accelerometer measures proper acceleration (including gravity reaction force)
  // When stationary in Earth frame, it reads [0, 0, +g] (upward normal force)
  // To get linear acceleration, subtract this: a_linear = a_measured - [0, 0, +g]
  float gE[3] = {0.0f, 0.0f, 9.81f};
  // We'll store raw v at each step to later correct drift linearly
  static float v_raw[ MAX_SAMPLES ][3];
  int i0 = idx_stride_begin_;
  int i1 = n_;
  if (i1 > MAX_SAMPLES) i1 = MAX_SAMPLES; // safety

  for (int i=i0; i<i1; ++i) {
    float aE[3];
    quat_rotate_vec(buf_qSE_[i], buf_acc_[i], aE);
    // Remove gravity: a_linear = a_measured - g_E
    aE[0] -= gE[0]; aE[1] -= gE[1]; aE[2] -= gE[2];
    float dt = buf_dt_[i];
    v[0] += aE[0]*dt; v[1] += aE[1]*dt; v[2] += aE[2]*dt;
    v_raw[i][0] = v[0]; v_raw[i][1] = v[1]; v_raw[i][2] = v[2];
  }
  float v_end[3] = { v[0], v[1], v[2] };

  // 2) linear drift correction: v_df(t) = v_raw(t) - ((t-t0_prev)/duration) * v_end
  float p[3] = {0,0,0}; // integrate corrected velocity to position
  for (int i=i0; i<i1; ++i) {
    float frac = (buf_t_[i] - t_prev) / duration;
    float vdf[3] = {
      v_raw[i][0] - frac * v_end[0],
      v_raw[i][1] - frac * v_end[1],
      v_raw[i][2] - frac * v_end[2]
    };
    float dt = buf_dt_[i];
    p[0] += vdf[0]*dt; p[1] += vdf[1]*dt; p[2] += vdf[2]*dt;
#if ENABLE_FPA_TRACE
  float pos_curr[3] = {p[0], p[1], p[2]};
  float t_rel = buf_t_[i] - t_prev;
  FPA_TRACE_SAMPLE(stride_index_trace, t_rel, dt, v_raw[i], vdf, pos_curr);
#endif
  }
  // stride horizontal displacement from t_prev..t0
  float dx = p[0], dy = p[1];
  float d_heading = atan2f(dy, dx); // δ_progression

  // 3) Estimate foot alignment from average gravity during foot-flat
  float zF_S[3] = {0,0,1};
  if (grav_count_ > 10) {
    zF_S[0] = grav_accum_[0] / (float)grav_count_;
    zF_S[1] = grav_accum_[1] / (float)grav_count_;
    zF_S[2] = grav_accum_[2] / (float)grav_count_;
    normalize3(zF_S);
  }
  // yF_S = zF_S x (0,-1,0)
  float yF_S[3]; float minusY[3] = {0.0f, -1.0f, 0.0f};
  cross3(zF_S, minusY, yF_S);
  normalize3(yF_S);
  // foot-to-sensor quaternion q_FS
  float q_FS[4]; rotm_to_quat_FS(yF_S, zF_S, q_FS);

  // 4) Foot yaw at previous t_rest (use sample closest to idx_stride_begin_)
  float q_SE0[4] = { buf_qSE_[i0][0], buf_qSE_[i0][1], buf_qSE_[i0][2], buf_qSE_[i0][3] };
  float q_FE0[4]; // foot in E
  quat_mul(q_SE0, q_FS, q_FE0);
  // forward axis of foot in E = q_FE0 * ex * q_FE0^-1
  float ex[3] = {1,0,0}, fwdE[3];
  quat_rotate_vec(q_FE0, ex, fwdE);
  float yaw_foot = atan2f(fwdE[1], fwdE[0]);
  float roll_foot = 0.0f, pitch_foot = 0.0f;
  quat_to_euler_zyx(q_FE0, &roll_foot, &pitch_foot, &yaw_foot);

  float fpa = wrap_pi(yaw_foot - d_heading);
  if (!is_left_) fpa = -fpa; // positive out-toeing for both feet

  last_.valid = true;
  last_.fpa_deg = rad2deg(fpa);
  last_.stride_duration_s = duration;
  last_.delta_xy_m[0] = dx; last_.delta_xy_m[1] = dy;
  last_.stride_index = stride_idx_ - 1; // the stride we just finished

  last_debug_result_ = last_;
  last_debug_valid_ = true;
  last_reject_reason_ = REJECT_NONE;
  last_foot_valid_ = true;
  last_foot_roll_deg_ = rad2deg(roll_foot);
  last_foot_pitch_deg_ = rad2deg(pitch_foot);
  last_foot_yaw_deg_ = rad2deg(yaw_foot);

  // Drop consumed samples to avoid buffer growth; keep buffer ready for next stride
  n_ = 0;
  idx_stride_begin_ = 0;
}

void FPA::feed(const float acc[3], const float gyr[3], const float q_SE[4], float dt) {
  if (dt <= 0) return;
  last_dt_ = dt;
  t_ += dt;

  // Save into buffers (ring-like but we simply clamp)
  int i = n_ < MAX_SAMPLES ? n_ : (MAX_SAMPLES - 1);
  buf_t_[i]   = t_;
  buf_dt_[i]  = dt;
  buf_acc_[i][0] = acc[0]; buf_acc_[i][1] = acc[1]; buf_acc_[i][2] = acc[2];
  buf_gyr_[i][0] = gyr[0]; buf_gyr_[i][1] = gyr[1]; buf_gyr_[i][2] = gyr[2];
  buf_qSE_[i][0] = q_SE[0]; buf_qSE_[i][1] = q_SE[1]; buf_qSE_[i][2] = q_SE[2]; buf_qSE_[i][3] = q_SE[3];
  if (n_ < MAX_SAMPLES) n_++;

  // Update foot-flat gravity average if currently in REST (state_==0)
  float acc_norm = sqrtf(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
  float acc_diff = fabsf(acc_norm - 9.81f);
  float gyr_norm = sqrtf(gyr[0]*gyr[0]+gyr[1]*gyr[1]+gyr[2]*gyr[2]);

  update_state_(acc_diff, gyr_norm);
  if (state_==0) { // rest
    grav_accum_[0] += acc[0];
    grav_accum_[1] += acc[1];
    grav_accum_[2] += acc[2];
    grav_count_++;
  }
}

bool FPA::poll(FPA_Result* out) {
  if (last_.valid) {
    if (out) *out = last_;
    last_.valid = false; // consume
    return true;
  }
  return false;
}

void FPA::getDebug(FPA_Debug* out) const {
  if (!out) return;
  out->time_s = t_;
  out->last_dt = last_dt_;
  out->state = state_;
  out->state_raw = state_raw_;
  out->have_thr = have_thr_;
  out->have_tfc = have_tfc_;
  out->have_trest = have_trest_;
  out->stride_index = stride_idx_;
  out->grav_count = grav_count_;
  out->sample_count = n_;
  out->acc_thr = acc_th_;
  out->gyr_thr = gyr_th_;
  out->last_result_valid = last_debug_valid_;
  out->last_result_fpa_deg = last_debug_valid_ ? last_debug_result_.fpa_deg : NAN;
  out->last_result_stride_duration_s = last_debug_valid_ ? last_debug_result_.stride_duration_s : 0.0f;
  out->last_result_dx = last_debug_valid_ ? last_debug_result_.delta_xy_m[0] : 0.0f;
  out->last_result_dy = last_debug_valid_ ? last_debug_result_.delta_xy_m[1] : 0.0f;
  out->last_result_stride_index = last_debug_valid_ ? last_debug_result_.stride_index : 0;
  out->last_reject_reason = last_reject_reason_;
  out->last_foot_valid = last_foot_valid_;
  out->last_foot_roll_deg = last_foot_valid_ ? last_foot_roll_deg_ : NAN;
  out->last_foot_pitch_deg = last_foot_valid_ ? last_foot_pitch_deg_ : NAN;
  out->last_foot_yaw_deg = last_foot_valid_ ? last_foot_yaw_deg_ : NAN;
}

#endif // FPA_NO_IMPLEMENTATION
