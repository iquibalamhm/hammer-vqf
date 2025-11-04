#pragma once
#include <cstdint>
#include <cmath>

namespace FPA {

// ---------- sizing (tunable via -D flags in platformio.ini) ----------
#ifndef FPA_SEGBUF_CAP
  #define FPA_SEGBUF_CAP 256
#endif
#ifndef FPA_TRAJ_CAP
  #define FPA_TRAJ_CAP 256
#endif
// --------------------------------------------------------------------

enum class EventType : uint8_t { FullContact, HeelRise, Rest, RestPredicted };
struct Event { EventType type; float t; };   // seconds

struct Params {
  // Hysteresis & debouncing
  float ha = 0.23f, hw = 0.23f;
  float ath_min = 1.8f, wth_min = 0.0f;
  float wa = 0.85f, ww = 0.80f;
  float alphaA_flat = 0.02f, alphaA_act = 0.02f;
  float alphaW_flat = 0.02f, alphaW_act = 0.02f;
  float T0_min = 0.120f, T1_min = 0.180f;

  // Optional REST prediction
  bool  predict_rest = true;
  float alphaHalfFF = 0.2f;

  // §3.9
  float g = 9.80665f;
  bool  quat_is_world_from_sensor = true; // VQF getQuat6D is world<-sensor
};

struct Vec3 { float x, y, z; };

struct TrajSample {
  float t;     // seconds
  Vec3  v_raw;   // uncorrected velocity, world [m/s]
  Vec3  v_df;    // drift-corrected velocity [m/s]
  Vec3  drift;   // linear drift subtracted [m/s]
  Vec3  p;       // position from t_rest (starts 0) [m]
};

class DetectorRT {
public:
  explicit DetectorRT(float samplePeriodSec, const Params& p = Params());
  ~DetectorRT(); // free buffers

  // Recommended: pass the quaternion you already have (q=[w,x,y,z]).
  void updateWithQuat(float t_sec,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      const float q[4], bool world_from_sensor = true);

  // Convenience: fetch quaternion from VQF (requires vqf.h in fpa.cpp).
  void update(float t_sec,
              float ax, float ay, float az,
              float gx, float gy, float gz);

  // Event FIFO
  bool available() const;
  bool pop(Event& e);

  // Trajectory FIFO (one row per sample when a stride closes)
  bool trajAvailable() const;
  bool popTraj(TrajSample& s);

  // Debug getters
  float lastAccelSig() const { return sA_; }
  float lastGyroSig()  const { return sW_; }
  float lastAth() const { return ath_; }
  float lastWth() const { return wth_; }
  uint8_t state() const { return r_; } // 0=flat, 1=active

  void reset();

private:
  static constexpr float g_ = 9.80665f;
  static inline float norm3(float x, float y, float z) {
    return std::sqrt(x*x + y*y + z*z);
  }

  // --------- tiny FIFOs ----------
  static constexpr int EV_CAP = 16;
  Event evq_[EV_CAP]; uint8_t ev_head_ = 0, ev_tail_ = 0;

  // Heap-allocated rings (PSRAM if available)
  struct SegBufSample { float t; Vec3 v_cont; };
  TrajSample*   traj_    = nullptr;
  SegBufSample* segbuf_  = nullptr;
  uint16_t      traj_cap_   = FPA_TRAJ_CAP;
  uint16_t      segbuf_cap_ = FPA_SEGBUF_CAP;
  uint16_t traj_head_ = 0, traj_tail_ = 0;
  uint16_t sb_head_   = 0, sb_tail_   = 0;

  // Params & timing
  float Ts_; Params P_;
  float t_now_ = 0.f;

  // Thresholding & state machine
  float sA_ = 0.f, sW_ = 0.f;
  float ath_ = 1.8f, wth_ = 0.f;
  float muA_flat_ = 0.3f, muA_act_ = 3.0f;
  float muW_flat_ = 0.2f, muW_act_ = 3.0f;
  uint8_t ra_ = 1, rw_ = 1, r_ = 1, cand_ = 1; int candCount_ = 0;
  int T0min_samp_ = 1, T1min_samp_ = 1;

  // Event times
  float t_last_fc_ = -1.f;

  // REST prediction
  bool  restPredArmed_ = false;
  float t_rest_pred_ = 0.f;
  float halfFF_ewma_ = 0.200f;

  // §3.9: strapdown & per-stride bookkeeping
  Vec3 v_cont_{0,0,0};
  Vec3 a_lin_prev_{0,0,0}; bool have_prev_a_ = false;

  bool   have_rest_prev_ = false;
  float  t_rest_prev_ = 0.f;

  // Helpers
  inline void updateAdaptiveThresholds();
  inline void updateChannel(uint8_t& rch, float s, float th, float h);
  inline void debounceAndEmit(uint8_t r_proposed);
  inline void checkPredictedRest();

  static inline Vec3 qRotate_world_from_sensor(const float q[4], const Vec3& v);
  static inline Vec3 qRotate_sensor_from_world(const float q[4], const Vec3& v);
  inline void strapdownStep_(const float q[4], bool world_from_sensor,
                             float ax, float ay, float az);

  void onRest_(float t_rest);
  void finishSegment_(float t0, float t1);
  bool  lookup_vcont_at(float t, Vec3& out) const;
  bool  for_each_sample_between(float t0, float t1,
                                void(*fn)(const SegBufSample&, void*),
                                void* user) const;

  void pushEvent(EventType type, float t);
  void pushTraj(const TrajSample& s);

  // allocation helpers (implemented in fpa.cpp)
  void allocateRings_();
  void freeRings_();
};

} // namespace FPA
