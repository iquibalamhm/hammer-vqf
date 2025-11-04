#include "fpa.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>       // std::nothrow

// PSRAM allocator (only compiled on ESP32)
#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_heap_caps.h>
#endif

// Optional: get quaternion from VQF in update()
extern "C" {
  #include "vqf.h"   // getQuat6D(vqf_real_t out[4]);
}

namespace FPA {

// ---------------- allocation helpers ----------------
void DetectorRT::allocateRings_() {
  // Free first if already allocated (re-init path)
  freeRings_();

  // Allocate segbuf_
#if defined(ARDUINO_ARCH_ESP32)
  segbuf_ = static_cast<SegBufSample*>(
    heap_caps_malloc(segbuf_cap_ * sizeof(SegBufSample),
                     MALLOC_CAP_8BIT
#if defined(BOARD_HAS_PSRAM)
                     | MALLOC_CAP_SPIRAM
#endif
    ));
  if (!segbuf_) segbuf_ = new (std::nothrow) SegBufSample[segbuf_cap_];
#else
  segbuf_ = new (std::nothrow) SegBufSample[segbuf_cap_];
#endif

  // Allocate traj_
#if defined(ARDUINO_ARCH_ESP32)
  traj_ = static_cast<TrajSample*>(
    heap_caps_malloc(traj_cap_ * sizeof(TrajSample),
                     MALLOC_CAP_8BIT
#if defined(BOARD_HAS_PSRAM)
                     | MALLOC_CAP_SPIRAM
#endif
    ));
  if (!traj_) traj_ = new (std::nothrow) TrajSample[traj_cap_];
#else
  traj_ = new (std::nothrow) TrajSample[traj_cap_];
#endif

  // If either allocation failed, shrink caps to zero to avoid use.
  if (!segbuf_) segbuf_cap_ = 0;
  if (!traj_)   traj_cap_   = 0;
}

void DetectorRT::freeRings_() {
  if (segbuf_) {
#if defined(ARDUINO_ARCH_ESP32)
    // we don't know which allocator provided it; free is fine for both
    free(segbuf_);
#else
    delete[] segbuf_;
#endif
    segbuf_ = nullptr;
  }
  if (traj_) {
#if defined(ARDUINO_ARCH_ESP32)
    free(traj_);
#else
    delete[] traj_;
#endif
    traj_ = nullptr;
  }
}

// ---------------- ctor/dtor/reset ----------------
DetectorRT::DetectorRT(float samplePeriodSec, const Params& p)
: Ts_(samplePeriodSec), P_(p)
{
  T0min_samp_ = std::max(1, (int)std::lround(P_.T0_min / Ts_));
  T1min_samp_ = std::max(1, (int)std::lround(P_.T1_min / Ts_));
  allocateRings_();
  reset();
}

DetectorRT::~DetectorRT() { freeRings_(); }

void DetectorRT::reset() {
  // event FIFO
  ev_head_ = ev_tail_ = 0;

  // trajectory FIFO
  traj_head_ = traj_tail_ = 0;

  // continuous velocity ring buffer
  sb_head_ = sb_tail_ = 0;

  // activity stats / thresholds
  sA_ = sW_ = 0.f;
  muA_flat_ = 0.3f; muA_act_ = 3.0f;
  muW_flat_ = 0.2f; muW_act_ = 3.0f;
  ath_ = std::max(P_.ath_min, P_.wa*muA_flat_ + (1.f-P_.wa)*muA_act_);
  wth_ = std::max(P_.wth_min, P_.ww*muW_flat_ + (1.f-P_.ww)*muW_act_);
  ra_ = rw_ = r_ = cand_ = 1; candCount_ = 0;

  // timing
  t_now_ = 0.f; t_last_fc_ = -1.f;

  // REST prediction
  restPredArmed_ = false; t_rest_pred_ = 0.f; halfFF_ewma_ = 0.200f;

  // strapdown
  v_cont_ = {0,0,0};
  a_lin_prev_ = {0,0,0}; have_prev_a_ = false;

  // stride segmentation
  have_rest_prev_ = false;
  t_rest_prev_ = 0.f;
}

// ---------------- tiny FIFOs ----------------
void DetectorRT::pushEvent(EventType type, float t) {
  uint8_t next = (uint8_t)((ev_head_ + 1) % EV_CAP);
  if (next == ev_tail_) ev_tail_ = (uint8_t)((ev_tail_ + 1) % EV_CAP);
  evq_[ev_head_] = Event{type, t};
  ev_head_ = next;
}

bool DetectorRT::available() const { return ev_head_ != ev_tail_; }

bool DetectorRT::pop(Event& e) {
  if (!available()) return false;
  e = evq_[ev_tail_];
  ev_tail_ = (uint8_t)((ev_tail_ + 1) % EV_CAP);
  return true;
}

void DetectorRT::pushTraj(const TrajSample& s) {
  if (traj_cap_ == 0) return;
  uint16_t next = (uint16_t)((traj_head_ + 1) % traj_cap_);
  if (next == traj_tail_) traj_tail_ = (uint16_t)((traj_tail_ + 1) % traj_cap_);
  traj_[traj_head_] = s;
  traj_head_ = next;
}

bool DetectorRT::trajAvailable() const { return (traj_cap_ != 0) && (traj_head_ != traj_tail_); }

bool DetectorRT::popTraj(TrajSample& s) {
  if (!trajAvailable()) return false;
  s = traj_[traj_tail_];
  traj_tail_ = (uint16_t)((traj_tail_ + 1) % traj_cap_);
  return true;
}

// --------------- thresholds & state ----------
inline void DetectorRT::updateAdaptiveThresholds() {
  if (r_ == 0) { // flat
    muA_flat_ += P_.alphaA_flat * (sA_ - muA_flat_);
    muW_flat_ += P_.alphaW_flat * (sW_ - muW_flat_);
  } else {       // active
    muA_act_  += P_.alphaA_act  * (sA_ - muA_act_);
    muW_act_  += P_.alphaW_act  * (sW_ - muW_act_);
  }
  float ath_cand = P_.wa * muA_flat_ + (1.f - P_.wa) * muA_act_;
  float wth_cand = P_.ww * muW_flat_ + (1.f - P_.ww) * muW_act_;
  ath_ = std::max(P_.ath_min, ath_cand);
  wth_ = std::max(P_.wth_min,  wth_cand);
}

inline void DetectorRT::updateChannel(uint8_t& rch, float s, float th, float h) {
  float lo = (1.f - h) * th, hi = (1.f + h) * th;
  if      (s > hi) rch = 1u;
  else if (s < lo) rch = 0u;
}

inline void DetectorRT::debounceAndEmit(uint8_t r_prop) {
  if (r_prop == r_) { candCount_ = 0; return; }

  if (r_prop != cand_) { cand_ = r_prop; candCount_ = 1; }
  else                 { ++candCount_; }

  int need = (cand_ == 0u) ? T0min_samp_ : T1min_samp_;
  if (candCount_ >= need) {
    uint8_t prev = r_;
    r_ = cand_;
    candCount_ = 0;

    if (prev == 1u && r_ == 0u) {           // 1->0 Full Contact
      t_last_fc_ = t_now_;
      pushEvent(EventType::FullContact, t_last_fc_);

      // arm predicted rest midway through foot-flat
      restPredArmed_ = P_.predict_rest;
      t_rest_pred_   = t_last_fc_ + halfFF_ewma_;
    }
    else if (prev == 0u && r_ == 1u) {      // 0->1 Heel Rise
      float t_hr = t_now_;
      pushEvent(EventType::HeelRise, t_hr);

      if (t_last_fc_ > 0.f) {
        float t_rest = 0.5f * (t_last_fc_ + t_hr);
        pushEvent(EventType::Rest, t_rest);
        onRest_(t_rest); // §3.9 segment handling

        // update half FF duration EWMA
        float halfFF = 0.5f * (t_hr - t_last_fc_);
        halfFF_ewma_ += P_.alphaHalfFF * (halfFF - halfFF_ewma_);
      }
      restPredArmed_ = false;
    }
  }
}

inline void DetectorRT::checkPredictedRest() {
  if (!P_.predict_rest || !restPredArmed_) return;
  if (r_ == 0u && t_now_ >= t_rest_pred_) {
    pushEvent(EventType::RestPredicted, t_rest_pred_);
    restPredArmed_ = false;
  }
}

// --------------- quaternion rotations ----------
inline Vec3 DetectorRT::qRotate_world_from_sensor(const float q[4], const Vec3& v) {
  // v_world = q * (0,v) * q_conj ; q = [w,x,y,z]
  float w=q[0], x=q[1], y=q[2], z=q[3];
  // q*(0,v)
  float rw = -x*v.x - y*v.y - z*v.z;
  float rx =  w*v.x + y*v.z - z*v.y;
  float ry =  w*v.y + z*v.x - x*v.z;
  float rz =  w*v.z + x*v.y - y*v.x;
  // (..)*q_conj
  Vec3 out;
  out.x = -rw*x + rx*w - ry*z + rz*y;
  out.y = -rw*y + ry*w - rz*x + rx*z;
  out.z = -rw*z + rz*w - rx*y + ry*x;
  return out;
}

inline Vec3 DetectorRT::qRotate_sensor_from_world(const float q[4], const Vec3& v) {
  // If q is world->sensor, use q_conj to rotate sensor->world
  float qc[4] = { q[0], -q[1], -q[2], -q[3] };
  return qRotate_world_from_sensor(qc, v);
}

// --------------- strapdown per-sample ----------
inline void DetectorRT::strapdownStep_(const float q[4], bool world_from_sensor,
                                       float ax, float ay, float az)
{
  if (!segbuf_cap_) return;

  // Rotate sensor acceleration into world frame, subtract gravity, integrate.
  Vec3 a_s{ax, ay, az};
  Vec3 a_w = world_from_sensor
             ? qRotate_world_from_sensor(q, a_s)
             : qRotate_sensor_from_world(q, a_s);

  Vec3 a_lin{ a_w.x, a_w.y, a_w.z - P_.g };

  if (have_prev_a_) {
    // trapezoid integration for continuous raw velocity
    v_cont_.x += 0.5f * (a_lin_prev_.x + a_lin.x) * Ts_;
    v_cont_.y += 0.5f * (a_lin_prev_.y + a_lin.y) * Ts_;
    v_cont_.z += 0.5f * (a_lin_prev_.z + a_lin.z) * Ts_;
  }
  a_lin_prev_ = a_lin; have_prev_a_ = true;

  // store in seg buffer (time, v_cont)
  uint16_t next = (uint16_t)((sb_head_ + 1) % segbuf_cap_);
  if (next == sb_tail_) sb_tail_ = (uint16_t)((sb_tail_ + 1) % segbuf_cap_);
  segbuf_[sb_head_] = SegBufSample{t_now_, v_cont_};
  sb_head_ = next;
}

// Iterate segbuf between [t0, t1]; call fn(sample, user)
bool DetectorRT::for_each_sample_between(float t0, float t1,
                                         void(*fn)(const SegBufSample&, void*),
                                         void* user) const
{
  if (t1 < t0 || !segbuf_cap_) return false;
  uint16_t i = sb_tail_;
  while (i != sb_head_) {
    const auto& s = segbuf_[i];
    if (s.t >= t0 && s.t <= t1) fn(s, user);
    i = (uint16_t)((i + 1) % segbuf_cap_);
  }
  return true;
}

// Linear interpolation of v_cont at time t using neighbors in segbuf_
bool DetectorRT::lookup_vcont_at(float t, Vec3& out) const {
  if (!segbuf_cap_) return false;
  bool havePrev = false; SegBufSample prev{}, next{};
  uint16_t i = sb_tail_;
  while (i != sb_head_) {
    const auto& s = segbuf_[i];
    if (s.t <= t) { havePrev = true; prev = s; }
    if (s.t >  t) { next = s; break; }
    i = (uint16_t)((i + 1) % segbuf_cap_);
  }
  if (!havePrev) return false;
  if (next.t <= prev.t + 1e-6f) { out = prev.v_cont; return true; }
  float a = (t - prev.t) / (next.t - prev.t);
  out.x = prev.v_cont.x + a * (next.v_cont.x - prev.v_cont.x);
  out.y = prev.v_cont.y + a * (next.v_cont.y - prev.v_cont.y);
  out.z = prev.v_cont.z + a * (next.v_cont.z - prev.v_cont.z);
  return true;
}

// Called at every exact REST instant (trest = 0.5*(t_fc + t_hr))
void DetectorRT::onRest_(float t_rest) {
  if (!have_rest_prev_) {
    have_rest_prev_ = true;
    t_rest_prev_ = t_rest;
    return; // need the next trest to close a segment
  }

  // Close previous segment [t_rest_prev_, t_rest]
  finishSegment_(t_rest_prev_, t_rest);

  // Advance window and purge very old samples
  t_rest_prev_ = t_rest;
  while (sb_tail_ != sb_head_ && segbuf_[sb_tail_].t < t_rest_prev_ - 3.0f) {
    sb_tail_ = (uint16_t)((sb_tail_ + 1) % segbuf_cap_);
  }
}

// Build drift-corrected velocity & position for a finished segment,
// then enqueue one TrajSample per sample in that segment.
void DetectorRT::finishSegment_(float t0, float t1) {
  if (!traj_cap_ || !segbuf_cap_) return;

  Vec3 v0; if (!lookup_vcont_at(t0, v0)) return;
  Vec3 v1; if (!lookup_vcont_at(t1, v1)) return;

  const float dt = std::max(1e-6f, t1 - t0);
  Vec3 slope{ v1.x / dt, v1.y / dt, v1.z / dt }; // linear drift per second

  Vec3 p{0,0,0};         // position from t0
  Vec3 vdf_prev{0,0,0};
  float last_t = 0.f;     bool have_prev = false;

  // Iterate samples in [t0,t1] and emit rows
  uint16_t i = sb_tail_;
  while (i != sb_head_) {
    const auto& s = segbuf_[i];
    if (s.t >= t0 && s.t <= t1) {
      // Raw segment velocity (relative to v0)
      Vec3 vraw{ s.v_cont.x - v0.x, s.v_cont.y - v0.y, s.v_cont.z - v0.z };
      float tau = s.t - t0;
      Vec3 drift{ slope.x * tau, slope.y * tau, slope.z * tau };
      Vec3 vdf{ vraw.x - drift.x, vraw.y - drift.y, vraw.z - drift.z };

      if (have_prev) {
        float dts = s.t - last_t;
        if (dts < 0.f) dts = Ts_;
        p.x += 0.5f * (vdf_prev.x + vdf.x) * dts;
        p.y += 0.5f * (vdf_prev.y + vdf.y) * dts;
        p.z += 0.5f * (vdf_prev.z + vdf.z) * dts;
      }
      vdf_prev = vdf; have_prev = true; last_t = s.t;

      pushTraj(TrajSample{ s.t, vraw, vdf, drift, p });
    }
    i = (uint16_t)((i + 1) % segbuf_cap_);
  }
}

// --------------- public update ---------------
void DetectorRT::updateWithQuat(float t_sec,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                const float q[4],
                                bool world_from_sensor)
{
  t_now_ = t_sec;

  // §3.3–3.4 activity signals & causal thresholds
  sA_ = std::fabs(norm3(ax,ay,az) - g_);
  sW_ = norm3(gx,gy,gz);
  updateAdaptiveThresholds();
  updateChannel(ra_, sA_, ath_, P_.ha);
  updateChannel(rw_, sW_, wth_, P_.hw);
  uint8_t r_prop = (ra_ | rw_) ? 1u : 0u;
  debounceAndEmit(r_prop);
  checkPredictedRest();

  // §3.9 strapdown step (raw continuous velocity accumulation)
  strapdownStep_(q, world_from_sensor, ax, ay, az);
}

// Convenience overload: fetch quaternion from VQF 6D
void DetectorRT::update(float t_sec,
                        float ax, float ay, float az,
                        float gx, float gy, float gz)
{
  float q[4];
  vqf_real_t q6[4];
  getQuat6D(q6); // VQF returns [w,x,y,z]
  q[0] = (float)q6[0];
  q[1] = (float)q6[1];
  q[2] = (float)q6[2];
  q[3] = (float)q6[3];
  updateWithQuat(t_sec, ax, ay, az, gx, gy, gz, q, P_.quat_is_world_from_sensor);
}

} // namespace FPA
