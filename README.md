# tec-TICTOCK
TEC-1's mechanical watch tick speed synchronizer



Here’s an expanded version of your answer, focusing on **how the software works** when measuring watch accuracy:

---

### 1. Manual Time Comparison (Basic Method)

When you synchronize a mechanical watch with a reference clock (like an atomic time app or an online NTP clock), you’re relying on **human observation**:

* You note the **start reference time** (say 12:00:00).
* After a set period (24 hours or longer), you check both the watch and the reference again.
* The difference is calculated in seconds, then divided by the number of days to give an average daily gain or loss.

This method is simple but limited by human precision—our eyes and reaction times introduce small errors.

---

### 2. Smartphone Apps (Microphone + DSP)

Modern watch accuracy apps work by turning your smartphone into a **timegrapher-lite**.
Here’s how they function internally:

1. **Microphone Input** – The app uses your phone’s microphone to record the “tick-tock” sound of the escapement.
2. **Signal Processing** – It applies **digital signal processing (DSP)** to filter background noise and isolate the sharp impulses from the balance wheel.
3. **Beat Detection** – Each tick is timestamped with millisecond precision. By measuring the interval between ticks, the software can:

   * Calculate **beat rate** (e.g., 21,600 or 28,800 beats per hour).
   * Check for **consistency**—whether the ticks are evenly spaced.
4. **Amplitude Estimation** – Some apps attempt to estimate the amplitude (the swing angle of the balance wheel) by analyzing the shape of the acoustic waveform.
5. **Deviation Calculation** – Over several seconds or minutes, the app extrapolates how many seconds per day the watch gains or loses.

This is much faster and more precise than human observation, since it can detect errors down to fractions of a second per day.

---

### 3. Professional Timegraphers (Specialized Hardware + Software)

A dedicated **timegrapher machine** is what professional watchmakers use. It works similarly to the apps but with more accuracy:

1. **High-Sensitivity Microphone** – A piezoelectric sensor or clamp picks up the ticking directly from the watch case.
2. **Real-Time DSP** – The machine runs dedicated software that analyzes ticks in real time, free of mobile hardware/software limitations.
3. **Parameters Measured:**

   * **Rate** – seconds gained/lost per day.
   * **Beat Error** – the difference in time between the tick and the tock (ideally 0 ms).
   * **Amplitude** – how far the balance wheel swings (typically 270°–310° for a healthy movement).
   * **Stability/Linearity** – whether the rate drifts during the test.
4. **Graphical Display** – The software plots tick data as sloping lines or dots on a screen. A straight, consistent line means a healthy watch; curved or scattered plots indicate issues.

The software inside a timegrapher not only measures intervals but also **classifies mechanical health**, giving insights into lubrication, wear, or misalignment.

---

### 4. Comparison of Methods

* **Manual method** – relies on human timing, accurate to maybe ±1s/day.
* **Smartphone app** – uses audio DSP, usually accurate to within ±0.5s/day if environment is quiet.
* **Timegrapher** – professional-grade DSP with specialized sensors, accurate to ±0.1s/day and provides full diagnostics (rate, beat error, amplitude).

---

👉 In short: the “software” in apps or timegraphers works by **capturing the sound of the escapement, applying digital signal processing, timestamping beats, and analyzing statistical patterns** to determine accuracy and health.

Would you like me to also draw an **ASCII-style flow diagram** showing how the signal goes from *watch tick → microphone → DSP → accuracy report*?


Here’s concise, implementation-ready **pseudocode** for a professional timegrapher’s software pipeline (item 3). It covers acquisition, DSP, beat detection, rate/beat-error/amplitude calculation, stability checks, and plotting.

```pseudocode
// ======== CONFIG & CONSTANTS ========
struct Settings {
  sample_rate_hz          = 44100            // or device-specific (e.g., 96 kHz)
  highpass_cut_hz         = 200              // remove low rumble
  lowpass_cut_hz          = 5000             // remove hiss
  agc_target_rms          = -20 dBFS         // automatic gain control target
  min_tick_separation_ms  = 10               // reject double-triggers
  window_ms               = 120              // rolling analysis window
  lift_angle_deg          = 52               // movement-dependent; user-adjustable
  expected_bph            = 28800            // user-set (e.g., 18000/21600/25200/28800 etc.)
  posture_labels          = ["DU","DD","CR","CL","CU","CD"] // dial up/down, crown positions
}

// ======== RUNTIME STATE ========
state {
  ring_buffer audio_rb                     // raw audio
  ring_buffer env_rb                       // envelope/energy
  list<float> tick_times_sec               // timestamps of detected ticks
  list<float> tick_polarity                // +1 tick, -1 tock (if classified)
  list<float> interbeat_intervals_sec      // IBIs (tick->tock or tick->next tick)
  list<float> rate_spd_history             // seconds/day over time
  float agc_gain                           // current gain for AGC
  float posture_rate[6], posture_amp[6], posture_beat_err[6]
  int current_posture_index
  bool running
}

// ======== INITIALIZATION ========
function init(settings):
  configure_audio_input(settings.sample_rate_hz)
  audio_rb.init(capacity = seconds_to_samples(10))
  env_rb.init(capacity   = seconds_to_samples(10))
  agc_gain = 1.0
  state.running = true

// ======== AUDIO ACQUISITION LOOP ========
thread audio_capture_thread():
  while state.running:
    frames = read_audio_device()
    audio_rb.push(frames)

// ======== SIGNAL PROCESSING ========
function dsp_process_block():
  // 1) Pull a block
  x = audio_rb.pop_block(N = seconds_to_samples(settings.window_ms/1000))
  if x.empty(): return

  // 2) Pre-filtering: HPF -> LPF (IIR or FIR)
  x = highpass_filter(x, cutoff = settings.highpass_cut_hz)
  x = lowpass_filter(x, cutoff = settings.lowpass_cut_hz)

  // 3) AGC: normalize to target RMS
  rms = compute_rms(x)
  agc_gain = smooth(agc_gain, target = db_to_linear(settings.agc_target_rms)/max(rms, eps), alpha=0.02)
  x = x * agc_gain

  // 4) Transient enhancement: differentiate + rectify + smooth (envelope)
  dx = diff_filter(x)                       // emphasize sharp edges of tick
  env = abs(dx)
  env = moving_average(env, win = ms_to_samples(2))  // quick smooth
  env_rb.push(env)

  // 5) Adaptive threshold on envelope
  thr = adaptive_threshold(env_rb.tail(seconds_to_samples(2))) // e.g., median + k*mad

  // 6) Peak picking with refractory period
  peaks = []
  last_peak_time = tick_times_sec.back_or(-inf)

  for i in indices(env):
    if env[i] > thr and is_local_max(env, i):
      t_sec = samples_to_sec(global_sample_index(i))
      if (t_sec - last_peak_time) * 1000 >= settings.min_tick_separation_ms:
        peaks.append(i)
        last_peak_time = t_sec

  // 7) Classify peaks as Tick/Tock (optional polarity via waveform sign around peak)
  for p in peaks:
    t_sec = samples_to_sec(global_sample_index(p))
    pol = classify_polarity(x, p)           // +1 or -1 using local template/sign
    state.tick_times_sec.append(t_sec)
    state.tick_polarity.append(pol)

// ======== METRICS: RATE, BEAT ERROR, AMPLITUDE ========
function update_metrics():
  // Build interbeat intervals from recent tick list
  times = state.tick_times_sec.tail(limit = 600) // ~ few minutes
  pols  = state.tick_polarity.tail(limit = 600)
  if length(times) < 4: return

  // Separate tick and tock streams if polarity available
  tick_list   = []
  tock_list   = []
  for k in 0..length(times)-1:
    if pols[k] > 0: tick_list.append(times[k]) else tock_list.append(times[k])

  // --- Beat Error (ms): difference between half-period and actual tick->tock spacing
  // For each full cycle: tick->tock and tock->next tick around half the period
  beat_errors_ms = []
  cycle_periods  = []
  for i in 0..min(length(tick_list), length(tock_list))-2:
    // Find nearest tock after tick[i], and next tick after that tock
    t_tick   = tick_list[i]
    t_tock   = first(t in tock_list where t > t_tick)
    t_tick2  = first(t in tick_list where t > t_tock)
    if not exists(t_tock) or not exists(t_tick2): continue

    period   = t_tick2 - t_tick             // full oscillation period
    half_per = period / 2.0
    err      = (t_tock - t_tick) - half_per // positive if tock is late
    beat_errors_ms.append(err * 1000.0)
    cycle_periods.append(period)

  beat_error_ms = robust_mean(beat_errors_ms)   // e.g., median
  period_sec    = robust_mean(cycle_periods)
  observed_bph  = 3600.0 / period_sec           // beats per hour

  // --- Rate (s/day): deviation from expected beat rate
  // Convert observed beats to seconds/day drift:
  // seconds_per_day_expected = 86400
  // error_ratio = (observed_bph / settings.expected_bph) - 1
  // rate_spd = error_ratio * 86400
  rate_spd = ((observed_bph / settings.expected_bph) - 1.0) * 86400.0
  state.rate_spd_history.append(rate_spd)

  // --- Amplitude (deg) estimation (acoustic proxy):
  // Common timegrapher approach uses "lift angle" and measured "lock time".
  // Approximation:
  // amplitude_deg ≈ f(lift_angle_deg, tick shape). One practical method:
  // 1) Measure "lift time" (contact phase) around the acoustic transient width.
  // 2) Use: amplitude_rad ≈ (π * lift_time / period) / sin(lift_angle_rad/2)
  // NOTE: real devices use calibrated mapping from waveform features -> amplitude.
  lift_time_sec = estimate_lift_time_from_waveform(env_rb, around_peaks = last_n_peaks(50))
  lift_angle_rad = deg_to_rad(settings.lift_angle_deg)
  amplitude_rad = clamp( (PI * lift_time_sec / max(period_sec, eps)) / max(sin(lift_angle_rad/2), eps),
                         min=0, max= (350 deg to rad) )
  amplitude_deg = rad_to_deg(amplitude_rad)

  // Store per-posture if enabled
  idx = state.current_posture_index
  if idx in 0..5:
    state.posture_rate[idx]      = smooth(state.posture_rate[idx],      rate_spd,       alpha=0.2)
    state.posture_amp[idx]       = smooth(state.posture_amp[idx],       amplitude_deg,  alpha=0.2)
    state.posture_beat_err[idx]  = smooth(state.posture_beat_err[idx],  beat_error_ms,  alpha=0.2)

  // Quality checks
  stability_ok = check_stability(state.rate_spd_history.tail(30)) // e.g., stddev < threshold
  dropouts_ok  = (count_gaps(times) < threshold)

// ======== ADAPTIVE THRESHOLDING ========
function adaptive_threshold(env_segment):
  median_val = median(env_segment)
  mad_val    = median_absolute_deviation(env_segment)
  k = 4.0                               // sensitivity
  return median_val + k * mad_val

// ======== POLARITY CLASSIFICATION (optional) ========
function classify_polarity(x, peak_index):
  // Look at a short window around the peak; compute signed lobe energy
  L = ms_to_samples(2)
  left  = sum(x[peak_index-L : peak_index])
  right = sum(x[peak_index : peak_index+L])
  return sign(right - left)             // crude but effective with calibration

// ======== LIFT-TIME ESTIMATION (proxy for amplitude) ========
function estimate_lift_time_from_waveform(env_rb, around_peaks):
  // Around each peak, measure FWHM of the envelope burst
  widths = []
  for p in around_peaks:
    burst = env_window_centered_at(p, width = ms_to_samples(8))
    w = full_width_half_max(burst)
    widths.append(samples_to_sec(w))
  return robust_mean(widths)

// ======== STABILITY & QUALITY ========
function check_stability(last_rates):
  if length(last_rates) < 10: return false
  return stddev(last_rates) < 0.5  // e.g., <0.5 s/day over last few windows

function count_gaps(times):
  // count missing beats > 2x median period
  if length(times) < 3: return 1e9
  periods = diff(times)
  med = median(periods)
  return count(p in periods where p > 2*med)

// ======== UI / PLOTTING ========
function render_ui():
  // 1) Live "dot trace": plot residuals vs time
  // residual = (instant_period - period_sec) or tick timing error vs fitted line
  residuals = compute_residuals(state.tick_times_sec)
  plot_scatter(time_axis = now_window(), y = residuals)

  // 2) Numeric readouts
  draw_label("Rate",        format("%.1f s/day",  last(state.rate_spd_history)))
  draw_label("Beat Error",  format("%.1f ms",     beat_error_ms))
  draw_label("Amplitude",   format("%.0f °",      amplitude_deg))
  draw_label("BPH",         format("%.0f",        observed_bph))
  draw_label("Posture",     Settings.posture_labels[state.current_posture_index])

  // 3) Status bars
  draw_quality_bar(stability_ok, dropouts_ok, rms_level = current_rms_dbfs())

// ======== MAIN LOOP ========
function main():
  init(Settings)
  start_thread(audio_capture_thread)

  while state.running:
    dsp_process_block()
    update_metrics()
    render_ui()
    sleep_ms(50)

// ======== POSTURE HANDLING ========
function set_posture(index):
  // Called when operator rotates the watch clamp
  flush_recent_buffers()
  state.current_posture_index = index
  // allow a settling delay
  wait_seconds(10)

// ======== UTILITIES ========
function seconds_to_samples(sec):  return int(sec * Settings.sample_rate_hz)
function ms_to_samples(ms):        return seconds_to_samples(ms/1000)
function samples_to_sec(n):        return n / Settings.sample_rate_hz
function robust_mean(v):           return median(trim_outliers(v, pct=10))
function smooth(prev, target, alpha): return prev*(1-alpha) + target*alpha
function current_rms_dbfs():       return linear_to_db(compute_rms(env_rb.tail(2048)))
```

### Notes for implementers

* **Lift angle:** essential for amplitude; expose it in UI per movement spec (e.g., 50–54° common).
* **Expected BPH:** user-selectable; affects rate calculation.
* **Calibration:** real devices map acoustic features → amplitude using reference movements; include a calibration table or factory constant.
* **Noise handling:** add a noise gate and a “quietness” indicator; prompt the user if SNR is poor.
* **Outlier rejection:** use median/MAD and trim outliers to stabilize rate and beat-error.
* **Warm-up:** ignore the first ~10–20 seconds after posture changes.

If you want, I can adapt this to a specific language (C++/Qt, Python/PyAudio, or embedded C on a DSP) with concrete data structures and a minimal working example.



