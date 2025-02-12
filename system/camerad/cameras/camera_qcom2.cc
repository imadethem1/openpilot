#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/cameras/spectra.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "CL/cl_ext_qcom.h"

#include "media/cam_defs.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"

#include "common/clutil.h"
#include "common/params.h"
#include "common/swaglog.h"


ExitHandler do_exit;


// high level camera state
class CameraState : public SpectraCamera {
public:
  std::mutex exp_lock;

  int exposure_time = 5;
  bool dc_gain_enabled = false;
  int dc_gain_weight = 0;
  int gain_idx = 0;
  float analog_gain_frac = 0;

  float cur_ev[3] = {};
  float best_ev_score = 0;
  int new_exp_g = 0;
  int new_exp_t = 0;

  Rect ae_xywh = {};
  float measured_grey_fraction = 0;
  float target_grey_fraction = 0.3;

  float fl_pix = 0;

  CameraState(SpectraMaster *master, const CameraConfig &config) : SpectraCamera(master, config) {};
  void update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain);
  void set_camera_exposure(float grey_frac);

  void set_exposure_rect();
  void run();

  void init() {
    fl_pix = cc.focal_len / sensor->pixel_size_mm;
    set_exposure_rect();

    dc_gain_weight = sensor->dc_gain_min_weight;
    gain_idx = sensor->analog_gain_rec_idx;
    cur_ev[0] = cur_ev[1] = cur_ev[2] = (1 + dc_gain_weight * (sensor->dc_gain_factor-1) / sensor->dc_gain_max_weight) * sensor->sensor_analog_gains[gain_idx] * exposure_time;
  };

  // TODO: this should move to SpectraCamera
  void handle_camera_event(void *evdat);
};


void CameraState::handle_camera_event(void *evdat) {
  if (!enabled) return;
  struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)evdat;
  assert(event_data->session_hdl == session_handle);
  assert(event_data->u.frame_msg.link_hdl == link_handle);

  uint64_t timestamp = event_data->u.frame_msg.timestamp;
  uint64_t main_id = event_data->u.frame_msg.frame_id;
  uint64_t real_id = event_data->u.frame_msg.request_id;

  if (real_id != 0) { // next ready
    if (real_id == 1) {idx_offset = main_id;}
    int buf_idx = (real_id - 1) % FRAME_BUF_COUNT;

    // check for skipped frames
    if (main_id > frame_id_last + 1 && !skipped) {
      LOGE("camera %d realign", cc.camera_num);
      clear_req_queue();
      enqueue_req_multi(real_id + 1, FRAME_BUF_COUNT - 1, 0);
      skipped = true;
    } else if (main_id == frame_id_last + 1) {
      skipped = false;
    }

    // check for dropped requests
    if (real_id > request_id_last + 1) {
      LOGE("camera %d dropped requests %ld %ld", cc.camera_num, real_id, request_id_last);
      enqueue_req_multi(request_id_last + 1 + FRAME_BUF_COUNT, real_id - (request_id_last + 1), 0);
    }

    // metas
    frame_id_last = main_id;
    request_id_last = real_id;

    auto &meta_data = buf.camera_bufs_metadata[buf_idx];
    meta_data.frame_id = main_id - idx_offset;
    meta_data.request_id = real_id;
    meta_data.timestamp_sof = timestamp;
    exp_lock.lock();
    meta_data.gain = analog_gain_frac * (1 + dc_gain_weight * (sensor->dc_gain_factor-1) / sensor->dc_gain_max_weight);
    meta_data.high_conversion_gain = dc_gain_enabled;
    meta_data.integ_lines = exposure_time;
    meta_data.measured_grey_fraction = measured_grey_fraction;
    meta_data.target_grey_fraction = target_grey_fraction;
    exp_lock.unlock();

    // dispatch
    enqueue_req_multi(real_id + FRAME_BUF_COUNT, 1, 1);
  } else { // not ready
    if (main_id > frame_id_last + 10) {
      LOGE("camera %d reset after half second of no response", cc.camera_num);
      clear_req_queue();
      enqueue_req_multi(request_id_last + 1, FRAME_BUF_COUNT, 0);
      frame_id_last = main_id;
      skipped = true;
    }
  }
}

void CameraState::set_exposure_rect() {
  // set areas for each camera, shouldn't be changed
  std::vector<std::pair<Rect, float>> ae_targets = {
    // (Rect, F)
    std::make_pair((Rect){96, 250, 1734, 524}, 567.0),  // wide
    std::make_pair((Rect){96, 160, 1734, 986}, 2648.0), // road
    std::make_pair((Rect){96, 242, 1736, 906}, 567.0)   // driver
  };
  int h_ref = 1208;
  /*
    exposure target intrinics is
    [
      [F, 0, 0.5*ae_xywh[2]]
      [0, F, 0.5*H-ae_xywh[1]]
      [0, 0, 1]
    ]
  */
  auto ae_target = ae_targets[cc.camera_num];
  Rect xywh_ref = ae_target.first;
  float fl_ref = ae_target.second;

  ae_xywh = (Rect){
    std::max(0, buf.out_img_width / 2 - (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::max(0, buf.out_img_height / 2 - (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y))),
    std::min((int)(fl_pix / fl_ref * xywh_ref.w), buf.out_img_width / 2 + (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::min((int)(fl_pix / fl_ref * xywh_ref.h), buf.out_img_height / 2 + (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y)))
  };
}

void CameraState::update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain) {
  float score = sensor->getExposureScore(desired_ev, exp_t, exp_g_idx, exp_gain, gain_idx);
  if (score < best_ev_score) {
    new_exp_t = exp_t;
    new_exp_g = exp_g_idx;
    best_ev_score = score;
  }
}

void CameraState::set_camera_exposure(float grey_frac) {
  if (!enabled) return;
  const float dt = 0.05;

  const float ts_grey = 10.0;
  const float ts_ev = 0.05;

  const float k_grey = (dt / ts_grey) / (1.0 + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0 + dt / ts_ev);

  // It takes 3 frames for the commanded exposure settings to take effect. The first frame is already started by the time
  // we reach this function, the other 2 are due to the register buffering in the sensor.
  // Therefore we use the target EV from 3 frames ago, the grey fraction that was just measured was the result of that control action.
  // TODO: Lower latency to 2 frames, by using the histogram outputted by the sensor we can do AE before the debayering is complete

  const float cur_ev_ = cur_ev[buf.cur_frame_data.frame_id % 3];

  // Scale target grey between 0.1 and 0.4 depending on lighting conditions
  float new_target_grey = std::clamp(0.4 - 0.3 * log2(1.0 + sensor->target_grey_factor*cur_ev_) / log2(6000.0), 0.1, 0.4);
  float target_grey = (1.0 - k_grey) * target_grey_fraction + k_grey * new_target_grey;

  float desired_ev = std::clamp(cur_ev_ * target_grey / grey_frac, sensor->min_ev, sensor->max_ev);
  float k = (1.0 - k_ev) / 3.0;
  desired_ev = (k * cur_ev[0]) + (k * cur_ev[1]) + (k * cur_ev[2]) + (k_ev * desired_ev);

  best_ev_score = 1e6;
  new_exp_g = 0;
  new_exp_t = 0;

  // Hysteresis around high conversion gain
  // We usually want this on since it results in lower noise, but turn off in very bright day scenes
  bool enable_dc_gain = dc_gain_enabled;
  if (!enable_dc_gain && target_grey < sensor->dc_gain_on_grey) {
    enable_dc_gain = true;
    dc_gain_weight = sensor->dc_gain_min_weight;
  } else if (enable_dc_gain && target_grey > sensor->dc_gain_off_grey) {
    enable_dc_gain = false;
    dc_gain_weight = sensor->dc_gain_max_weight;
  }

  if (enable_dc_gain && dc_gain_weight < sensor->dc_gain_max_weight) {dc_gain_weight += 1;}
  if (!enable_dc_gain && dc_gain_weight > sensor->dc_gain_min_weight) {dc_gain_weight -= 1;}

  std::string gain_bytes, time_bytes;
  if (env_ctrl_exp_from_params) {
    static Params params;
    gain_bytes = params.get("CameraDebugExpGain");
    time_bytes = params.get("CameraDebugExpTime");
  }

  if (gain_bytes.size() > 0 && time_bytes.size() > 0) {
    // Override gain and exposure time
    gain_idx = std::stoi(gain_bytes);
    exposure_time = std::stoi(time_bytes);

    new_exp_g = gain_idx;
    new_exp_t = exposure_time;
    enable_dc_gain = false;
  } else {
    // Simple brute force optimizer to choose sensor parameters
    // to reach desired EV
    for (int g = std::max((int)sensor->analog_gain_min_idx, gain_idx - 1); g <= std::min((int)sensor->analog_gain_max_idx, gain_idx + 1); g++) {
      float gain = sensor->sensor_analog_gains[g] * (1 + dc_gain_weight * (sensor->dc_gain_factor-1) / sensor->dc_gain_max_weight);

      // Compute optimal time for given gain
      int t = std::clamp(int(std::round(desired_ev / gain)), sensor->exposure_time_min, sensor->exposure_time_max);

      // Only go below recommended gain when absolutely necessary to not overexpose
      if (g < sensor->analog_gain_rec_idx && t > 20 && g < gain_idx) {
        continue;
      }

      update_exposure_score(desired_ev, t, g, gain);
    }
  }

  exp_lock.lock();

  measured_grey_fraction = grey_frac;
  target_grey_fraction = target_grey;

  analog_gain_frac = sensor->sensor_analog_gains[new_exp_g];
  gain_idx = new_exp_g;
  exposure_time = new_exp_t;
  dc_gain_enabled = enable_dc_gain;

  float gain = analog_gain_frac * (1 + dc_gain_weight * (sensor->dc_gain_factor-1) / sensor->dc_gain_max_weight);
  cur_ev[buf.cur_frame_data.frame_id % 3] = exposure_time * gain;

  exp_lock.unlock();

  // Processing a frame takes right about 50ms, so we need to wait a few ms
  // so we don't send i2c commands around the frame start.
  int ms = (nanos_since_boot() - buf.cur_frame_data.timestamp_sof) / 1000000;
  if (ms < 60) {
    util::sleep_for(60 - ms);
  }
  // LOGE("ae - camera %d, cur_t %.5f, sof %.5f, dt %.5f", cc.camera_num, 1e-9 * nanos_since_boot(), 1e-9 * buf.cur_frame_data.timestamp_sof, 1e-9 * (nanos_since_boot() - buf.cur_frame_data.timestamp_sof));

  auto exp_reg_array = sensor->getExposureRegisters(exposure_time, new_exp_g, dc_gain_enabled);
  sensors_i2c(exp_reg_array.data(), exp_reg_array.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, sensor->data_word);
}

void CameraState::run() {
  util::set_thread_name(cc.publish_name);

  std::vector<const char*> pubs = {cc.publish_name};
  if (cc.stream_type == VISION_STREAM_ROAD) pubs.push_back("thumbnail");
  PubMaster pm(pubs);

  for (uint32_t cnt = 0; !do_exit; ++cnt) {
    // Acquire the buffer; continue if acquisition fails
    if (!buf.acquire()) continue;

    MessageBuilder msg;
    auto framed = (msg.initEvent().*cc.init_camera_state)();
    framed.setFrameId(buf.cur_frame_data.frame_id);
    framed.setRequestId(buf.cur_frame_data.request_id);
    framed.setTimestampEof(buf.cur_frame_data.timestamp_eof);
    framed.setTimestampSof(buf.cur_frame_data.timestamp_sof);
    framed.setIntegLines(buf.cur_frame_data.integ_lines);
    framed.setGain(buf.cur_frame_data.gain);
    framed.setHighConversionGain(buf.cur_frame_data.high_conversion_gain);
    framed.setMeasuredGreyFraction(buf.cur_frame_data.measured_grey_fraction);
    framed.setTargetGreyFraction(buf.cur_frame_data.target_grey_fraction);
    framed.setProcessingTime(buf.cur_frame_data.processing_time);

    const float ev = cur_ev[buf.cur_frame_data.frame_id % 3];
    const float perc = util::map_val(ev, sensor->min_ev, sensor->max_ev, 0.0f, 100.0f);
    framed.setExposureValPercent(perc);
    framed.setSensor(sensor->image_sensor);

    // Log raw frames for road camera
    if (env_log_raw_frames && cc.stream_type == VISION_STREAM_ROAD && cnt % 100 == 5) {  // no overlap with qlog decimation
      framed.setImage(get_raw_frame_image(&buf));
    }

    // Process camera registers and set camera exposure
    sensor->processRegisters((uint8_t *)buf.cur_camera_buf->addr, framed);
    set_camera_exposure(set_exposure_target(&buf, ae_xywh, 2, cc.stream_type != VISION_STREAM_DRIVER ? 2 : 4));

    // Send the message
    pm.send(cc.publish_name, msg);
    if (cc.stream_type == VISION_STREAM_ROAD && cnt % 100 == 3) {
      publish_thumbnail(&pm, &buf);  // this takes 10ms???
    }
  }
}

void camerad_thread() {
  /*
    TODO: future cleanups
    - centralize enabled handling
    - open/init/etc mess
    - no ISP stuff in this file
  */

  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  const cl_context_properties props[] = {CL_CONTEXT_PRIORITY_HINT_QCOM, CL_PRIORITY_HINT_HIGH_QCOM, 0};
  cl_context ctx = CL_CHECK_ERR(clCreateContext(props, 1, &device_id, NULL, NULL, &err));

  VisionIpcServer v("camerad", device_id, ctx);

  // *** initial ISP init ***
  SpectraMaster m;
  m.init();

  // *** per-cam init ***
  std::vector<CameraState*> cams = {
   new CameraState(&m, ROAD_CAMERA_CONFIG),
   new CameraState(&m, WIDE_ROAD_CAMERA_CONFIG),
   new CameraState(&m, DRIVER_CAMERA_CONFIG),
  };
  for (auto cam : cams) cam->camera_open();
  for (auto cam : cams) cam->camera_init(&v, device_id, ctx);
  for (auto cam : cams) cam->init();
  v.start_listener();

  LOG("-- Starting threads");
  std::vector<std::thread> threads;
  for (auto cam : cams) {
    if (cam->enabled) threads.emplace_back(&CameraState::run, cam);
  }

  // start devices
  LOG("-- Starting devices");
  for (auto cam : cams) cam->sensors_start();

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    struct pollfd fds[1] = {{0}};

    fds[0].fd = m.video0_fd;
    fds[0].events = POLLPRI;

    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    if (!fds[0].revents) continue;

    struct v4l2_event ev = {0};
    ret = HANDLE_EINTR(ioctl(fds[0].fd, VIDIOC_DQEVENT, &ev));
    if (ret == 0) {
      if (ev.type == V4L_EVENT_CAM_REQ_MGR_EVENT) {
        struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)ev.u.data;
        if (env_debug_frames) {
          printf("sess_hdl 0x%6X, link_hdl 0x%6X, frame_id %lu, req_id %lu, timestamp %.2f ms, sof_status %d\n", event_data->session_hdl, event_data->u.frame_msg.link_hdl,
                 event_data->u.frame_msg.frame_id, event_data->u.frame_msg.request_id, event_data->u.frame_msg.timestamp/1e6, event_data->u.frame_msg.sof_status);
          do_exit = do_exit || event_data->u.frame_msg.frame_id > (1*20);
        }

        for (auto cam : cams) {
          if (event_data->session_hdl == cam->session_handle) {
            cam->handle_camera_event(event_data);
            break;
          }
        }
      } else {
        LOGE("unhandled event %d\n", ev.type);
      }
    } else {
      LOGE("VIDIOC_DQEVENT failed, errno=%d", errno);
    }
  }

  LOG(" ************** STOPPING **************");
  for (auto &t : threads) t.join();
  for (auto cam : cams) delete cam;
}
