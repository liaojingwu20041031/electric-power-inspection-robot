#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dg_hvs_hbn_handle dg_hvs_hbn_handle_t;

enum {
  DG_HVS_HBN_PIXEL_FORMAT_UNKNOWN = 0,
  DG_HVS_HBN_PIXEL_FORMAT_NV12 = 8
};

typedef struct dg_hvs_hbn_frame {
  const uint8_t *evs_data;
  size_t evs_size;
  int evs_width;
  int evs_height;
  uint32_t evs_frame_id;
  uint64_t evs_timestamp_ns;
  uint64_t evs_sys_timestamp_ns;
  const uint8_t *aps_data;
  size_t aps_size;
  int aps_width;
  int aps_height;
  int aps_stride;
  int aps_format;
  int aps_plane_count;
  uint32_t aps_frame_id;
  uint64_t aps_timestamp_ns;
  uint64_t aps_sys_timestamp_ns;
} dg_hvs_hbn_frame_t;

int dg_hvs_hbn_open(
  dg_hvs_hbn_handle_t **out_handle,
  int config_index,
  int requested_evs_sensor_index,
  int requested_i2c_bus,
  int settle_value,
  int buffer_count);

int dg_hvs_hbn_get_evs_frame(
  dg_hvs_hbn_handle_t *handle,
  uint32_t timeout_ms,
  dg_hvs_hbn_frame_t *out_frame);

int dg_hvs_hbn_get_aps_frame(
  dg_hvs_hbn_handle_t *handle,
  uint32_t timeout_ms,
  dg_hvs_hbn_frame_t *out_frame);

void dg_hvs_hbn_close(dg_hvs_hbn_handle_t *handle);

#ifdef __cplusplus
}
#endif
