#include "dg_hvs_hbn_backend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef DG_HVS_DUAL_VC_VIN_SOURCE
#error "DG_HVS_DUAL_VC_VIN_SOURCE must point at the vendor dual_vc_vin.c"
#endif

#define main dg_hvs_original_main
#include DG_HVS_DUAL_VC_VIN_SOURCE
#undef main

struct dg_hvs_hbn_handle {
  dual_vc_pipe_t pipes[VC_PIPE_NUM];
  int created_count;
  int mem_open;
  int buffer_count;
  uint8_t *evs_copy;
  size_t evs_capacity;
  uint8_t *aps_copy;
  size_t aps_capacity;
  int aps_layout_reported;
};

static int ensure_capacity(uint8_t **buffer, size_t *capacity, size_t required)
{
  if (required <= *capacity) {
    return 0;
  }
  uint8_t *next = (uint8_t *)realloc(*buffer, required);
  if (next == NULL) {
    return -1;
  }
  *buffer = next;
  *capacity = required;
  return 0;
}

static void copy_rows(uint8_t *dst, const uint8_t *src, int rows, int row_bytes, int src_stride)
{
  for (int row = 0; row < rows; ++row) {
    memcpy(dst + (size_t)row * row_bytes, src + (size_t)row * src_stride, (size_t)row_bytes);
  }
}

int dg_hvs_hbn_open(dg_hvs_hbn_handle_t **out_handle, int config_index,
  int requested_evs_sensor_index, int requested_i2c_bus, int settle_value, int buffer_count)
{
  if (out_handle == NULL || buffer_count <= 0 || requested_i2c_bus < 0) {
    return -1;
  }
  *out_handle = NULL;
  dg_hvs_hbn_handle_t *handle = (dg_hvs_hbn_handle_t *)calloc(1, sizeof(*handle));
  if (handle == NULL) return -2;
  handle->buffer_count = buffer_count;

  const hvs_config_pair_t *config = hvs_config_get(config_index);
  if (config == NULL) { free(handle); return -3; }
  const char *config_files[VC_PIPE_NUM] = {config->vc0_cfg, config->vc1_cfg};
  int sensor_indexes[VC_PIPE_NUM] = {-1, -1};
  for (int i = 0; i < VC_PIPE_NUM; ++i) {
    sensor_indexes[i] = find_sensor_index_by_config_file(config_files[i]);
    if (sensor_indexes[i] < 0) { free(handle); return -4; }
  }
  if (requested_evs_sensor_index >= 0 && requested_evs_sensor_index != sensor_indexes[0]) {
    fprintf(stderr, "dg_hvs_hbn: requested sensor_index=%d resolves to %d for HVS config %d\n",
      requested_evs_sensor_index, sensor_indexes[0], config_index);
  }
  for (int i = 0; i < VC_PIPE_NUM; ++i) {
    if (configure_sensor(&handle->pipes[i], sensor_indexes[i]) != 0) { free(handle); return -5; }
  }
  (void)requested_i2c_bus;
  if (settle_value >= 0 && settle_value <= 127) settle = settle_value;
  if (bind_same_mipi_host(handle->pipes) != 0 || check_dual_vc_config(handle->pipes) != 0) {
    free(handle); return -6;
  }
  if (hb_mem_module_open() != 0) { free(handle); return -7; }
  handle->mem_open = 1;
  for (int i = 0; i < VC_PIPE_NUM; ++i) {
    if (create_and_run_vflow(&handle->pipes[i]) != 0) {
      for (int j = handle->created_count - 1; j >= 0; --j) stop_and_destroy_pipe(&handle->pipes[j]);
      hb_mem_module_close(); free(handle); return -8;
    }
    ++handle->created_count;
  }
  *out_handle = handle;
  return 0;
}

int dg_hvs_hbn_get_evs_frame(dg_hvs_hbn_handle_t *handle, uint32_t timeout_ms,
  dg_hvs_hbn_frame_t *out_frame)
{
  if (handle == NULL || out_frame == NULL || handle->created_count != VC_PIPE_NUM) return -1;
  memset(out_frame, 0, sizeof(*out_frame));
  hbn_vnode_image_t evs = {0};
  int evs_ret = hbn_vnode_getframe(handle->pipes[0].pipe_contex.vin_node_handle, 0, timeout_ms, &evs);
  if (evs_ret != 0) return evs_ret;

  const size_t evs_size = (size_t)evs.buffer.size[0];
  if (evs.buffer.virt_addr[0] == NULL || evs_size == 0) {
    hbn_vnode_releaseframe(handle->pipes[0].pipe_contex.vin_node_handle, 0, &evs);
    return -20;
  }
  if (ensure_capacity(&handle->evs_copy, &handle->evs_capacity, evs_size) != 0) {
    hbn_vnode_releaseframe(handle->pipes[0].pipe_contex.vin_node_handle, 0, &evs);
    return -21;
  }
  memcpy(handle->evs_copy, evs.buffer.virt_addr[0], evs_size);

  out_frame->evs_data = handle->evs_copy;
  out_frame->evs_size = evs_size;
  out_frame->evs_width = evs.buffer.width;
  out_frame->evs_height = evs.buffer.height;
  out_frame->evs_frame_id = evs.info.frame_id;
  out_frame->evs_timestamp_ns = evs.info.timestamps;
  out_frame->evs_sys_timestamp_ns = evs.info.sys_timestamps;

  hbn_vnode_releaseframe(handle->pipes[0].pipe_contex.vin_node_handle, 0, &evs);
  return 0;
}

int dg_hvs_hbn_get_aps_frame(dg_hvs_hbn_handle_t *handle, uint32_t timeout_ms,
  dg_hvs_hbn_frame_t *out_frame)
{
  if (handle == NULL || out_frame == NULL || handle->created_count != VC_PIPE_NUM) return -1;
  memset(out_frame, 0, sizeof(*out_frame));
  hbn_vnode_image_t aps = {0};
  const int aps_ret = hbn_vnode_getframe(
    handle->pipes[1].pipe_contex.isp_node_handle, 0, timeout_ms, &aps);
  if (aps_ret != 0) return aps_ret;

  const int aps_width = aps.buffer.width;
  const int aps_height = aps.buffer.height;
  const int aps_stride = aps.buffer.stride;
  const int aps_planes = aps.buffer.plane_cnt;
  if (!handle->aps_layout_reported) {
    fprintf(stderr,
      "dg_hvs_hbn: APS layout format=%d planes=%d width=%d height=%d stride=%d "
      "size0=%llu size1=%llu plane1=%s\n",
      aps.buffer.format, aps_planes, aps_width, aps_height, aps_stride,
      (unsigned long long)aps.buffer.size[0], (unsigned long long)aps.buffer.size[1],
      aps.buffer.virt_addr[1] != NULL ? "present" : "missing");
    handle->aps_layout_reported = 1;
  }
  const size_t aps_y_required =
    aps_stride > 0 && aps_height > 0 ? (size_t)aps_stride * aps_height : 0U;
  const size_t aps_uv_required =
    aps_stride > 0 && aps_height > 0 ? (size_t)aps_stride * (aps_height / 2) : 0U;
  if (aps.buffer.format != MEM_PIX_FMT_NV12 || aps.buffer.virt_addr[0] == NULL ||
      aps.buffer.virt_addr[1] == NULL || aps_width <= 0 || aps_height <= 0 ||
      aps_stride < aps_width || aps_planes != 2 || aps.buffer.size[0] < aps_y_required ||
      aps.buffer.size[1] < aps_uv_required) {
    hbn_vnode_releaseframe(handle->pipes[1].pipe_contex.isp_node_handle, 0, &aps);
    return -20;
  }
  const size_t normalized_aps_size =
    (size_t)aps_width * aps_height + (size_t)aps_width * (aps_height / 2);
  if (ensure_capacity(&handle->aps_copy, &handle->aps_capacity, normalized_aps_size) != 0) {
    hbn_vnode_releaseframe(handle->pipes[1].pipe_contex.isp_node_handle, 0, &aps);
    return -21;
  }
  for (int plane = 0; plane < aps_planes && plane < 2; ++plane) {
    if (aps.buffer.virt_addr[plane] != NULL && aps.buffer.size[plane] > 0) {
      hb_mem_invalidate_buf_with_vaddr(
        (uint64_t)aps.buffer.virt_addr[plane], aps.buffer.size[plane]);
    }
  }
  copy_rows(handle->aps_copy, aps.buffer.virt_addr[0], aps_height, aps_width, aps_stride);
  copy_rows(handle->aps_copy + (size_t)aps_width * aps_height, aps.buffer.virt_addr[1],
    aps_height / 2, aps_width, aps_stride);

  out_frame->aps_data = handle->aps_copy;
  out_frame->aps_size = normalized_aps_size;
  out_frame->aps_width = aps_width;
  out_frame->aps_height = aps_height;
  out_frame->aps_stride = aps_width;
  out_frame->aps_format = DG_HVS_HBN_PIXEL_FORMAT_NV12;
  out_frame->aps_plane_count = aps_planes;
  out_frame->aps_frame_id = aps.info.frame_id;
  out_frame->aps_timestamp_ns = aps.info.timestamps;
  out_frame->aps_sys_timestamp_ns = aps.info.sys_timestamps;

  hbn_vnode_releaseframe(handle->pipes[1].pipe_contex.isp_node_handle, 0, &aps);
  return 0;
}

void dg_hvs_hbn_close(dg_hvs_hbn_handle_t *handle)
{
  if (handle == NULL) return;
  for (int i = handle->created_count - 1; i >= 0; --i) stop_and_destroy_pipe(&handle->pipes[i]);
  if (handle->mem_open) hb_mem_module_close();
  free(handle->evs_copy);
  free(handle->aps_copy);
  free(handle);
}
