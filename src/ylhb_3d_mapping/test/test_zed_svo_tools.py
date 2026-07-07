import json
from pathlib import Path

from ylhb_3d_mapping import zed_svo_tools


def test_capture_records_svo_without_spatial_mapping(tmp_path):
    class Sl:
        class ERROR_CODE:
            SUCCESS = 'SUCCESS'

        class DEPTH_MODE:
            NONE = 'none'

        class RESOLUTION:
            HD720 = 'hd720'

        class SVO_COMPRESSION_MODE:
            H264 = 'h264'

        class InitParameters:
            pass

        class RecordingParameters:
            pass

        class Camera:
            def __init__(self):
                self.calls = []

            def open(self, params):
                self.calls.append(('open', params.depth_mode, params.camera_resolution, params.camera_fps))
                return 'SUCCESS'

            def enable_recording(self, params):
                self.calls.append(('enable_recording', Path(params.video_filename).name, params.compression_mode))
                return 'SUCCESS'

            def grab(self):
                self.calls.append('grab')
                return 'SUCCESS'

            def disable_recording(self):
                self.calls.append('disable_recording')

            def close(self):
                self.calls.append('close')

    metadata = zed_svo_tools.capture_svo(
        ['output_root:=' + str(tmp_path), 'duration_sec:=0.01'],
        sl=Sl,
        now=lambda: 10.0,
        monotonic=iter([0.0, 0.02]).__next__,
    )

    assert metadata['svo_file'].endswith('capture.svo2')
    assert metadata['parameters']['depth_mode'] == 'NONE'
    assert metadata['parameters']['camera_fps'] == 30
    assert metadata['state'] == 'succeeded'
    assert (Path(metadata['output_dir']) / 'metadata.json').exists()
    assert json.loads((Path(metadata['output_dir']) / 'status.json').read_text())['state'] == 'succeeded'


def test_reconstruct_uses_svo_offline_mode_and_profile(tmp_path):
    svo_file = tmp_path / 'capture.svo2'
    svo_file.write_text('svo', encoding='utf-8')

    class Sl:
        class ERROR_CODE:
            SUCCESS = 'SUCCESS'
            END_OF_SVOFILE_REACHED = 'END'

        class DEPTH_MODE:
            NEURAL = 'neural'
            NEURAL_PLUS = 'neural_plus'

        class RESOLUTION:
            HD720 = 'hd720'

        class UNIT:
            METER = 'meter'

        class COORDINATE_SYSTEM:
            RIGHT_HANDED_Z_UP = 'z_up'

        class RuntimeParameters:
            pass

        class InitParameters:
            def set_from_svo_file(self, path):
                self.svo_file = path

        class SpatialMappingParameters:
            pass

        class MESH_FILE_FORMAT:
            PLY = 'PLY'
            OBJ = 'OBJ'

        class SPATIAL_MAP_TYPE:
            FUSED_POINT_CLOUD = 'fused'
            MESH = 'mesh'

        class FusedPointCloud:
            vertices = [(1, 2, 3), (4, 5, 6)]

            def save(self, output_file, file_format):
                Path(output_file).write_text('ply', encoding='utf-8')
                return True

        class Camera:
            def __init__(self):
                self.grabs = 0

            def open(self, params):
                self.init_params = params
                return 'SUCCESS'

            def enable_positional_tracking(self, params):
                return 'SUCCESS'

            def enable_spatial_mapping(self, params):
                self.spatial_params = params
                return 'SUCCESS'

            def grab(self, params):
                self.grabs += 1
                return 'SUCCESS' if self.grabs == 1 else 'END'

            def extract_whole_spatial_map(self, spatial_map):
                self.spatial_map = spatial_map

            def get_svo_number_of_frames(self):
                return 2

            def disable_spatial_mapping(self):
                pass

            def disable_positional_tracking(self):
                pass

            def close(self):
                pass

        class PositionalTrackingParameters:
            pass

    metadata = zed_svo_tools.reconstruct_svo(
        ['input:=' + str(svo_file), 'output_root:=' + str(tmp_path), 'profile:=quality_plus'],
        sl=Sl,
        now=lambda: 20.0,
    )

    assert metadata['state'] == 'succeeded'
    assert metadata['svo_file'] == str(svo_file)
    assert metadata['svo_frame_count'] == 2
    assert metadata['reconstruct_profile'] == 'quality_plus'
    assert metadata['parameters']['depth_mode'] == 'NEURAL_PLUS'
    assert metadata['parameters']['spatial_mapping_max_memory_mb'] == 1024
    assert metadata['export_point_count'] == 2
    assert metadata['output_file'].endswith('pointcloud.ply')
