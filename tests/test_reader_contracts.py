"""Independent wire, pattern, core-order, parameter and file-lifetime contracts."""

import base64
import copy
from datetime import datetime, timedelta, timezone
import json
import struct
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest

from ardupilot_log_reader import Ardupilot
from ardupilot_log_reader import reader


@pytest.mark.parametrize("mode", ["base64", "list", "records"])
def test_supported_float_column_modes_have_independent_values_and_round_trip(mode):
    values = pd.Series([1.25, -2.5, 0.], name="Roll")
    encoded = Ardupilot.write_column(values, mode=mode)
    expected = (base64.b64encode(struct.pack("=ddd", 1.25, -2.5, 0.)).decode()
                if mode == "base64" else values.tolist() if mode == "list" else {0: 1.25, 1: -2.5, 2: 0.})
    assert encoded == expected
    pd.testing.assert_series_equal(Ardupilot.process_column("Roll", encoded), values)


@pytest.mark.parametrize("dtype", ["int8", "uint16", "float32", "object"])
def test_default_serialization_keeps_non_float64_columns_as_lists(dtype):
    values = pd.Series([1, 2, 3], dtype=dtype, name="value")
    assert Ardupilot.write_column(values) == [1, 2, 3]


@pytest.mark.parametrize("value", [None, 3, 1.5, (1, 2), b"bytes"])
def test_column_type_errors_name_the_target_column(value):
    with pytest.raises(TypeError, match="Unsupported data type for column Roll"):
        Ardupilot.process_column("Roll", value)


@pytest.mark.parametrize("patterns, excluded, expected", [
    (None, None, ["ATT", "XKF1", "XKF2", "IMU"]),
    (["XKF*", "ATT"], ["*2"], ["ATT", "XKF1"]),
    (["missing"], None, []), ([], None, []),
    (["*"], ["*"], []),
])
def test_message_patterns_keep_available_order_and_apply_exclusions(patterns, excluded, expected):
    available = ["ATT", "XKF1", "XKF2", "IMU"]
    assert Ardupilot.process_patterns(available, patterns, excluded) == expected
    assert available == ["ATT", "XKF1", "XKF2", "IMU"]


def test_grouped_legacy_core_messages_sort_by_microseconds_then_core_without_mutation():
    payload = {"filename": "synthetic", "source": "web_legacy", "data": {
        "XKF1[0]": {"time_boot_s": {"0": 2, "1": 1}, "C": {"0": 0, "1": 0}, "Roll": {"0": 20, "1": 10}},
        "XKF1[1]": {"time_boot_s": {"0": 1, "1": 2}, "C": {"0": 1, "1": 1}, "Roll": {"0": 11, "1": 21}},
        "EMPTY": {"TimeUS": [], "Roll": []},
    }}
    before = copy.deepcopy(payload)
    log = Ardupilot.from_dict(payload)
    assert list(log.dfs) == ["XKF1"]
    assert log.XKF1.TimeUS.tolist() == [1_000_000, 1_000_000, 2_000_000, 2_000_000]
    assert log.XKF1.C.tolist() == [0, 1, 0, 1]
    assert log.XKF1.Roll.tolist() == [10, 11, 20, 21]
    assert log.XKF1.timestamp.tolist() == [1., 1., 2., 2.]
    assert payload == before
    with pytest.raises(AttributeError, match="No such attribute: UNKNOWN"):
        _ = log.UNKNOWN


def test_parameter_history_keeps_only_first_value_and_actual_changes():
    frame = pd.DataFrame({"Name": ["A"] * 5 + ["B"] * 3,
                          "Value": [1, 1, 2, 2, 1, 5, 5, 6],
                          "timestamp": np.arange(8, dtype=float), "TimeUS": np.arange(8) * 1_000_000})
    before = frame.copy(deep=True)
    parameters = Ardupilot("synthetic", {"PARM": frame}, "bin_file").parameters()
    assert set(parameters) == {"A", "B"}
    assert parameters["A"].index.tolist() == [0., 2., 4.]
    assert parameters["A"].Value.tolist() == [1, 2, 1]
    assert parameters["B"].index.tolist() == [5., 7.]
    assert parameters["B"].TimeUS.tolist() == [5_000_000, 7_000_000]
    pd.testing.assert_frame_equal(frame, before)


def test_existing_cache_reopens_without_binary_parser_or_optional_dependency(tmp_path, monkeypatch):
    cache = tmp_path / "cached.json"
    cache.write_text(json.dumps({"filename": "cached.bin", "source": "web_array", "data": {
        "ATT": {"TimeUS": [2_000_000, 1_000_000], "Roll": [2, 1]},
    }}))
    monkeypatch.setattr(reader, "HAS_PYMAVLINK", False)
    parsed = Ardupilot.parse(tmp_path / "absent.bin", cache_file=cache)
    assert parsed.filename == "cached.bin" and parsed.source == "web_array"
    assert parsed.ATT.Roll.tolist() == [1, 2]
    with pytest.raises(ImportError, match="pymavlink is required"):
        Ardupilot.parse(tmp_path / "absent.bin")


def test_binary_parser_closes_its_handle_when_message_decoding_raises(tmp_path, monkeypatch):
    path = tmp_path / "invalid.bin"
    path.write_bytes(b"synthetic failure")
    handle = path.open("rb")

    def fail(**kwargs):
        raise ValueError("controlled malformed message")

    monkeypatch.setattr(reader, "HAS_PYMAVLINK", True)
    monkeypatch.setattr(reader, "DFReader_binary", lambda *args, **kwargs: SimpleNamespace(
        name_to_id={"ATT": 1}, filehandle=handle, recv_match=fail,
    ))
    monkeypatch.setenv("MAVLINK20", "1")
    try:
        with pytest.raises(ValueError, match="controlled malformed message"):
            Ardupilot.parse(path, types=["ATT"])
        assert handle.closed
    finally:
        handle.close()


@pytest.mark.parametrize("gps_step_ms", [1, 5, 20, 200, 1000])
def test_gps_milliseconds_have_one_conversion_independent_of_sample_rate(gps_step_ms):
    # The DataFlash GWk/GMS pair is week + milliseconds (also passed unchanged
    # by pymavlink DFReader_clock_usec.find_time_base to _gpsTimeToTime).
    epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    first_gps = (epoch + timedelta(weeks=2300, milliseconds=120_000, seconds=-18)).timestamp()
    first_boot = 2_000_000
    log = Ardupilot.from_dict({"data": {
        "GPS": {"GWk": [2300, 2300], "GMS": [120_000, 120_000 + gps_step_ms],
                "TimeUS": [first_boot, first_boot + gps_step_ms * 1000]},
        "ATT": {"TimeUS": [1_000_000, 2_000_000, 3_000_000], "Roll": [0, 1, 2]},
    }})
    assert log.ATT.timestamp.tolist() == [first_gps - 1, first_gps, first_gps + 1]
    assert log.GPS.timestamp.iloc[0] == first_gps
