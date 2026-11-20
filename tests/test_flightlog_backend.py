"""The flightlog backend gives the same DataFrames as the pymavlink one."""

from pathlib import Path

import pandas as pd
import pytest

from ardupilot_log_reader import Ardupilot, reader

pytest.importorskip("flightlog")
pytest.importorskip("pymavlink")

TEST_INPUTS = Path(__file__).parent / "test_inputs"
TYPES = ["POS", "ATT", "IMU", "XKF1", "XKF2", "XKQ1", "ERR", "GPS", "ORGN",
         "RCIN", "RCOU", "BARO", "MODE", "MAG", "MSG", "EV"]


def assert_same_frames(a: Ardupilot, b: Ardupilot):
    assert set(a.dfs) == set(b.dfs)
    for name, df in a.dfs.items():
        pd.testing.assert_frame_equal(
            df.reset_index(drop=True), b.dfs[name].reset_index(drop=True),
            check_exact=False, rtol=1e-12, atol=1e-9, obj=name,
        )


@pytest.mark.parametrize("log_name", ["raw_web_bin.BIN", "test_log_00000052.BIN", "00000129.BIN"])
@pytest.mark.parametrize("zero_time_base", [True, False])
def test_flightlog_frames_equal_pymavlink_frames(log_name, zero_time_base):
    path = TEST_INPUTS / log_name
    by_pymavlink = Ardupilot.parse(path, types=TYPES, zero_time_base=zero_time_base, backend="pymavlink")
    by_flightlog = Ardupilot.parse(path, types=TYPES, zero_time_base=zero_time_base, backend="flightlog")
    assert by_flightlog.source == "bin_file"
    assert_same_frames(by_pymavlink, by_flightlog)
    pd.testing.assert_frame_equal(by_pymavlink.parameters()["SERVO1_MIN"],
                                  by_flightlog.parameters()["SERVO1_MIN"])


def test_patterns_and_exclusions_select_the_same_types():
    path = TEST_INPUTS / "raw_web_bin.BIN"
    kwargs = {"types": ["XKF*", "AT*"], "nottypes": ["XKF3"], "zero_time_base": True}
    assert_same_frames(Ardupilot.parse(path, backend="pymavlink", **kwargs),
                       Ardupilot.parse(path, backend="flightlog", **kwargs))


def test_auto_uses_flightlog_unless_mavlink_filters_are_given(monkeypatch):
    calls = []
    real = Ardupilot._parse_flightlog

    def spy(*args):
        calls.append(args)
        return real(*args)

    monkeypatch.setattr(Ardupilot, "_parse_flightlog", staticmethod(spy))
    path = TEST_INPUTS / "raw_web_bin.BIN"
    Ardupilot.parse(path, types=["ATT"])
    assert len(calls) == 1
    with pytest.raises(ValueError, match="need backend='pymavlink'"):
        Ardupilot.parse(path, types=["ATT"], source_system=1, backend="flightlog")
    monkeypatch.setattr(reader, "HAS_FLIGHTLOG", False)
    Ardupilot.parse(path, types=["ATT"])
    assert len(calls) == 1


def test_backend_errors(monkeypatch):
    path = TEST_INPUTS / "raw_web_bin.BIN"
    with pytest.raises(ValueError, match="Unknown backend"):
        Ardupilot.parse(path, backend="other")
    monkeypatch.setattr(reader, "HAS_FLIGHTLOG", False)
    with pytest.raises(ImportError, match="flightlog is required"):
        Ardupilot.parse(path, backend="flightlog")


def test_damaged_format_record_still_reads_the_other_types(tmp_path):
    data = bytearray((TEST_INPUTS / "raw_web_bin.BIN").read_bytes())
    # FMT record: header A3 95 80, then Type, Length, Name[4], Format[16], Columns[64]
    pos = data.index(b"\xa3\x95\x80")
    while bytes(data[pos + 5:pos + 9]).rstrip(b"\0") != b"BARO":
        pos = data.index(b"\xa3\x95\x80", pos + 1)
    data[pos + 9] = ord("~")  # not a DataFlash format character
    path = tmp_path / "damaged.BIN"
    path.write_bytes(bytes(data))

    with pytest.raises(Exception, match="Unsupported format char"):
        Ardupilot.parse(path, types=["ATT", "BARO"], backend="pymavlink")
    log = Ardupilot.parse(path, types=["ATT", "BARO"], zero_time_base=True, backend="flightlog")
    assert "BARO" not in log.dfs
    expected = Ardupilot.parse(TEST_INPUTS / "raw_web_bin.BIN", types=["ATT"], zero_time_base=True,
                               backend="pymavlink")
    pd.testing.assert_frame_equal(log.ATT, expected.ATT)
