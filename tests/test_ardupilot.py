"""
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

import base64
import struct
from json import load
from pathlib import Path

from pytest import fixture, mark, raises

from ardupilot_log_reader.reader import Ardupilot
import numpy as np
import pandas as pd

TEST_INPUTS = Path(__file__).parent / "test_inputs"


@fixture(scope="session")
def type_request():
    return [
        "XKF1",
        "XKQ1",
        "ARSP",
        "GPS",
        "RCIN",
        "RCOU",
        "IMU",
        "BARO",
        "MODE",
        "RPM",
        "MAG",
    ]


@fixture(scope="session")
def log(type_request):
    return Ardupilot.parse(
        Path(__file__).parent / "test_inputs/test_log_00000052.BIN",
        types=type_request,
        zero_time_base=True,
    )


@fixture(scope="session")
def log2(type_request):
    return Ardupilot.parse(
        Path(__file__).parent / "test_inputs/00000129.BIN",
        types=type_request,
        zero_time_base=True,
    )


def test_dfs(type_request, log):
    assert set(log.dfs.keys()) == set(type_request + ["PARM"])


@fixture(scope="session")
def old_web_bin():
    with (TEST_INPUTS / "old_web_bin.json").open("r") as f:
        return Ardupilot.from_dict(load(f))


@fixture(scope="session")
def new_web_bin():
    with (TEST_INPUTS / "new_web_bin.json").open("r") as f:
        return Ardupilot.from_dict(load(f))

@fixture(scope="session")
def raw_bin():
    return Ardupilot.parse(
        TEST_INPUTS / "raw_web_bin.BIN",
        types=["POS", "ATT", "XKF1", "XKF2", "IMU", "GPS", "ERR"],
        zero_time_base=True,
    )


def assert_dfs_match(bin1: Ardupilot, bin2: Ardupilot):
    matches = {}
    throw = False
    for k in bin1.dfs:
        try:
            pd.testing.assert_frame_equal(
                bin1.dfs[k].reset_index(drop=True),
                bin2.dfs[k].reset_index(drop=True),
                check_dtype=False,
                rtol=0.1,
            )
            matches[k] = True
        except AssertionError as e:
            matches[k] = f"DataFrames for {k} are not equal: {e}"
            throw = True
    if throw:
        match_summary = "\n".join(
            [f"\n{v}" for v in matches.values() if v is not True]
        )
        raise AssertionError(f"DataFrames {','.join(m for m in matches if matches[m] is not True)} do not match:\n{match_summary}")


def test_old_web_bin_matches_raw_bin(raw_bin: Ardupilot, old_web_bin: Ardupilot):
    assert_dfs_match(raw_bin, old_web_bin)

def test_new_web_bin_matches_raw_bin(
    raw_bin: Ardupilot, new_web_bin: Ardupilot
):
    assert_dfs_match(raw_bin, new_web_bin)


@mark.xfail(
    raises=TypeError,
    strict=True,
    reason="D04/coordinator review by 2026-11-26: CompressedBinColumn decoding pending",
)
def test_schema_compressed_web_bin_matches_raw_bin(raw_bin: Ardupilot):
    """Keep the identified compressed counterpart visible until decoding exists."""
    with (Path(__file__).parent / "test_inputs/new_web_bin_compressed.json").open() as f:
        compressed_web_bin = Ardupilot.from_dict(load(f))
    assert_dfs_match(raw_bin, compressed_web_bin)


def test_base64_web_bin_columns_match_checked_in_array_data():
    """The reader's base64 form is native float64 bytes, encoded as base64."""
    with (Path(__file__).parent / "test_inputs/new_web_bin.json").open() as f:
        array_data = load(f)

    columns = ("TimeUS", "Lat", "Lng", "Alt")
    expected = {
        name: array_data["POS"][name][:5]
        for name in columns
    }
    encoded = {
        name: base64.b64encode(
            struct.pack("=" + "d" * len(values), *values)
        ).decode("ascii")
        for name, values in expected.items()
    }

    parsed = Ardupilot.from_dict(
        {"filename": "small-web-bin", "source": "web_compressed", "data": {"POS": encoded}}
    )

    frame = parsed.dfs["POS"]
    assert list(parsed.dfs) == ["POS"]
    assert frame.shape == (5, 5)  # four supplied columns plus derived timestamp
    assert frame.TimeUS.tolist() == expected["TimeUS"]
    for name in ("Lat", "Lng", "Alt"):
        assert frame[name].tolist() == expected[name]
    assert frame.timestamp.tolist() == [value / 1e6 for value in expected["TimeUS"]]


def test_base64_web_bin_column_rejects_incomplete_float64_bytes():
    incomplete_double = base64.b64encode(b"\x00").decode("ascii")
    with raises(ValueError):
        Ardupilot.process_column("Alt", incomplete_double)
