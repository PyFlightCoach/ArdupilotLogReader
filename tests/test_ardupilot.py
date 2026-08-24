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

from json import load
from pathlib import Path

from pytest import fixture

from ardupilot_log_reader.reader import Ardupilot
import numpy as np
import pandas as pd


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
    with open("tests/test_inputs/old_web_bin.json", "r") as f:
        return Ardupilot.from_dict(load(f))


@fixture(scope="session")
def new_web_bin():
    with open("tests/test_inputs/new_web_bin.json", "r") as f:
        return Ardupilot.from_dict(load(f))

@fixture(scope="session")
def compressed_web_bin():
    with open("tests/test_inputs/new_compressed_web_bin.json", "r") as f:
        return Ardupilot.from_dict(load(f))



@fixture(scope="session")
def raw_bin():
    return Ardupilot.parse(
        "tests/test_inputs/raw_web_bin.BIN",
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

def test_compressed_web_bin_matches_raw_bin(
    raw_bin: Ardupilot, compressed_web_bin: Ardupilot
):
    assert_dfs_match(raw_bin, compressed_web_bin)