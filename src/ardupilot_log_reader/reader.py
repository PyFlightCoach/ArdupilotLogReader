from __future__ import annotations

import base64
import fnmatch
import logging
import os
from dataclasses import dataclass
from json import dump, load
from pathlib import Path
from typing import Literal

import numpy as np
import numpy.typing as npt
import pandas as pd

try:
    from pymavlink.DFReader import DFReader_binary

    HAS_PYMAVLINK = True
except ImportError:
    type DFReader_binary = None
    HAS_PYMAVLINK = False

logger = logging.getLogger(__name__)


@dataclass
class Ardupilot:
    filename: str
    dfs: dict[str, pd.DataFrame]

    def __getattr__(self, name):
        if name in self.dfs:
            return self.dfs[name]
        raise AttributeError(f"No such attribute: {name}")

    @staticmethod
    def process_patterns(
        available: list[str],
        patterns: list[str] | None = None,
        exclude_patterns: list[str] | None = None,
    ):
        def match_type(mtype, patterns):
            for p in patterns:
                if fnmatch.fnmatch(mtype, p):
                    return True
            return False

        patterns = available if patterns is None else patterns
        exclude_patterns = [] if exclude_patterns is None else exclude_patterns
        return [
            k
            for k in available
            if match_type(k, patterns) and not match_type(k, exclude_patterns)
        ]

    @staticmethod
    def parse(
        bin_file,
        types=None,
        nottypes=None,
        zero_time_base=False,
        source_system=None,
        source_component=None,
        link=None,
        mav10=False,
        cache_file: str | Path | bool = False,
    ) -> Ardupilot:
        """
        Parses a binary file into an Ardupilot object.

        Parameters:
        bin_file (str): The binary file to parse.
        types (list[str], optional): List of types or patterns to include in the parsing. Defaults to None.
        nottypes (list[str], optional): List of types or patterns to exclude from the parsing. Defaults to None.
        zero_time_base (bool, optional): If True, sets the time base to zero. Defaults to False.
        source_system (int, optional): The source system ID to filter messages by. Defaults to None (all systems in log).
        source_component (int, optional): The source component ID to filter messages by. Defaults to None (all components in log).
        link (int, optional): The link to filter messages by. Defaults to None.
        mav10 (bool, optional): If True, uses MAVLink 1.0. Defaults to False.

        Returns:
        Ardupilot: The parsed Ardupilot object.
        """
        if types is None:
            types = ["POS", "ATT", "IMU", "XKF1", "XKF2", "ERR", "GPS", "ORGN", "RCOU", "RCIN"]
        if cache_file:
            cache_file = (
                Path(bin_file).with_suffix(".json")
                if cache_file is True
                else Path(cache_file)
            )
            if cache_file.exists():
                with open(cache_file, "r") as f:
                    data = load(f)
                return Ardupilot.from_dict(data)

        if not HAS_PYMAVLINK:
            raise ImportError(
                "pymavlink is required to parse Ardupilot logs. Please install it."
            )
        if not mav10:
            os.environ["MAVLINK20"] = "1"

        mlog: DFReader_binary = DFReader_binary(
            str(bin_file), zero_time_base=zero_time_base
        )

        match_types = Ardupilot.process_patterns(
            list(mlog.name_to_id.keys()), list(set(types + ["PARM"])), nottypes
        )

        log = Ardupilot._parse(mlog, match_types, source_system, source_component, link)

        mlog.filehandle.close()

        if cache_file:
            with open(cache_file, "w") as f:
                dump(log.to_dict(), f, indent=4)

        return log

    @staticmethod
    def _parse(
        mlog: DFReader_binary,
        cols: list[str],
        src_system=None,
        src_component=None,
        link=None,
    ):
        dfs_dicts = {}

        while True:
            m = mlog.recv_match(blocking=False, type=cols)
            if m is None:
                break
            if src_system is not None and src_system != m.get_srcSystem():
                continue
            if src_component is not None and src_component != m.get_srcComponent():
                continue
            if link is not None and link != m._link:
                continue

            key = m.get_type()

            if key not in dfs_dicts:
                if key == "BAD_DATA":
                    continue
                dfs_dicts[key] = {}
                dfs_dicts[key]["timestamp"] = []
                for field in m.get_fieldnames():
                    dfs_dicts[key][field] = []

            dfs_dicts[key]["timestamp"].append(getattr(m, "_timestamp", 0.0))

            if key == "XKF2":
                pass

            for field in m.get_fieldnames():
                dfs_dicts[key][field].append(getattr(m, field))

        return Ardupilot(
            mlog.filehandle.name, {k: pd.DataFrame(v) for k, v in dfs_dicts.items()}
        )._correct_timestamps()

    def parameters(self) -> dict[str, pd.DataFrame]:
        gb = self.PARM.groupby("Name")

        parms = {}
        for gn in gb.groups:
            gr = gb.get_group(gn)
            parms[gn] = gr.loc[
                abs(gr.Value.diff().fillna(1)) > 0, ["timestamp", "TimeUS", "Value"]
            ].set_index("timestamp")
        return parms

    def to_dict(self, **kwargs) -> dict[str, dict[str, list]]:
        return {
            "filename": self.filename,
            "data": {k: Ardupilot.write_df(v, **kwargs) for k, v in self.dfs.items()},
        }

    @staticmethod
    def write_df(df: pd.DataFrame, **kwargs) -> dict[str, list | str]:
        return {k: Ardupilot.write_column(v, **kwargs) for k, v in df.items()}

    @staticmethod
    def write_column(
        data: pd.Series, mode: Literal["list", "base64", "records"] = "base64"
    ) -> str | list | dict:
        if mode == "records":
            return data.to_dict(orient="records")
        elif mode == "list":
            return data.to_dict(orient="list")
        elif mode == "base64":
            if data.dtype == np.float64:
                return base64.b64encode(data.to_numpy().tobytes()).decode("utf-8")
            else:
                return data.tolist()

    @staticmethod
    def from_dict(data: dict[str, dict[str, list]]) -> Ardupilot:

        if "filename" in data and "data" in data:
            return Ardupilot._from_dict(data)
        else:
            return Ardupilot._from_web_dict(data)

    @staticmethod
    def _from_dict(data: dict[str, dict[str, list]]) -> Ardupilot:
        return Ardupilot(
            data["filename"],
            {k: Ardupilot.parse_df(v) for k, v in data["data"].items()},
        )

    @staticmethod
    def _from_web_dict(bindata: dict[str, dict[str, list]]) -> Ardupilot:
        # dfs = {k: pd.DataFrame(v) for k,v in bindata.items()}

        dfs: dict[str, pd.DataFrame] = {}
        groups = {}
        for k, v in bindata.items():
            new_df = Ardupilot.parse_df(v)
            if new_df is not None:
                if "[" not in k:
                    dfs[k] = new_df
                else:
                    nk = k.split("[")[0]
                    if nk not in groups:
                        groups[nk] = []
                    groups[nk].append(new_df)

        for k, v in groups.items():
            dfs[k] = pd.concat(v)
            colsort = ["time_boot_s"]
            for core_col in ["I", "C"]:
                if core_col in dfs[k].columns:
                    colsort.append(core_col)
                    break
            dfs[k] = dfs[k].sort_values(colsort)

        def process_df(df: pd.DataFrame) -> pd.DataFrame:
            df.insert(0, "TimeUS", np.floor(df.time_boot_s * 1e6).astype(int))  # ms
            df.insert(0, "timestamp", df.time_boot_s)

            return df.drop(columns="time_boot_s")

        dfs = {k: process_df(v) for k, v in dfs.items() if not v.empty}

        return Ardupilot("web_dfs", dfs)._correct_timestamps()

    def _correct_timestamps(self):
        if "GPS" in self.dfs:
            gps = self.dfs["GPS"]

            start_time = (
                Ardupilot._gpsTimeToTime(
                    gps.GWk.iloc[0],
                    gps.GMS.iloc[0] / 1e3
                    if gps.GMS.diff().mean() > 10
                    else gps.GMS.iloc[0],
                )
                - (gps.TimeUS / 1e6).iloc[0]
            )
        else:
            start_time = 0

        return Ardupilot(
            self.filename,
            {
                k: v.assign(timestamp=v.timestamp + start_time)
                for k, v in self.dfs.items()
            },
        )

    @staticmethod
    def _gpsTimeToTime(week, msec):
        """convert GPS week and TOW to a time in seconds since 1970"""
        epoch = 86400 * (10 * 365 + int((1980 - 1969) / 4) + 1 + 6 - 2)
        return epoch + 86400 * 7 * week + msec * 0.001 - 18

    @staticmethod
    def parse_df(data: dict[str, list | str], **kwargs) -> pd.DataFrame:
        df = pd.DataFrame(
            {k: Ardupilot.process_column(k, v) for k, v in data.items()}, **kwargs
        )

        return df

    @staticmethod
    def process_column(name: str, data: str | list | dict) -> pd.Series:
        if isinstance(data, str):
            _data = np.frombuffer(base64.b64decode(data), dtype=np.float64)
            return pd.Series(_data, name=name)
        elif isinstance(data, list):
            return pd.Series(data, name=name)
        elif isinstance(data, dict):
            return pd.Series(data, name=name).reset_index(drop=True)
        else:
            raise TypeError(f"Unsupported data type for column {name}: {type(data)}")
