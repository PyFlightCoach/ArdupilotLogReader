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
from ardupilot_log_reader.reader import Ardupilot
from pathlib import Path
import pytest


@pytest.fixture(scope="session")
def type_request():
    return ['XKF1', 'XKQ1', 'ARSP', 'GPS', 'RCIN', 'RCOU', 'IMU', 'BARO', 'MODE', 'RPM', 'MAG']

@pytest.fixture(scope="session")
def log(type_request):
    return Ardupilot.parse(Path(__file__).parent / 'test_inputs/test_log_00000052.BIN', types=type_request, zero_time_base=True)

@pytest.fixture(scope="session")
def log2(type_request):
    return Ardupilot.parse(Path(__file__).parent / 'test_inputs/00000129.BIN', types=type_request, zero_time_base=True)

def test_dfs(type_request, log):
    assert set(log.dfs.keys()) == set(type_request + ['PARM'])





