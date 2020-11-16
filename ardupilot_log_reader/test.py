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
import unittest



class TestReader(unittest.TestCase):
    def setUp(self):
        self.readtypes = ['ARSP', 'ATT']
        self.res = Ardupilot('./test/00000054.BIN', types=self.readtypes)

    def test_dfs(self):
        dfs = self.res.dfs
        self.assertEqual(set(dfs.keys()), set(self.readtypes + ['PARM']))

    def test_full_df(self):
        fulldf = self.res.full_df()
        self.assertEqual(len(fulldf), 1221)

if __name__ == "__main__":
    unittest.main()