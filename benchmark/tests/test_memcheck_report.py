# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from benchmark.lib.report import render_memcheck_markdown
from benchmark.lib.leak_parse import MemcheckSummary


def test_memcheck_md():
    md = render_memcheck_markdown(MemcheckSummary(1024, 2048, 4))
    assert "definitely lost" in md.lower() and "1024" in md
