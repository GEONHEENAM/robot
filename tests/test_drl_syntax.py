"""drl/ 폴더의 .drl 파일이 문법적으로 파싱되는지만 확인한다.

주의: DRL 함수(movej, posx 등)가 실제로 존재하는지, 좌표가 안전한지는 검사하지 못한다.
들여쓰기·괄호·콜론 누락 같은 '로봇에 올리기 전 바보 같은 실수'만 걸러낸다.
"""

from pathlib import Path

import pytest

DRL_FILES = sorted((Path(__file__).parent.parent / "drl").glob("**/*.drl"))


@pytest.mark.parametrize("path", DRL_FILES, ids=lambda p: p.name)
def test_drl_parses(path):
    compile(path.read_text(encoding="utf-8"), str(path), "exec")
