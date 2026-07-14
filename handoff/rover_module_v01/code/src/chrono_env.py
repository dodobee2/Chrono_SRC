"""PyChrono 실행 환경 부트스트랩.

이 프로젝트는 지난 학기 연습 레포의 소스 빌드 PyChrono(Chrono 9.0)를 재사용한다.
해당 빌드는 dylib 경로가 빌드 당시 절대경로로 박혀 있어, 임포트 전에
PYTHONPATH 와 DYLD_LIBRARY_PATH 가 모두 설정되어 있어야 한다.

macOS 에서는 이미 실행 중인 프로세스에 DYLD_LIBRARY_PATH 를 주입할 수 없으므로,
필요 시 환경변수를 채워서 현재 스크립트를 re-exec 한다.

사용법: pychrono 를 임포트하는 모든 진입점(스크립트/테스트)에서 가장 먼저

    from chrono_env import ensure_chrono
    ensure_chrono()
    import pychrono as chrono

환경변수 ROVER_CHRONO_BUILD_DIR 로 빌드 위치를 override 할 수 있다.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

DEFAULT_CHRONO_BUILD_DIR = Path(
    "~/Documents/Praxis/Project_Chrono_Practice/chrono_build"
).expanduser()

_REEXEC_GUARD = "ROVER_CHRONO_ENV_REEXEC"


def chrono_build_dir() -> Path:
    return Path(
        os.environ.get("ROVER_CHRONO_BUILD_DIR", str(DEFAULT_CHRONO_BUILD_DIR))
    ).expanduser()


def _libomp_dir() -> Path:
    return Path("/opt/homebrew/opt/libomp/lib")


def _fix_data_path(build: Path) -> None:
    """Chrono 데이터 경로 교정.

    빌드에 박힌 기본 데이터 경로가 옛 폴더명(Pneuma)이라 셰이더/컬러맵을
    못 읽는다 (VSG 시각화 segfault 원인). 실제 존재하는 경로로 재설정.
    """
    import pychrono as chrono

    for cand in (build / "data", build.parent / "chrono" / "data"):
        if cand.exists():
            chrono.SetChronoDataPath(str(cand) + "/")
            return


def _preload_dylibs(lib_dir: Path) -> bool:
    """chrono dylib 들을 ctypes 로 전역 로드한다. 하나라도 로드되면 True."""
    import ctypes

    omp = _libomp_dir() / "libomp.dylib"
    candidates = ([omp] if omp.exists() else []) + sorted(lib_dir.glob("*.dylib"))
    pending = list(candidates)
    loaded_any = False
    for _ in range(4):  # 의존성 순서 해소용 멀티패스
        still = []
        for f in pending:
            try:
                ctypes.CDLL(str(f), mode=ctypes.RTLD_GLOBAL)
                loaded_any = True
            except OSError:
                still.append(f)
        if not still:
            break
        pending = still
    return loaded_any


def ensure_chrono() -> None:
    """pychrono 임포트가 가능하도록 보장한다. 필요하면 re-exec 한다."""
    build = chrono_build_dir()
    bin_dir = build / "bin"
    lib_dir = build / "lib"

    if not bin_dir.exists():
        raise SystemExit(
            f"Chrono 빌드를 찾지 못했습니다: {build}\n"
            "ROVER_CHRONO_BUILD_DIR 환경변수로 chrono_build 위치를 지정하세요."
        )

    if str(bin_dir) not in sys.path:
        sys.path.insert(0, str(bin_dir))

    try:
        import pychrono  # noqa: F401
        _fix_data_path(build)
        return
    except ImportError:
        pass

    # 1차 시도: dylib 을 ctypes 로 직접 사전 로드 (DYLD 환경변수 불필요 →
    # pytest 등 이미 실행 중인 프로세스에서도 동작).
    # 빌드에 박힌 절대경로가 달라도, 같은 install name 으로 이미 로드된
    # 이미지를 dyld 가 재사용하므로 임포트가 성공한다.
    # 의존성 순서를 모르는 채 로드하므로 성공할 때까지 멀티패스로 재시도.
    if _preload_dylibs(lib_dir):
        try:
            import pychrono  # noqa: F401
            _fix_data_path(build)
            return
        except ImportError:
            pass

    if os.environ.get(_REEXEC_GUARD):
        raise ImportError(
            "재실행 후에도 pychrono 임포트 실패 — chrono_build 상태를 확인하세요."
        )

    # DYLD_LIBRARY_PATH 를 채워서 동일 스크립트를 재실행
    env = dict(os.environ)
    dyld_parts = [str(lib_dir)]
    if _libomp_dir().exists():
        dyld_parts.append(str(_libomp_dir()))
    if env.get("DYLD_LIBRARY_PATH"):
        dyld_parts.append(env["DYLD_LIBRARY_PATH"])
    env["DYLD_LIBRARY_PATH"] = ":".join(dyld_parts)
    env["PYTHONPATH"] = (
        str(bin_dir) + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    )
    env[_REEXEC_GUARD] = "1"
    os.execve(sys.executable, [sys.executable] + sys.argv, env)
