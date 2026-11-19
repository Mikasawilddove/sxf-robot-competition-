"""Compatibility entry for suat_mission.navigation.navigation_test_geometry."""

if __name__ == '__main__':
    from runpy import run_module

    run_module('suat_mission.navigation.navigation_test_geometry', run_name='__main__', alter_sys=True)
else:
    from importlib import import_module
    import sys

    sys.modules[__name__] = import_module('suat_mission.navigation.navigation_test_geometry')
