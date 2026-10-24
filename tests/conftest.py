"""Test-suite defaults.

The filter's step deadline (``FilterConfig.step_deadline``, contract v1.5) is
wall-clock time, so with it on a test's result can depend on machine load: under
pytest-xdist each worker runs its own Numba thread pool, and a cluster can miss
its deadline in one run of a test and not in another (PR #26, Windows check).
The suite therefore runs with the deadline off by default, like the
verification scripts, so results reproduce on any machine. The runtime default
stays on; ``tests/test_filter_deadline.py`` turns it on explicitly and tests the
deadline behaviour, including forced misses.

The default is set in ``pytest_configure``, before the test modules import, so
module-level configs (``BRAKING = FilterConfig(barrier="braking")``) and the
``FilterConfig()`` that ``apply_filter`` and ``run_episode`` build when given
none get it too.
"""

import dataclasses


def pytest_configure(config):
    from utm_services.safety.filter import FilterConfig

    names = [f.name for f in dataclasses.fields(FilterConfig)]
    defaults = list(FilterConfig.__init__.__defaults__)
    defaults[names.index("step_deadline") - (len(names) - len(defaults))] = False
    FilterConfig.__init__.__defaults__ = tuple(defaults)
