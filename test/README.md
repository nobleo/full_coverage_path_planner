Test for Full coverage path planner
===================================

The full coverage path planner consists of several parts that are each tested separately.

The move_base_flex plugin consists of several parts, each unit-tested separately:
- test_common: tests common.h
- test_boustrophedon_stc: tests static functions of boustrophedon_stc.h

Besides unittests, there are also some launch files that both illustrate how to use the
- BoustrophedonSTC-plugin, in test/full_coverage_path_planner/test_full_coverage_path_planner.launch

Note that the .launch-files do not do any automatic testing or verification of anything,
they are there to make manual testing easier.
