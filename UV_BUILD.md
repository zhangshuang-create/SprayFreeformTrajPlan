UV planning build option
========================

BUILD_UV_TRAJ_PLAN defaults to OFF. Slicing remains available.
OFF excludes UVTrajPlan, TrajModule, OptCutsCore and their bundled TBB from the build; the workflow selector offers slicing only. UV actions requested directly are rejected. Source and submodule declarations remain tracked. Independent TBB users are unaffected.

To restore UV, initialize the nested TBB submodule and configure with -DBUILD_UV_TRAJ_PLAN=ON. Enabling UV requires its full dependencies and has not been validated by the OFF-only checks.
