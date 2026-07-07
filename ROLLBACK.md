# ROLLBACK — camera-file changes (R12)

Owner authorized edits to the formerly PROTECTED camera files
(RESUME_PLAN.md R12). Every touched file was backed up first; none of
these changes can be fully verified without the live AR0234, so each
commit is tagged NEEDS-HARDWARE where applicable.

**Register/format values are byte-identical** in all changes: device
path, 1920x1080@30, BA10 format, black level 2752, WB gains, intrinsics
(constants.hpp) and the V4L2 S_FMT/S_PARM setup are untouched.

## Backups (taken 2026-07-07 06:36, pre-change state = commit 21f380f)

| File | Backup | Restore command |
|------|--------|-----------------|
| src/cuas_fusion/src/drivers/camera_driver.cpp | camera_backups/camera_driver.cpp.20260707-063643.bak | `cp camera_backups/camera_driver.cpp.20260707-063643.bak src/cuas_fusion/src/drivers/camera_driver.cpp` |
| src/cuas_fusion/src/drivers/camera_node.cpp | camera_backups/camera_node.cpp.20260707-063643.bak | `cp camera_backups/camera_node.cpp.20260707-063643.bak src/cuas_fusion/src/drivers/camera_node.cpp` |
| src/cuas_fusion/src/cuas_color_correct_node.cpp | camera_backups/cuas_color_correct_node.cpp.20260707-063643.bak | `cp camera_backups/cuas_color_correct_node.cpp.20260707-063643.bak src/cuas_fusion/src/cuas_color_correct_node.cpp` |
| src/cuas_fusion/src/color_correct_engine.cpp | camera_backups/color_correct_engine.cpp.20260707-063643.bak | `cp camera_backups/color_correct_engine.cpp.20260707-063643.bak src/cuas_fusion/src/color_correct_engine.cpp` |
| src/cuas_fusion/include/cuas_fusion/common/constants.hpp | camera_backups/constants.hpp.20260707-063643.bak | `cp camera_backups/constants.hpp.20260707-063643.bak src/cuas_fusion/include/cuas_fusion/common/constants.hpp` |

Alternative restore for any file: `git checkout 21f380f -- <file>` then
rebuild (`colcon build --packages-select cuas_fusion`).

## Per-commit log

(One camera fix per commit; entries appended as commits land.)

## Hardware verification procedure (after each of fixes d/e/f)

```
ros2 launch cuas_fusion full.launch.py
ros2 topic hz /camera/image_raw      # expect ~30 Hz
```
Image must look identical (colors, exposure, no tearing); inference
must still detect. If anything looks off: restore from the table above
(or `git revert <commit>`), rebuild, relaunch.
