PATH FOLLOWING FIX

Replace/add these two files in the same MATLAB project folder:
1. main.m
2. purePursuitPathController.m

The new controller uses monotonic arc-length progress and always chooses its
look-ahead target from the planned path. It does not aim directly at the goal.

Important:
- Keep purePursuitPathController.m in the same folder as main.m.
- The old waypointController.m may remain, but main.m no longer calls it.
- Run main.m from this project folder.
