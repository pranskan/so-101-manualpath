# Quick Reference - New Commands

## Delete Features

### Paths
```bash
del_path <name>      # Delete one saved path
del_all_paths        # Delete all saved paths (asks for confirmation)
```

### Camera Calibration
```bash
cam_cal_list         # Show all collected points (numbered 1-N)
cam_cal_del <n>      # Delete point #n (e.g., cam_cal_del 3)
cam_cal_clear        # Delete all points (asks for confirmation)
cam_cal_reset        # Delete transform file (asks for confirmation)
```

## Path Recording

### Current (Improved)
```bash
start <name>         # Start recording
add                  # Record current position
undo                 # Remove last waypoint ← NEW
save                 # Save path
```

### Execution
```bash
exec <name>          # Execute path (jump between waypoints)
smooth <name> [sec]  # Execute with smooth curves ← NEW
                     # Optional: duration in seconds (default 5)
```

## Examples

### Fixing a Bad Recording
```
[SO-101]> start assembly
[SO-101]> move 1 45
[SO-101]> add
[SO-101]> move 2 70
[SO-101]> add
[SO-101]> move 3 90
[SO-101]> add              # Oops, wrong position
[SO-101]> undo             # Remove that one
[SO-101]> move 3 85
[SO-101]> add              # Correct now
[SO-101]> save
```

### Recalibrating Camera After Moving It
```
[SO-101]> cam_cal_list     # See old points? Delete them
[SO-101]> cam_cal_clear    # Start fresh
[SO-101]> rs_init
[SO-101]> cam_cal          # New calibration
[SO-101]> cam_cal_list     # Review points
[SO-101]> cam_cal_del 2    # Remove bad one
[SO-101]> cam_cal          # Recalibrate with remaining
```

### Smooth Motion
```
[SO-101]> smooth pick 5.0    # Execute over 5 seconds
[SO-101]> smooth pick 10.0   # Execute over 10 seconds (slower)
[SO-101]> smooth pick        # Use default 5 seconds
```

## Key Points

- **Undo** works during recording (before save)
- **Smooth** uses cubic spline interpolation for fluid curves
- **Delete** commands ask for confirmation on destructive ops
- **cam_cal_del** uses 1-based numbering (matches display)
- All new commands integrated into help menu
