
Occlusion detection

- NCC_THRESH = 0.5 — appearance similarity floor. Raise it (0.6–0.7) if the tracker is too permissive with obstacles; lower it (0.3–0.4) if it falsely detects occlusion during legitimate camera motion
with lighting change.
- NCC_REDETECT_THRESH = 0.7 — must stay above this to allow feature re-detection. Always keep it higher than NCC_THRESH. Closing the gap between the two makes the tracker more permissive about refreshing
features; widening it makes migration to distractors harder.
- INLIER_RATIO_THRESH = 0.5 — fraction of RANSAC inliers required. Raise it (0.6–0.7) to be stricter about motion consistency; lower it if the tracker falsely triggers occlusion when many features are
legitimately lost during fast camera motion.
Tracking quality
- FB_THRESH = 1.0 — forward-backward round-trip tolerance in pixels. Lower it (0.5) for cleaner but fewer surviving tracks; raise it (2.0) if the tracker loses points too fast on fast motion.
- MIN_FEATURES = 10 — minimum surviving points before declaring lost. Raise it for stricter tracking; lower it to survive heavy occlusion.
Stability
- EMA_ALPHA = 0.4 — corner smoothing weight on the new measurement. Raise toward 1.0 for faster response to real motion at the cost of more jitter; lower toward 0.2 for a smoother but laggier box.
Practical starting points by symptom:
┌────────────────────────────────┬────────────────────────────────────────────────────────────────────┐
│            Symptom             │                             Adjustment                             │
├────────────────────────────────┼────────────────────────────────────────────────────────────────────┤
│ Box still drifts to obstacle   │ Raise NCC_REDETECT_THRESH to 0.8, raise INLIER_RATIO_THRESH to 0.6 │
├────────────────────────────────┼────────────────────────────────────────────────────────────────────┤
│ False occlusion on camera move │ Lower NCC_THRESH to 0.35, lower INLIER_RATIO_THRESH to 0.35        │
├────────────────────────────────┼────────────────────────────────────────────────────────────────────┤
│ Box jitters on static camera   │ Lower EMA_ALPHA to 0.2                                             │
├────────────────────────────────┼────────────────────────────────────────────────────────────────────┤
│ Loses track too easily         │ Lower MIN_FEATURES to 6, raise FB_THRESH to 2.0                    │
├────────────────────────────────┼────────────────────────────────────────────────────────────────────┤
│ Tracks garbage features        │ Raise qualityLevel in FEATURE_PARAMS to 0.03–0.05                  │
└────────────────────────────────┴────────────────────────────────────────────────────────────────────┘
