# Yard Mapping Project

Drone mission planner + workflow for mapping a yard with a DJI Mavic Air 2 via Litchi.

## Files

- `yard_mission_planner.py` — the planner script
- `yard_polygon.csv` — yard polygon vertices (lat,lon per row). **Edit this with your actual corners.**
- `example_yard_polygon.csv` — sample polygon for reference
- `mission_config.yaml` — mission parameters (annotated; all settings have script defaults)
- `mission_output/` — generated CSVs + KML preview

## Setup

```bash
pip install shapely simplekml pyyaml
```

## Usage

```bash
# Edit yard_polygon.csv with your actual yard corners first
python yard_mission_planner.py
```

Outputs land in `./mission_output/`:
- `mission_nadir.csv` — Litchi import for the top-down grid mission
- `mission_orbit.csv` — Litchi import for the perimeter oblique orbit
- `mission_combined.csv` — both missions concatenated, single import for one continuous flight
- `missions_preview.kml` — open in Google Earth to verify before flying

## Getting yard coordinates

Open Google Maps satellite view, right-click each corner of your yard, click the
lat/lon at the top of the menu (auto-copies). Walk the perimeter in one direction
(CW or CCW both fine) and paste each pair as a row in `yard_polygon.csv`.

## Pre-flight: camera setup (DJI Fly)

Litchi's camera control surface for the Mavic Air 2 is limited — set these in
DJI Fly first, then switch to Litchi. Settings persist on the drone.

- Photo mode: **Single Shot** (not AEB, HDR, Burst, Hyperlight)
- Format: **JPG only** (JPG+RAW writes are too slow and cause dropped photos)

Format the SD card in DJI Fly before flying (exFAT, U3/V30 or better). Copy
off any photos you want to keep first — format wipes the card.

## Pre-flight: Litchi global mission settings

**This is the #1 source of grief.** The Litchi CSV format only encodes
*per-waypoint* values. Global mission settings (Path Mode, Heading Mode,
Cruising Speed, etc.) are **not stored in the CSV** — they live in the Litchi
app/Mission Hub and will silently override per-waypoint values if misconfigured.

The symptom of getting this wrong: drone flies through waypoints without
stopping, and photos either don't trigger or only the first one fires.

After importing the CSV into Litchi, set these globals on the mission:

| Setting | Value | Why |
|---|---|---|
| Path Mode | **Straight Lines** | Without this, the drone smooths through waypoints and never "arrives" — Take Photo actions never fire |
| Default Curve Size | **0%** | Forces a full stop at every waypoint; overrides per-wp curvesize if non-zero |
| Heading Mode | **Custom** | Uses per-waypoint heading values from the CSV (not POI, not Auto) |
| Default Gimbal Pitch Mode | **Interpolate** | Smoothly transitions gimbal between nadir (−90°) and orbit (−58°) sections |
| Rotation Directions | **Managed** | Picks shortest rotation per turn |
| Photo Capture Interval | **disabled** | We use per-waypoint Take Photo actions; intervals would double-shoot |
| Cruising Speed | **3 m/s** | Low enough that the drone can brake to a full stop at each waypoint |
| Finish Action | **RTH** | Drone returns home when mission ends (instead of hovering on the perimeter waiting for input) |

## Pre-flight: drone & app hygiene

- **Re-import the CSV every time you regenerate it.** Litchi snapshots the
  mission on import — saving a new file on disk does *not* update what Litchi
  will fly. Delete the old mission from Mission Hub, re-import the new CSV.
- **Force-close DJI Fly before opening Litchi.** Background DJI Fly intercepts
  the camera and breaks Litchi's photo triggers.
- **Power-cycle the drone and controller between flights** — clears wedged
  camera/gimbal state from the previous run.
- **Verify camera triggers on the first 2–3 waypoints** before letting the
  mission run unattended. Look for the photo counter incrementing or the
  shutter feedback (brief screen flash / audible click).

## Pre-flight: altitude vs obstacles

The mission altitudes come from `mission_config.yaml`. Standard rule of thumb
is **5–10 m clearance** above the tallest obstacle (trees, wires, structures).
GPS altitude drift is ±1–2 m and wind can push the drone down further, so a
2–3 m margin is too thin.

The KML preview (`missions_preview.kml`, open in Google Earth) is your final
sanity check before flying.

## Post-flight: process photos

1. Pull the SD card. Photo count should roughly match the waypoint count
   (planner prints both nadir and orbit waypoint counts on each run).
2. Spot-check ~10 photos at full res for sharpness; verify EXIF includes GPS
   coordinates (right-click → Get Info on Mac).
3. Process with **WebODM** (free, runs locally via Docker): drop both nadir
   and orbit photo sets into one task → orthomosaic + 3D model + point cloud.
4. View / analyze the orthomosaic GeoTIFF in **QGIS** (free).

## Mavic Air 2 specifics

The Mavic Air 2 / Air 2S use **Virtual Stick Control** in Litchi — the mission
isn't stored on the drone, it's driven moment-to-moment from the controller
via DJI's MSDK 5. This makes timing-sensitive operations (waypoint stops,
photo triggers) inherently less reliable than older drones (Mavic 2 Pro,
Phantom 4) where missions are stored onboard.

Expect occasional photo drops even with everything configured correctly. If
photos drop badly despite the checklist above, the fallback is **DroneLink**
(free tier supports Air 2 with native waypoint missions and a different SDK
path).
