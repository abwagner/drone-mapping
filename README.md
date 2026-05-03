# Yard Mapping Project

Drone mission planner + workflow checklists for mapping a yard with a DJI Mavic Air 2.

## Files

- `yard_mission_planner.py` — the planner script
- `yard_polygon.csv` — yard polygon vertices (lat,lon per row). **Edit this with your actual corners.**
- `mission_config.yaml` — mission parameters (annotated example; all settings have script defaults if omitted)
- `yard_mapping_checklist.md` — the full workflow from prep through QGIS analysis
- `example_output/` — sample output from the placeholder polygon, so you can see what to expect

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
- `missions_preview.kml` — open in Google Earth to verify before flying

## Getting yard coordinates

Open Google Maps satellite view, right-click each corner of your yard, click the
lat/lon at the top of the menu (auto-copies). Walk the perimeter in one direction
(CW or CCW both fine) and paste each pair as a row in `yard_polygon.csv`.
