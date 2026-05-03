"""
Yard mission planner for DJI Mavic Air 2.

Generates two missions over a user-defined yard polygon:
  1. Nadir grid (camera straight down) for orthomosaic
  2. Perimeter orbit (camera angled inward/down) for oblique coverage

Inputs:
  - yard_polygon.csv  -- one row per polygon vertex: lat,lon
  - mission_config.yaml -- mission parameters (GSD target, overlap, etc.)

Outputs:
  - Litchi-compatible CSV files (one per mission)
  - KML file with both missions overlaid for Google Earth visualization

Usage:
    python yard_mission_planner.py
    python yard_mission_planner.py --polygon my_yard.csv --config my_config.yaml
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import simplekml
import yaml
from shapely.geometry import LineString, Polygon

# =============================================================================
# DJI MAVIC AIR 2 CAMERA SPECS  (constants -- update for drone)
# =============================================================================

SENSOR_WIDTH_MM = 6.4
SENSOR_HEIGHT_MM = 4.8
FOCAL_LENGTH_MM = 4.5
IMAGE_WIDTH_PX = 4000
IMAGE_HEIGHT_PX = 3000

EARTH_RADIUS_M = 6_378_137.0


# =============================================================================
# INPUT LOADING
# =============================================================================

def load_polygon_csv(path: Path) -> list[tuple[float, float]]:
    """Load polygon vertices from a CSV with lat,lon per row.

    Header rows, comments (#), and blank lines are skipped.
    Vertices should be in order around the perimeter (CW or CCW both fine).
    """
    vertices: list[tuple[float, float]] = []
    with path.open() as f:
        reader = csv.reader(f)
        for row_num, row in enumerate(reader, start=1):
            if not row or not row[0].strip() or row[0].strip().startswith("#"):
                continue
            if len(row) < 2:
                raise ValueError(f"{path}:{row_num}: expected lat,lon, got {row!r}")
            try:
                lat = float(row[0].strip())
                lon = float(row[1].strip())
            except ValueError:
                # Likely a header row; skip if it's the first non-comment row
                if not vertices:
                    continue
                raise ValueError(f"{path}:{row_num}: couldn't parse {row!r} as lat,lon")
            vertices.append((lat, lon))

    if len(vertices) < 3:
        raise ValueError(f"{path}: need at least 3 vertices, got {len(vertices)}")
    return vertices


def load_config(path: Path) -> dict:
    """Load YAML config, providing defaults for missing keys."""
    defaults = {
        "nadir": {
            "target_gsd_cm_per_px": 1.8,    # determines altitude
            "front_overlap": 0.75,
            "side_overlap": 0.65,
            "speed_mps": 6.0,
            "heading_deg": "auto",          # "auto" = align to polygon's long axis
            "edge_buffer_m": 5.0,           # how far outside polygon to extend sweep grid
        },
        "orbit": {
            "altitude_m": 24.0,             # 80 ft
            "inset_m": 5.0,
            "photo_spacing_m": 4.0,
            "gimbal_pitch_deg": -45.0,
            "speed_mps": 3.0,
        },
        "output_dir": "./mission_output",
    }
    with path.open() as f:
        loaded = yaml.safe_load(f) or {}

    def merge(base: dict, override: dict) -> dict:
        result = dict(base)
        for k, v in override.items():
            if isinstance(v, dict) and isinstance(result.get(k), dict):
                result[k] = merge(result[k], v)
            else:
                result[k] = v
        return result

    return merge(defaults, loaded)


# =============================================================================
# DERIVED PARAMETERS
# =============================================================================

def altitude_for_gsd(gsd_cm_per_px: float) -> float:
    """Altitude (m) needed to achieve the given ground sample distance.

    GSD = (sensor_width_mm * altitude_m * 100) / (focal_length_mm * image_width_px)
    """
    return (gsd_cm_per_px * FOCAL_LENGTH_MM * IMAGE_WIDTH_PX) / (
        SENSOR_WIDTH_MM * 100
    )


def ground_footprint_m(altitude_m: float) -> tuple[float, float]:
    width = (SENSOR_WIDTH_MM * altitude_m) / FOCAL_LENGTH_MM
    height = (SENSOR_HEIGHT_MM * altitude_m) / FOCAL_LENGTH_MM
    return width, height


def auto_heading_deg(yard_local: Polygon) -> float:
    """Heading aligned with the polygon's long axis -> fewer turns, less battery.

    Uses the minimum-rotated-rectangle's longer edge orientation.
    Returns degrees clockwise from north.
    """
    mrr = yard_local.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)[:4]
    edges = [(coords[i], coords[(i + 1) % 4]) for i in range(4)]
    longest = max(edges, key=lambda e: math.dist(e[0], e[1]))
    (x0, y0), (x1, y1) = longest
    dx, dy = x1 - x0, y1 - y0
    # Compass: 0=N (positive y), 90=E (positive x). Mod 180 = direction-agnostic.
    return math.degrees(math.atan2(dx, dy)) % 180


# =============================================================================
# COORDINATE CONVERSION
# =============================================================================

@dataclass
class LocalOrigin:
    lat: float
    lon: float

    def to_local(self, lat: float, lon: float) -> tuple[float, float]:
        d_lat = math.radians(lat - self.lat)
        d_lon = math.radians(lon - self.lon)
        north = d_lat * EARTH_RADIUS_M
        east = d_lon * EARTH_RADIUS_M * math.cos(math.radians(self.lat))
        return east, north

    def to_latlon(self, east: float, north: float) -> tuple[float, float]:
        d_lat = north / EARTH_RADIUS_M
        d_lon = east / (EARTH_RADIUS_M * math.cos(math.radians(self.lat)))
        return self.lat + math.degrees(d_lat), self.lon + math.degrees(d_lon)


def make_origin(polygon_latlon: list[tuple[float, float]]) -> LocalOrigin:
    avg_lat = sum(p[0] for p in polygon_latlon) / len(polygon_latlon)
    avg_lon = sum(p[1] for p in polygon_latlon) / len(polygon_latlon)
    return LocalOrigin(avg_lat, avg_lon)


def polygon_to_local(
    polygon_latlon: list[tuple[float, float]], origin: LocalOrigin
) -> Polygon:
    coords = [origin.to_local(lat, lon) for lat, lon in polygon_latlon]
    return Polygon(coords)


# =============================================================================
# NADIR GRID GENERATION
# =============================================================================

def generate_nadir_grid(
    yard_local: Polygon,
    altitude_m: float,
    front_overlap: float,
    side_overlap: float,
    buffer_m: float,
    heading_deg: float,
) -> list[tuple[float, float]]:
    """Serpentine grid waypoints in local (east, north) meters."""
    fp_w, fp_h = ground_footprint_m(altitude_m)
    line_spacing = fp_w * (1 - side_overlap)
    photo_spacing = fp_h * (1 - front_overlap)

    expanded = yard_local.buffer(buffer_m)
    theta = math.radians(heading_deg)

    def rotate(pt: tuple[float, float], angle_rad: float) -> tuple[float, float]:
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        x, y = pt
        return (c * x - s * y, s * x + c * y)

    rotated_coords = [rotate(c, -theta) for c in expanded.exterior.coords]
    rotated_poly = Polygon(rotated_coords)
    min_x, min_y, max_x, max_y = rotated_poly.bounds

    waypoints_rotated: list[tuple[float, float]] = []
    x = min_x
    flip = False
    while x <= max_x:
        line = LineString([(x, min_y - 1), (x, max_y + 1)])
        clipped = line.intersection(rotated_poly)
        if clipped.is_empty:
            x += line_spacing
            continue

        if clipped.geom_type == "LineString":
            segments = [clipped]
        elif clipped.geom_type == "MultiLineString":
            segments = list(clipped.geoms)
        else:
            x += line_spacing
            continue

        for seg in segments:
            ys = sorted([seg.coords[0][1], seg.coords[-1][1]])
            y_start, y_end = ys
            seg_pts: list[tuple[float, float]] = []
            y = y_start
            while y <= y_end:
                seg_pts.append((x, y))
                y += photo_spacing
            if seg_pts and seg_pts[-1][1] < y_end:
                seg_pts.append((x, y_end))
            if flip:
                seg_pts.reverse()
            waypoints_rotated.extend(seg_pts)

        flip = not flip
        x += line_spacing

    return [rotate(p, theta) for p in waypoints_rotated]


# =============================================================================
# PERIMETER ORBIT GENERATION
# =============================================================================

def generate_perimeter_orbit(
    yard_local: Polygon, inset_m: float, photo_spacing_m: float
) -> list[tuple[float, float, float]]:
    """Inward-offset orbit. Returns (east, north, heading_deg) tuples."""
    inset_poly = yard_local.buffer(-inset_m)
    if inset_poly.is_empty:
        raise ValueError(
            f"Inset of {inset_m}m collapses the polygon -- yard too small or inset too large."
        )

    ring = inset_poly.exterior
    perimeter = ring.length
    n_points = max(8, int(perimeter / photo_spacing_m))

    centroid = yard_local.centroid
    cx, cy = centroid.x, centroid.y

    waypoints = []
    for i in range(n_points):
        d = (i / n_points) * perimeter
        pt = ring.interpolate(d)
        dx, dy = cx - pt.x, cy - pt.y
        heading = math.degrees(math.atan2(dx, dy)) % 360
        waypoints.append((pt.x, pt.y, heading))
    return waypoints


# =============================================================================
# LITCHI CSV EXPORT
# =============================================================================

LITCHI_HEADER = ["latitude", "longitude", "altitude(m)", "heading(deg)",
                 "curvesize(m)", "rotationdir", "gimbalmode", "gimbalpitchangle"]
for _i in range(1, 16):
    LITCHI_HEADER.extend([f"actiontype{_i}", f"actionparam{_i}"])
LITCHI_HEADER.extend(["altitudemode", "speed(m/s)", "poi_latitude", "poi_longitude",
                      "poi_altitude(m)", "poi_altitudemode",
                      "photo_timeinterval", "photo_distinterval"])


def write_litchi_csv(
    path: Path,
    waypoints_latlon: list[tuple[float, float, float, float]],
    altitude_m: float,
    speed_mps: float,
) -> None:
    """waypoints_latlon: list of (lat, lon, heading_deg, gimbal_pitch_deg)."""
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(LITCHI_HEADER)
        for lat, lon, heading, gimbal in waypoints_latlon:
            row = [
                lat, lon, altitude_m, heading,
                0.2,    # curvesize: small for sharp corners
                0,      # rotationdir
                2,      # gimbalmode: interpolate
                gimbal,
                0, 0,   # actiontype1=take_photo, actionparam1
            ]
            row.extend([-1, 0] * 14)  # 14 unused action slots
            row.extend([
                0,             # altitudemode: above takeoff
                speed_mps,
                0, 0, 0, 0,    # POI fields unused
                -1, -1,        # photo intervals (per-wp action handles this)
            ])
            writer.writerow(row)


# =============================================================================
# KML EXPORT
# =============================================================================

def write_kml(
    path: Path,
    yard_polygon_latlon: list[tuple[float, float]],
    nadir_waypoints: list[tuple[float, float]],
    orbit_waypoints: list[tuple[float, float]],
    nadir_altitude_m: float,
    orbit_altitude_m: float,
) -> None:
    kml = simplekml.Kml()

    yard = kml.newpolygon(name="Yard boundary")
    yard_coords = [(lon, lat) for lat, lon in yard_polygon_latlon]
    yard_coords.append(yard_coords[0])
    yard.outerboundaryis = yard_coords
    yard.style.linestyle.color = simplekml.Color.yellow
    yard.style.linestyle.width = 3
    yard.style.polystyle.color = simplekml.Color.changealphaint(50, simplekml.Color.yellow)

    nadir_line = kml.newlinestring(name=f"Nadir grid ({nadir_altitude_m:.0f}m AGL)")
    nadir_line.coords = [(lon, lat, nadir_altitude_m) for lat, lon in nadir_waypoints]
    nadir_line.altitudemode = simplekml.AltitudeMode.relativetoground
    nadir_line.style.linestyle.color = simplekml.Color.cyan
    nadir_line.style.linestyle.width = 2

    orbit_line = kml.newlinestring(name=f"Perimeter orbit ({orbit_altitude_m:.0f}m AGL)")
    orbit_coords = [(lon, lat, orbit_altitude_m) for lat, lon in orbit_waypoints]
    if orbit_coords:
        orbit_coords.append(orbit_coords[0])
    orbit_line.coords = orbit_coords
    orbit_line.altitudemode = simplekml.AltitudeMode.relativetoground
    orbit_line.style.linestyle.color = simplekml.Color.magenta
    orbit_line.style.linestyle.width = 2

    for i, (lat, lon) in enumerate(nadir_waypoints):
        pt = kml.newpoint(name=f"N{i+1}", coords=[(lon, lat, nadir_altitude_m)])
        pt.altitudemode = simplekml.AltitudeMode.relativetoground
        pt.style.iconstyle.scale = 0.4
        pt.style.iconstyle.color = simplekml.Color.cyan
        pt.style.labelstyle.scale = 0

    for i, (lat, lon) in enumerate(orbit_waypoints):
        pt = kml.newpoint(name=f"O{i+1}", coords=[(lon, lat, orbit_altitude_m)])
        pt.altitudemode = simplekml.AltitudeMode.relativetoground
        pt.style.iconstyle.scale = 0.5
        pt.style.iconstyle.color = simplekml.Color.magenta
        pt.style.labelstyle.scale = 0

    kml.save(str(path))


# =============================================================================
# MAIN
# =============================================================================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plan yard mapping missions for Mavic Air 2")
    p.add_argument("--polygon", type=Path, default=Path("yard_polygon.csv"),
                   help="CSV with polygon vertices (lat,lon per row)")
    p.add_argument("--config", type=Path, default=Path("mission_config.yaml"),
                   help="YAML mission parameters")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    polygon_latlon = load_polygon_csv(args.polygon)
    config = load_config(args.config)

    output_dir = Path(config["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)

    origin = make_origin(polygon_latlon)
    yard_local = polygon_to_local(polygon_latlon, origin)

    print(f"Loaded polygon: {len(polygon_latlon)} vertices from {args.polygon}")
    print(f"Loaded config: {args.config}")
    print(f"Yard area: {yard_local.area:.0f} m^2  ({yard_local.area / 4046.86:.3f} acres)")
    bounds = yard_local.bounds
    print(f"Bounding box: {bounds[2] - bounds[0]:.1f}m x {bounds[3] - bounds[1]:.1f}m")

    # --- Nadir mission ---
    nadir_cfg = config["nadir"]
    target_gsd = nadir_cfg["target_gsd_cm_per_px"]
    nadir_altitude = altitude_for_gsd(target_gsd)
    fp_w, fp_h = ground_footprint_m(nadir_altitude)
    nadir_buffer = float(nadir_cfg["edge_buffer_m"])

    if nadir_cfg["heading_deg"] == "auto":
        nadir_heading = auto_heading_deg(yard_local)
        heading_label = f"{nadir_heading:.1f}° (auto, aligned to long axis)"
    else:
        nadir_heading = float(nadir_cfg["heading_deg"])
        heading_label = f"{nadir_heading:.1f}° (manual)"

    print(f"\nNadir mission:")
    print(f"  Target GSD: {target_gsd} cm/px -> altitude {nadir_altitude:.1f}m "
          f"({nadir_altitude * 3.281:.0f}ft)")
    print(f"  Ground footprint: {fp_w:.1f}m x {fp_h:.1f}m per photo")
    print(f"  Line spacing: {fp_w * (1 - nadir_cfg['side_overlap']):.1f}m  "
          f"(side overlap {nadir_cfg['side_overlap'] * 100:.0f}%)")
    print(f"  Photo spacing: {fp_h * (1 - nadir_cfg['front_overlap']):.1f}m  "
          f"(front overlap {nadir_cfg['front_overlap'] * 100:.0f}%)")
    print(f"  Heading: {heading_label}")
    print(f"  Edge buffer: {nadir_buffer:.1f}m")

    nadir_local = generate_nadir_grid(
        yard_local, nadir_altitude,
        nadir_cfg["front_overlap"], nadir_cfg["side_overlap"],
        nadir_buffer, nadir_heading,
    )
    nadir_latlon = [
        (lat, lon, 0.0, -90.0)
        for east, north in nadir_local
        for lat, lon in [origin.to_latlon(east, north)]
    ]
    print(f"  Waypoints: {len(nadir_latlon)}")
    if len(nadir_local) > 1:
        path_len = sum(
            math.hypot(nadir_local[i+1][0] - nadir_local[i][0],
                       nadir_local[i+1][1] - nadir_local[i][1])
            for i in range(len(nadir_local) - 1)
        )
        flight_time = path_len / nadir_cfg["speed_mps"] + len(nadir_local) * 1.5
        print(f"  Path length: {path_len:.0f}m, est. flight time: {flight_time / 60:.1f} min")

    # --- Orbit ---
    orbit_cfg = config["orbit"]
    print(f"\nOrbit mission @ {orbit_cfg['altitude_m']:.0f}m AGL:")
    orbit_local = generate_perimeter_orbit(
        yard_local, orbit_cfg["inset_m"], orbit_cfg["photo_spacing_m"]
    )
    orbit_latlon = [
        (lat, lon, heading, orbit_cfg["gimbal_pitch_deg"])
        for east, north, heading in orbit_local
        for lat, lon in [origin.to_latlon(east, north)]
    ]
    print(f"  Waypoints: {len(orbit_latlon)}")
    if len(orbit_local) > 1:
        path_len = sum(
            math.hypot(orbit_local[(i+1) % len(orbit_local)][0] - orbit_local[i][0],
                       orbit_local[(i+1) % len(orbit_local)][1] - orbit_local[i][1])
            for i in range(len(orbit_local))
        )
        flight_time = path_len / orbit_cfg["speed_mps"] + len(orbit_local) * 1.5
        print(f"  Path length: {path_len:.0f}m, est. flight time: {flight_time / 60:.1f} min")

    # --- Outputs ---
    nadir_csv = output_dir / "mission_nadir.csv"
    orbit_csv = output_dir / "mission_orbit.csv"
    kml_path = output_dir / "missions_preview.kml"

    write_litchi_csv(nadir_csv, nadir_latlon, nadir_altitude, nadir_cfg["speed_mps"])
    write_litchi_csv(orbit_csv, orbit_latlon, orbit_cfg["altitude_m"], orbit_cfg["speed_mps"])
    write_kml(
        kml_path, polygon_latlon,
        [(lat, lon) for lat, lon, _, _ in nadir_latlon],
        [(lat, lon) for lat, lon, _, _ in orbit_latlon],
        nadir_altitude, orbit_cfg["altitude_m"],
    )

    print(f"\nWrote:")
    print(f"  {nadir_csv}")
    print(f"  {orbit_csv}")
    print(f"  {kml_path}")


if __name__ == "__main__":
    main()
