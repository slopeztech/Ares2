#!/usr/bin/env python3
"""ARES telemetry post-processor and visualisation tool.

Reads CSV flight logs produced by the ARES RTT script and generates
plots, 3-D trajectories, and interactive maps.

Dependencies (install via pip):
    pip install pandas matplotlib plotly pydeck folium
"""

import argparse
import os
import math
import json

try:
    import pandas as pd
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pandas and matplotlib are required. Run: pip install pandas matplotlib"
    ) from exc

# Expected column schema
COLUMNS = [
    "timestamp", "mode",
    "acc_x", "acc_y", "acc_z", "acc_total",
    "gyro_x", "gyro_y", "gyro_z",
    "imu_temp",
    "altitude",
    "bmp_temp",
    "pressure",
    "lat", "lon",
    "gps_alt",
    "speed"
]

# Minimum columns for modes that only require GPS data.
GPS_MIN_COLUMNS = ["timestamp", "lat", "lon", "gps_alt"]


def resolve_csv_path(path):
    """Resolve a CSV path by searching common project locations."""
    if os.path.isabs(path) and os.path.isfile(path):
        return path

    # 1) Relative path from the current working directory
    if os.path.isfile(path):
        return path

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, path),
        os.path.join(script_dir, "downloads", os.path.basename(path)),
        os.path.join(os.getcwd(), "scripts", os.path.basename(path)),
        os.path.join(os.getcwd(), "scripts", "downloads", os.path.basename(path)),
    ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    # Keep the original name for a clear error message.
    return path


def load_csv(path):
    resolved_path = resolve_csv_path(path)

    if not os.path.isfile(resolved_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base = os.path.basename(path)
        raise FileNotFoundError(
            f"CSV not found: '{path}'. "
            f"Try: '{os.path.join('scripts', base)}' or "
            f"'{os.path.join('scripts', 'downloads', base)}'. "
            f"Script directory: '{script_dir}'"
        )

    df = pd.read_csv(resolved_path)

    # Compatibility with current telemetry header format.
    header_map = {
        "t_ms": "timestamp",
        "state": "mode",
        "imu_ax": "acc_x",
        "imu_ay": "acc_y",
        "imu_az": "acc_z",
        "imu_amag": "acc_total",
        "imu_gx": "gyro_x",
        "imu_gy": "gyro_y",
        "imu_gz": "gyro_z",
        "baro_alt": "altitude",
        "baro_temp": "bmp_temp",
        "baro_pressure": "pressure",
        "gps_lat": "lat",
        "gps_lon": "lon",
        "gps_speed": "speed",
    }
    df = df.rename(columns=header_map)

    # Fallback: headerless CSV in legacy column order.
    if "timestamp" not in df.columns:
        df = pd.read_csv(resolved_path, names=COLUMNS)

    missing = [c for c in COLUMNS if c not in df.columns]
    if missing:
        # Tolerate GPS-only CSVs if they contain at least the minimum columns.
        gps_missing = [c for c in GPS_MIN_COLUMNS if c not in df.columns]
        if gps_missing:
            raise ValueError(
                f"CSV has incompatible columns. Missing: {missing}. "
                f"Detected columns: {list(df.columns)}"
            )
        # Fill missing sensor columns with NaN to preserve schema.
        for c in missing:
            df[c] = float("nan")

    # Convert to numeric
    for c in COLUMNS:
        if c not in ["mode"]:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    # Preserve extra columns (e.g. wind_mps) without breaking
    # compatibility with existing modes.
    extra_cols = [c for c in df.columns if c not in COLUMNS]
    df = df[COLUMNS + extra_cols]

    df["time_s"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 1000
    return df


# ----------------------
# MATPLOTLIB PLOTS
# ----------------------
def plot_altitude(df, output):
    plt.figure(figsize=(10, 5))
    plt.plot(df["time_s"], df["altitude"])
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude during flight")
    plt.grid(True)
    plt.savefig(output)
    print(f"Plot saved to {output}")


def plot_acceleration(df, output):
    plt.figure(figsize=(10, 5))
    plt.plot(df["time_s"], df["acc_total"])
    plt.xlabel("Tiempo (s)")
    plt.ylabel("m/s²")
    plt.title("Total acceleration")
    plt.grid(True)
    plt.savefig(output)
    print(f"Plot saved to {output}")


def plot_gyro(df, output):
    plt.figure(figsize=(10, 5))
    plt.plot(df["time_s"], df["gyro_x"], label="gyro_x")
    plt.plot(df["time_s"], df["gyro_y"], label="gyro_y")
    plt.plot(df["time_s"], df["gyro_z"], label="gyro_z")
    plt.legend()
    plt.grid(True)
    plt.savefig(output)
    print(f"Gyro plot saved to {output}")


# ----------------------
# SUMMARY TABLE
# ----------------------
def generate_summary(df, output):
    summary = {
        "duration_s": df["time_s"].max(),
        "altitude_max": df["altitude"].max(),
        "speed_max": df["speed"].max(),
        "acceleration_max": df["acc_total"].max(),
        "temperature_mean": df["imu_temp"].mean()
    }

    summary_df = pd.DataFrame([summary])
    summary_df.to_csv(output, index=False)
    print(f"Summary saved to {output}")


# ----------------------
# GPS MAP
# ----------------------
def generate_map(df, output):
    import folium

    gps = df.dropna(subset=["lat", "lon"])

    if gps.empty:
        print("No valid GPS data")
        return

    m = folium.Map(
        location=[gps.iloc[0]["lat"], gps.iloc[0]["lon"]],
        zoom_start=16
    )

    coords = gps[["lat", "lon"]].values.tolist()
    folium.PolyLine(coords, color="red").add_to(m)

    m.save(output)
    print(f"Map saved to {output}")


def generate_map_3d(df, output):
    import plotly.graph_objects as go

    gps = df.dropna(subset=["lat", "lon"]).copy()
    if gps.empty:
        print("No valid GPS data")
        return

    # Prefer GPS altitude; fall back to barometric.
    gps["z_alt"] = gps["gps_alt"].fillna(gps["altitude"])
    gps = gps.dropna(subset=["z_alt"])
    if gps.empty:
        print("No valid altitude data for 3D map")
        return

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=gps["lon"],
                y=gps["lat"],
                z=gps["z_alt"],
                mode="lines+markers",
                line={"color": gps["z_alt"], "colorscale": "Turbo", "width": 4},
                marker={"size": 2, "color": gps["z_alt"], "colorscale": "Turbo"},
                name="Trajectory",
            )
        ]
    )

    fig.update_layout(
        title="3D Trajectory (lon/lat/altitude)",
        scene={
            "xaxis_title": "Longitude",
            "yaxis_title": "Latitude",
            "zaxis_title": "Altitude (m)",
            "aspectmode": "data",
        },
    )

    fig.write_html(output)
    print(f"3D map saved to {output}")


def generate_map_real(df, output, satellite=False, animation=False):
    import pydeck as pdk
    import folium

    def write_deck_animation_html(gps_df, events, out_path, use_satellite):
        # Data serialised for JS.
        trip_path = gps_df[["lon", "lat", "z_agl"]].values.tolist()
        trip_timestamps = (gps_df["time_s"] * 1000.0).round(1).tolist()
        duration_ms = float(max(trip_timestamps)) if trip_timestamps else 0.0

        point_records = []
        for _, r in gps_df.iterrows():
            point_records.append(
                {
                    "position": [float(r["lon"]), float(r["lat"]), float(r["z_agl"])],
                    "time": float(r["time_s"] * 1000.0),
                    "time_s": float(r["time_s"]),
                    "alt": float(r["z_agl"]),
                    "speed": float(r["speed"]),
                    "g": float(r["g_force"]),
                }
            )

        event_records = []
        for ev in events:
            event_records.append(
                {
                    "name": ev["name"],
                    "position": [ev["lon"], ev["lat"], ev["z_agl"]],
                    "time_s": ev["time_s"],
                    "speed": ev["speed"],
                    "g": ev["g_force"],
                }
            )

        # Map style for MapLibre (no token required).
        style_obj = {
            "version": 8,
            "sources": {
                "base": {
                    "type": "raster",
                    "tiles": [
                        (
                            "https://server.arcgisonline.com/ArcGIS/rest/services/"
                            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
                        )
                        if use_satellite
                        else "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
                    ],
                    "tileSize": 256,
                    "attribution": "Esri" if use_satellite else "OpenStreetMap",
                }
            },
            "layers": [{"id": "base", "type": "raster", "source": "base"}],
        }

        html = f"""<!doctype html>
<html>
<head>
    <meta charset=\"utf-8\" />
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
    <title>Real 3D animated map</title>
    <link href=\"https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.css\" rel=\"stylesheet\" />
    <script src=\"https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.js\"></script>
    <script src=\"https://unpkg.com/deck.gl@9.0.25/dist.min.js\"></script>
    <script src=\"https://unpkg.com/three@0.161.0/build/three.min.js\"></script>
    <style>
        html, body, #map {{ margin: 0; width: 100%; height: 100%; overflow: hidden; background: #101418; }}
        #hud {{
            position: absolute; left: 12px; top: 12px; z-index: 10;
            background: rgba(0,0,0,0.62); color: #fff; padding: 10px 12px;
            border-radius: 8px; font-family: Segoe UI, sans-serif; font-size: 13px;
        }}
        #controls {{
            position: absolute; left: 12px; top: 92px; z-index: 11;
            background: rgba(0,0,0,0.62); color: #fff; padding: 8px;
            border-radius: 8px; font-family: Segoe UI, sans-serif;
            display: flex; gap: 6px;
        }}
        #controls button {{
            border: 1px solid rgba(255,255,255,0.3);
            background: rgba(255,255,255,0.08);
            color: #fff;
            border-radius: 6px;
            padding: 6px 10px;
            cursor: pointer;
            font-size: 12px;
        }}
        #controls button:hover {{
            background: rgba(255,255,255,0.18);
        }}
        #attitude {{
            position: absolute; right: 14px; top: 14px; z-index: 12;
            width: 188px;
            background: rgba(0,0,0,0.58);
            border: 1px solid rgba(255,255,255,0.18);
            border-radius: 10px;
            padding: 8px;
            color: #f2f5f8;
            font-family: Segoe UI, sans-serif;
        }}
        #attitudeTitle {{
            text-align: center;
            font-size: 12px;
            margin-bottom: 4px;
            letter-spacing: 0.2px;
        }}
        #attitude3d {{
            display: block;
            width: 170px;
            height: 170px;
            margin: 0 auto;
            border-radius: 8px;
            background: radial-gradient(circle at 30% 30%, rgba(120,160,220,0.12), rgba(10,14,20,0.55));
            border: 1px solid rgba(255,255,255,0.12);
        }}
        #attitudeMeta {{
            margin-top: 4px;
            text-align: center;
            font-size: 12px;
            line-height: 1.35;
        }}
    </style>
</head>
<body>
    <div id=\"map\"></div>
    <div id=\"hud\">Initialising animation...</div>
    <div id=\"controls\">
      <button id=\"btnStart\">Start</button>
      <button id=\"btnPause\">Pause</button>
      <button id=\"btnStop\">Stop</button>
      <button id=\"btnReset\">Reset</button>
    </div>
        <div id=\"attitude\">
            <div id=\"attitudeTitle\">Attitude 3D</div>
            <div id=\"attitude3d\"></div>
            <div id=\"attitudeMeta\">Hdg 0° | Pitch 90°</div>
        </div>
    <script>
        const TRIP_PATH = {json.dumps(trip_path)};
        const TRIP_TIMES = {json.dumps(trip_timestamps)};
        const POINTS = {json.dumps(point_records)};
        const EVENTS = {json.dumps(event_records)};
        const STYLE = {json.dumps(style_obj)};
        const DURATION_MS = {duration_ms:.1f};

        const center = [TRIP_PATH[0][0], TRIP_PATH[0][1]];
        const hud = document.getElementById('hud');
        const btnStart = document.getElementById('btnStart');
        const btnPause = document.getElementById('btnPause');
        const btnStop = document.getElementById('btnStop');
        const btnReset = document.getElementById('btnReset');
        const attitudePanel = document.getElementById('attitude');
        const attitude3d = document.getElementById('attitude3d');
        const attitudeMeta = document.getElementById('attitudeMeta');

        // Mini 3D attitude viewer (rocket in corner).
        let miniReady = false;
        let miniScene = null;
        let miniCamera = null;
        let miniRenderer = null;
        let ring = null;
        let rocketGroup = null;

        function renderMini3D() {{
            if (!miniReady) {{
                return;
            }}
            miniRenderer.render(miniScene, miniCamera);
        }}

        try {{
            if (!window.THREE) {{
                throw new Error('THREE not available');
            }}

            miniScene = new THREE.Scene();
            miniCamera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);
            miniCamera.position.set(0, 2.2, 5.4);
            miniCamera.lookAt(0, 0, 0);

            miniRenderer = new THREE.WebGLRenderer({{ antialias: true, alpha: true }});
            miniRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
            miniRenderer.setSize(170, 170);
            attitude3d.appendChild(miniRenderer.domElement);

            const amb = new THREE.AmbientLight(0xffffff, 0.72);
            miniScene.add(amb);
            const dir1 = new THREE.DirectionalLight(0xffffff, 0.9);
            dir1.position.set(2, 3, 2);
            miniScene.add(dir1);
            const dir2 = new THREE.DirectionalLight(0x88aaff, 0.35);
            dir2.position.set(-2, -1, 1.5);
            miniScene.add(dir2);

            const ringGeom = new THREE.TorusGeometry(1.65, 0.03, 16, 96);
            const ringMat = new THREE.MeshBasicMaterial({{ color: 0x93a9c0, opacity: 0.45, transparent: true }});
            ring = new THREE.Mesh(ringGeom, ringMat);
            ring.rotation.x = Math.PI / 2;
            miniScene.add(ring);

            rocketGroup = new THREE.Group();
            miniScene.add(rocketGroup);

            // Body (local +Y axis = rocket nose).
            const body = new THREE.Mesh(
                new THREE.CylinderGeometry(0.17, 0.19, 2.15, 18),
                new THREE.MeshStandardMaterial({{ color: 0xe8edf5, metalness: 0.18, roughness: 0.42 }})
            );
            rocketGroup.add(body);

            const nose = new THREE.Mesh(
                new THREE.ConeGeometry(0.18, 0.58, 18),
                new THREE.MeshStandardMaterial({{ color: 0xffffff, metalness: 0.12, roughness: 0.35 }})
            );
            nose.position.y = 1.36;
            rocketGroup.add(nose);

            const finMat = new THREE.MeshStandardMaterial({{ color: 0x9fb5ca, metalness: 0.12, roughness: 0.5 }});
            for (let k = 0; k < 4; k++) {{
                const fin = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.35, 0.45), finMat);
                const a = (k * Math.PI) / 2;
                fin.position.set(Math.cos(a) * 0.2, -0.9, Math.sin(a) * 0.2);
                fin.rotation.y = -a;
                rocketGroup.add(fin);
            }}

            // Simple exhaust flame.
            const flame = new THREE.Mesh(
                new THREE.ConeGeometry(0.09, 0.32, 14),
                new THREE.MeshBasicMaterial({{ color: 0xffa31a, opacity: 0.85, transparent: true }})
            );
            flame.position.y = -1.28;
            flame.rotation.x = Math.PI;
            rocketGroup.add(flame);

            miniReady = true;
            renderMini3D();
        }} catch (err) {{
            console.warn('Failed to initialise mini 3D viewer:', err);
            if (attitudePanel) {{
                attitudePanel.style.display = 'none';
            }}
        }}

        const deckgl = new deck.DeckGL({{
            container: 'map',
            mapStyle: STYLE,
            mapLib: maplibregl,
            initialViewState: {{
                longitude: center[0],
                latitude: center[1],
                zoom: 15.8,
                pitch: 67,
                bearing: 22,
            }},
            controller: true,
            getTooltip: ({{object}}) => object && object.tooltip ? {{html: object.tooltip}} : null,
        }});

        const eventLayer = new deck.ScatterplotLayer({{
            id: 'events',
            data: EVENTS.map(e => ({{
                position: e.position,
                radius: 7,
                color: [255, 230, 120, 240],
                tooltip: `<b>${{e.name}}</b><br/>t=${{e.time_s.toFixed(2)}}s<br/>v=${{e.speed.toFixed(2)}} m/s<br/>g=${{e.g.toFixed(2)}}`
            }})),
            getPosition: d => d.position,
            getRadius: d => d.radius,
            radiusMinPixels: 5,
            radiusMaxPixels: 11,
            getFillColor: d => d.color,
            pickable: true,
        }});

        const basePathLayer = new deck.PathLayer({{
            id: 'path-base',
            data: [{{path: TRIP_PATH}}],
            getPath: d => d.path,
            getColor: [120, 120, 120, 120],
            widthMinPixels: 2,
            getWidth: 3,
            pickable: false,
        }});

        const fmt = (ms) => (ms / 1000).toFixed(2);

        const mPerDegLat = 111320.0;
        const mPerDegLon = 111320.0 * Math.cos(center[1] * Math.PI / 180.0);

        function clamp(v, lo, hi) {{
            return Math.max(lo, Math.min(hi, v));
        }}

        function lerp(a, b, t) {{
            return a + (b - a) * t;
        }}

        function getCurrentPose(currentTime) {{
            if (POINTS.length === 0) {{
                return null;
            }}
            if (POINTS.length === 1 || currentTime <= POINTS[0].time) {{
                return {{ pos: POINTS[0].position, dir: [0, 0, 1] }};
            }}

            let i = 0;
            while (i < POINTS.length - 1 && POINTS[i + 1].time < currentTime) {{
                i += 1;
            }}
            const j = Math.min(i + 1, POINTS.length - 1);
            const p0 = POINTS[i];
            const p1 = POINTS[j];

            const dt = Math.max(1e-6, p1.time - p0.time);
            const alpha = clamp((currentTime - p0.time) / dt, 0, 1);

            const lon = lerp(p0.position[0], p1.position[0], alpha);
            const lat = lerp(p0.position[1], p1.position[1], alpha);
            const z = lerp(p0.position[2], p1.position[2], alpha);

            // Orientation vector in metres (avoids mixing degrees with metres).
            const vxM = (p1.position[0] - p0.position[0]) * mPerDegLon;
            const vyM = (p1.position[1] - p0.position[1]) * mPerDegLat;
            const vzM = (p1.position[2] - p0.position[2]);

            let norm = Math.sqrt(vxM * vxM + vyM * vyM + vzM * vzM);
            let dir = [0, 0, 1];
            if (norm > 1e-6) {{
                dir = [vxM / norm, vyM / norm, vzM / norm];
            }}

            // At launch, force vertical to represent guided ascent.
            if (currentTime <= 2000) {{
                dir = [0, 0, 1];
            }}

            return {{ pos: [lon, lat, z], dir: dir }};
        }}

        function updateAttitude(pose) {{
            if (!pose || !pose.dir) {{
                return;
            }}
            // dir: [east, north, vertical]
            const dx = pose.dir[0];
            const dy = pose.dir[1];
            const dz = clamp(pose.dir[2], -1, 1);

            // Heading: 0°=N, 90°=E
            let headingDeg = (Math.atan2(dx, dy) * 180 / Math.PI + 360) % 360;
            if (!Number.isFinite(headingDeg)) headingDeg = 0;

            // Pitch from horizon (90° = straight up).
            const horiz = Math.sqrt(dx * dx + dy * dy);
            const pitchDeg = Math.atan2(dz, Math.max(1e-6, horiz)) * 180 / Math.PI;

            // Orient 3D model: +Y axis → flight direction vector in ENU world.
            if (miniReady) {{
                const target = new THREE.Vector3(dx, dy, dz).normalize();
                const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), target);
                rocketGroup.quaternion.slerp(q, 0.28);

                // Slow ring rotation gives a heading reference.
                ring.rotation.z = -headingDeg * Math.PI / 180.0;
                renderMini3D();
            }}

            attitudeMeta.textContent = `Hdg ${{headingDeg.toFixed(0)}}° | Pitch ${{pitchDeg.toFixed(0)}}°`;
        }}

        let isRunning = true;
        let playTimeMs = 0;
        let lastFrameTs = null;
        let rafId = null;

        function renderAt(currentTime) {{
            const t = Math.max(0, Math.min(currentTime, DURATION_MS));

            const tripLayer = new deck.TripsLayer({{
                id: 'trip-anim',
                data: [{{path: TRIP_PATH, timestamps: TRIP_TIMES}}],
                getPath: d => d.path,
                getTimestamps: d => d.timestamps,
                getColor: [30, 210, 255],
                opacity: 0.95,
                widthMinPixels: 5,
                rounded: true,
                trailLength: 1500,
                currentTime: t,
                capRounded: true,
                jointRounded: true,
            }});

            const visible = POINTS.filter(p => p.time <= t);
            const pointLayer = new deck.ScatterplotLayer({{
                id: 'points-anim',
                data: visible.map(p => ({{
                    position: p.position,
                    radius: 3,
                    color: [255, 120, 70, 180],
                    tooltip: `t=${{p.time_s.toFixed(2)}}s<br/>alt=${{p.alt.toFixed(1)}}m<br/>v=${{p.speed.toFixed(2)}} m/s<br/>g=${{p.g.toFixed(2)}}`
                }})),
                getPosition: d => d.position,
                getRadius: d => d.radius,
                radiusMinPixels: 2,
                radiusMaxPixels: 5,
                getFillColor: d => d.color,
                pickable: true,
            }});

            const pose = getCurrentPose(t);
            let rocketAxisLayer = null;
            let rocketHeadLayer = null;
            if (pose) {{
                updateAttitude(pose);
                const bodyLenM = 28.0;
                const tipLon = pose.pos[0] + (pose.dir[0] * bodyLenM) / mPerDegLon;
                const tipLat = pose.pos[1] + (pose.dir[1] * bodyLenM) / mPerDegLat;
                const tipZ = pose.pos[2] + pose.dir[2] * bodyLenM;

                rocketAxisLayer = new deck.PathLayer({{
                    id: 'rocket-axis',
                    data: [{{ path: [pose.pos, [tipLon, tipLat, tipZ]] }}],
                    getPath: d => d.path,
                    getColor: [255, 255, 255, 240],
                    widthMinPixels: 4,
                    getWidth: 6,
                    pickable: false,
                }});

                rocketHeadLayer = new deck.ScatterplotLayer({{
                    id: 'rocket-head',
                    data: [{{
                        position: [tipLon, tipLat, tipZ],
                        radius: 8,
                        color: [255, 235, 59, 250],
                        tooltip: `<b>Rocket</b><br/>t=${{fmt(t)}}s`
                    }}],
                    getPosition: d => d.position,
                    getRadius: d => d.radius,
                    radiusMinPixels: 6,
                    radiusMaxPixels: 12,
                    getFillColor: d => d.color,
                    pickable: true,
                }});
            }}

            const layers = [basePathLayer, tripLayer, pointLayer, eventLayer];
            if (rocketAxisLayer) layers.push(rocketAxisLayer);
            if (rocketHeadLayer) layers.push(rocketHeadLayer);
            deckgl.setProps({{ layers: layers }});

            hud.innerHTML =
                `<b>{'Satellite' if use_satellite else 'Map'} 3D animated</b><br/>` +
                `t = ${{fmt(t)}} s / ${{fmt(DURATION_MS)}} s<br/>` +
                `Estado: ${{isRunning ? 'running' : 'paused'}}`;
        }}

        function renderFrame(ts) {{
            if (lastFrameTs === null) lastFrameTs = ts;
            const dt = ts - lastFrameTs;
            lastFrameTs = ts;

            if (isRunning) {{
                playTimeMs += dt;
                if (playTimeMs >= DURATION_MS) {{
                    playTimeMs = DURATION_MS;
                    isRunning = false;
                }}
            }}

            renderAt(playTimeMs);
            rafId = requestAnimationFrame(renderFrame);
        }}

        function ensureLoop() {{
            if (rafId === null) {{
                rafId = requestAnimationFrame(renderFrame);
            }}
        }}

        btnStart.addEventListener('click', () => {{
            if (playTimeMs >= DURATION_MS) {{
                playTimeMs = 0;
            }}
            isRunning = true;
            lastFrameTs = performance.now();
            ensureLoop();
        }});

        btnPause.addEventListener('click', () => {{
            isRunning = false;
            ensureLoop();
        }});

        btnStop.addEventListener('click', () => {{
            isRunning = false;
            playTimeMs = 0;
            renderAt(playTimeMs);
            ensureLoop();
        }});

        btnReset.addEventListener('click', () => {{
            playTimeMs = 0;
            lastFrameTs = performance.now();
            renderAt(playTimeMs);
            ensureLoop();
        }});

        ensureLoop();
    </script>
</body>
</html>
"""

        with open(out_path, "w", encoding="utf-8") as f:
            f.write(html)

    gps = df.dropna(subset=["lat", "lon"]).copy()
    if gps.empty:
        print("No valid GPS data")
        return

    # Prefer GPS altitude; fall back to barometric.
    gps["z_alt"] = gps["gps_alt"].fillna(gps["altitude"])
    gps = gps.dropna(subset=["z_alt"])
    if gps.empty:
        print("No valid altitude data")
        return

    z0 = float(gps["z_alt"].iloc[0])
    gps["z_agl"] = gps["z_alt"] - z0
    gps["acc_total"] = gps["acc_total"].fillna(0.0)
    gps["speed"] = gps["speed"].fillna(0.0)
    gps["g_force"] = gps["acc_total"] / 9.80665
    gps["timestamp_s"] = (gps["timestamp"] / 1000.0).round(2)

    max_alt_idx = gps["z_agl"].idxmax()
    max_g_idx = gps["g_force"].idxmax()
    launch_idx = gps.index[0]
    landing_idx = gps.index[-1]

    recovery_rows = gps[gps["mode"] == "RECOVERY"]
    chute_idx = recovery_rows.index[0] if not recovery_rows.empty else None

    event_rows = [
        {
            "name": "Launch",
            "lon": float(gps.loc[launch_idx, "lon"]),
            "lat": float(gps.loc[launch_idx, "lat"]),
            "z_agl": float(gps.loc[launch_idx, "z_agl"]),
            "time_s": float(gps.loc[launch_idx, "time_s"]),
            "speed": float(gps.loc[launch_idx, "speed"]),
            "g_force": float(gps.loc[launch_idx, "g_force"]),
            "color": [60, 220, 60, 240],
        },
        {
            "name": "Apogee",
            "lon": float(gps.loc[max_alt_idx, "lon"]),
            "lat": float(gps.loc[max_alt_idx, "lat"]),
            "z_agl": float(gps.loc[max_alt_idx, "z_agl"]),
            "time_s": float(gps.loc[max_alt_idx, "time_s"]),
            "speed": float(gps.loc[max_alt_idx, "speed"]),
            "g_force": float(gps.loc[max_alt_idx, "g_force"]),
            "color": [255, 160, 30, 245],
        },
        {
            "name": "Peak g",
            "lon": float(gps.loc[max_g_idx, "lon"]),
            "lat": float(gps.loc[max_g_idx, "lat"]),
            "z_agl": float(gps.loc[max_g_idx, "z_agl"]),
            "time_s": float(gps.loc[max_g_idx, "time_s"]),
            "speed": float(gps.loc[max_g_idx, "speed"]),
            "g_force": float(gps.loc[max_g_idx, "g_force"]),
            "color": [255, 40, 40, 245],
        },
        {
            "name": "Landing",
            "lon": float(gps.loc[landing_idx, "lon"]),
            "lat": float(gps.loc[landing_idx, "lat"]),
            "z_agl": float(gps.loc[landing_idx, "z_agl"]),
            "time_s": float(gps.loc[landing_idx, "time_s"]),
            "speed": float(gps.loc[landing_idx, "speed"]),
            "g_force": float(gps.loc[landing_idx, "g_force"]),
            "color": [255, 255, 255, 245],
        },
    ]

    if chute_idx is not None:
        event_rows.append(
            {
                "name": "Parachute deployment",
                "lon": float(gps.loc[chute_idx, "lon"]),
                "lat": float(gps.loc[chute_idx, "lat"]),
                "z_agl": float(gps.loc[chute_idx, "z_agl"]),
                "time_s": float(gps.loc[chute_idx, "time_s"]),
                "speed": float(gps.loc[chute_idx, "speed"]),
                "g_force": float(gps.loc[chute_idx, "g_force"]),
                "color": [70, 180, 255, 245],
            }
        )

    # Satellite and/or animation mode for the georeferenced 2D map (Folium).
    # Default mode (no satellite, no animation): georeferenced 3D viewer (PyDeck).
    if animation:
        write_deck_animation_html(gps, event_rows, output, satellite)
        print(
            "Flight summary: "
            f"alt_max={gps.loc[max_alt_idx, 'z_agl']:.1f}m, "
            f"g_max={gps.loc[max_g_idx, 'g_force']:.2f}g, "
            f"duration={gps['time_s'].max():.1f}s"
        )
        print(f"Real map saved to {output}")
        return

    if satellite:
        tiles_url = (
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        )
        attr = "Esri World Imagery"

        m = folium.Map(
            location=[float(gps.iloc[0]["lat"]), float(gps.iloc[0]["lon"])],
            zoom_start=16,
            tiles=tiles_url,
            attr=attr,
            control_scale=True,
        )

        coords_2d = gps[["lat", "lon"]].values.tolist()
        folium.PolyLine(coords_2d, color="#d4145a", weight=4, opacity=0.9).add_to(m)

        # Key markers with quick summary.
        for ev in event_rows:
            popup = (
                f"<b>{ev['name']}</b><br>"
                f"t={ev['time_s']:.2f}s<br>"
                f"alt={ev['z_agl']:.1f}m AGL<br>"
                f"spd={ev['speed']:.2f}m/s<br>"
                f"g={ev['g_force']:.2f}"
            )
            folium.CircleMarker(
                location=[ev["lat"], ev["lon"]],
                radius=6,
                color="#ffffff",
                weight=2,
                fill=True,
                fill_color="#ff7f50",
                fill_opacity=0.95,
                popup=popup,
            ).add_to(m)

        title = "Real map (satellite)" if satellite else "Real map"
        title_html = (
            "<h4 style='position: fixed; top: 8px; left: 50px; z-index: 1000; "
            "background: rgba(0,0,0,0.6); color: #fff; padding: 6px 10px; "
            "border-radius: 6px;'>"
            f"{title} | Max alt: {gps.loc[max_alt_idx, 'z_agl']:.1f} m | "
            f"Max g: {gps.loc[max_g_idx, 'g_force']:.2f} g"
            "</h4>"
        )
        m.get_root().html.add_child(folium.Element(title_html))
        m.save(output)

        print(
            "Flight summary: "
            f"alt_max={gps.loc[max_alt_idx, 'z_agl']:.1f}m, "
            f"g_max={gps.loc[max_g_idx, 'g_force']:.2f}g, "
            f"duration={gps['time_s'].max():.1f}s"
        )
        print(f"Real map saved to {output}")
        return

    # Single georeferenced 3D viewer (default mode).
    path_coords = gps[["lon", "lat", "z_agl"]].values.tolist()
    path_data = [{"name": "Trajectory", "path": path_coords}]

    # Colour points by altitude for easy ascent/descent reading.
    pts = gps[["lon", "lat", "z_agl", "time_s", "timestamp_s", "speed", "g_force"]].copy()
    max_agl = max(float(pts["z_agl"].max()), 1.0)

    def point_color(z):
        # Simple blue-to-red RGB gradient, no extra dependencies.
        ratio = min(max(float(z) / max_agl, 0.0), 1.0)
        r = int(40 + 215 * ratio)
        g = int(120 + 80 * (1.0 - ratio))
        b = int(255 - 220 * ratio)
        return [r, g, b, 200]

    pts["color"] = pts["z_agl"].apply(point_color)
    pts["time_s"] = pts["time_s"].round(2)
    pts["z_agl"] = pts["z_agl"].round(2)
    pts["speed"] = pts["speed"].round(2)
    pts["g_force"] = pts["g_force"].round(2)

    path_layer = pdk.Layer(
        "PathLayer",
        data=path_data,
        get_path="path",
        get_width=5,
        width_scale=1,
        width_min_pixels=2,
        get_color=[220, 20, 60, 220],
        pickable=True,
    )

    point_layer = pdk.Layer(
        "ScatterplotLayer",
        data=pts,
        get_position="[lon, lat, z_agl]",
        get_fill_color="color",
        get_radius=1.8,
        radius_min_pixels=2,
        radius_max_pixels=5,
        pickable=True,
    )

    events_layer = pdk.Layer(
        "ScatterplotLayer",
        data=event_rows,
        get_position="[lon, lat, z_agl]",
        get_fill_color="color",
        get_radius=6.0,
        radius_min_pixels=6,
        radius_max_pixels=12,
        pickable=True,
    )

    text_layer = pdk.Layer(
        "TextLayer",
        data=event_rows,
        get_position="[lon, lat, z_agl]",
        get_text="name",
        get_color=[245, 245, 245, 230],
        get_size=14,
        get_alignment_baseline="bottom",
        get_pixel_offset=[0, -16],
        pickable=False,
    )

    view_state = pdk.ViewState(
        latitude=float(gps["lat"].mean()),
        longitude=float(gps["lon"].mean()),
        zoom=16,
        pitch=60,
        bearing=25,
    )

    deck = pdk.Deck(
        map_style="road",
        initial_view_state=view_state,
        layers=[path_layer, point_layer, events_layer, text_layer],
        tooltip={
            "html": (
                "<b>Event:</b> {name}<br/>"
                "<b>t:</b> {time_s}s<br/>"
                "<b>timestamp:</b> {timestamp_s}s<br/>"
                "<b>Alt AGL:</b> {z_agl} m<br/>"
                "<b>Speed:</b> {speed} m/s<br/>"
                "<b>g:</b> {g_force}"
            )
        },
    )

    deck.to_html(output)

    print(
        "Flight summary: "
        f"alt_max={gps.loc[max_alt_idx, 'z_agl']:.1f}m, "
        f"g_max={gps.loc[max_g_idx, 'g_force']:.2f}g, "
        f"duration={gps['time_s'].max():.1f}s"
    )
    print(f"Real map saved to {output}")


# ----------------------
# INTERACTIVE PLOTLY HTML
# ----------------------
def generate_dashboard(df, output):
    import plotly.express as px

    fig = px.line(
        df,
        x="time_s",
        y=["altitude", "speed", "acc_total"],
        title="Interactive dashboard"
    )

    fig.write_html(output)
    print(f"Dashboard saved to {output}")


# ----------------------
# EVENT DETECTOR
# ----------------------
def detect_events(df):
    print("=== DETECTED EVENTS ===")

    launch = df[df["acc_total"] > 15]
    if not launch.empty:
        print(f"Possible launch: t={launch.iloc[0]['time_s']:.2f}s")

    apogee = df["altitude"].idxmax()
    print(f"Apogee: {df.iloc[apogee]['altitude']}m at t={df.iloc[apogee]['time_s']:.2f}s")


# ----------------------
# MAIN
# ----------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="Path to the CSV file")
    parser.add_argument(
        "mode",
        choices=[
            "altitude_plot",
            "acc_plot",
            "gyro_plot",
            "summary",
            "map",
            "map3d",
            "map_real",
            "dashboard",
            "events"
        ],
        help="Output type"
    )
    parser.add_argument("--output", default="output")
    parser.add_argument(
        "--satellite",
        action="store_true",
        help="Use satellite base map (map_real only)",
    )
    parser.add_argument(
        "--animation",
        action="store_true",
        help="Enable animated trajectory playback (map_real only)",
    )

    args = parser.parse_args()

    df = load_csv(args.csv_file)

    if args.mode == "altitude_plot":
        plot_altitude(df, args.output + ".png")

    elif args.mode == "acc_plot":
        plot_acceleration(df, args.output + ".png")

    elif args.mode == "gyro_plot":
        plot_gyro(df, args.output + ".png")

    elif args.mode == "summary":
        generate_summary(df, args.output + ".csv")

    elif args.mode == "map":
        generate_map(df, args.output + ".html")

    elif args.mode == "map3d":
        generate_map_3d(df, args.output + ".html")

    elif args.mode == "map_real":
        generate_map_real(
            df,
            args.output + ".html",
            satellite=args.satellite,
            animation=args.animation,
        )

    elif args.mode == "dashboard":
        generate_dashboard(df, args.output + ".html")

    elif args.mode == "events":
        detect_events(df)


if __name__ == "__main__":
    main()
