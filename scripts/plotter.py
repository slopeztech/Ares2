#!/usr/bin/env python3
"""Serve an interactive dashboard for ARES flight telemetry CSV data."""

from __future__ import annotations

import argparse
import difflib
import sys
from pathlib import Path

try:
    import dash
    import numpy as np
    import pandas as pd
    import plotly.graph_objects as go
    from dash import Input, Output, State, dcc, html
    from plotly.subplots import make_subplots
except ImportError as exc:
    dependency = exc.name or "required plotting package"
    print(
        f"Missing dependency '{dependency}'. Install the plotter dependencies with:\n"
        "  python -m pip install dash numpy pandas plotly",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc

class RocketDataAnalyzer:
    G_ACCEL = 9.80665

    # Fallback header maps if exact names differ
    TIME_KEYS = ["t_ms", "timestamp", "time", "t"]
    BARO_KEYS = ["baro_alt", "altitude", "alt", "height"]
    AX_KEYS = ["accel_x", "ax", "acc_x"]
    AY_KEYS = ["accel_y", "ay", "acc_y"]
    AZ_KEYS = ["accel_z", "az", "acc_z"]

    def __init__(self, csv_path, relative_time=True):
        self.csv_path = Path(csv_path)
        self.relative_time = relative_time
        self.df = None
        self._map = {}

    def _resolve_headers(self, columns):
        """Intelligently maps flexible CSV headers to known data fields."""
        def find_best_match(keys, options):
            for k in keys:
                matches = difflib.get_close_matches(k, options, n=1, cutoff=0.6)
                if matches:
                    return matches[0]
            for opt in options:
                if any(k in opt.lower() for k in keys):
                    return opt
            return None

        self._map["time"] = find_best_match(self.TIME_KEYS, columns)
        self._map["baro"] = find_best_match(self.BARO_KEYS, columns)
        self._map["ax"] = find_best_match(self.AX_KEYS, columns)
        self._map["ay"] = find_best_match(self.AY_KEYS, columns)
        self._map["az"] = find_best_match(self.AZ_KEYS, columns)

        print("\n--- Data Column Mapping ---")
        for k, v in self._map.items():
            print(f"  {k.upper()}: {v if v else 'NOT FOUND'}")
        print("---------------------------\n")

    def load_data(self):
        """Loads CSV, resolves headers, and performs telemetry calculations."""
        if not self.csv_path.exists():
            raise FileNotFoundError(f"Target telemetry file not found: {self.csv_path.resolve()}")

        print(f"Parsing telemetry from: {self.csv_path.resolve()}")
        self.df = pd.read_csv(self.csv_path, skipinitialspace=True)
        self._resolve_headers(self.df.columns)

        # Process Time Domain
        t_col = self._map["time"]
        if not t_col:
            raise KeyError("Could not identify a valid time column in CSV.")
        
        # Check if time is already in seconds or ms
        if self.df[t_col].max() > 10000: # Presume ms
            conversion = 1000.0
        else:
            conversion = 1.0

        if self.relative_time:
            self.df["t_sec"] = (self.df[t_col] - self.df[t_col].iloc[0]) / conversion
        else:
            self.df["t_sec"] = self.df[t_col] / conversion

        # Process IMU / G-Forces
        has_imu = all([self._map["ax"], self._map["ay"], self._map["az"]])
        if has_imu:
            # Detect if already in Gs or raw m/s^2
            raw_max = max(self.df[self._map["ax"]].abs().max(), self.df[self._map["ay"]].abs().max())
            divisor = self.G_ACCEL if raw_max > 3.0 else 1.0

            self.df["accel_x_g"] = self.df[self._map["ax"]] / divisor
            self.df["accel_y_g"] = self.df[self._map["ay"]] / divisor
            self.df["accel_z_g"] = self.df[self._map["az"]] / divisor

            self.df["accel_total_g"] = np.sqrt(
                self.df["accel_x_g"]**2 + self.df["accel_y_g"]**2 + self.df["accel_z_g"]**2
            )
            self.df["accel_total_g_smooth"] = self.df["accel_total_g"].rolling(window=5, center=True).mean().fillna(self.df["accel_total_g"])

    def build_dashboard(self):
        """Launches a localized interactive Web Dashboard containing all display modes."""
        if self.df is None:
            self.load_data()

        app = dash.Dash(__name__, title="ARES Flight Data Analysis System")

        # Configurations
        has_baro = self._map["baro"] is not None
        has_imu = "accel_total_g" in self.df.columns

        # Dashboard layout using standard dark-mode styling variables
        app.layout = html.Div(style={"backgroundColor": "#1e1e24", "color": "#f1f1f1", "fontFamily": "sans-serif", "padding": "20px"}, children=[
            html.H1("ARES Flight Data Analysis System", style={"textAlign": "center", "color": "#00adb5"}),
            html.P(f"Source file: {self.csv_path.name}", style={"textAlign": "center", "color": "#eee"}),
            
            html.Div(style={"display": "flex", "justifyContent": "center", "gap": "20px", "marginBottom": "20px"}, children=[
                html.Div([
                    html.Label("Visualization Mode: ", style={"fontWeight": "bold"}),
                    dcc.Dropdown(
                        id="view-mode-dropdown",
                        options=[
                            {"label": "Combined Telemetry (Baro + IMU)", "value": "both", "disabled": not (has_baro and has_imu)},
                            {"label": "Barometric Altitude Only", "value": "baro", "disabled": not has_baro},
                            {"label": "IMU Acceleration Forces Only", "value": "imu", "disabled": not has_imu},
                        ],
                        value="both" if (has_baro and has_imu) else ("baro" if has_baro else "imu"),
                        style={"width": "300px", "color": "#000"}
                    )
                ]),
                html.Div([
                    html.Label("Playback Controls (Video Mode): ", style={"fontWeight": "bold", "display": "block"}),
                    html.Button("Play", id="play-btn", n_clicks=0, style={"marginRight": "5px"}),
                    html.Button("Pause", id="pause-btn", n_clicks=0, style={"marginRight": "5px"}),
                    html.Button("Reset", id="reset-btn", n_clicks=0),
                ])
            ]),

            dcc.Interval(id="playback-timer", interval=150, n_intervals=0, disabled=True),
            dcc.Slider(
                id="time-slider",
                min=0,
                max=len(self.df) - 1,
                value=len(self.df) - 1,
                marks={i: f"{self.df['t_sec'].iloc[i]:.1f}s" for i in range(0, len(self.df), max(1, len(self.df)//10))},
                step=1
            ),

            html.Div(style={"display": "flex", "flexWrap": "wrap", "marginTop": "20px"}, children=[
                html.Div(dcc.Graph(id="telemetry-plot"), style={"flex": "2", "minWidth": "600px"}),
                html.Div(dcc.Graph(id="3d-imu-plot"), style={"flex": "1", "minWidth": "400px", "display": "block" if has_imu else "none"}),
            ])
        ])

        # Playback logic callback
        @app.callback(
            Output("playback-timer", "disabled"),
            Output("playback-timer", "n_intervals"),
            Input("play-btn", "n_clicks"),
            Input("pause-btn", "n_clicks"),
            Input("reset-btn", "n_clicks"),
            State("playback-timer", "n_intervals"),
            prevent_initial_call=True
        )
        def control_playback(play, pause, reset, current_interval):
            ctx = dash.callback_context
            if not ctx.triggered:
                return True, current_interval
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]

            if button_id == "play-btn":
                return False, current_interval
            elif button_id == "pause-btn":
                return True, current_interval
            elif button_id == "reset-btn":
                return True, 0
            return True, current_interval

        # Link Playback Interval Updates to Timeline Slider position
        @app.callback(
            Output("time-slider", "value"),
            Input("playback-timer", "n_intervals"),
            State("time-slider", "value"),
            prevent_initial_call=True
        )
        def step_time(n_intervals, current_val):
            if current_val >= len(self.df) - 1:
                return 0
            return current_val + max(1, len(self.df) // 100) # Advances ~1% of timeline per frame

        # Master Plot Renderer
        @app.callback(
            Output("telemetry-plot", "figure"),
            Output("3d-imu-plot", "figure"),
            Input("view-mode-dropdown", "value"),
            Input("time-slider", "value")
        )
        def update_charts(mode, upper_idx):
            sliced_df = self.df.iloc[:upper_idx + 1]
            t_data = sliced_df["t_sec"]

            # Initialize Figures
            if mode == "both":
                fig = make_subplots(rows=2, cols=1, sharedx=True, vertical_spacing=0.08)
                show_b, show_i = True, True
            elif mode == "baro":
                fig = make_subplots(rows=1, cols=1)
                show_b, show_i = True, False
            else:
                fig = make_subplots(rows=1, cols=1)
                show_b, show_i = False, True

            # 1. Baro Plotting
            if show_b:
                r = 1
                fig.add_trace(go.Scatter(x=t_data, y=sliced_df[self._map["baro"]], name="Altitude", line=dict(color="#00adb5", width=2.5)), row=r, col=1)
                fig.update_yaxes(title_text="Altitude (m)", row=r, col=1, gridcolor="#393e46")
                if mode == "both": 
                    fig.update_subplots(rows=1, cols=1, row_title="Baro")

            # 2. IMU Plotting
            if show_i:
                r = 2 if mode == "both" else 1
                fig.add_trace(go.Scatter(x=t_data, y=sliced_df["accel_x_g"], name="Accel X", line=dict(color="#ff2e63", width=1), opacity=0.5), row=r, col=1)
                fig.add_trace(go.Scatter(x=t_data, y=sliced_df["accel_y_g"], name="Accel Y", line=dict(color="#25d366", width=1), opacity=0.5), row=r, col=1)
                fig.add_trace(go.Scatter(x=t_data, y=sliced_df["accel_z_g"], name="Accel Z", line=dict(color="#f8b500", width=1), opacity=0.5), row=r, col=1)
                fig.add_trace(go.Scatter(x=t_data, y=sliced_df["accel_total_g_smooth"], name="Total Magnitude", line=dict(color="#ffffff", width=2.5)), row=r, col=1)
                
                # Threshold highlight (>3G)
                high_g = sliced_df[sliced_df["accel_total_g_smooth"] > 3.0]
                if not high_g.empty:
                    fig.add_trace(go.Scatter(x=high_g["t_sec"], y=high_g["accel_total_g_smooth"], mode="markers", name=">3 G Event", marker=dict(color="red", size=4)), row=r, col=1)

                fig.update_yaxes(title_text="Acceleration (G)", row=r, col=1, gridcolor="#393e46")
                if mode == "both": 
                    fig.update_subplots(rows=2, cols=1, row_title="IMU")

            fig.update_layout(
                title="Flight Telemetry Timeline",
                template="plotly_dark",
                paper_bgcolor="#1e1e24",
                plot_bgcolor="#222831",
                xaxis=dict(title="Relative Time (s)" if self.relative_time else "Time (s)", gridcolor="#393e46"),
                hovermode="x unified",
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
            )

            # 3. Spatial 3D IMU G-Force Vector Map
            fig_3d = go.Figure()
            if has_imu:
                fig_3d.add_trace(go.Scatter3d(
                    x=sliced_df["accel_x_g"],
                    y=sliced_df["accel_y_g"],
                    z=sliced_df["accel_z_g"],
                    mode="lines+markers",
                    marker=dict(
                        size=3,
                        color=sliced_df["accel_total_g_smooth"],
                        colorscale="Inferno",
                        showscale=True,
                        colorbar=dict(title="G Force", thickness=15, len=0.6)
                    ),
                    line=dict(color="#ffffff", width=1.5),
                    name="G-Vector Space"
                ))
            
            fig_3d.update_layout(
                title="3D G-Force Vector Mapping",
                template="plotly_dark",
                paper_bgcolor="#1e1e24",
                scene=dict(
                    xaxis=dict(title="X (G)", backgroundcolor="#222831", gridcolor="#393e46"),
                    yaxis=dict(title="Y (G)", backgroundcolor="#222831", gridcolor="#393e46"),
                    zaxis=dict(title="Z (G)", backgroundcolor="#222831", gridcolor="#393e46"),
                ),
                margin=dict(l=0, r=0, b=0, t=40)
            )

            return fig, fig_3d

        print("\n--> Starting analysis server. Open your web browser and navigate to: http://127.0.0.1:8050/\n")
        app.run(debug=True, use_reloader=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ARES Flight Data Analysis System Engine.")
    parser.add_argument("csv_file", type=str, nargs="?", default="mission_datalog_ams2.csv", help="Path to telemetry log file.")
    args = parser.parse_args()

    # Automatic absolute pathing relative to execution script if plain string provided
    target_path = Path(args.csv_file)
    if not target_path.is_absolute():
        target_path = Path(__file__).resolve().parent / target_path

    plotter = RocketDataAnalyzer(csv_path=target_path, relative_time=True)
    plotter.build_dashboard()