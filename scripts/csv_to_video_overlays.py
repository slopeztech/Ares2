#!/usr/bin/env python3
"""
Genera vídeos y secuencias PNG animadas a partir de datos temporales de un CSV.

Características principales
---------------------------
- Modos: graph, bar, graph-bar y hud.
- Temas normales y overlays transparentes/translúcidos.
- Transparencia real en PNG, WebM VP9 y MOV ProRes 4444.
- Selección de columnas, unidades, resolución, FPS, velocidad y colores.
- La línea reproducida utiliza todos los puntos originales alcanzados, evitando
  que la trayectoria pendiente sobresalga como una falsa sombra.

Ejemplos
--------
Vídeo MP4 normal:
    python csv_to_video_overlays.py log.csv -o replay.mp4 --theme spacex

Overlay blanco transparente (canal alfa):
    python csv_to_video_overlays.py log.csv -o overlay.webm \
        --theme transparent-white --mode graph-bar --no-header

Barra vertical translúcida negra:
    python csv_to_video_overlays.py log.csv -o altitude_bar.mov \
        --theme translucent-black --mode bar --bar-min 0 --bar-max 1000
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import matplotlib

# Permite ejecutar el script sin entorno gráfico.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle, Rectangle
import numpy as np
import pandas as pd


Color = str | tuple[float, float, float, float]


@dataclass(frozen=True)
class Theme:
    background: Color
    axes_background: Color
    text: Color
    muted_text: Color
    line: Color
    cursor: Color
    track: Color
    grid: Color
    spine: Color
    panel: Color
    panel_border: Color
    marker_face: Color


THEMES: dict[str, Theme] = {
    "spacex": Theme(
        background="#0b0e13",
        axes_background="#0b0e13",
        text="#e6edf3",
        muted_text="#8b949e",
        line="#4cc9f0",
        cursor="#ff4d6d",
        track="#30363d",
        grid="#21262d",
        spine="#30363d",
        panel="#10151dcc",
        panel_border="#30363d",
        marker_face="#ffffff",
    ),
    "neon": Theme(
        background="#090014",
        axes_background="#090014",
        text="#f6efff",
        muted_text="#b8a5c9",
        line="#00f5d4",
        cursor="#ff4ecd",
        track="#39284f",
        grid="#2a173d",
        spine="#65437f",
        panel="#160c24e6",
        panel_border="#65437f",
        marker_face="#ffffff",
    ),
    "light": Theme(
        background="#f5f7fa",
        axes_background="#ffffff",
        text="#17202a",
        muted_text="#667085",
        line="#1769aa",
        cursor="#d7263d",
        track="#cbd5e1",
        grid="#dfe5ec",
        spine="#aab4c0",
        panel="#fffffff2",
        panel_border="#cbd5e1",
        marker_face="#ffffff",
    ),
    "minimal": Theme(
        background="#ffffff",
        axes_background="#ffffff",
        text="#202124",
        muted_text="#6b7280",
        line="#111827",
        cursor="#ef4444",
        track="#e5e7eb",
        grid="#eeeeee",
        spine="#d1d5db",
        panel="#f8fafcf2",
        panel_border="#d1d5db",
        marker_face="#ffffff",
    ),
    # Fondo completamente transparente y elementos blancos. Adecuado para
    # superponer sobre vídeos oscuros.
    "transparent-white": Theme(
        background="#00000000",
        axes_background="#00000000",
        text="#ffffffff",
        muted_text="#ffffffb3",
        line="#ffffffff",
        cursor="#ffffffff",
        track="#ffffff35",
        grid="#ffffff28",
        spine="#ffffff70",
        panel="#00000000",
        panel_border="#ffffff65",
        marker_face="#ffffffff",
    ),
    # Fondo completamente transparente y elementos negros. Adecuado para
    # superponer sobre vídeos claros.
    "transparent-black": Theme(
        background="#ffffff00",
        axes_background="#ffffff00",
        text="#000000ff",
        muted_text="#000000a8",
        line="#000000ff",
        cursor="#000000ff",
        track="#00000032",
        grid="#00000024",
        spine="#00000065",
        panel="#ffffff00",
        panel_border="#00000060",
        marker_face="#ffffffff",
    ),
    # Tarjetas blancas semitransparentes y trazado oscuro.
    "translucent-white": Theme(
        background="#ffffff00",
        axes_background="#ffffffe0",
        text="#111827ff",
        muted_text="#374151c7",
        line="#111827ff",
        cursor="#111827e6",
        track="#1118272e",
        grid="#11182724",
        spine="#11182755",
        panel="#ffffffe8",
        panel_border="#11182752",
        marker_face="#ffffffff",
    ),
    # Tarjetas negras semitransparentes y trazado blanco.
    "translucent-black": Theme(
        background="#00000000",
        axes_background="#05070ab8",
        text="#ffffffff",
        muted_text="#ffffffb8",
        line="#ffffffff",
        cursor="#ffffffff",
        track="#ffffff35",
        grid="#ffffff24",
        spine="#ffffff55",
        panel="#05070ac7",
        panel_border="#ffffff50",
        marker_face="#ffffffff",
    ),
}

MODES = ("graph", "bar", "graph-bar", "hud")
TRANSPARENT_VIDEO_EXTENSIONS = {".webm", ".mov"}


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("debe ser un entero mayor que cero")
    return number


def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("debe ser un número mayor que cero")
    return number


def opacity_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or not 0.0 <= number <= 1.0:
        raise argparse.ArgumentTypeError("debe estar entre 0 y 1")
    return number


def parse_delimiter(value: str) -> str | None:
    aliases = {
        "auto": None,
        "comma": ",",
        "coma": ",",
        "semicolon": ";",
        "puntoycoma": ";",
        "tab": "\t",
        "tabulador": "\t",
        "pipe": "|",
    }
    return aliases.get(value.lower(), value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Genera un vídeo, overlay transparente o secuencia PNG animada "
            "a partir de una serie temporal almacenada en CSV."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("csv_file", type=Path, help="Archivo CSV de entrada")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("replay.mp4"),
        help="Vídeo de salida: .mp4, .webm o .mov",
    )

    data = parser.add_argument_group("Datos del CSV")
    data.add_argument("--time-column", default="t_ms", help="Columna de tiempo")
    data.add_argument("--value-column", default="baro_alt", help="Columna numérica representada")
    data.add_argument("--state-column", default="state", help="Columna de estado")
    data.add_argument(
        "--time-unit",
        choices=("s", "ms", "us", "ns"),
        default="ms",
        help="Unidad de la columna temporal",
    )
    data.add_argument(
        "--delimiter",
        default="auto",
        help="Separador: auto, comma, semicolon, tab, pipe o un carácter literal",
    )
    data.add_argument("--encoding", default="utf-8-sig", help="Codificación del CSV")

    video = parser.add_argument_group("Vídeo y renderizado")
    video.add_argument("--fps", type=positive_float, default=30.0, help="Fotogramas por segundo")
    video.add_argument("--speed", type=positive_float, default=1.0, help="Velocidad: 2 = doble velocidad")
    video.add_argument("--width", type=positive_int, default=1280, help="Anchura en píxeles")
    video.add_argument("--height", type=positive_int, default=720, help="Altura en píxeles")
    video.add_argument("--dpi", type=positive_int, default=120, help="DPI de renderizado")
    video.add_argument(
        "--codec",
        default="auto",
        help="Códec de FFmpeg; auto escoge uno compatible con el formato y el alfa",
    )
    video.add_argument("--preset", default="medium", help="Preset para H.264")
    video.add_argument("--crf", type=int, default=18, help="Calidad: menor = más calidad")
    video.add_argument("--ffmpeg", default="ffmpeg", help="Ejecutable o ruta de FFmpeg")
    video.add_argument("--frames-dir", type=Path, help="Carpeta para los PNG")
    video.add_argument("--keep-frames", action="store_true", help="Conservar los PNG")
    video.add_argument("--frames-only", action="store_true", help="Generar solo PNG, sin vídeo")
    video.add_argument("--max-frames", type=positive_int, default=100_000, help="Límite de seguridad")

    design = parser.add_argument_group("Diseño")
    design.add_argument("--mode", choices=MODES, default="graph", help="Tipo de visualización")
    design.add_argument("--theme", choices=tuple(THEMES), default="spacex", help="Tema visual")
    design.add_argument("--title", default="ARES FLIGHT REPLAY", help="Título principal")
    design.add_argument("--subtitle", default="BAROMETRIC TELEMETRY", help="Subtítulo")
    design.add_argument("--no-header", action="store_true", help="Ocultar título y subtítulo")
    design.add_argument("--x-label", default="TIME (s)", help="Etiqueta del eje X")
    design.add_argument("--y-label", default="ALTITUDE (m)", help="Etiqueta del eje Y")
    design.add_argument("--value-name", default="ALTITUDE", help="Nombre de la magnitud")
    design.add_argument("--value-unit", default="m", help="Unidad de la magnitud")
    design.add_argument("--decimals", type=int, default=2, help="Decimales mostrados")
    design.add_argument("--line-width", type=positive_float, default=3.0, help="Grosor de línea")
    design.add_argument("--margin-ratio", type=float, default=0.15, help="Margen vertical proporcional")
    design.add_argument("--minimum-margin", type=positive_float, default=2.0, help="Margen vertical mínimo")
    design.add_argument("--no-panel", action="store_true", help="Quitar el fondo/borde de paneles informativos")
    design.add_argument("--no-grid", action="store_true", help="Ocultar cuadrícula")
    design.add_argument("--hide-full-track", action="store_true", help="Ocultar el recorrido pendiente")

    bar = parser.add_argument_group("Barra vertical")
    bar.add_argument("--bar-min", type=float, help="Valor inferior fijo de la barra")
    bar.add_argument("--bar-max", type=float, help="Valor superior fijo de la barra")
    bar.add_argument("--bar-ticks", type=positive_int, default=5, help="Número de marcas de escala")
    bar.add_argument(
        "--bar-clamp",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Limitar visualmente la barra al mínimo y máximo",
    )

    opacity = parser.add_argument_group("Transparencia")
    opacity.add_argument(
        "--canvas-opacity",
        type=opacity_float,
        help="Opacidad del lienzo completo: 0 transparente, 1 opaco",
    )
    opacity.add_argument(
        "--axes-opacity",
        type=opacity_float,
        help="Opacidad del fondo de la gráfica/barra",
    )
    opacity.add_argument(
        "--panel-opacity",
        type=opacity_float,
        help="Opacidad de paneles informativos",
    )

    colors = parser.add_argument_group("Colores personalizados")
    colors.add_argument("--background")
    colors.add_argument("--axes-background")
    colors.add_argument("--text-color")
    colors.add_argument("--muted-text-color")
    colors.add_argument("--line-color")
    colors.add_argument("--cursor-color")
    colors.add_argument("--track-color")
    colors.add_argument("--grid-color")
    colors.add_argument("--panel-color")
    colors.add_argument("--panel-border-color")
    colors.add_argument("--marker-face-color")

    return parser


def validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if not args.csv_file.is_file():
        parser.error(f"no existe el archivo CSV: {args.csv_file}")
    if args.width % 2 or args.height % 2:
        parser.error("--width y --height deben ser pares para los formatos de vídeo habituales")
    if not 0 <= args.crf <= 51:
        parser.error("--crf debe estar entre 0 y 51")
    if args.decimals < 0 or args.decimals > 10:
        parser.error("--decimals debe estar entre 0 y 10")
    if args.margin_ratio < 0 or not math.isfinite(args.margin_ratio):
        parser.error("--margin-ratio debe ser finito y mayor o igual que cero")
    if args.bar_min is not None and not math.isfinite(args.bar_min):
        parser.error("--bar-min debe ser finito")
    if args.bar_max is not None and not math.isfinite(args.bar_max):
        parser.error("--bar-max debe ser finito")
    if args.bar_min is not None and args.bar_max is not None and args.bar_min >= args.bar_max:
        parser.error("--bar-min debe ser menor que --bar-max")


def load_data(args: argparse.Namespace) -> tuple[pd.DataFrame, np.ndarray, np.ndarray]:
    delimiter = parse_delimiter(args.delimiter)
    read_kwargs: dict[str, Any] = {
        "filepath_or_buffer": args.csv_file,
        "encoding": args.encoding,
    }
    if delimiter is None:
        read_kwargs.update({"sep": None, "engine": "python"})
    else:
        read_kwargs["sep"] = delimiter

    try:
        df = pd.read_csv(**read_kwargs)
    except UnicodeDecodeError as exc:
        raise RuntimeError(
            f"No se pudo leer el CSV con {args.encoding!r}. Prueba --encoding latin-1."
        ) from exc
    except Exception as exc:
        raise RuntimeError(f"No se pudo leer el CSV: {exc}") from exc

    df.columns = [str(column).strip() for column in df.columns]
    required = [args.time_column, args.value_column]
    missing = [column for column in required if column not in df.columns]
    if missing:
        available = ", ".join(df.columns) or "ninguna"
        raise RuntimeError(
            f"Faltan columnas obligatorias: {', '.join(missing)}. "
            f"Columnas disponibles: {available}."
        )

    df[args.time_column] = pd.to_numeric(df[args.time_column], errors="coerce")
    df[args.value_column] = pd.to_numeric(df[args.value_column], errors="coerce")
    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.dropna(subset=required).copy()

    if args.state_column in df.columns:
        df[args.state_column] = df[args.state_column].fillna("N/A").astype(str)
    else:
        print(
            f"Aviso: no existe {args.state_column!r}; se mostrará N/A.",
            file=sys.stderr,
        )
        df[args.state_column] = "N/A"

    # np.interp necesita tiempos estrictamente crecientes.
    df = df.sort_values(args.time_column, kind="stable")
    df = df.drop_duplicates(subset=args.time_column, keep="last").reset_index(drop=True)

    if len(df) < 2:
        raise RuntimeError("El CSV necesita al menos dos muestras numéricas con tiempos distintos.")

    divisors = {"s": 1.0, "ms": 1_000.0, "us": 1_000_000.0, "ns": 1_000_000_000.0}
    raw_times = df[args.time_column].to_numpy(dtype=float)
    times = (raw_times - raw_times[0]) / divisors[args.time_unit]
    values = df[args.value_column].to_numpy(dtype=float)

    duration = float(times[-1])
    if not math.isfinite(duration) or duration <= 0:
        raise RuntimeError("La duración calculada debe ser mayor que cero.")

    return df, times, values


def set_color_alpha(color: Color, alpha: float | None) -> Color:
    if alpha is None:
        return color
    red, green, blue, _ = to_rgba(color)
    return red, green, blue, alpha


def resolve_theme(args: argparse.Namespace) -> Theme:
    theme = THEMES[args.theme]
    overrides = {
        "background": args.background,
        "axes_background": args.axes_background,
        "text": args.text_color,
        "muted_text": args.muted_text_color,
        "line": args.line_color,
        "cursor": args.cursor_color,
        "track": args.track_color,
        "grid": args.grid_color,
        "panel": args.panel_color,
        "panel_border": args.panel_border_color,
        "marker_face": args.marker_face_color,
    }
    theme = replace(theme, **{key: value for key, value in overrides.items() if value is not None})
    return replace(
        theme,
        background=set_color_alpha(theme.background, args.canvas_opacity),
        axes_background=set_color_alpha(theme.axes_background, args.axes_opacity),
        panel=set_color_alpha(theme.panel, args.panel_opacity),
    )


def canvas_has_alpha(theme: Theme) -> bool:
    return to_rgba(theme.background)[3] < 0.999


def build_video_times(duration: float, fps: float, speed: float, max_frames: int) -> np.ndarray:
    playback_duration = duration / speed
    frame_count = max(2, int(math.ceil(playback_duration * fps)) + 1)
    if frame_count > max_frames:
        raise RuntimeError(
            f"Se generarían {frame_count:,} frames, por encima de {max_frames:,}. "
            "Aumenta --speed, baja --fps o eleva --max-frames conscientemente."
        )
    return np.linspace(0.0, duration, frame_count, dtype=float)


def clean_old_frames(frames_dir: Path) -> None:
    frames_dir.mkdir(parents=True, exist_ok=True)
    for old_frame in frames_dir.glob("frame_*.png"):
        old_frame.unlink()


def calculate_ranges(
    args: argparse.Namespace,
    values: np.ndarray,
) -> tuple[float, float, float, float]:
    value_min = float(np.min(values))
    value_max = float(np.max(values))
    value_range = value_max - value_min

    margin = max(value_range * args.margin_ratio, args.minimum_margin)
    if value_range == 0:
        margin = max(abs(value_min) * 0.05, args.minimum_margin)

    graph_min = value_min - margin
    graph_max = value_max + margin

    bar_min = graph_min if args.bar_min is None else float(args.bar_min)
    bar_max = graph_max if args.bar_max is None else float(args.bar_max)

    if bar_min >= bar_max:
        raise RuntimeError("La escala final de la barra no es válida: el mínimo debe ser menor al máximo.")

    return graph_min, graph_max, bar_min, bar_max


def style_graph_axis(
    ax: plt.Axes,
    args: argparse.Namespace,
    theme: Theme,
    duration: float,
    ymin: float,
    ymax: float,
) -> None:
    ax.set_facecolor(theme.axes_background)
    ax.set_xlim(0.0, duration)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel(args.x_label, color=theme.text)
    ax.set_ylabel(args.y_label, color=theme.text)
    ax.tick_params(colors=theme.text)

    for spine in ax.spines.values():
        spine.set_color(theme.spine)

    if not args.no_grid:
        ax.grid(True, color=theme.grid, alpha=1.0, linewidth=0.6)


def style_info_axis(ax: plt.Axes, args: argparse.Namespace, theme: Theme) -> None:
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    if args.no_panel:
        ax.set_facecolor((0, 0, 0, 0))
        for spine in ax.spines.values():
            spine.set_visible(False)
    else:
        ax.set_facecolor(theme.panel)
        for spine in ax.spines.values():
            spine.set_color(theme.panel_border)
            spine.set_linewidth(1.0)


def format_number(value: float, decimals: int) -> str:
    return f"{value:,.{decimals}f}"


def create_graph_artists(
    ax: plt.Axes,
    args: argparse.Namespace,
    theme: Theme,
    initial_time: float,
    initial_value: float,
) -> dict[str, Any]:
    future_line = None
    if not args.hide_full_track:
        (future_line,) = ax.plot(
            [],
            [],
            color=theme.track,
            linewidth=max(1.0, args.line_width * 0.68),
            alpha=1.0,
            zorder=1,
        )

    (travelled_line,) = ax.plot(
        [],
        [],
        color=theme.line,
        linewidth=args.line_width,
        zorder=5,
    )
    cursor_line = ax.axvline(
        initial_time,
        color=theme.cursor,
        linewidth=max(1.0, args.line_width * 0.38),
        alpha=0.9,
        zorder=6,
    )
    marker = ax.scatter(
        [initial_time],
        [initial_value],
        s=120,
        color=theme.marker_face,
        edgecolor=theme.line,
        linewidth=2,
        zorder=20,
    )

    return {
        "future_line": future_line,
        "travelled_line": travelled_line,
        "cursor_line": cursor_line,
        "marker": marker,
    }


def update_graph_artists(
    artists: dict[str, Any],
    times: np.ndarray,
    values: np.ndarray,
    current_time: float,
    current_value: float,
) -> None:
    reached = int(np.searchsorted(times, current_time, side="right"))

    if reached > 0 and np.isclose(times[reached - 1], current_time, rtol=0.0, atol=1e-12):
        travelled_x = times[:reached]
        travelled_y = values[:reached]
    else:
        travelled_x = np.concatenate((times[:reached], [current_time]))
        travelled_y = np.concatenate((values[:reached], [current_value]))

    artists["travelled_line"].set_data(travelled_x, travelled_y)

    future_line = artists["future_line"]
    if future_line is not None:
        future_start = int(np.searchsorted(times, current_time, side="right"))
        future_x = np.concatenate(([current_time], times[future_start:]))
        future_y = np.concatenate(([current_value], values[future_start:]))
        future_line.set_data(future_x, future_y)

    artists["cursor_line"].set_xdata([current_time, current_time])
    artists["marker"].set_offsets(np.array([[current_time, current_value]]))


def create_bar_artists(
    ax: plt.Axes,
    args: argparse.Namespace,
    theme: Theme,
    bar_min: float,
    bar_max: float,
    compact: bool,
) -> dict[str, Any]:
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_xticks([])
    ax.set_yticks([])

    if args.no_panel:
        ax.set_facecolor((0, 0, 0, 0))
        for spine in ax.spines.values():
            spine.set_visible(False)
    else:
        ax.set_facecolor(theme.axes_background)
        for spine in ax.spines.values():
            spine.set_color(theme.spine)

    track_x = 0.36 if compact else 0.40
    track_width = 0.24 if compact else 0.20
    track_bottom = 0.12
    track_height = 0.76

    track = Rectangle(
        (track_x, track_bottom),
        track_width,
        track_height,
        transform=ax.transAxes,
        facecolor=theme.track,
        edgecolor=theme.spine,
        linewidth=1.2,
        zorder=1,
    )
    fill = Rectangle(
        (track_x, track_bottom),
        track_width,
        0,
        transform=ax.transAxes,
        facecolor=theme.line,
        edgecolor="none",
        zorder=3,
    )
    ax.add_patch(track)
    ax.add_patch(fill)

    (marker_line,) = ax.plot(
        [track_x - 0.07, track_x + track_width + 0.07],
        [track_bottom, track_bottom],
        transform=ax.transAxes,
        color=theme.cursor,
        linewidth=2.0,
        zorder=5,
    )
    marker_dot = Circle(
        (track_x + track_width / 2, track_bottom),
        radius=0.027 if compact else 0.024,
        transform=ax.transAxes,
        facecolor=theme.marker_face,
        edgecolor=theme.line,
        linewidth=2,
        zorder=6,
    )
    ax.add_patch(marker_dot)

    # Escala fija de referencia.
    for tick_index in range(args.bar_ticks + 1):
        ratio = tick_index / args.bar_ticks
        y = track_bottom + ratio * track_height
        tick_value = bar_min + ratio * (bar_max - bar_min)
        ax.plot(
            [track_x + track_width + 0.02, track_x + track_width + 0.07],
            [y, y],
            transform=ax.transAxes,
            color=theme.spine,
            linewidth=1,
            zorder=2,
        )
        if not compact or tick_index in (0, args.bar_ticks):
            ax.text(
                track_x + track_width + 0.09,
                y,
                format_number(tick_value, args.decimals),
                transform=ax.transAxes,
                va="center",
                ha="left",
                fontsize=8 if compact else 9,
                color=theme.muted_text,
            )

    value_text = ax.text(
        0.5,
        0.96,
        "",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=12 if compact else 16,
        weight="bold",
        color=theme.text,
    )
    name_text = ax.text(
        0.5,
        0.035,
        args.value_name,
        transform=ax.transAxes,
        ha="center",
        va="bottom",
        fontsize=9 if compact else 11,
        color=theme.muted_text,
    )

    return {
        "fill": fill,
        "marker_line": marker_line,
        "marker_dot": marker_dot,
        "value_text": value_text,
        "name_text": name_text,
        "track_bottom": track_bottom,
        "track_height": track_height,
        "bar_min": bar_min,
        "bar_max": bar_max,
    }


def update_bar_artists(
    artists: dict[str, Any],
    args: argparse.Namespace,
    current_value: float,
) -> None:
    bar_min = float(artists["bar_min"])
    bar_max = float(artists["bar_max"])
    ratio = (current_value - bar_min) / (bar_max - bar_min)
    visual_ratio = float(np.clip(ratio, 0.0, 1.0)) if args.bar_clamp else ratio

    bottom = float(artists["track_bottom"])
    height = float(artists["track_height"])
    marker_y = bottom + visual_ratio * height

    artists["fill"].set_height(max(0.0, visual_ratio * height))
    artists["marker_line"].set_ydata([marker_y, marker_y])
    artists["marker_dot"].center = (artists["marker_dot"].center[0], marker_y)
    artists["value_text"].set_text(
        f"{format_number(current_value, args.decimals)} {args.value_unit}".rstrip()
    )


def create_telemetry_text(
    ax: plt.Axes,
    args: argparse.Namespace,
    theme: Theme,
    large: bool,
) -> dict[str, Any]:
    if large:
        value_text = ax.text(
            0.07,
            0.72,
            "",
            transform=ax.transAxes,
            va="center",
            ha="left",
            fontsize=38,
            weight="bold",
            color=theme.text,
        )
        unit_text = ax.text(
            0.08,
            0.53,
            args.value_unit,
            transform=ax.transAxes,
            va="center",
            ha="left",
            fontsize=18,
            color=theme.muted_text,
        )
        time_text = ax.text(
            0.08,
            0.31,
            "",
            transform=ax.transAxes,
            va="center",
            ha="left",
            fontsize=15,
            family="monospace",
            color=theme.text,
        )
        state_text = ax.text(
            0.08,
            0.14,
            "",
            transform=ax.transAxes,
            va="center",
            ha="left",
            fontsize=12,
            family="monospace",
            color=theme.muted_text,
        )
        label_text = ax.text(
            0.07,
            0.92,
            args.value_name,
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=11,
            color=theme.muted_text,
        )
        return {
            "value": value_text,
            "unit": unit_text,
            "time": time_text,
            "state": state_text,
            "label": label_text,
            "large": True,
        }

    text = ax.text(
        0.08,
        0.94,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        family="monospace",
        color=theme.text,
        linespacing=1.35,
    )
    return {"text": text, "large": False}


def update_telemetry_text(
    artists: dict[str, Any],
    args: argparse.Namespace,
    current_time: float,
    current_value: float,
    state: str,
    sample_index: int,
    sample_count: int,
) -> None:
    if len(state) > 28:
        state = state[:25] + "..."

    if artists["large"]:
        artists["value"].set_text(format_number(current_value, args.decimals))
        artists["time"].set_text(f"T+ {current_time:07.2f} s")
        artists["state"].set_text(f"STATE  {state}\nSAMPLE {sample_index + 1}/{sample_count}")
        return

    artists["text"].set_text(
        "TELEMETRY\n"
        "----------------\n"
        f"T+ {current_time:07.2f} s\n\n"
        f"{args.value_name}\n"
        f"{format_number(current_value, args.decimals)} {args.value_unit}\n\n"
        "STATE\n"
        f"{state}\n\n"
        "SAMPLE\n"
        f"{sample_index + 1}/{sample_count}"
    )


def create_layout(
    fig: plt.Figure,
    args: argparse.Namespace,
    theme: Theme,
    duration: float,
    graph_min: float,
    graph_max: float,
    bar_min: float,
    bar_max: float,
    initial_time: float,
    initial_value: float,
) -> dict[str, Any]:
    top = 0.93 if args.no_header else 0.84
    bottom = 0.12
    content_height = top - bottom

    result: dict[str, Any] = {
        "graph": None,
        "bar": None,
        "telemetry": None,
    }

    if not args.no_header:
        fig.text(0.035, 0.94, args.title, fontsize=22, weight="bold", color=theme.text)
        fig.text(0.037, 0.895, args.subtitle, fontsize=10, color=theme.muted_text)

    if args.mode == "graph":
        if args.no_panel:
            graph_ax = fig.add_axes([0.075, bottom, 0.89, content_height])
        else:
            graph_ax = fig.add_axes([0.075, bottom, 0.66, content_height])
            info_ax = fig.add_axes([0.77, bottom + 0.03, 0.20, content_height - 0.06])
            style_info_axis(info_ax, args, theme)
            result["telemetry"] = create_telemetry_text(info_ax, args, theme, large=False)

        style_graph_axis(graph_ax, args, theme, duration, graph_min, graph_max)
        result["graph"] = create_graph_artists(
            graph_ax, args, theme, initial_time, initial_value
        )

    elif args.mode == "graph-bar":
        graph_ax = fig.add_axes([0.065, bottom, 0.67, content_height])
        bar_ax = fig.add_axes([0.765, bottom, 0.205, content_height])
        style_graph_axis(graph_ax, args, theme, duration, graph_min, graph_max)
        result["graph"] = create_graph_artists(
            graph_ax, args, theme, initial_time, initial_value
        )
        result["bar"] = create_bar_artists(
            bar_ax, args, theme, bar_min, bar_max, compact=True
        )

    elif args.mode == "bar":
        bar_ax = fig.add_axes([0.12, bottom, 0.32, content_height])
        info_ax = fig.add_axes([0.50, bottom + 0.08, 0.42, content_height - 0.16])
        style_info_axis(info_ax, args, theme)
        result["bar"] = create_bar_artists(
            bar_ax, args, theme, bar_min, bar_max, compact=False
        )
        result["telemetry"] = create_telemetry_text(info_ax, args, theme, large=True)

    elif args.mode == "hud":
        bar_ax = fig.add_axes([0.06, bottom + 0.03, 0.20, content_height - 0.06])
        info_ax = fig.add_axes([0.30, bottom + 0.08, 0.64, content_height - 0.16])
        style_info_axis(info_ax, args, theme)
        result["bar"] = create_bar_artists(
            bar_ax, args, theme, bar_min, bar_max, compact=True
        )
        result["telemetry"] = create_telemetry_text(info_ax, args, theme, large=True)

    return result


def render_frames(
    args: argparse.Namespace,
    theme: Theme,
    df: pd.DataFrame,
    times: np.ndarray,
    values: np.ndarray,
    video_times: np.ndarray,
    frames_dir: Path,
) -> None:
    interpolated = np.interp(video_times, times, values)
    sample_indices = np.searchsorted(times, video_times, side="right") - 1
    sample_indices = np.clip(sample_indices, 0, len(df) - 1)

    graph_min, graph_max, bar_min, bar_max = calculate_ranges(args, values)

    fig = plt.figure(
        figsize=(args.width / args.dpi, args.height / args.dpi),
        dpi=args.dpi,
        facecolor=theme.background,
    )
    fig.patch.set_facecolor(theme.background)
    fig.patch.set_edgecolor("none")

    layout = create_layout(
        fig=fig,
        args=args,
        theme=theme,
        duration=float(times[-1]),
        graph_min=graph_min,
        graph_max=graph_max,
        bar_min=bar_min,
        bar_max=bar_max,
        initial_time=float(video_times[0]),
        initial_value=float(interpolated[0]),
    )

    total_frames = len(video_times)
    progress_step = max(1, total_frames // 20)

    try:
        for frame, current_time_raw in enumerate(video_times):
            current_time = float(current_time_raw)
            current_value = float(interpolated[frame])
            sample_index = int(sample_indices[frame])
            state = str(df[args.state_column].iloc[sample_index])

            if layout["graph"] is not None:
                update_graph_artists(
                    layout["graph"], times, values, current_time, current_value
                )

            if layout["bar"] is not None:
                update_bar_artists(layout["bar"], args, current_value)

            if layout["telemetry"] is not None:
                update_telemetry_text(
                    layout["telemetry"],
                    args,
                    current_time,
                    current_value,
                    state,
                    sample_index,
                    len(df),
                )

            filename = frames_dir / f"frame_{frame:06d}.png"
            fig.savefig(
                filename,
                dpi=args.dpi,
                facecolor=fig.get_facecolor(),
                edgecolor="none",
                transparent=False,
            )

            if frame == 0 or (frame + 1) % progress_step == 0 or frame + 1 == total_frames:
                percentage = (frame + 1) / total_frames * 100
                print(
                    f"Frames: {frame + 1:>{len(str(total_frames))}}/"
                    f"{total_frames} ({percentage:5.1f}%)"
                )
    finally:
        plt.close(fig)


def find_ffmpeg(executable: str) -> str:
    found = shutil.which(executable)
    if found:
        return found

    supplied = Path(executable)
    if supplied.is_file():
        return str(supplied)

    raise RuntimeError(
        "No se encontró FFmpeg. Instálalo y añádelo al PATH, "
        "o indica ffmpeg.exe con --ffmpeg."
    )


def validate_output_format(args: argparse.Namespace, transparent_canvas: bool) -> None:
    """Comprueba el contenedor antes de invertir tiempo generando frames."""
    if args.frames_only:
        return

    extension = args.output.suffix.lower()
    if extension not in {".mp4", ".webm", ".mov"}:
        raise RuntimeError("La salida debe terminar en .mp4, .webm o .mov.")

    if transparent_canvas and extension not in TRANSPARENT_VIDEO_EXTENSIONS:
        raise RuntimeError(
            "El tema usa un lienzo transparente, pero MP4/H.264 no conserva alfa. "
            "Usa una salida .webm o .mov, o genera PNG con --frames-only."
        )


def build_ffmpeg_command(
    args: argparse.Namespace,
    ffmpeg_path: str,
    frames_dir: Path,
    transparent_canvas: bool,
) -> list[str]:
    extension = args.output.suffix.lower()
    if extension not in {".mp4", ".webm", ".mov"}:
        raise RuntimeError("La salida debe terminar en .mp4, .webm o .mov.")

    if transparent_canvas and extension not in TRANSPARENT_VIDEO_EXTENSIONS:
        raise RuntimeError(
            "El tema usa un lienzo transparente, pero MP4/H.264 no conserva alfa. "
            "Usa una salida .webm o .mov, o genera PNG con --frames-only."
        )

    base = [
        ffmpeg_path,
        "-y",
        "-framerate",
        str(args.fps),
        "-i",
        str(frames_dir / "frame_%06d.png"),
    ]

    codec = args.codec
    if codec == "auto":
        if transparent_canvas and extension == ".webm":
            codec = "libvpx-vp9"
        elif transparent_canvas and extension == ".mov":
            codec = "prores_ks"
        elif extension == ".webm":
            codec = "libvpx-vp9"
        else:
            codec = "libx264"

    command = base + ["-c:v", codec]

    if transparent_canvas and extension == ".webm":
        command += [
            "-pix_fmt",
            "yuva420p",
            "-auto-alt-ref",
            "0",
            "-metadata:s:v:0",
            "alpha_mode=1",
            "-b:v",
            "0",
            "-crf",
            str(args.crf),
        ]
    elif transparent_canvas and extension == ".mov":
        command += [
            "-profile:v",
            "4",
            "-pix_fmt",
            "yuva444p10le",
            "-alpha_bits",
            "16",
        ]
    elif codec == "libvpx-vp9":
        command += [
            "-pix_fmt",
            "yuv420p",
            "-b:v",
            "0",
            "-crf",
            str(args.crf),
        ]
    else:
        command += [
            "-preset",
            args.preset,
            "-crf",
            str(args.crf),
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
        ]

    command.append(str(args.output))
    return command


def run_ffmpeg(
    args: argparse.Namespace,
    frames_dir: Path,
    transparent_canvas: bool,
) -> None:
    ffmpeg_path = find_ffmpeg(args.ffmpeg)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = build_ffmpeg_command(args, ffmpeg_path, frames_dir, transparent_canvas)

    print("\nGenerando vídeo con FFmpeg...")
    print(" ".join(f'"{part}"' if " " in part else part for part in command))

    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"FFmpeg terminó con el código {exc.returncode}.") from exc


def prepare_frames_directory(
    args: argparse.Namespace,
) -> tuple[Path, tempfile.TemporaryDirectory[str] | None]:
    if args.frames_dir is not None:
        frames_dir = args.frames_dir.resolve()
        clean_old_frames(frames_dir)
        return frames_dir, None

    if args.keep_frames:
        frames_dir = (args.output.parent / f"{args.output.stem}_frames").resolve()
        clean_old_frames(frames_dir)
        return frames_dir, None

    temporary = tempfile.TemporaryDirectory(prefix="csv_video_frames_")
    return Path(temporary.name), temporary


def preserve_temporary_frames(
    args: argparse.Namespace,
    frames_dir: Path,
) -> Path:
    final_dir = (args.output.parent / f"{args.output.stem}_frames").resolve()
    clean_old_frames(final_dir)
    for frame_file in frames_dir.glob("frame_*.png"):
        shutil.move(str(frame_file), final_dir / frame_file.name)
    return final_dir


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    try:
        df, times, values = load_data(args)
        theme = resolve_theme(args)
        transparent_canvas = canvas_has_alpha(theme)
        validate_output_format(args, transparent_canvas)
        video_times = build_video_times(
            float(times[-1]), args.fps, args.speed, args.max_frames
        )

        print(f"CSV              : {args.csv_file}")
        print(f"Muestras válidas : {len(df)}")
        print(f"Duración datos   : {times[-1]:.2f} s")
        print(f"Duración vídeo   : {times[-1] / args.speed:.2f} s aprox.")
        print(f"FPS              : {args.fps:g}")
        print(f"Frames           : {len(video_times)}")
        print(f"Tema             : {args.theme}")
        print(f"Modo             : {args.mode}")
        print(f"Canal alfa       : {'sí' if transparent_canvas else 'no'}")

        frames_dir, temporary_context = prepare_frames_directory(args)

        try:
            print(f"Carpeta de frames: {frames_dir}\n")
            render_frames(args, theme, df, times, values, video_times, frames_dir)

            if args.frames_only:
                if temporary_context is not None:
                    final_dir = preserve_temporary_frames(args, frames_dir)
                else:
                    final_dir = frames_dir
                print(f"\nOK: PNG generados en {final_dir}")
                return 0

            run_ffmpeg(args, frames_dir, transparent_canvas)
            print(f"\nOK: {args.output.resolve()}")
            if args.keep_frames or args.frames_dir is not None:
                print(f"Frames conservados en: {frames_dir}")
            return 0
        finally:
            if temporary_context is not None:
                temporary_context.cleanup()

    except KeyboardInterrupt:
        print("\nProceso cancelado por el usuario.", file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
