#!/usr/bin/env python3
"""Genera un vídeo animado a partir de una serie temporal almacenada en CSV."""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path

import matplotlib

# Evita depender de una interfaz gráfica, algo importante al ejecutar en servidores.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass(frozen=True)
class Theme:
    background: str
    axes_background: str
    text: str
    muted_text: str
    line: str
    cursor: str
    track: str
    grid: str
    spine: str
    panel: str
    panel_border: str
    marker_face: str


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
}


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
    lowered = value.lower()
    return aliases.get(lowered, value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Genera un MP4 animado a partir de dos columnas de un CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("csv_file", type=Path, help="Archivo CSV de entrada")
    parser.add_argument("-o", "--output", type=Path, default=Path("replay.mp4"), help="Vídeo MP4 de salida")

    data = parser.add_argument_group("Datos del CSV")
    data.add_argument("--time-column", default="t_ms", help="Columna de tiempo")
    data.add_argument("--value-column", default="baro_alt", help="Columna numérica representada")
    data.add_argument("--state-column", default="state", help="Columna de estado; se muestra como N/A si no existe")
    data.add_argument(
        "--time-unit",
        choices=("s", "ms", "us", "ns"),
        default="ms",
        help="Unidad de la columna de tiempo",
    )
    data.add_argument(
        "--delimiter",
        default="auto",
        help="Separador: auto, comma, semicolon, tab, pipe o un carácter literal",
    )
    data.add_argument("--encoding", default="utf-8-sig", help="Codificación del CSV")

    video = parser.add_argument_group("Vídeo")
    video.add_argument("--fps", type=positive_float, default=30.0, help="Fotogramas por segundo")
    video.add_argument("--speed", type=positive_float, default=1.0, help="Velocidad de reproducción: 2 reproduce al doble")
    video.add_argument("--width", type=positive_int, default=1280, help="Anchura en píxeles")
    video.add_argument("--height", type=positive_int, default=720, help="Altura en píxeles")
    video.add_argument("--dpi", type=positive_int, default=120, help="DPI usados para renderizar")
    video.add_argument("--codec", default="libx264", help="Códec de FFmpeg")
    video.add_argument("--preset", default="medium", help="Preset de FFmpeg")
    video.add_argument("--crf", type=int, default=18, help="Calidad H.264: menor valor = más calidad y más peso")
    video.add_argument("--ffmpeg", default="ffmpeg", help="Ejecutable o ruta de FFmpeg")
    video.add_argument("--frames-dir", type=Path, help="Carpeta de frames; si se omite se usa una carpeta temporal")
    video.add_argument("--keep-frames", action="store_true", help="No borrar los PNG después de generar el vídeo")
    video.add_argument("--frames-only", action="store_true", help="Generar únicamente los PNG, sin ejecutar FFmpeg")
    video.add_argument("--max-frames", type=positive_int, default=100_000, help="Límite de seguridad de fotogramas")

    chart = parser.add_argument_group("Diseño de la gráfica")
    chart.add_argument("--theme", choices=tuple(THEMES), default="spacex", help="Tema visual")
    chart.add_argument("--title", default="ARES FLIGHT REPLAY", help="Título principal")
    chart.add_argument("--subtitle", default="BAROMETRIC TELEMETRY", help="Subtítulo")
    chart.add_argument("--x-label", default="TIME (s)", help="Etiqueta del eje X")
    chart.add_argument("--y-label", default="ALTITUDE (m)", help="Etiqueta del eje Y")
    chart.add_argument("--value-name", default="ALTITUDE", help="Nombre mostrado en el panel")
    chart.add_argument("--value-unit", default="m", help="Unidad mostrada en el panel")
    chart.add_argument("--line-width", type=positive_float, default=3.0, help="Grosor de la trayectoria")
    chart.add_argument("--margin-ratio", type=float, default=0.15, help="Margen vertical proporcional")
    chart.add_argument("--minimum-margin", type=positive_float, default=2.0, help="Margen vertical mínimo")
    chart.add_argument("--no-panel", action="store_true", help="Ocultar el panel de telemetría")
    chart.add_argument("--no-grid", action="store_true", help="Ocultar la cuadrícula")
    chart.add_argument("--hide-full-track", action="store_true", help="Ocultar la trayectoria completa de fondo")

    colors = parser.add_argument_group("Sobrescritura de colores")
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

    return parser


def validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if not args.csv_file.is_file():
        parser.error(f"no existe el archivo CSV: {args.csv_file}")
    if args.width % 2 != 0 or args.height % 2 != 0:
        parser.error("--width y --height deben ser números pares para yuv420p")
    if not 0 <= args.crf <= 51:
        parser.error("--crf debe estar entre 0 y 51")
    if args.margin_ratio < 0 or not math.isfinite(args.margin_ratio):
        parser.error("--margin-ratio debe ser un número finito mayor o igual que cero")


def load_data(args: argparse.Namespace) -> tuple[pd.DataFrame, np.ndarray, np.ndarray]:
    delimiter = parse_delimiter(args.delimiter)
    read_kwargs: dict[str, object] = {
        "filepath_or_buffer": args.csv_file,
        "encoding": args.encoding,
    }
    if delimiter is None:
        read_kwargs.update({"sep": None, "engine": "python"})
    else:
        read_kwargs.update({"sep": delimiter})

    try:
        df = pd.read_csv(**read_kwargs)
    except UnicodeDecodeError as exc:
        raise RuntimeError(
            f"No se pudo leer el CSV con la codificación {args.encoding!r}. "
            "Prueba, por ejemplo, con --encoding latin-1."
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
            f"Aviso: no existe la columna {args.state_column!r}; el panel mostrará STATE = N/A.",
            file=sys.stderr,
        )
        df[args.state_column] = "N/A"

    # np.interp exige tiempos crecientes. Se ordena y, si hay tiempos repetidos,
    # se conserva la última muestra de cada instante.
    df = df.sort_values(args.time_column, kind="stable")
    df = df.drop_duplicates(subset=args.time_column, keep="last").reset_index(drop=True)

    if len(df) < 2:
        raise RuntimeError("El CSV necesita al menos dos muestras numéricas con tiempos distintos.")

    time_divisors = {"s": 1.0, "ms": 1_000.0, "us": 1_000_000.0, "ns": 1_000_000_000.0}
    raw_times = df[args.time_column].to_numpy(dtype=float)
    times = (raw_times - raw_times[0]) / time_divisors[args.time_unit]
    values = df[args.value_column].to_numpy(dtype=float)

    duration = float(times[-1])
    if not math.isfinite(duration) or duration <= 0:
        raise RuntimeError("La duración calculada debe ser mayor que cero.")

    return df, times, values


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
    }
    return replace(theme, **{key: value for key, value in overrides.items() if value is not None})


def build_video_times(duration: float, fps: float, speed: float, max_frames: int) -> np.ndarray:
    playback_duration = duration / speed
    frame_count = max(1, int(math.ceil(playback_duration * fps)))
    if frame_count > max_frames:
        raise RuntimeError(
            f"Se generarían {frame_count:,} frames, por encima del límite de {max_frames:,}. "
            "Aumenta --speed, baja --fps o eleva --max-frames conscientemente."
        )

    video_times = np.arange(frame_count, dtype=float) * speed / fps
    return np.minimum(video_times, duration)


def clean_old_frames(frames_dir: Path) -> None:
    frames_dir.mkdir(parents=True, exist_ok=True)
    for old_frame in frames_dir.glob("frame_*.png"):
        old_frame.unlink()


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

    value_min = float(np.min(values))
    value_max = float(np.max(values))
    value_range = value_max - value_min
    margin = max(value_range * args.margin_ratio, args.minimum_margin)
    if value_range == 0:
        margin = max(abs(value_min) * 0.05, args.minimum_margin)
    ymin = value_min - margin
    ymax = value_max + margin

    fig = plt.figure(
        figsize=(args.width / args.dpi, args.height / args.dpi),
        dpi=args.dpi,
        facecolor=theme.background,
    )

    if args.no_panel:
        ax = fig.add_axes([0.08, 0.13, 0.88, 0.72])
        panel_ax = None
    else:
        ax = fig.add_axes([0.075, 0.13, 0.66, 0.72])
        panel_ax = fig.add_axes([0.77, 0.16, 0.20, 0.63])
        panel_ax.set_facecolor(theme.panel)
        panel_ax.set_xticks([])
        panel_ax.set_yticks([])
        for spine in panel_ax.spines.values():
            spine.set_color(theme.panel_border)
            spine.set_linewidth(1.0)

    ax.set_facecolor(theme.axes_background)
    ax.set_xlim(0, float(times[-1]))
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel(args.x_label, color=theme.text)
    ax.set_ylabel(args.y_label, color=theme.text)
    ax.tick_params(colors=theme.text)
    for spine in ax.spines.values():
        spine.set_color(theme.spine)

    if not args.no_grid:
        ax.grid(True, color=theme.grid, alpha=0.65, linewidth=0.6)

    fig.text(0.035, 0.94, args.title, fontsize=22, weight="bold", color=theme.text)
    fig.text(0.037, 0.895, args.subtitle, fontsize=10, color=theme.muted_text)

    # La trayectoria gris se dibuja únicamente desde el instante actual hacia
    # delante. Antes se dibujaba completa por debajo de la azul usando los
    # datos originales, mientras que la azul usaba datos remuestreados a los
    # FPS del vídeo. Eso hacía que algunos picos grises sobresalieran.
    future_line = None
    if not args.hide_full_track:
        future_line, = ax.plot(
            [],
            [],
            color=theme.track,
            linewidth=2.0,
            alpha=0.8,
            zorder=1,
        )

    travelled_line, = ax.plot([], [], color=theme.line, linewidth=args.line_width, zorder=5)
    cursor_line = ax.axvline(0, color=theme.cursor, linewidth=1.2, alpha=0.85, zorder=6)
    marker = ax.scatter(
        [video_times[0]],
        [interpolated[0]],
        s=120,
        color=theme.marker_face,
        edgecolor=theme.line,
        linewidth=2,
        zorder=20,
    )

    panel_text = None
    if panel_ax is not None:
        panel_text = panel_ax.text(
            0.08,
            0.94,
            "",
            transform=panel_ax.transAxes,
            va="top",
            ha="left",
            fontsize=11,
            family="monospace",
            color=theme.text,
            linespacing=1.35,
        )

    total_frames = len(video_times)
    progress_step = max(1, total_frames // 20)

    try:
        for frame, current_time in enumerate(video_times):
            sample_index = int(sample_indices[frame])
            current_value = float(interpolated[frame])

            # La línea azul debe recorrer todos los puntos originales ya
            # alcanzados, no solo una muestra por frame. De lo contrario, al
            # remuestrear a 30/60 FPS puede saltarse máximos y mínimos del CSV.
            reached = int(np.searchsorted(times, current_time, side="right"))

            if reached > 0 and np.isclose(
                times[reached - 1],
                current_time,
                rtol=0.0,
                atol=1e-12,
            ):
                travelled_x = times[:reached]
                travelled_y = values[:reached]
            else:
                travelled_x = np.concatenate((times[:reached], [current_time]))
                travelled_y = np.concatenate((values[:reached], [current_value]))

            travelled_line.set_data(travelled_x, travelled_y)

            # La línea gris representa solo la parte pendiente. De esta forma
            # nunca queda dibujada debajo del tramo azul ya reproducido.
            if future_line is not None:
                future_start = int(np.searchsorted(times, current_time, side="right"))
                future_x = np.concatenate(([current_time], times[future_start:]))
                future_y = np.concatenate(([current_value], values[future_start:]))
                future_line.set_data(future_x, future_y)

            cursor_line.set_xdata([current_time, current_time])
            marker.set_offsets(np.array([[current_time, current_value]]))

            if panel_text is not None:
                state = str(df[args.state_column].iloc[sample_index])
                if len(state) > 22:
                    state = state[:19] + "..."
                panel_text.set_text(
                    "TELEMETRY\n"
                    "----------------\n"
                    f"T+ {current_time:07.2f} s\n\n"
                    f"{args.value_name}\n"
                    f"{current_value:10.2f} {args.value_unit}\n\n"
                    "STATE\n"
                    f"{state}\n\n"
                    "SAMPLE\n"
                    f"{sample_index + 1}/{len(df)}"
                )

            filename = frames_dir / f"frame_{frame:06d}.png"
            fig.savefig(filename, dpi=args.dpi, facecolor=theme.background)

            if frame == 0 or (frame + 1) % progress_step == 0 or frame + 1 == total_frames:
                percentage = (frame + 1) / total_frames * 100
                print(f"Frames: {frame + 1:>{len(str(total_frames))}}/{total_frames} ({percentage:5.1f}%)")
    finally:
        plt.close(fig)


def run_ffmpeg(args: argparse.Namespace, frames_dir: Path) -> None:
    ffmpeg_path = shutil.which(args.ffmpeg)
    if ffmpeg_path is None:
        supplied_path = Path(args.ffmpeg)
        if supplied_path.is_file():
            ffmpeg_path = str(supplied_path)
        else:
            raise RuntimeError(
                "No se encontró FFmpeg. Instálalo y asegúrate de que 'ffmpeg' está en PATH, "
                "o indica su ruta con --ffmpeg."
            )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg_path,
        "-y",
        "-framerate",
        str(args.fps),
        "-i",
        str(frames_dir / "frame_%06d.png"),
        "-c:v",
        args.codec,
        "-preset",
        args.preset,
        "-crf",
        str(args.crf),
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(args.output),
    ]

    print("\nGenerando MP4 con FFmpeg...")
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"FFmpeg terminó con el código de error {exc.returncode}.") from exc


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    try:
        df, times, values = load_data(args)
        theme = resolve_theme(args)
        video_times = build_video_times(float(times[-1]), args.fps, args.speed, args.max_frames)

        print(f"CSV              : {args.csv_file}")
        print(f"Muestras válidas : {len(df)}")
        print(f"Duración datos   : {times[-1]:.2f} s")
        print(f"Duración vídeo   : {times[-1] / args.speed:.2f} s")
        print(f"FPS              : {args.fps:g}")
        print(f"Frames           : {len(video_times)}")
        print(f"Tema             : {args.theme}")

        if args.frames_dir is not None:
            frames_dir = args.frames_dir.resolve()
            clean_old_frames(frames_dir)
            temporary_context = None
        elif args.keep_frames:
            frames_dir = (args.output.parent / f"{args.output.stem}_frames").resolve()
            clean_old_frames(frames_dir)
            temporary_context = None
        else:
            temporary_context = tempfile.TemporaryDirectory(prefix="csv_video_frames_")
            frames_dir = Path(temporary_context.name)

        try:
            print(f"Carpeta de frames: {frames_dir}\n")
            render_frames(args, theme, df, times, values, video_times, frames_dir)

            if args.frames_only:
                print(f"\nOK: frames generados en {frames_dir}")
                # Una carpeta temporal no puede borrarse si el usuario pidió conservar solo frames.
                if temporary_context is not None:
                    final_frames_dir = (args.output.parent / f"{args.output.stem}_frames").resolve()
                    clean_old_frames(final_frames_dir)
                    for frame_file in frames_dir.glob("frame_*.png"):
                        shutil.move(str(frame_file), final_frames_dir / frame_file.name)
                    print(f"Frames trasladados a: {final_frames_dir}")
                return 0

            run_ffmpeg(args, frames_dir)
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