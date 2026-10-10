"""Match viewer-saved RAW frames to this run's lolenc detection datasets.

This opens no board sockets. Keep the existing FLARE viewer as the sole RAW
receiver, launched with --shot-metadata and the updated receiver binary.
Dependencies: h5py, numpy (already used by the lolenc result environment).
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import time
import zlib

import h5py
import numpy as np


RESULT_FIELDS = (
    "index", "shot_id", "config_token", "result_sequence", "state_mask",
    "valid_mask", "low_confidence_mask", "saturation_mask", "error_flags",
    "result_code", "ion_count", "config_current", "classifier_complete",
    "frame_valid", "armed_ok", "raw_enabled", "armed_us", "result_us",
    "auto_ack_delta", "board_pass",
)
SETTINGS_FIELDS = (
    "config_token", "ion_count", "background_enable", "background_scale_q20",
    "signal_width", "signal_height", "background_width", "background_height",
    "background_dx", "background_dy", "confidence_margin", "x", "y", "threshold",
)


def read_run(path: Path) -> dict:
    with h5py.File(path, "r") as saved:
        group = saved["datasets"]

        def dataset(suffix: str):
            matches = [name for name in group if name == suffix or name.endswith("_" + suffix)]
            if len(matches) != 1:
                raise ValueError(f"Expected one {suffix} dataset, found {len(matches)}")
            rows = np.asarray(group[matches[0]][()])
            if rows.ndim != 2 or not np.isfinite(rows).all():
                raise ValueError(f"Invalid dataset shape/values: {suffix}")
            if not np.equal(rows, np.trunc(rows)).all():
                raise ValueError(f"Noninteger protocol data: {suffix}")
            return rows.astype(np.int64)

        summary = dataset("flare_camera_summary")
        settings = dataset("flare_camera_settings")
        has_results = any(name == "flare_camera_results" or name.endswith("_flare_camera_results") for name in group)
        results = dataset("flare_camera_results") if has_results else np.empty((0, 20), dtype=np.int64)
        if summary.shape != (1, 8) or settings.shape != (1, 14) or results.shape[1] != 20:
            raise ValueError("Unexpected camera dataset schema")
        return {
            "h5": str(path.resolve()),
            "summary": dict(zip(("board_pass", "failure_mask", "failed_phase", "requested",
                                 "attempted", "passed", "config_token", "raw_policy"),
                                map(int, summary[0]))),
            "settings": dict(zip(SETTINGS_FIELDS, map(int, settings[0]))),
            "results": [dict(zip(RESULT_FIELDS, map(int, row))) for row in results],
        }


def evaluate_run(run: dict, shot_dir: Path, shots: int, fresh_since: float | None = None) -> dict:
    """Missing files are pending; malformed or inconsistent evidence fails."""
    problems = []
    missing = []
    checked = []
    summary = run["summary"]
    settings = run["settings"]
    results = run["results"]
    if (summary["board_pass"] != 1 or summary["failure_mask"] or
            summary["requested"] != shots or summary["attempted"] != shots or
            summary["passed"] != shots or summary["raw_policy"] != 2 or len(results) != shots):
        problems.append(f"Board detection test failed or has wrong count: {summary}")
    if settings["ion_count"] != 1 or settings["background_enable"] != 0:
        problems.append("This verifier expects the test's one-ROI/background-disabled configuration")
    if settings["config_token"] != summary["config_token"] or not settings["config_token"]:
        problems.append("Settings/summary token mismatch")
    shot_ids = [result["shot_id"] for result in results]
    if len(set(shot_ids)) != len(shot_ids) or 0 in shot_ids:
        problems.append("Duplicate/zero detection shot IDs")
    sequences = [result["result_sequence"] for result in results]
    if len(set(sequences)) != len(sequences):
        problems.append("Duplicate detection sequences")
    x, y = settings["x"], settings["y"]
    width, height = settings["signal_width"], settings["signal_height"]
    if not (0 <= x < 512 and 0 <= y < 512 and width > 0 and height > 0 and
            x + width <= 512 and y + height <= 512):
        problems.append("ROI is outside the full camera frame")
    if problems:
        return {"pass": False, "pending": False, "problems": problems,
                "missing_shots": missing, "shots": checked, "run": run}

    for result in results:
        shot_id = result["shot_id"]
        prefix = f"shot{shot_id:010d}_frame"
        candidates = sorted(path for path in shot_dir.glob(prefix + "*.raw.json")
                            if fresh_since is None or path.stat().st_mtime >= fresh_since)
        if not candidates:
            missing.append(shot_id)
            continue
        if len(candidates) != 1:
            problems.append(f"shot {shot_id}: {len(candidates)} RAW metadata files; ambiguous frame")
            continue
        meta_path = candidates[0]
        raw_path = meta_path.with_suffix("")
        try:
            try:
                metadata = json.loads(meta_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                # The receiver may have opened the sidecar in this exact
                # polling turn. Retry rather than fail an in-progress write.
                missing.append(shot_id)
                continue
            if (metadata.get("schema") != 1 or metadata.get("protocol_version") != 1 or metadata.get("shot_id") != shot_id or
                    metadata.get("width") != 512 or metadata.get("height") != 512 or
                    metadata.get("stride") != 1024 or metadata.get("payload_length") != 524288 or
                    metadata.get("pixel_format") != 0 or metadata.get("pixel_bit_depth") != 16 or
                    metadata.get("streaming") is not False or metadata.get("frame_valid") is not True or
                    metadata.get("error_flags") != 0 or metadata.get("header_crc_verified") is not True or
                    metadata.get("payload_crc_present") is not True or
                    metadata.get("flags", 0) & 9 or not metadata.get("flags", 0) & 2):
                raise ValueError("Invalid RAW header/geometry/CRC evidence")
            # Current daemon sets metadata_partial: its config fields are zeros,
            # not a source of the Aurora token. Match by shot ID, not those zeros.
            if not metadata.get("metadata_partial") and (
                    metadata.get("config_id") != 0 or
                    metadata.get("config_version") != (settings["config_token"] & 65535)):
                raise ValueError("RAW configuration identity mismatch")
            payload = raw_path.read_bytes()
            if fresh_since is not None and raw_path.stat().st_mtime < fresh_since:
                missing.append(shot_id)
                continue
            if len(payload) != 524288:
                raise ValueError(f"RAW size {len(payload)} instead of 524288")
            checksum = zlib.crc32(payload) & 0xffffffff
            if checksum != metadata.get("payload_crc32"):
                raise ValueError("Saved RAW payload CRC mismatch")
            image = np.frombuffer(payload, dtype="<u2").reshape(512, 512)
            # Schema 1: offset=0, polarity=0. With background disabled, the
            # score is the integer ROI sum. This small window cannot overflow.
            score = int(image[y:y+height, x:x+width].sum(dtype=np.uint64))
            expected_state = int(score >= settings["threshold"])
            expected_low_confidence = int(abs(score - settings["threshold"]) < settings["confidence_margin"])
            required = {"valid_mask": 1, "error_flags": 0, "result_code": 0,
                        "ion_count": 1, "config_current": 1, "classifier_complete": 1,
                        "frame_valid": 1, "armed_ok": 1, "raw_enabled": 1,
                        "auto_ack_delta": 1, "board_pass": 1,
                        "state_mask": expected_state, "low_confidence_mask": expected_low_confidence,
                        "config_token": settings["config_token"]}
            for field, value in required.items():
                if result[field] != value:
                    raise ValueError(f"{field}: FPGA {result[field]}, expected {value}")
            checked.append({"shot_id": shot_id, "frame_id": metadata["frame_id"],
                            "raw": str(raw_path.resolve()), "crc32": checksum,
                            "roi_sum": score, "state": expected_state,
                            "low_confidence": expected_low_confidence,
                            "saturation_mask": result["saturation_mask"],
                            "pixel_min": int(image.min()), "pixel_max": int(image.max()),
                            "result_us": result["result_us"]})
        except (OSError, ValueError, KeyError, TypeError) as error:
            problems.append(f"shot {shot_id}: {error}")
    return {"pass": not problems and not missing and len(checked) == shots,
            "pending": bool(missing) and not problems, "problems": problems,
            "missing_shots": missing, "shots": checked, "run": run}


def new_runs(root: Path, started: float, shots: int):
    paths = sorted((path for path in root.rglob("*.h5") if path.stat().st_mtime >= started),
                   key=lambda path: path.stat().st_mtime, reverse=True)
    for path in paths:
        try:
            run = read_run(path)
            if run["summary"]["requested"] == shots:
                yield run
        except (OSError, ValueError, KeyError):
            continue  # Unrelated result or a file still being written.


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shots", type=int, choices=(1, 10), required=True)
    parser.add_argument("--shot-dir", type=Path, required=True, help="same directory as viewer --save-dir")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--h5", type=Path, help="verify an already completed lolenc run")
    source.add_argument("--results-root", type=Path, help="watch for a NEW lolenc HDF5 run; start before experiment")
    parser.add_argument("--timeout", type=float, default=300, help="wall-clock limit in seconds")
    parser.add_argument("--report", type=Path, help="JSON report (new file only)")
    args = parser.parse_args(argv)
    if not args.shot_dir.is_dir() or args.timeout <= 0:
        parser.error("--shot-dir must exist; --timeout must be positive")
    if args.results_root is not None and not args.results_root.is_dir():
        parser.error("--results-root must exist")
    if args.report is not None and args.report.exists():
        parser.error("--report already exists; choose a new file")
    started = time.time()
    deadline = time.monotonic() + args.timeout
    report = None
    if args.h5 is None:
        print("READY: watching new lolenc result files. Run the camera experiment now.", flush=True)
    while True:
        if args.h5 is not None:
            run = read_run(args.h5)
        else:
            run = next(new_runs(args.results_root, started, args.shots), None)
        if run is not None:
            report = evaluate_run(run, args.shot_dir, args.shots,
                                  fresh_since=started if args.h5 is None else None)
            if not report["pending"] or args.h5 is not None:
                break
        if time.monotonic() >= deadline:
            if report is None:
                report = {"pass": False, "pending": False, "problems": ["No new camera HDF5 result before timeout"]}
            else:
                report["pending"] = False
                report["problems"].append("RAW timeout: missing .raw/.raw.json files; check viewer --shot-metadata")
            break
        time.sleep(0.25)
    report_path = args.report or args.shot_dir / ("flare_camera_check_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
    with report_path.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, ensure_ascii=False)
    for shot in report.get("shots", []):
        print(f"shot {shot['shot_id']}: RAW CRC OK, ROI sum {shot['roi_sum']}, "
              f"state {shot['state']}, low_conf {shot['low_confidence']}, detection MATCH")
    for problem in report.get("problems", []):
        print("FAIL:", problem)
    if report.get("missing_shots"):
        print("Missing RAW shot IDs:", report["missing_shots"])
    print(f"FLARE camera {'PASS' if report['pass'] else 'FAIL'}: "
          f"{len(report.get('shots', []))}/{args.shots} RAW/detection pairs; report {report_path}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
