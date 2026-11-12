"""Read-only review of a finished probe_contact_first run; never builds Genesis.

The checks use the probe's new events and sampled command/actual-joint trace.
They establish sampled controller ordering for this pickup diagnostic only,
not a force-closure, material, or complete wiping-task acceptance result.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


SIDES = ("left", "right")
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def truth(value):
    return value is True or str(value).strip().lower() == "true"


def descending(phase):
    return "descend with open fingers" in phase or "lower open fingers to cloth contact" in phase


def review_trace(rows, events, probe):
    indexed = [{**e, "probe_event_index": i} for i, e in enumerate(events)]
    closing = next((e for e in indexed if "close with contact feedback" in e["phase"]), None)
    close_index = closing["probe_event_index"] if closing else len(events)
    descents = [e for e in indexed[:close_index] if descending(e["phase"])]
    descent_results = []
    tolerance = 1e-9
    for event in descents:
        event_index = event["probe_event_index"]
        selected = [r for r in rows if int(r["probe_event_index"]) == event_index
                    and r["phase"] == event["phase"]]
        result = {"phase": event["phase"], "start_time_s": event["time_s"],
                  "event_index": event_index, "samples": len(selected), "hands": {}}
        for side in SIDES:
            if not selected:
                result["hands"][side] = {"command_not_decreased": None}
                continue
            command = np.array([[float(r[f"{side}_command_finger_{i}_m"]) for i in range(2)] for r in selected])
            actual = np.array([[float(r[f"{side}_actual_finger_{i}_m"]) for i in range(2)] for r in selected])
            delta = command-command[0]
            violations = np.flatnonzero(np.any(delta < -tolerance, axis=1))
            result["hands"][side] = {
                "command_start_per_finger_m": command[0].tolist(),
                "command_min_per_finger_m": command.min(0).tolist(),
                "command_min_change_from_start_m": delta.min(0).tolist(),
                "command_not_decreased": len(violations) == 0,
                "violating_sample_times_s": [float(selected[i]["time_s"]) for i in violations],
                "actual_min_per_finger_m": actual.min(0).tolist(),
                "actual_joint_sum_min_m": float(actual.sum(1).min()),
                "actual_joint_sum_start_m": float(actual[0].sum()),
                "actual_joint_sum_end_m": float(actual[-1].sum())}
        descent_results.append(result)
    ready_arrivals = [e for e in indexed[:close_index] if "measured open-finger arrival" in e["phase"]]
    close_gate = [e for e in indexed[:close_index] if "check cloth contact before closing" in e["phase"]]
    first_close = {}
    for side in SIDES:
        first_close[side] = None
        for previous, current in zip(rows, rows[1:]):
            if truth(current[side+"_attached"]):
                continue
            a = np.array([float(previous[f"{side}_command_finger_{i}_m"]) for i in range(2)])
            b = np.array([float(current[f"{side}_command_finger_{i}_m"]) for i in range(2)])
            if np.any(b < a-tolerance):
                first_close[side] = {"time_s": float(current["time_s"]), "frame": int(current["frame"]),
                                     "phase": current["phase"], "probe_event_index": int(current["probe_event_index"]),
                                     "command_before_m": a.tolist(), "command_after_m": b.tolist(),
                                     "sampling_note": "First observed command decrease, not necessarily the first native step of closure."}
                break
    lift = next((e for e in indexed if e["phase"].startswith("04 Lift cloth")
                 or e["phase"].startswith("11 Lift wet cloth")), None)
    confirms = [e for e in indexed if "confirmed" in e["phase"]
                and all(e.get("contacts", {}).get(s, {}).get("attached") is True for s in SIDES)]
    before_lift = [e for e in confirms if lift and e["probe_event_index"] < lift["probe_event_index"]]
    lift_rows = [r for r in rows if lift and int(r["probe_event_index"]) == lift["probe_event_index"]]
    descent_ok = (all(h["command_not_decreased"] is True for d in descent_results for h in d["hands"].values())
                  if descent_results else None)
    ready_ok = (bool(ready_arrivals and close_gate
                     and ready_arrivals[-1].get("approach", {}).get("ready") is True
                     and close_gate[-1].get("approach", {}).get("ready") is True) if closing else None)
    sampled_close_ok = (all(v and v["probe_event_index"] >= close_index for v in first_close.values())
                        if closing else None)
    lift_ok = bool(before_lift) if lift else None
    attached_at_lift = (all(truth(lift_rows[0][s+"_attached"]) for s in SIDES) if lift_rows else None)
    checks = {"descent_sampled_commands_never_below_descent_start": descent_ok,
              "first_closure_event_follows_ready_arrival_and_ready_close_gate": ready_ok,
              "first_sampled_command_decrease_not_before_closing_event": sampled_close_ok,
              "lift_event_follows_bilateral_confirmation": lift_ok,
              "both_sides_attached_at_lift_event_sample": attached_at_lift}
    # A closure-checkpoint continuation contains no approach motion. Keep its
    # descent result unknown, and require a separate predecessor review before
    # describing the composite approach-to-lift sequence as verified.
    omitted = (["descent_sampled_commands_never_below_descent_start"]
               if probe.get("mode") == "dry_closure_resume" and not descents else [])
    applicable = {key: value for key, value in checks.items() if key not in omitted}
    usable = probe.get("requested_stage_reached") is True and probe.get("exception") is None
    return {"scope": __doc__, "current_probe_sequence_verified": usable and all(v is True for v in applicable.values()),
            "requested_stage_reached": probe.get("requested_stage_reached"), "probe_exception": probe.get("exception"),
            "checks": checks, "command_tolerance_m": tolerance, "descent_intervals": descent_results,
            "applicable_checks": list(applicable), "unobserved_prefix_checks": omitted,
            "full_approach_to_lift_witnessed_in_current_probe": bool(descents and lift),
            "first_closing_event": closing, "ready_arrival_before_close": ready_arrivals[-1] if ready_arrivals else None,
            "ready_gate_before_close": close_gate[-1] if close_gate else None,
            "first_sampled_command_decrease": first_close,
            "bilateral_confirmation_before_lift": before_lift[-1] if before_lift else None,
            "lift_event": lift,
            "limitations": ["Only new probe_events.json is used; inherited historical controller events are excluded.",
                            "Periodic samples are every 25 native outer steps, with extra event-boundary samples; unsampled motion is not independently measured.",
                            "Finger joint sum is a joint-opening measure, not an independently measured physical pad-face clearance.",
                            "Current probe ordering verification does not establish force closure, strict material acceptance or completion of a fresh full wiping task."]}


def extract_four_stages(source, output, review, probe, config):
    video = source / "dual_arm_wipe.mp4"
    cap = cv2.VideoCapture(str(video))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    dt = float(config["simulation"]["dt"])
    stride = max(1, round(1/(15*dt)))
    first_step = (int(probe["start_frame"])//stride+1)*stride
    times = (first_step+np.arange(count)*stride)*dt
    if not count:
        raise ValueError("Finished video contains no decoded frames")
    specs = []
    desc = review["descent_intervals"]
    arrival = review["ready_arrival_before_close"]
    close = review["first_closing_event"]
    confirm = review["bilateral_confirmation_before_lift"]
    if desc and arrival:
        specs.append(("张开夹爪下降", (desc[0]["start_time_s"]+arrival["time_s"])/2, "nearest"))
    if close:
        specs.append(("到达布角，尚未开始闭合", close["time_s"], "before"))
    if confirm:
        specs.append(("闭合后双侧夹持确认", confirm["time_s"], "after"))
    if probe.get("stage_reached_time_s") is not None:
        specs.append(("双侧确认后提起布料", probe["stage_reached_time_s"], "before"))
    selections = []
    for title, target, mode in specs:
        if mode == "before":
            ids = np.flatnonzero(times <= target+1e-10)
            index = int(ids[-1]) if len(ids) else 0
        elif mode == "after":
            ids = np.flatnonzero(times >= target-1e-10)
            index = int(ids[0]) if len(ids) else count-1
        else:
            index = int(np.argmin(np.abs(times-target)))
        selections.append({"label": title, "requested_time_s": float(target), "selection_mode": mode,
                           "source_frame_index": index, "actual_frame_time_s": float(times[index]),
                           "frame_minus_requested_time_s": float(times[index]-target)})
    selected = {r["source_frame_index"] for r in selections}
    decoded = {}
    for index in range(count):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Video decode ended before its declared frame count")
        if index in selected:
            decoded[index] = frame
    cap.release()
    montage = Image.new("RGB", (1280, 1180), "#f2f4f6")
    draw = ImageDraw.Draw(montage)
    title_font = ImageFont.truetype(FONT, 27)
    text_font = ImageFont.truetype(FONT, 18)
    draw.text((20, 12), "抓取顺序诊断：张开下降 → 到位 → 闭合确认 → 提起", font=title_font, fill="#17252c")
    draw.text((20, 54), "原视频帧按实际时间提取；小于一帧的事件偏差在下方标出。仅验证当前抓取诊断。", font=text_font, fill="#17252c")
    for i, selection in enumerate(selections):
        frame = decoded[selection["source_frame_index"]]
        path = output / f"stage_{i+1:02d}.png"
        cv2.imwrite(str(path), frame)
        selection["image"] = str(path)
        selection["sha256"] = sha(path)
        x, y = (i%2)*640, 95+(i//2)*535
        draw.text((x+12, y), f"{i+1}. {selection['label']}", font=title_font, fill="#17252c")
        view = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize((624, 468))
        # Keep full source frame; no crop of grippers, cloth or existing native labels.
        montage.paste(view, (x+8, y+36))
        draw.text((x+12, y+510), f"画面 {selection['actual_frame_time_s']:.3f}s / 事件目标 {selection['requested_time_s']:.3f}s",
                  font=text_font, fill="#17252c")
    montage.save(output / "four_stage_evidence.png")
    review["four_stage_frames"] = selections
    review["video_frame_time_derivation"] = {"start_checkpoint_frame": probe["start_frame"],
        "first_render_step": first_step, "native_stride": stride, "dt_s": dt, "source_frames": count,
        "source_video_sha256": sha(video), "source_modified": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--extract-frames", action="store_true")
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if source == output:
        parser.error("Use a separate review output directory")
    required = ["probe_report.json", "probe_events.json", "contact_trace.csv", "config_used.json", "source_manifest.json"]
    for name in required:
        if not (source/name).exists():
            parser.error(f"Finished probe is required; missing {name}")
    output.mkdir(parents=True, exist_ok=True)
    probe = json.loads((source/"probe_report.json").read_text())
    events = json.loads((source/"probe_events.json").read_text())
    config = json.loads((source/"config_used.json").read_text())
    with (source/"contact_trace.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    review = review_trace(rows, events, probe)
    review["source_directory"] = str(source)
    review["source_sha256"] = {name: sha(source/name) for name in required}
    review["sampling_dt_s"] = 25*float(config["simulation"]["dt"])
    review["trace_rows"] = len(rows)
    if args.extract_frames:
        extract_four_stages(source, output, review, probe, config)
    (output/"contact_first_review.json").write_text(json.dumps(review, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"current_probe_sequence_verified": review["current_probe_sequence_verified"],
                      "checks": review["checks"], "review_file": str(output/"contact_first_review.json")},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
