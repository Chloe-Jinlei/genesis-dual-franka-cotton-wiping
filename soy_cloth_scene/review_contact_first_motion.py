"""Read-only endpoint strain and sampled hold-motion review of a finished probe.

This does not load Genesis or reconstruct an unsampled cloth trajectory.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from review_contact_first import FONT, sha


def strain_summary(path, config):
    with np.load(path) as data:
        positions, reference, faces = [data[key] for key in ("positions", "reference", "faces")]
    edges = np.unique(np.sort(faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0)
    reference_delta = reference[edges[:, 1]]-reference[edges[:, 0]]
    strain = np.linalg.norm(positions[edges[:, 1]]-positions[edges[:, 0]], axis=1)/np.linalg.norm(reference_delta, axis=1)-1

    def values(selected):
        return {"edge_count": int(len(selected)), "extension_p99": float(np.percentile(selected, 99)),
                "maximum_extension": float(selected.max()), "minimum_signed_strain": float(selected.min()),
                "edges_over_5pct_extension": int(np.count_nonzero(selected > .05))}

    result = {"source_sha256": sha(path), "whole_cloth": values(strain), "material_corner_patches": {}}
    directions = {"warp": np.abs(reference_delta[:, 1]) < 1e-9,
                  "weft": np.abs(reference_delta[:, 0]) < 1e-9}
    directions["bias"] = ~(directions["warp"] | directions["weft"])
    result["directions"] = {key: values(strain[mask]) for key, mask in directions.items()}
    center = np.asarray(config["cloth"]["position_xy"])
    radius = float(config["robot_task"].get("material_patch_radius_m", .022))
    for side, offset in config["robot_task"]["grasp_offsets_xy_m"].items():
        inside = np.linalg.norm(reference[:, :2]-(center+offset), axis=1) <= radius
        edge_mask = np.any(inside[edges], axis=1)
        result["material_corner_patches"][side] = values(strain[edge_mask])
    result["local_patch_definition"] = "True mesh edges incident to immutable reference nodes within the configured material-corner radius."
    result["material_corner_radius_m"] = radius
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    if source == output:
        parser.error("Use a separate output directory")
    probe = json.loads((source/"probe_report.json").read_text())
    if probe.get("requested_stage_reached") is not True or probe.get("exception") is not None:
        parser.error("The requested stage was not successfully reached; do not present this as a lifted hold")
    config = json.loads((source/"config_used.json").read_text())
    events = json.loads((source/"probe_events.json").read_text())
    start = next(e["time_s"] for e in events if e["phase"].startswith("Probe: stationary hold"))
    dt = float(config["simulation"]["dt"])
    end = probe["final_frame"]*dt
    output.mkdir(parents=True, exist_ok=True)
    report = {"scope": "Current pickup diagnostic only; endpoint strain and its recorded additional stationary hold.",
              "source_directory": str(source), "hold_interval_s": [start, end], "hold_duration_s": end-start,
              "stage_strain": {}, "sampled_hold": {}, "not_a_four_second_settling_acceptance": True}
    for name in ("01_initial", "03_grasped", "04_cloth_lifted", "12_wet_cloth_lifted", "contact_first_probe_final"):
        path = source/(name+"_cloth.npz")
        if path.exists():
            report["stage_strain"][name] = strain_summary(path, config)
    with (source/"contact_trace.csv").open() as stream:
        trace = [r for r in csv.DictReader(stream) if start-1e-9 <= float(r["time_s"]) <= end+1e-9]
    report["sampled_hold"]["trace_rows"] = len(trace)
    for side in ("left", "right"):
        tcp = np.array([[float(r[f"{side}_tcp_{axis}_m"]) for axis in "xyz"] for r in trace])
        report["sampled_hold"][side+"_tcp_coordinate_ranges_m"] = np.ptp(tcp, axis=0).tolist()
        report["sampled_hold"][side+"_tcp_max_displacement_from_first_sample_m"] = float(np.linalg.norm(tcp-tcp[0], axis=1).max())
    with (source/"mass_and_motion.csv").open() as stream:
        rows = [r for r in csv.DictReader(stream) if start-1e-9 <= float(r["time_s"]) <= end+1e-9]
    min_z = np.array([float(r["cloth_min_z_m"]) for r in rows])
    report["sampled_hold"].update(motion_rows=len(rows), cloth_min_z_range_m=[float(min_z.min()),float(min_z.max())],
                                  cloth_min_z_peak_to_peak_m=float(np.ptp(min_z)))
    video = source/"dual_arm_wipe.mp4"
    cap = cv2.VideoCapture(str(video)); count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    stride = max(1, round(1/(15*dt))); first = (int(probe["start_frame"])//stride+1)*stride
    times = (first+np.arange(count)*stride)*dt
    selected = np.flatnonzero((times >= start-1e-9) & (times <= end+1e-9))
    chosen = set(map(int, selected)); frames=[]
    for index in range(count):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Video decode stopped before its reported frame count")
        if index in chosen:
            name = f"hold_frame_{index:05d}.png"
            cv2.imwrite(str(output/name), frame)
            frames.append((index, frame, name))
    cap.release()
    columns=4; width=480; panel_height=390
    canvas=Image.new("RGB",(columns*width,70+panel_height*((len(frames)+columns-1)//columns)),"#eef2f5")
    draw=ImageDraw.Draw(canvas); title_font=ImageFont.truetype(FONT,24); font=ImageFont.truetype(FONT,18)
    draw.text((15,8),f"提起后保持：{start:.3f}–{end:.3f} 秒（仅本次 {end-start:.2f} 秒观察）",font=title_font,fill="#14222b")
    draw.text((15,40),"每格为原录像完整画面；这不是 4 秒稳定验收。",font=font,fill="#14222b")
    frame_records=[]
    for n,(index,bgr,name) in enumerate(frames):
        x,y=(n%columns)*width,70+(n//columns)*panel_height
        image=Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)); image.thumbnail((480,360))
        canvas.paste(image,(x,y));draw.text((x+8,y+364),f"{times[index]:.3f}s",font=font,fill="#14222b")
        frame_records.append({"source_frame_index":index,"time_s":float(times[index]),"image":str(output/name),"sha256":sha(output/name)})
    canvas.save(output/"hold_frame_strip.png")
    report["hold_video_frames"]=frame_records
    report["limitations"]=["Minimum cloth height is one scalar; it does not quantify all-node oscillation or prove absence of flutter.",
        "Endpoint snapshots are not a sampled cloth trajectory; differences in strain can reflect contact relaxation as well as motion.",
        "The visual assessment is added only after separately inspecting the extracted source frames."]
    report["source_sha256"]={name:sha(source/name) for name in ("probe_report.json","probe_events.json","contact_trace.csv","mass_and_motion.csv","dual_arm_wipe.mp4","config_used.json")}
    (output/"motion_and_strain_review.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps({"hold_interval_s":report["hold_interval_s"],"sampled_hold":report["sampled_hold"],"stage_strain":{key:value["whole_cloth"] for key,value in report["stage_strain"].items()}},indent=2))


if __name__ == "__main__":
    main()
