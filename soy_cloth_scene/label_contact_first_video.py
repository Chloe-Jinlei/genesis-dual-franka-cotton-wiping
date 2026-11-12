"""Add Chinese diagnostic labels outside a completed native probe video.

Retains every source frame exactly once at the original encoded frame rate.
No Genesis initialization, interpolation, simulation edits, or speed changes.
"""
import argparse
import csv
import json
import subprocess
from fractions import Fraction
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from review_contact_first import FONT, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args=parser.parse_args()
    source,target=args.source.resolve(),args.output.resolve()
    if target.exists() or target.parent == source:
        parser.error("Use a new output video outside the original run directory")
    probe=json.loads((source/"probe_report.json").read_text())
    if not probe.get("requested_stage_reached") or probe.get("exception"):
        parser.error("Requested pickup stage was not reached without exception")
    events=json.loads((source/"probe_events.json").read_text())
    config=json.loads((source/"config_used.json").read_text())
    video=source/"dual_arm_wipe.mp4"; before=sha(video)
    native=json.loads(subprocess.check_output(["ffprobe","-v","error","-select_streams","v:0",
        "-show_entries","stream=width,height,avg_frame_rate,nb_frames","-of","json",str(video)],text=True))["streams"][0]
    width,height,count=[int(native[k]) for k in ("width","height","nb_frames")]
    fps=native["avg_frame_rate"];dt=float(config["simulation"]["dt"]);stride=max(1,round(1/(15*dt)))
    first=(int(probe["start_frame"])//stride+1)*stride
    wet=probe["mode"]=="wet_final_pickup"
    title=("湿布再次抓取" if wet else "干布抓取")+"：张开下降 → 接触 → 闭合 → 提起"
    footer="原生连续片段 · 原速 · 仅本次抓取诊断；不代表完整擦拭回归"
    if probe.get("resumed_from"):
        footer="从真实检查点恢复 · 原速 · 仅本次湿布抓取诊断" if wet else "从真实检查点恢复 · 原速 · 仅本次干布抓取诊断"
    target.parent.mkdir(parents=True,exist_ok=True)
    font=ImageFont.truetype(FONT,23); small=ImageFont.truetype(FONT,18)
    command=["ffmpeg","-v","error","-y","-f","rawvideo","-pixel_format","rgb24","-video_size",f"{width}x{height+112}",
             "-framerate",fps,"-i","pipe:0","-an","-c:v","libx264","-preset","medium","-crf","18","-pix_fmt","yuv420p",
             "-threads","2","-movflags","+faststart",str(target)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE);cap=cv2.VideoCapture(str(video))
    frame_map=[]
    try:
        for i in range(count):
            ok,bgr=cap.read()
            if not ok: raise RuntimeError("Source video ended early")
            t=(first+i*stride)*dt
            current=[e for e in events if e["time_s"]<=t+1e-10]
            phase=current[-1]["phase"] if current else ""
            if "stationary hold" in phase: label=f"提起后保持（本次额外观察 {probe['additional_hold_s']:.1f} 秒）"
            elif phase.startswith("04 Lift") or phase.startswith("11 Lift"): label="双手夹持检查通过后，提起布料"
            elif "confirmed" in phase: label="双手原夹持检查均通过"
            elif "Ease compressed" in phase: label="保持已夹持的一侧，小幅调整另一侧接触"
            elif "Grasp contact" in phase or "close with contact" in phase: label="到位接触后，逐步闭合并检查夹持"
            elif "open fingers" in phase: label="保持夹爪张开，下降至布角接触"
            else: label="双臂接近布料，保持夹爪张开"
            canvas=Image.new("RGB",(width,height+112),"#14222b")
            canvas.paste(Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)),(0,72));draw=ImageDraw.Draw(canvas)
            draw.text((15,7),title,font=font,fill="white");draw.text((15,42),f"{label}  |  仿真时间 {t:.3f}s",font=small,fill="#deebf2")
            draw.text((15,height+82),footer,font=small,fill="#deebf2")
            process.stdin.write(np.asarray(canvas).tobytes())
            frame_map.append([i,i,first+i*stride,f"{t:.9f}"])
        process.stdin.close()
        if process.wait()!=0: raise RuntimeError("Video encoding failed")
    finally:
        cap.release()
        if process.poll() is None:
            process.kill();process.wait()
    if sha(video)!=before: raise RuntimeError("Original source changed during review")
    cap=cv2.VideoCapture(str(target));actual=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
    if actual!=count: raise RuntimeError("Output lost or duplicated source frames")
    mapping=target.with_suffix(".frames.csv")
    with mapping.open("w",newline="") as stream:
        writer=csv.writer(stream);writer.writerow(["output_frame_index","source_frame_index","simulation_step","simulation_time_s"]);writer.writerows(frame_map)
    provenance={"source_video":str(video),"source_sha256":before,"output_video":str(target),"output_sha256":sha(target),
       "probe_mode":probe["mode"],"resumed_from":probe.get("resumed_from"),"source_frames":count,"output_frames":actual,
       "output_fps_rational":fps,"duration_s":float(count/Fraction(fps)),"playback_speed_relative_to_source":1.,
       "simulation_time_range_s":[first*dt,(first+(count-1)*stride)*dt],"source_frame_map":str(mapping),"source_frame_map_sha256":sha(mapping),
       "generated_simulation_frames":False,"interpolated_frames":False,"duplicated_frames":False,"source_image_crop_or_scale":False,
       "only_change":"Chinese text panels outside the source frame, then lossy video reencoding at original source fps.",
       "scope":"Current native pickup diagnostic only, not a fresh full wiping task or four-second settling/material acceptance."}
    target.with_suffix(".provenance.json").write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(provenance,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
