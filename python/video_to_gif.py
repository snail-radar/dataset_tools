
# save as video_to_gif.py
import cv2
import imageio.v2 as imageio # pip install "imageio[ffmpeg]"
import numpy as np
import argparse
import math

def mp4_segment_to_gif(in_path, out_path, start_s=0.0, duration_s=10.0, fps_out=12, max_width=640):
    cap = cv2.VideoCapture(in_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {in_path}")

    fps_in   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    start_f  = max(0, int(round(start_s * fps_in)))
    end_f    = n_frames if duration_s <= 0 else min(n_frames, int(round((start_s + duration_s) * fps_in)))
    total_in = max(0, end_f - start_f)
    if total_in == 0:
        raise ValueError("No frames in the requested segment (check start/duration).")

    # Seek to start (note: some codecs seek to nearest keyframe)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_f)

    # Sample frames to match desired output FPS
    step = max(1.0, fps_in / float(fps_out))  # input frames per 1 output frame
    next_take = 0.0
    taken = 0
    out_frames = []

    # Read frames until end_f
    while True:
        cur_f = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        if cur_f >= end_f:
            break
        ret, frame_bgr = cap.read()
        if not ret:
            break

        rel = cur_f - start_f
        if rel >= next_take - 1e-6:
            # BGR -> RGB
            frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            # Optional resize to control file size
            if max_width and frame.shape[1] > max_width:
                scale = max_width / frame.shape[1]
                new_wh = (max_width, int(round(frame.shape[0] * scale)))
                frame = cv2.resize(frame, new_wh, interpolation=cv2.INTER_AREA)
            out_frames.append(frame)
            taken += 1
            next_take += step

    cap.release()

    if not out_frames:
        raise RuntimeError("No frames collected—try increasing duration or adjusting start time.")

    # Write GIF (duration per frame in seconds)
    imageio.mimsave(out_path, out_frames, duration=1.0 / float(fps_out), loop=0)
    print(f"Saved GIF: {out_path}  | frames={len(out_frames)}, fps={fps_out}")

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Trim an MP4 segment and convert to GIF.")
    p.add_argument("input", help="Input video file (e.g., input.mp4)")
    p.add_argument("output", help="Output GIF file (e.g., clip.gif)")
    p.add_argument("--start", type=float, default=0.0, help="Start time in seconds")
    p.add_argument("--dur",   type=float, default=10.0, help="Duration in seconds (<=0 means to end)")
    p.add_argument("--fps",   type=float, default=12,   help="Output GIF FPS")
    p.add_argument("--width", type=int,   default=640,  help="Max output width (px), 0=keep original")
    args = p.parse_args()

    mp4_segment_to_gif(args.input, args.output, args.start, args.dur, args.fps, args.width)
