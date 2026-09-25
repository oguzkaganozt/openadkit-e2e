#!/usr/bin/env python3
"""Sample rig imagery at a fixed rate for log correlation.

Run on the rig host (CARLA venv) next to a fresh-world run. Every period it
saves the latest frame of each source as <out>/<src>/<wall_ms>.jpg and one row
in <out>/index.csv, so a log line at wall time T maps to the nearest image:
- front: the bridge's ego camera preview (http://127.0.0.1:8090/stream);
- vp:    the VP HUD with its lane overlay (http://127.0.0.1:8080/mjpeg),
         present only with a viewer config;
- chase: optional (--chase) third-person camera behind and above the hero,
         the one view that shows lane position and guardrails directly.

The HTTP sources are read-only. --chase attaches one extra RGB sensor to the
hero (sensor_tick = period, small resolution): it never ticks the world or
writes control, but it is an actor in the world and costs CARLA render time,
so record whether it was on when comparing timing-sensitive runs.
"""

import argparse
import csv
import os
import threading
import time
import urllib.request


class MjpegReader(threading.Thread):
    """Keep the latest JPEG of a multipart MJPEG stream; reconnect on loss."""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.latest = None
        self.lock = threading.Lock()

    def run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as stream:
                    buf = b""
                    while True:
                        chunk = stream.read(16384)
                        if not chunk:
                            break
                        buf += chunk
                        # Frames are delimited by JPEG SOI/EOI markers; this
                        # works for both the bridge and the VP boundary format.
                        end = buf.rfind(b"\xff\xd9")
                        start = buf.rfind(b"\xff\xd8", 0, end) if end > 0 else -1
                        if start >= 0:
                            with self.lock:
                                self.latest = buf[start:end + 2]
                            buf = buf[end + 2:]
                        elif len(buf) > 4_000_000:
                            buf = b""
            except OSError:
                pass
            time.sleep(1.0)

    def take(self):
        with self.lock:
            frame, self.latest = self.latest, None
        return frame


def attach_chase(host, port, period, width, height, out_dir):
    import carla

    client = carla.Client(host, port)
    client.set_timeout(3.0)
    world = None
    for _ in range(240):
        try:
            world = client.get_world()
            break
        except RuntimeError:
            time.sleep(0.5)
    if world is None:
        raise SystemExit("CARLA server not reachable")
    hero = None
    for _ in range(600):
        hero = next((a for a in world.get_actors().filter("vehicle.*")
                     if a.attributes.get("role_name") == "hero"), None)
        if hero is not None:
            break
        time.sleep(0.5)
    if hero is None:
        raise SystemExit("no hero vehicle in world")
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", "90")
    bp.set_attribute("sensor_tick", str(period))
    # Far and shallow enough to show the hero, its lane markings and ~40 m of
    # road ahead (a steeper view hides the upcoming curve).
    pose = carla.Transform(carla.Location(x=-12.0, z=7.0), carla.Rotation(pitch=-18.0))
    sensor = world.spawn_actor(bp, pose, attach_to=hero,
                               attachment_type=carla.AttachmentType.SpringArmGhost)
    rows = []
    lock = threading.Lock()

    def on_image(image):
        # The CARLA venv has no numpy/cv2: let CARLA encode the JPEG itself.
        # sensor_tick already limits this callback to one image per period.
        wall_ms = int(time.time() * 1000)
        rel = os.path.join("chase", "%d.jpg" % wall_ms)
        image.save_to_disk(os.path.join(out_dir, rel))
        with lock:
            rows.append((wall_ms, "chase", rel, "%.3f" % image.timestamp))

    sensor.listen(on_image)

    def drain():
        with lock:
            taken = rows[:]
            rows.clear()
        return taken

    return sensor, drain


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--front-url", default="http://127.0.0.1:8090/stream")
    parser.add_argument("--vp-url", default="http://127.0.0.1:8080/mjpeg")
    parser.add_argument("--chase", action="store_true", help="attach a chase camera")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--size", default="800x450", help="chase camera WxH")
    args = parser.parse_args()

    readers = {}
    for name, url in (("front", args.front_url), ("vp", args.vp_url)):
        if url:
            readers[name] = MjpegReader(url)
            readers[name].start()
    for name in list(readers) + (["chase"] if args.chase else []):
        os.makedirs(os.path.join(args.out, name), exist_ok=True)
    sensor = drain_chase = None
    if args.chase:
        width, height = (int(v) for v in args.size.split("x"))
        sensor, drain_chase = attach_chase(args.host, args.port, args.period,
                                           width, height, args.out)

    counts = dict.fromkeys(list(readers) + (["chase"] if args.chase else []), 0)
    deadline = time.monotonic() + args.seconds
    next_at = time.monotonic()
    try:
        with open(os.path.join(args.out, "index.csv"), "w", newline="",
                  encoding="utf-8") as index:
            writer = csv.writer(index)
            writer.writerow(("wall_ms", "source", "file", "sim_s"))
            while time.monotonic() < deadline:
                next_at += args.period
                time.sleep(max(0.0, next_at - time.monotonic()))
                wall_ms = int(time.time() * 1000)
                for name, reader in readers.items():
                    frame = reader.take()
                    if not frame:
                        continue
                    rel = os.path.join(name, "%d.jpg" % wall_ms)
                    with open(os.path.join(args.out, rel), "wb") as f:
                        f.write(frame)
                    writer.writerow((wall_ms, name, rel, ""))
                    counts[name] += 1
                if drain_chase:
                    for row in drain_chase():
                        writer.writerow(row)
                        counts["chase"] += 1
                index.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if sensor is not None:
            sensor.stop()
            sensor.destroy()
    print("frames: " + " ".join("%s=%d" % kv for kv in counts.items()), flush=True)


if __name__ == "__main__":
    main()
