"""buoy_mapper_node — detections in, a map of Task 1 buoys and their states out.

    ros2 run uav_perception buoy_mapper

  in   /uav/perception/buoy_detections   (detector_node; each frame's boxes WITH
                                          the pose camera_node recorded for it)
       /uav/fcu_status                   (disarm = write the map files now)
  out  /uav/perception/buoy_map          (the WHOLE map, map_rate_hz)
       /uav/perception/clear_buoy_map    (std_srvs/Trigger: save, then start fresh)
       http://<jetson>:<http_port>/      (GET only: /state, /buoys.{csv,kml,plan,json})

All the logic lives in mapping_core, geolocate, buoy_tracker and map_export,
which are pure and shared with tools/scripts/map_session.py. This file is only
the wiring: which topic feeds what, when files are written, and a lock.

NO TUNING SURFACE, AND THAT IS A REQUIREMENT, not an omission. The operator
flies alone with a controller in hand; a map that needs adjusting in the air is
a map that is wrong in the air. Every constant is read-only and MEASURED once
(field of view, gimbal yaw sign and mount offset, launch height above the
surface). If a map is off, the evidence is on the map itself -- each buoy's
spread -- and the fix is replaying the recorded sightings with a corrected
constant after landing, not a slider.

WHAT IS WRITTEN, into export_dir, under a stem named for when this map began:

  <stem>_sightings.csv   every detection, every input, and what became of it.
                         The replay tool's input. Flushed every tick.
  <stem>_buoys.csv/.kml/.plan/.json
                         the map, rewritten when it changes (at most every
                         export_min_interval_s) and immediately on disarm, so the
                         files on disk are the final map of the sortie.

Clearing the map writes the old map one last time and starts a new stem, so a
clear is never how a map is lost.
"""
import os
import threading
import time
from datetime import datetime, timezone

from rclpy.node import Node
from std_srvs.srv import Trigger

from uav_msgs.msg import Buoy, BuoyDetections, BuoyMap, FcuStatus

from uav_camera.recorder_core import session_stem
from uav_common import config as uav_config
from uav_common.node_main import run_node
from uav_common.param_utils import declare_from_config
from uav_perception import map_export
from uav_perception.buoy_tracker import TrackerConfig
from uav_perception.geolocate import Projector
from uav_perception.map_server import MapServer
from uav_perception.mapping_core import (
    PROJECTOR_KEYS, TRACKER_KEYS, USED, Mapper, nan_to_none, sighting_rows,
    sightings_header)

PARAM_SPEC = {
    # ---- geometry (geolocate.Projector). Measured constants, not knobs.
    "hfov_deg": dict(read_only=True, lo=10.0, hi=170.0,
                     description="camera horizontal field of view, MEASURED "
                                 "(81 on the A8 mini); focal length follows "
                                 "from each frame's width"),
    "nadir_pitch_deg": dict(read_only=True, lo=-180.0, hi=180.0,
                            description="= camera_node.gimbal_pitch_deg"),
    "max_off_nadir_deg": dict(read_only=True, lo=0.5, hi=45.0,
                              description="frames with the gimbal further "
                                          "off nadir than this are not mapped"),
    "gimbal_yaw_mode": dict(read_only=True,
                            description="body | earth | aircraft: what the "
                                        "gimbal's yaw is measured against. "
                                        "Check once on the bench"),
    "gimbal_yaw_sign": dict(read_only=True, lo=-1.0, hi=1.0,
                            description="+1 or -1: the SIYI yaw direction on "
                                        "THIS unit. Check once on the bench"),
    "mount_yaw_offset_deg": dict(read_only=True, lo=-180.0, hi=180.0,
                                 description="fixed angle between the gimbal's "
                                             "yaw zero and the nose. Measured "
                                             "once; replay solves for it"),
    "launch_height_above_surface_m": dict(read_only=True, lo=-50.0, hi=50.0,
                                          description="how far home sits above "
                                                      "the surface buoys float "
                                                      "on. 0 on grass; the dock "
                                                      "height on water"),
    "target_height_m": dict(read_only=True, lo=0.0, hi=5.0,
                            description="height of the buoy top above that "
                                        "surface"),
    "min_alt_m": dict(read_only=True, lo=0.0, hi=100.0,
                      description="no mapping below this altitude above home"),
    "max_pose_age_s": dict(read_only=True, lo=0.05, hi=5.0,
                           description="older pose = frame not mapped"),
    "max_gimbal_age_s": dict(read_only=True, lo=0.05, hi=5.0,
                             description="older gimbal reading = not mapped"),
    "max_gimbal_yaw_rate_dps": dict(read_only=True, lo=1.0, hi=720.0,
                                    description="camera turning faster than "
                                                "this = not mapped"),
    # ---- state and association (buoy_tracker.TrackerConfig)
    "min_conf": dict(read_only=True, lo=0.05, hi=0.99,
                     description="detections below this confidence are "
                                 "not mapped"),
    "assoc_radius_m": dict(read_only=True, lo=0.2, hi=20.0,
                           description="a sighting this close to a buoy IS that "
                                       "buoy. Keep under half the closest buoy "
                                       "spacing (3 m gates -> 1.5)"),
    "merge_radius_m": dict(read_only=True, lo=0.1, hi=20.0,
                           description="two buoys this close are one; <= "
                                       "assoc_radius_m"),
    "min_sightings": dict(read_only=True, lo=1, hi=1000,
                          description="sightings before a buoy appears on the "
                                      "map"),
    "min_observe_s": dict(read_only=True, lo=0.5, hi=60.0,
                          description="full-view watch time before a state is "
                                      "decided (>= two 2 s flash periods)"),
    "min_samples": dict(read_only=True, lo=2, hi=1000,
                        description="full-view colour samples before a state is "
                                    "decided"),
    "max_sample_gap_s": dict(read_only=True, lo=0.1, hi=10.0,
                             description="gaps longer than this do not count as "
                                         "watch time"),
    "solid_min_lit": dict(read_only=True, lo=0.5, hi=1.0,
                          description="lit share at or above this = SOLID"),
    "off_max_lit": dict(read_only=True, lo=0.0, hi=0.5,
                        description="lit share at or below this = OFF"),
    "min_flash_transitions": dict(read_only=True, lo=1, hi=100,
                                  description="lit<->dark changes needed to "
                                              "call a buoy FLASHING"),
    "min_colour_agreement": dict(read_only=True, lo=0.0, hi=1.0,
                                 description="share of lit samples that must "
                                             "agree on the colour"),
    "lock_state": dict(read_only=True,
                       description="freeze a buoy's state once decided "
                                   "(Advanced tier). false for Disruptive"),
    # ---- output
    "map_rate_hz": dict(read_only=True, lo=0.1, hi=10.0,
                        description="BuoyMap publish rate"),
    "export_dir": dict(read_only=True,
                       description="where the sightings log and map files go"),
    "export_min_interval_s": dict(read_only=True, lo=0.5, hi=600.0,
                                  description="map files rewritten at most this "
                                              "often while changing"),
    "http_port": dict(read_only=True, lo=1024, hi=65535,
                      description="GET-only map server port; must differ from "
                                  "8090/8091/8092"),
    "bind_host": dict(read_only=True, description="map server bind address"),
    "waypoint_alt_m": dict(read_only=True, lo=1.0, hi=60.0,
                           description=".plan waypoint altitude above home"),
    "waypoint_hold_s": dict(read_only=True, lo=0.0, hi=120.0,
                            description=".plan hover time at each buoy"),
}

def _nan(v):
    return float("nan") if v is None else float(v)


class BuoyMapperNode(Node):

    def __init__(self):
        super().__init__("buoy_mapper")
        p = declare_from_config(self, uav_config.node_params("buoy_mapper"),
                                PARAM_SPEC)
        self.p = p
        self.mapper = Mapper(Projector(**{k: p[k] for k in PROJECTOR_KEYS}),
                             TrackerConfig(**{k: p[k] for k in TRACKER_KEYS}),
                             min_conf=float(p["min_conf"]))
        self._plan_opts = {"alt_m": float(p["waypoint_alt_m"]),
                           "hold_s": float(p["waypoint_hold_s"])}

        # Validated here, not at the first detection: an unwritable export_dir
        # found mid-flight is a sortie with no map files.
        self.export_dir = str(p["export_dir"])
        try:
            os.makedirs(self.export_dir, exist_ok=True)
            probe = os.path.join(self.export_dir, ".buoy_mapper_write_test")
            with open(probe, "w"):
                pass
            os.remove(probe)
        except OSError as e:
            raise ValueError("export_dir %r is not writable: %s"
                             % (self.export_dir, e)) from e

        # One lock for the mapper, the pending log rows and the stem: the ROS
        # executor writes them, the HTTP threads read them.
        self._lock = threading.Lock()
        self._pending = []
        self._dirty = False
        self._last_export = 0.0
        self._armed = None
        self._log = None
        self._stem = ""
        self._open_map()

        self.map_pub = self.create_publisher(BuoyMap, "/uav/perception/buoy_map", 10)
        self.create_subscription(BuoyDetections, "/uav/perception/buoy_detections",
                                 self._on_detections, 10)
        self.create_subscription(FcuStatus, "/uav/fcu_status", self._on_fcu, 10)
        self.create_service(Trigger, "/uav/perception/clear_buoy_map",
                            self._on_clear)
        self.create_timer(1.0 / float(p["map_rate_hz"]), self._tick)

        self.server = MapServer(self._snapshot, self._export)
        self.server.start(int(p["http_port"]), str(p["bind_host"]))
        self.get_logger().info(
            "buoy map %s: yaw mode %s sign %+.0f offset %+.1f deg, launch %+.2f m "
            "above surface; files in %s; downloads on :%d"
            % (self._stem, p["gimbal_yaw_mode"], p["gimbal_yaw_sign"],
               p["mount_yaw_offset_deg"], p["launch_height_above_surface_m"],
               self.export_dir, int(p["http_port"])))

    # ------------------------------------------------------------ files

    def _open_map(self):
        """Start a new stem and its sightings log. Caller holds no lock at init;
        _on_clear holds self._lock."""
        self._stem = session_stem(datetime.now(timezone.utc))
        path = os.path.join(self.export_dir, self._stem + "_sightings.csv")
        try:
            self._log = open(path, "w", buffering=1, encoding="utf-8")
            self._log.write(sightings_header() + "\n")
        except OSError as e:
            self._log = None
            self.get_logger().error(
                "cannot open %s: %s -- mapping continues, but this map "
                "cannot be replayed" % (path, e))

    def _flush_log(self, rows):
        if self._log is None or not rows:
            return
        try:
            self._log.write("\n".join(rows) + "\n")
        except OSError as e:
            self.get_logger().error("sightings log write failed: %s" % e,
                                    throttle_duration_sec=10.0)

    def _write_exports(self, buoys, stem):
        try:
            map_export.write_all(self.export_dir, stem, buoys, self._plan_opts)
        except OSError as e:
            self.get_logger().error("map export failed: %s" % e,
                                    throttle_duration_sec=10.0)

    # ------------------------------------------------------------ inputs

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_detections(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = nan_to_none({
            "lat": msg.latitude, "lon": msg.longitude,
            "alt_rel": msg.altitude_rel, "roll": msg.roll, "pitch": msg.pitch,
            "yaw": msg.yaw, "pose_age_s": msg.pose_age_s,
            "gimbal_pitch": msg.gimbal_pitch, "gimbal_yaw": msg.gimbal_yaw,
            "gimbal_yaw_rate": msg.gimbal_yaw_rate,
            "gimbal_age_s": msg.gimbal_age_s,
        })
        frame["session"] = msg.session
        frame["frame_idx"] = int(msg.frame_idx) if msg.frame_meta_ok else None
        dets = [{"class_name": d.class_name, "confidence": float(d.confidence),
                 "x0": float(d.x0), "y0": float(d.y0), "x1": float(d.x1),
                 "y1": float(d.y1), "full_view": bool(d.full_view)}
                for d in msg.detections]
        w, h = int(msg.image_width), int(msg.image_height)
        with self._lock:
            results = self.mapper.ingest(t, frame, dets, w, h)
            self._pending.extend(sighting_rows(t, frame, dets, w, h, results))
            if any(r["outcome"] == USED for r in results):
                self._dirty = True

    def _on_fcu(self, msg):
        """Disarm writes the map straight away: the files on disk after landing
        are then the sortie's final map, whether or not anyone downloads them."""
        armed = bool(msg.armed)
        if self._armed and not armed:
            buoys, stem = self._save()
            self.get_logger().info("disarmed: map %s written (%d buoys)"
                                   % (stem, len(buoys)))
        self._armed = armed

    def _map_now(self):
        """(buoys, stem) as they stand. Caller holds self._lock."""
        return self.mapper.buoys(self._now_s()), self._stem

    def _save(self):
        """Flush the sightings log and write the map files. -> (buoys, stem)."""
        with self._lock:
            buoys, stem = self._map_now()
            rows, self._pending = self._pending, []
            self._dirty = False
        self._flush_log(rows)
        self._write_exports(buoys, stem)
        return buoys, stem

    def _on_clear(self, request, response):
        # All under ONE lock hold, unlike _save: a detection landing between
        # "saved" and "cleared" would otherwise be in neither map.
        with self._lock:
            buoys, old = self._map_now()
            rows, self._pending = self._pending, []
            self._flush_log(rows)
            self._write_exports(buoys, old)
            if self._log is not None:
                self._log.close()
            self.mapper.clear()
            self._dirty = False
            self._open_map()
            new = self._stem
        response.success = True
        response.message = ("map %s saved (%d buoys) and cleared; new map %s"
                            % (old, len(buoys), new))
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------ output

    def _tick(self):
        with self._lock:
            buoys, stem = self._map_now()
            stats = self.mapper.stats()
            rows, self._pending = self._pending, []
            export = (self._dirty and time.monotonic() - self._last_export
                      >= float(self.p["export_min_interval_s"]))
            if export:
                self._dirty = False
                self._last_export = time.monotonic()
        self._flush_log(rows)
        if export:
            self._write_exports(buoys, stem)

        msg = BuoyMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.export_stem = stem
        msg.detections_used = stats["detections_used"]
        msg.detections_partial = stats["detections_partial"]
        msg.detections_low_conf = stats["detections_low_conf"]
        msg.frames_rejected = stats["frames_rejected"]
        msg.last_reject_reason = stats["last_reject_reason"]
        for b in buoys:
            m = Buoy()
            m.id, m.latitude, m.longitude = int(b["id"]), b["lat"], b["lon"]
            m.state, m.colour, m.label = b["state"], b["colour"], b["label"]
            m.locked = bool(b["locked"])
            m.lit_fraction = float(b["lit_fraction"])
            m.colour_agreement = float(b["colour_agreement"])
            m.samples = int(b["samples"])
            m.flash_transitions = int(b["flash_transitions"])
            m.observed_s = float(b["observed_s"])
            m.sightings = int(b["sightings"])
            m.spread_m = float(b["spread_m"])
            m.last_seen_age_s = _nan(b["last_seen_age_s"])
            msg.buoys.append(m)
        self.map_pub.publish(msg)

    def _snapshot(self):
        with self._lock:
            buoys, stem = self._map_now()
            return {"stem": stem, "buoys": buoys, "stats": self.mapper.stats()}

    def _export(self, fmt):
        if fmt not in map_export.FORMATS:
            return None
        with self._lock:
            buoys, stem = self._map_now()
        ctype, text = map_export.render(fmt, buoys, stem, self._plan_opts)
        return ctype, text, "%s_buoys.%s" % (stem, fmt)

    def destroy_node(self):
        """Write the map one last time on every exit path."""
        try:
            self._save()
            if self._log is not None:
                self._log.close()
        except Exception:
            pass
        try:
            self.server.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(BuoyMapperNode, args=args)


if __name__ == "__main__":
    main()
