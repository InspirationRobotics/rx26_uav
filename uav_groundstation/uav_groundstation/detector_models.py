"""Which model detector_node loads -- the Camera tab's Model choice.

No ROS imports, so bench_gcs exercises these rules and not a copy of them.

detector_node takes ONE model and reads it once, at start-up. Switching is a
restart with a launch override, `--ros-args -p model_path:=...`, which wins
over the YAML (uav_common.param_utils.declare). Nothing here edits a file, so a
ground-station restart or a reboot always comes back to the YAML's model.

THE YAML's detector_node.model_path IS THE BUOY MODEL. It is the one
buoy_mapper and search_node are built on. Every other model (the Task 2/3 tins
and delivery circles, first) is for looking at and recording over, never for
mapping: buoy_mapper keeps every class the detector sends it, so with another
model loaded a tin becomes a buoy, and the buoy map is what Crusader steers by.
Both directions are therefore refused here: no mapping node starts while
another model is chosen, and no other model is chosen while one is running.
"""
import os

MAPPING_NODES = ("buoy_mapper", "search_node")


def parse(entries, buoy_path):
    """[{"label", "path", "buoy"}] from the YAML's 'Label = /path' strings.

    -> (models, problems). The buoy model is always offered, and always first,
    labelled from its entry or else by file name: a list that forgot it must
    never leave the page without a way back to it. A bad entry is reported and
    skipped rather than raised -- a typo in a label must not take the ground
    station down with it.
    """
    models, problems, seen = [], [], set()
    for e in entries or []:
        label, sep, path = str(e).partition("=")
        label, path = label.strip(), path.strip()
        if not (sep and label and path):
            problems.append("detector_models entry %r: expected "
                            "'Label = /path/to/model.pt'" % e)
            continue
        if path in seen:
            problems.append("detector_models lists %s twice" % path)
            continue
        seen.add(path)
        models.append({"label": label, "path": path, "buoy": path == buoy_path})
    if buoy_path not in seen:
        models.append({"label": os.path.basename(buoy_path), "path": buoy_path,
                       "buoy": True})
    models.sort(key=lambda m: not m["buoy"])          # stable: buoy model first
    return models, problems


def label_of(path, models):
    for m in models:
        if m["path"] == path:
            return m["label"]
    return os.path.basename(path or "") or "?"


def other_model(chosen, buoy_path, models):
    """The chosen model's label when it is NOT the buoy model, else None --
    the pre-flight checklist's cue that no mapping this flight is deliberate."""
    return None if chosen == buoy_path else label_of(chosen, models)


def launch_args(path):
    """The override appended to `ros2 run uav_perception detector_node`."""
    return ["--ros-args", "-p", "model_path:=%s" % path]


def mapping_refusal(name, chosen, buoy_path, models):
    """Why `name` may not start with `chosen` as the detector's model, or ""."""
    if name in MAPPING_NODES and chosen != buoy_path:
        return ("%s maps BUOYS, and the detector's model is set to %s. Pick "
                "the buoy model on the Camera tab first: with this one a tin "
                "or a circle would land on the buoy map."
                % (name, label_of(chosen, models)))
    return ""


def switch_refusal(path, buoy_path, models, running):
    """Why the detector may not be switched to `path` now, or ""."""
    if path not in {m["path"] for m in models}:
        # Only the YAML's list is loadable: this endpoint never takes a file
        # path from a browser and hands it to a process.
        return "that is not one of the models in detector_models"
    busy = [n for n in MAPPING_NODES if n in running]
    if path != buoy_path and busy:
        return ("%s %s running and would map whatever %s finds as buoys. Stop "
                "%s first." % (" and ".join(busy), "is" if len(busy) == 1
                                else "are", label_of(path, models),
                                " and ".join(busy)))
    return ""
