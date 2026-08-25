"""power_client — ask the HOST to power down, from inside the container.

WHY THIS IS NOT `subprocess.run(["sudo", "shutdown"])`. The ground station runs
inside `uav_ekko`. A container shares the host's kernel but has no init of its own to
talk to, so `shutdown` and `systemctl poweroff` inside it either fail outright
or, if the container is privileged enough, do something worse than nothing. The
container genuinely cannot power off the machine, and no amount of sudo changes
that.

The alternative usually reached for is running the container --privileged with
the host PID namespace, which does work — by handing every process in the
container root on the Jetson. That is a large permanent grant to buy one
button, and it applies to the MAVLink bridge too.

SO THE REQUEST IS A FILE. This writes <request_dir>/shutdown.request into the
workspace bind mount; systemd .path units on the host watch for exactly those
two names and run a oneshot that deletes the file and powers off.

  tools/systemd/uav-shutdown.path      watches
  tools/systemd/uav-shutdown.service   acts
  tools/systemd/uav-reboot.path / .service

THE PRIVILEGED SURFACE IS THE SET OF FILENAMES, and it has two members. Nothing
this module writes is ever executed, parsed, or interpolated into a command —
the host side runs a fixed argv compiled into the unit file, and the only thing
it reads from the request is that it exists. The `reason` written into the body
is for a human running `cat` after the fact; no code on either side reads it.

WHY NOT A UNIX SOCKET, which is what this was until 2026-08-24: the socket
needed its own bind mount, and a bind mount of a FILE pins an inode — so every
restart of the helper daemon silently orphaned the container's end, leaving a
System tab that said "unreachable" with nothing in either journal explaining
why. Worse, when Docker was asked to mount a socket path that did not exist yet
it invented a DIRECTORY there, and the container then could not start at all.
The file-drop design has no socket, no daemon, no extra mount, and no group to
get wrong: the request rides the workspace mount that must already work for the
node's own code to be here. Borrowed from robotx_graey_2026, where it has flown.

THE ACKNOWLEDGEMENT IS THE FILE DISAPPEARING. The host unit deletes it before it
acts, so a request that vanishes was picked up by systemd — that is a stronger
signal than the old socket's reply, which only proved a daemon was listening. A
request still sitting there after the timeout means the .path unit is not
running, and this module says so rather than reporting a shutdown that will
never come.
"""
import os
import time

#: Container-side path. The host sees the same directory through the workspace
#: bind mount, at <workspace>/logs.
DEFAULT_REQUEST_DIR = "/root/robotx_ws/logs"
VERBS = ("shutdown", "reboot")

#: How long to wait for the host to consume the request. Generous: the unit is
#: oneshot and deletes first, so this normally resolves in well under a second,
#: and the cost of being wrong is telling an operator nothing happened when it
#: is about to.
ACK_TIMEOUT_S = 5.0
ACK_POLL_S = 0.1


class PowerUnavailable(Exception):
    """The host side is not reachable. Carries text meant for the operator."""


def _request_path(verb, request_dir):
    return os.path.join(request_dir, f"{verb}.request")


def available(request_dir=DEFAULT_REQUEST_DIR):
    """(ok, reason). Cheap enough to call on every /state poll."""
    if not os.path.isdir(request_dir):
        return False, (
            f"power request directory {request_dir} does not exist. The "
            "container cannot power off the Jetson on its own; it asks by "
            "dropping a file the host watches for. Run "
            "setup/install_jetson_host.sh, which creates the directory and "
            "installs uav-shutdown.path / uav-reboot.path.")
    if not os.access(request_dir, os.W_OK):
        return False, (
            f"power request directory {request_dir} is not writable, so the "
            "shutdown request cannot be dropped. Check the workspace bind "
            "mount and its ownership on the host.")
    return True, ""


def request(verb, request_dir=DEFAULT_REQUEST_DIR, reason=""):
    """Ask the host to run `verb`. Returns a short status string.

    The verb is validated HERE as well as being constrained by the host, which
    has a unit for each of exactly two filenames and no way to be talked into a
    third. Both ends checking is not redundant: this end gives a clear error
    before anything crosses a privilege boundary, and that end cannot be made
    to run something else no matter what reaches it.
    """
    if verb not in VERBS:
        raise ValueError(f"refusing unknown power verb {verb!r}; "
                         f"only {VERBS} exist")
    ok, why = available(request_dir)
    if not ok:
        raise PowerUnavailable(why)

    path = _request_path(verb, request_dir)
    # Clear any unconsumed request first, so systemd sees a genuine creation
    # rather than a file that was already sitting there. A leftover means the
    # .path unit was down when it was written, and re-using it would leave the
    # operator waiting on an event that already failed to fire once.
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    except OSError as e:
        raise PowerUnavailable(
            f"could not clear the previous {verb} request: {e}") from e

    try:
        with open(path, "w") as f:
            # Body is forensic only — nothing reads it back.
            f.write(f"{verb} requested at {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                    f"{reason[:200]}\n")
    except OSError as e:
        raise PowerUnavailable(f"could not write {path}: {e}") from e

    deadline = time.monotonic() + ACK_TIMEOUT_S
    while time.monotonic() < deadline:
        if not os.path.exists(path):
            return "accepted by the host (request consumed)"
        time.sleep(ACK_POLL_S)

    # Not consumed. Take it back rather than leaving a live request on disk for
    # the next boot to find — the tmpfiles.d sweep is the backstop, not the
    # plan.
    try:
        os.unlink(path)
    except OSError:
        pass
    raise PowerUnavailable(
        f"the host did not act on {path} within {ACK_TIMEOUT_S:.0f}s, so the "
        f"request was withdrawn. uav-{verb}.path is probably not running: "
        f"check `systemctl status uav-{verb}.path` on the Jetson.")
