"""siyi_client — gimbal pointing, attitude readback, and SD-card recording.

WHY THIS IS AN ADAPTER AND NOT A PROTOCOL IMPLEMENTATION. SIYI's wire format is
simple enough to write from scratch:

    STX(2)=0x5566  CTRL(1)  DATALEN(2)  SEQ(2)  CMD_ID(1)  DATA(N)  CRC16(2)
    little-endian, UDP port 37260

but the COMMAND ID TABLE is not published. SIYI removed the SDK command section
from the A8 mini manual at v1.8 and does not ship a standalone protocol document
-- the Download Centre carries only the user manual and a firmware pack. Writing
IDs from guesswork would produce a client that either silently does nothing or
sends a wrong command to a gimbal on an airframe, and the failure would look
like a hardware fault.

So the wire protocol comes from `mzahana/siyi_sdk` (MIT), which carries verified
IDs for the A8 mini, and this file is the seam between that library and the node:
the node names four operations, and only this file knows how they are spelled.

VENDOR OR PIN THE LIBRARY. Do not track its default branch -- this sits on a
flight computer, and an upstream rename between two `pip install`s is not a
thing to discover on a flight line.

NO TRACKING LOOP. set_angles() points the gimbal where it is told, once, and
returns. There is no loop that keeps aiming at a moving thing: the gimbal holds
its own attitude against its IMU, and Tasks 1 and 3 need a fixed view and a
lat/lon per detection, not a camera that follows a target. Commanding an angle
is a discrete act; tracking would be a control path and is a separate argument.

WHO MAY CALL set_angles. The node exposes it on a topic only when
gimbal_control_enabled is true, which is false in the flight config. See
camera_node's header -- this file is happy to point, and the decision about
whether anything is allowed to ask lives there.

NADIR IS NOT ALWAYS -90. SIYI documents -90 as full-down and +25 as full-up,
and units in the field have been found reporting the opposite sign, so
commanding -90 aims at the sky. The angle is therefore a PARAMETER, not a
constant: uav_params.yaml carries the value measured on the airframe it is
bolted to. The default below is SIYI's documented convention and is the thing
most likely to be wrong for any given unit.
"""

import socket
import struct
import time

NADIR_PITCH_DEG = -90.0

# Absolute-angle command. Payload is yaw then pitch, int16 little-endian, in
# TENTHS of a degree. Identified from mzahana/siyi_sdk and confirmed against
# this airframe's unit with tools/siyi_gimbal.py.
CMD_SET_ATTITUDE = 0x0E


def _crc16_xmodem(data):
    """SIYI's frame checksum: CRC16-XMODEM, poly 0x1021, init 0x0000.

    Not documented as such anywhere SIYI publishes; identified by testing the
    ten common CRC16 variants against a frame from their own manual, of which
    exactly one matched. A wrong variant does not error -- the gimbal simply
    ignores every frame, which looks like a dead camera.
    """
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc

# The gimbal's travel puts nadir exactly on a mechanical stop, whichever sign
# convention the unit uses. A gimbal that reports -87 while commanded to -90 is
# not broken, it is against the limit with a calibration offset -- and
# geo-projection has to use what it reports, not what it was told. This is the
# tolerance beyond which the node says the gimbal is not where it was asked.
NADIR_TOLERANCE_DEG = 2.0


class SiyiUnavailable(RuntimeError):
    """The SDK is not installed, or the camera did not answer."""


class SiyiClient:
    """Thin, synchronous adapter over siyi_sdk.

    Every method returns a value or None/False rather than raising, because the
    node treats a silent gimbal as a degraded-but-flyable condition: the camera
    still streams and still records with the gimbal wherever it happens to be,
    and losing the footage because the pointing link dropped would be the wrong
    trade. Construction is the exception -- see connect().
    """

    def __init__(self, ip="192.168.144.25", port=37260, timeout_s=3.0,
                 nadir_pitch_deg=NADIR_PITCH_DEG):
        self.ip = ip
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        # Carried per instance, not read from the module constant at call time,
        # so at_nadir() can never check against a different angle from the one
        # set_nadir() commanded. That pair going out of step would report a
        # healthy gimbal as misaimed, or worse, the reverse.
        self.nadir_pitch_deg = float(nadir_pitch_deg)
        self._sdk = None

    # ------------------------------------------------------------------ life

    def connect(self):
        """Open the UDP session. Raises SiyiUnavailable with a fix in the text.

        The import is here rather than at module scope so this file imports on a
        laptop without the SDK, and so the node's own import errors stay legible
        -- the same reason telemetry_bridge imports pymavlink inside __init__.
        """
        try:
            from siyi_sdk import SIYISDK
        except ImportError as e:
            raise SiyiUnavailable(
                "siyi_sdk is not importable (%s).\n"
                "  It is not on PyPI. Vendor it, or install a PINNED commit:\n"
                "    pip install "
                "'git+https://github.com/mzahana/siyi_sdk@<commit>'\n"
                "  Set siyi_enabled=false in uav_params.yaml to run the camera "
                "without gimbal control." % e) from e
        try:
            self._sdk = SIYISDK(server_ip=self.ip, port=self.port)
            if not self._sdk.connect():
                raise SiyiUnavailable(
                    "no answer from the gimbal at %s:%d.\n"
                    "  The RTSP stream and the SDK are separate paths -- video "
                    "can work while this does not.\n"
                    "  Check the camera is on this subnet and that nothing else "
                    "holds the SDK port." % (self.ip, self.port))
        except SiyiUnavailable:
            raise
        except Exception as e:
            raise SiyiUnavailable(
                "siyi_sdk failed to connect to %s:%d: %s" % (self.ip, self.port, e)
            ) from e
        return self

    def close(self):
        if self._sdk is not None:
            try:
                self._sdk.disconnect()
            except Exception:
                pass
            self._sdk = None

    @property
    def connected(self) -> bool:
        return self._sdk is not None

    # --------------------------------------------------------------- pointing

    def set_angles(self, yaw_deg, pitch_deg) -> bool:
        """Point at an absolute yaw and pitch, in degrees. One command.

        NOT VALIDATED AGAINST A RANGE HERE, deliberately. The documented travel
        is -90..+25 in pitch, but that assumes SIYI's sign convention and units
        have been found using the opposite one -- a clamp written against the
        wrong convention silently refuses the only angle that works. The gimbal
        stops at its own mechanical limits regardless, and attitude() reports
        where it actually ended up, which is the check that means something.

        WHY NOT self._sdk.setGimbalRotation(). That helper validates against a
        HARDCODED -90..+25 pitch range in SIYI's documented sign convention, and
        this airframe's unit uses the opposite one -- so the only angle that
        actually aims at the water, +90, is the one the helper refuses. Worse,
        it refuses by PRINTING and returning, not by raising, so the caller sees
        success while the gimbal never moved. The command frame itself is four
        bytes of payload and is sent here directly; the gimbal's own mechanical
        stops are the real limit, and attitude() reports where it ended up.
        """
        if self._sdk is None:
            return False
        data = struct.pack("<hh", int(round(float(yaw_deg) * 10)),
                           int(round(float(pitch_deg) * 10)))
        return self._command(CMD_SET_ATTITUDE, data) is not None

    # ------------------------------------------------------------ wire format

    def _command(self, cmd_id, data=b"", seq=1):
        """Send one SIYI frame and return the reply payload, or None.

        Its own short-lived socket rather than the SDK's: the SDK owns a
        receive thread on its socket, and a reply read from under it would be a
        reply the SDK never sees. Sending from a separate ephemeral port keeps
        the two conversations disjoint -- the gimbal answers whoever asked.

        MATCHING THE RETURNED CMD_ID IS NOT OPTIONAL. The gimbal also emits
        unsolicited attitude frames, and their CRC is valid, so accepting the
        first datagram that arrives yields a well-formed reply to a question
        nobody asked.
        """
        body = (b"\x55\x66\x01" + struct.pack("<H", len(data))
                + struct.pack("<H", seq) + bytes([cmd_id]) + data)
        pkt = body + struct.pack("<H", _crc16_xmodem(body))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.settimeout(self.timeout_s)
            deadline = time.monotonic() + self.timeout_s
            sock.sendto(pkt, (self.ip, self.port))
            while time.monotonic() < deadline:
                try:
                    reply, _ = sock.recvfrom(1024)
                except socket.timeout:
                    return None
                if len(reply) < 10 or reply[:2] != b"\x55\x66":
                    continue
                n = struct.unpack("<H", reply[3:5])[0]
                if len(reply) < 10 + n or reply[7] != cmd_id:
                    continue
                return reply[8:8 + n]
            return None
        except OSError:
            return None
        finally:
            sock.close()

    def set_nadir(self) -> bool:
        """Point straight down. One command; the gimbal's own IMU holds it.

        This is called once at startup and after a reconnect, NOT on a timer.
        The A8 mini is 3-axis stabilised against its own IMU, so it holds nadir
        through aircraft roll and pitch without being told again -- re-commanding
        it every tick would add a control loop that has nothing to correct.
        """
        return self.set_angles(0.0, self.nadir_pitch_deg)

    def attitude(self):
        """-> (yaw, pitch, roll) in degrees, MEASURED, or None.

        The node publishes pitch from here rather than the commanded -90 because
        geo-projection turns a pixel into a lat/lon using this angle: three
        degrees of error is about a metre on the water at 20 m, and substituting
        the command would hide exactly the error the projection cannot absorb.
        """
        if self._sdk is None:
            return None
        try:
            att = self._sdk.getAttitude()
        except Exception:
            return None
        if att is None:
            return None
        try:
            yaw, pitch, roll = (float(att[0]), float(att[1]), float(att[2]))
        except (TypeError, ValueError, IndexError):
            return None
        return yaw, pitch, roll

    def at_nadir(self, tolerance_deg=NADIR_TOLERANCE_DEG):
        """-> (is_nadir, measured_pitch). measured_pitch is None if unknown.

        Compared against THIS INSTANCE's nadir angle, so a unit configured with
        an inverted sign is judged against the angle it was actually sent.
        """
        att = self.attitude()
        if att is None:
            return False, None
        pitch = att[1]
        return abs(pitch - self.nadir_pitch_deg) <= tolerance_deg, pitch

    # -------------------------------------------------------------- recording

    def start_recording(self) -> bool:
        """Start recording to the camera's own SD card.

        Independent of the Jetson-side recording on purpose: the two fail
        differently. A Jetson crash costs the .mkv and keeps the card; a missing
        or full card costs the card and keeps the .mkv. Running both is why a
        sortie is unlikely to come back with nothing.
        """
        if self._sdk is None:
            return False
        try:
            self._sdk.requestRecording()
            return True
        except Exception:
            return False

    def stop_recording(self) -> bool:
        """Stop SD recording. Same toggle command as start on this firmware."""
        if self._sdk is None:
            return False
        try:
            self._sdk.requestRecording()
            return True
        except Exception:
            return False


class NullSiyiClient:
    """Stands in when siyi_enabled is false, or on a bench with no camera.

    Exists so the node has no `if self.siyi is None` branches: an absent gimbal
    is a client that politely answers "no" to everything, which is also what a
    present-but-silent one does. One code path, exercised either way.
    """

    connected = False

    def __init__(self, *args, **kwargs):
        # Same constructor shape as SiyiClient so the node can build either
        # without knowing which it got.
        self.nadir_pitch_deg = float(kwargs.get("nadir_pitch_deg",
                                                NADIR_PITCH_DEG))

    def connect(self):
        return self

    def close(self):
        pass

    def set_angles(self, yaw_deg, pitch_deg) -> bool:
        return False

    def set_nadir(self) -> bool:
        return False

    def attitude(self):
        return None

    def at_nadir(self, tolerance_deg=NADIR_TOLERANCE_DEG):
        return False, None

    def start_recording(self) -> bool:
        return False

    def stop_recording(self) -> bool:
        return False
