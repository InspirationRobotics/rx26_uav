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

WHAT THIS DELIBERATELY DOES NOT DO. It does not aim the gimbal at anything.
Tasks 1 and 3 need a fixed nadir view and a lat/lon per detection; nothing needs
the camera to track. A pointing loop would be a control path and would have to
be argued against README safety constraint 6, so the absence is the design.
"""

NADIR_PITCH_DEG = -90.0

# The A8 mini's travel is -90..+25 in pitch, so nadir sits exactly on the stop.
# A gimbal that reports -87 while commanded to -90 is not broken, it is against
# the limit with a calibration offset -- and geo-projection has to use what it
# reports, not what it was told. This is the tolerance beyond which the node
# says the gimbal is not where it was asked to be.
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

    def __init__(self, ip="192.168.144.25", port=37260, timeout_s=3.0):
        self.ip = ip
        self.port = int(port)
        self.timeout_s = float(timeout_s)
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

    def set_nadir(self) -> bool:
        """Point straight down. One command; the gimbal's own IMU holds it.

        This is called once at startup and after a reconnect, NOT on a timer.
        The A8 mini is 3-axis stabilised against its own IMU, so it holds nadir
        through aircraft roll and pitch without being told again -- re-commanding
        it every tick would add a control loop that has nothing to correct.
        """
        if self._sdk is None:
            return False
        try:
            self._sdk.setGimbalRotation(0.0, NADIR_PITCH_DEG)
            return True
        except Exception:
            return False

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
        """-> (is_nadir, measured_pitch). measured_pitch is None if unknown."""
        att = self.attitude()
        if att is None:
            return False, None
        pitch = att[1]
        return abs(pitch - NADIR_PITCH_DEG) <= tolerance_deg, pitch

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

    def connect(self):
        return self

    def close(self):
        pass

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
