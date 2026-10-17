"""Record every NetworkTables topic from the robot into a .wpilog (opens in AdvantageScope).

Run on the Driver Station laptop before the match, Ctrl+C after:
    python tools/nt_recorder.py                 # team 3173 (10.31.73.2, works on the field too)
    python tools/nt_recorder.py --address 127.0.0.1   # sim
    python tools/nt_recorder.py --selftest      # checks the file writer, no robot needed

Needs: pip install pyntcore
"""

import argparse
import datetime
import pathlib
import struct
import time

import ntcore

T = ntcore.NetworkTableType


def _str_array(items):
    out = struct.pack("<I", len(items))
    for s in items:
        b = s.encode()
        out += struct.pack("<I", len(b)) + b
    return out


# NT value -> WPILOG payload bytes, per the WPILOG spec.
ENCODERS = {
    T.kBoolean: lambda v: struct.pack("<?", v),
    T.kInteger: lambda v: struct.pack("<q", v),
    T.kFloat: lambda v: struct.pack("<f", v),
    T.kDouble: lambda v: struct.pack("<d", v),
    T.kString: lambda v: v.encode(),
    T.kRaw: bytes,
    T.kBooleanArray: lambda v: struct.pack(f"<{len(v)}?", *v),
    T.kIntegerArray: lambda v: struct.pack(f"<{len(v)}q", *v),
    T.kFloatArray: lambda v: struct.pack(f"<{len(v)}f", *v),
    T.kDoubleArray: lambda v: struct.pack(f"<{len(v)}d", *v),
    T.kStringArray: _str_array,
}


class WpilogWriter:
    """Minimal WPILOG v1.0 writer. Fixed field widths (4B id, 4B size, 8B timestamp)."""

    def __init__(self, path):
        self.f = open(path, "wb")
        self.f.write(b"WPILOG" + struct.pack("<HI", 0x0100, 0))
        self.ids = {}

    def _record(self, entry, ts_us, payload):
        self.f.write(struct.pack("<BIIQ", 0x7F, entry, len(payload), ts_us) + payload)

    def start(self, name, type_str, ts_us):
        entry = len(self.ids) + 1
        self.ids[name] = entry
        n, t = name.encode(), type_str.encode()
        payload = (
            struct.pack("<BI", 0, entry)
            + struct.pack("<I", len(n)) + n
            + struct.pack("<I", len(t)) + t
            + struct.pack("<I", 0)
        )
        self._record(0, ts_us, payload)
        return entry

    def append(self, name, type_str, ts_us, payload):
        entry = self.ids.get(name) or self.start(name, type_str, ts_us)
        self._record(entry, ts_us, payload)

    def close(self):
        self.f.close()


def record(inst, out_path, stop=lambda: False):
    # sendAll: every update, not just the latest; 20 ms matches the robot loop.
    sub = ntcore.MultiSubscriber(
        inst, [""], ntcore.PubSubOptions(sendAll=True, periodic=0.02)
    )
    poller = ntcore.NetworkTableListenerPoller(inst)
    poller.addListener(sub, ntcore.EventFlags.kValueRemote)
    poller.addConnectionListener(False)

    log = WpilogWriter(out_path)
    count, last_flush = 0, time.monotonic()
    try:
        while not stop():
            for ev in poller.readQueue():
                if ev.is_(ntcore.EventFlags.kConnection):
                    state = "CONNECTED" if ev.is_(ntcore.EventFlags.kConnected) else "DISCONNECTED"
                    print(f"[{datetime.datetime.now():%H:%M:%S}] {state} {ev.data.remote_ip}")
                    continue
                v = ev.data.value
                encode = ENCODERS.get(v.type())
                if encode is None:
                    continue
                topic = ev.data.topic
                log.append(topic.getName(), topic.getTypeString(), v.time(), encode(v.value()))
                count += 1
            if time.monotonic() - last_flush > 1:
                log.f.flush()  # survive a laptop crash / pulled cable with at most 1 s lost
                last_flush = time.monotonic()
                print(f"\r{count} values, {len(log.ids)} topics", end="", flush=True)
            time.sleep(0.01)
    finally:
        log.close()
        poller.close()
        sub.close()
    return count


def selftest(tmpdir):
    """Publish a few values on a local server, record them, read them back."""
    from wpiutil.log import DataLogReader

    server = ntcore.NetworkTableInstance.create()
    server.startServer(listen_address="127.0.0.1", port4=5899)
    client = ntcore.NetworkTableInstance.create()
    client.setServer("127.0.0.1", 5899)
    client.startClient4("nt-recorder-selftest")

    path = tmpdir / "selftest.wpilog"
    pubs = {
        "/t/d": server.getDoubleTopic("/t/d").publish(),
        "/t/s": server.getStringArrayTopic("/t/s").publish(),
        "/t/b": server.getBooleanTopic("/t/b").publish(),
    }
    deadline = time.monotonic() + 3

    def drive():
        # wait for the connection, then publish
        if client.isConnected() and not hasattr(drive, "sent"):
            time.sleep(0.2)
            pubs["/t/d"].set(1.5)
            pubs["/t/s"].set(["a", "bc"])
            pubs["/t/b"].set(True)
            drive.sent = True
        return time.monotonic() > deadline

    record(client, path, stop=drive)

    got = {}
    names = {}
    for r in DataLogReader(str(path)):
        if r.isStart():
            s = r.getStartData()
            names[s.entry] = (s.name, s.type)
        elif not r.isControl():
            name, typ = names[r.getEntry()]
            got[name] = {
                "double": r.getDouble,
                "string[]": r.getStringArray,
                "boolean": r.getBoolean,
            }[typ]()
    assert got.get("/t/d") == 1.5, got
    assert list(got.get("/t/s", [])) == ["a", "bc"], got
    assert got.get("/t/b") is True, got
    print("\nselftest OK")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--team", type=int, default=3173)
    p.add_argument("--address", help="robot IP instead of team number (e.g. 127.0.0.1 for sim)")
    p.add_argument("--out", default=str(pathlib.Path.home() / "Documents" / "nt_logs"))
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()

    out_dir = pathlib.Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    if a.selftest:
        selftest(out_dir)
        return

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("nt-recorder")
    if a.address:
        inst.setServer(a.address)
    else:
        inst.setServerTeam(a.team)

    path = out_dir / f"nt_{datetime.datetime.now():%Y%m%d_%H%M%S}.wpilog"
    print(f"Recording to {path}  (Ctrl+C to stop)")
    try:
        record(inst, path)
    except KeyboardInterrupt:
        pass
    print(f"\nSaved {path}")


if __name__ == "__main__":
    main()
