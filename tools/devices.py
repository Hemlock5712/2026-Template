"""Talks to the Phoenix diagnostic server the running robot program already hosts.

    python tools/devices.py list                 # what is on the bus, at what ID, on what firmware
    python tools/devices.py blink 31             # make one device flash, so you can see which it is
    python tools/devices.py setid 0 31           # give a device a new CAN ID

The robot (or sim) must be RUNNING - this is its diagnostic server, not a standalone tool.
Add --host 10.TE.AM.2 to talk to a real robot instead of a local sim.

Devices are addressed by model + canbus + id, all three; `list` looks them up for you. Note that
CANbus comes back as an empty string in simulation, which is why a hand-built URL usually 404s.
"""

import argparse
import json
import pathlib
import re
import sys
import urllib.parse
import urllib.request

VENDORDEPS = pathlib.Path("vendordeps")


def request(host, params, timeout=15):
    url = f"http://{host}:1250/?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode())


def devices(host):
    body = request(host, {"action": "getdevices"})
    return body.get("DeviceArray", []), body.get("BusUtilPerc")


def find(host, device_id):
    matches = [d for d in devices(host)[0] if d["ID"] == device_id]
    if not matches:
        sys.exit(f"No device at ID {device_id}. Run `list` to see what is there.")
    if len(matches) > 1:
        models = ", ".join(sorted(d["Model"] for d in matches))
        sys.exit(f"ID {device_id} is used by more than one device ({models}) - fix that first.")
    return matches[0]


def act(host, device, action, **extra):
    params = {
        "action": action,
        "model": device["Model"],
        "canbus": device["CANbus"],
        "id": device["ID"],
        **extra,
    }
    result = request(host, params).get("GeneralReturn", {})
    return result.get("Error", -1), result.get("ErrorMessage", "")


def expected_firmware():
    """The device firmware the pinned Phoenix vendordep expects, e.g. 26.50.0-alpha-1 -> 26.

    MAJOR version only. An alpha vendordep's minor version runs ahead of any released device
    firmware - 26.50.0-alpha-1 has no matching 26.50.x CRF and never will - so comparing more
    than the major flags every device on the bus forever.
    """
    for path in VENDORDEPS.glob("Phoenix6*.json"):
        version = json.loads(path.read_text()).get("version", "")
        match = re.match(r"(\d+)\.", version)
        if match:
            return match.group(1), version
    return None, None


def cmd_list(args):
    found, utilization = devices(args.host)
    if not found:
        print("No devices. Is the robot program running?")
        return
    want, raw = expected_firmware()
    # Simulation reports -1; only print a real number.
    busy = f", bus {utilization}% utilized" if utilization and utilization >= 0 else ""
    print(f"{len(found)} devices{busy}")
    if want:
        print(f"vendordep {raw} expects firmware {want}.x\n")
    # Compare the major only - see expected_firmware.
    want = want + "." if want else want

    seen = {}
    for device in sorted(found, key=lambda d: (d["Model"], d["ID"])):
        firmware = device.get("CurrentVers", "")
        stale = want and not firmware.startswith(want)
        notes = []
        if stale:
            notes.append("FIRMWARE MISMATCH")
        if device["ID"] == 0:
            notes.append("ID 0 - factory default, probably the one you just plugged in")
        key = (device["Model"], device["ID"])
        if key in seen:
            notes.append("DUPLICATE ID")
        seen[key] = True
        flag = "  <- " + "; ".join(notes) if notes else ""
        print(f"  ID {device['ID']:>3}  {device['Model']:<22} {firmware:<24}{flag}")


def cmd_blink(args):
    device = find(args.host, args.id)
    code, message = act(args.host, device, "blink")
    if code == 0:
        print(f"Blinking {device['Model']} at ID {args.id}. Which one is flashing?")
    else:
        sys.exit(f"blink failed: {code} {message}")


def cmd_setid(args):
    device = find(args.host, args.old)
    print(f"{device['Model']} at ID {args.old} -> ID {args.new}")
    code, message = act(args.host, device, "setid", newid=args.new)
    if code == 0:
        print("Done. Update the LoggedTalonFX/LoggedCANcoder id in the subsystem to match.")
        return
    if code == -109 and device.get("SoftStatus", "").startswith("Simulated"):
        sys.exit(
            "setid failed: this is a simulated device. Simulated devices always refuse an ID"
            " change - there is no non-volatile storage to write it into. Needs real hardware."
        )
    sys.exit(f"setid failed: {code} {message}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="localhost", help="robot address (default: local sim)")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="show every device, its ID and its firmware")

    blink = sub.add_parser("blink", help="flash one device so you can see which it is")
    blink.add_argument("id", type=int)

    setid = sub.add_parser("setid", help="change a device's CAN ID")
    setid.add_argument("old", type=int)
    setid.add_argument("new", type=int)

    args = parser.parse_args()
    try:
        {"list": cmd_list, "blink": cmd_blink, "setid": cmd_setid}[args.command](args)
    except urllib.error.URLError as error:
        sys.exit(f"No diagnostic server at {args.host}:1250 - is the robot program running? ({error})")


if __name__ == "__main__":
    main()
