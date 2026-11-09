"""Parse the `FBOXES` lines printed by sim/run_fleet_sim.py (Day 22), pure Python."""


def parse_fboxes(line):
    """-> {"t": float, "robots": {ns: {"lift": float, "held": str}},
           "boxes": {id: ((x, y, z), holder_or_"-")}}"""
    parts = line.split()
    if not parts or parts[0] != "FBOXES":
        raise ValueError("not an FBOXES line")
    out = {"t": None, "robots": {}, "boxes": {}}
    for tok in parts[1:]:
        if tok.startswith("t="):
            out["t"] = float(tok[2:])
        elif ":lift=" in tok:
            ns, rest = tok.split(":", 1)
            kv = dict(x.split("=", 1) for x in rest.split(","))
            out["robots"][ns] = {"lift": float(kv["lift"]), "held": kv["held"]}
        else:
            name, rest = tok.split("=", 1)
            xyz, holder = rest.split("@", 1)
            out["boxes"][name] = (tuple(float(v) for v in xyz.split(",")), holder)
    return out
