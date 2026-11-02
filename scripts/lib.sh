# Shared helpers for sim scripts. Source after env.sh.
# NOTE: never run these pkill patterns inline in an interactive `bash -c` whose
# own command line contains the pattern text - pkill -f would match that shell.

_KIT_TELEMETRY_PAT="isaacsim-env/.*omni[.]telemetry[.]transmitte[r]"

# Kit spawns a detached telemetry transmitter (from the shared Isaac Sim
# install) around app shutdown. It outlives the sim, so poll ~15 s and stop it.
kill_kit_leftovers() {
  for _ in $(seq 1 15); do
    pkill -f "$_KIT_TELEMETRY_PAT" 2>/dev/null
    sleep 1
  done
  pkill -9 -f "$_KIT_TELEMETRY_PAT" 2>/dev/null
  true
}

# Print any sim processes from this project still running (empty = clean).
list_sim_leftovers() {
  pgrep -af "$_KIT_TELEMETRY_PAT|warehouse-bot-sim/sim/[a-z_]+[.]py" | grep -v -E "pgrep|bash -c" || true
}
