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
  # some ros2 CLI verbs (e.g. `topic hz`) start a ros2 daemon; stop ours
  # (only the daemon of this shell's ROS_DOMAIN_ID=21 is affected)
  command -v ros2 >/dev/null && timeout 10 ros2 daemon stop >/dev/null 2>&1
  true
}

# Print any sim processes from this project still running (empty = clean).
list_sim_leftovers() {
  pgrep -af "$_KIT_TELEMETRY_PAT|warehouse-bot-sim/sim/[a-z_]+[.]py" | grep -v -E "pgrep|bash -c" || true
}
