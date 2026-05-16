#!/bin/bash
. $(dirname "$0")/color_variables.sh
. $(dirname "$0")/fix_ld_issue.sh

printf "${BIRed} Make sure you are in the Performance Mode!!! ${Color_Off} \n"

RTOS_MODE=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)

echo $RTOS_MODE

for mode in $RTOS_MODE
do
    if [ "${mode}" = "powersave" ]; then
	printf "${BIRed} Not in Performance Mode, will cause errors for franka codebase!!! ${Color_Off} \n"
    fi
done

LOG_DIR="$(dirname "$0")/../logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/franka_interface_$(date +%Y%m%d_%H%M%S).log"
printf "${BIYellow} Logging franka-interface stdout+stderr to: ${LOG_FILE} ${Color_Off}\n"

# Enable core dumps so we can gdb the abort if needed.
ulimit -c unlimited 2>/dev/null || true

while true
do
    START_TS="$(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    echo "============================================================"
    echo "=== franka-interface starting at ${START_TS}"
    echo "============================================================"
    # stdbuf -oL -eL forces line buffering on stdout/stderr so partial
    # buffers aren't lost when the process aborts. 2>&1 merges stderr into
    # stdout so libfranka's exception text appears in the same stream.
    # tee -a writes to the log while still showing output in the terminal.
    stdbuf -oL -eL bin/franka-interface "$@" 2>&1 | tee -a "$LOG_FILE"
    EXIT_CODE=${PIPESTATUS[0]}
    END_TS="$(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    echo "============================================================"
    echo "=== franka-interface exited at ${END_TS} with code ${EXIT_CODE}"
    if [ "${EXIT_CODE}" -ge 128 ]; then
        SIG=$((EXIT_CODE - 128))
        echo "===   (terminated by signal ${SIG}; 6=SIGABRT, 11=SIGSEGV, 15=SIGTERM)"
    fi
    echo "============================================================"
    sleep 1
done
