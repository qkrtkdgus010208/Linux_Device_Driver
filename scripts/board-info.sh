#!/usr/bin/env bash
set -euo pipefail

printf 'Kernel: %s\n' "$(uname -r)"
if [[ -r /proc/device-tree/model ]]; then
    printf 'Board: '
    tr -d '\0' < /proc/device-tree/model
    printf '\n'
fi
printf '\nGPIO controllers (base is dynamic):\n'
for chip in /sys/bus/gpio/devices/gpiochip*; do
    [[ -e "$chip" ]] || continue
    if [[ -r "$chip/label" && -r "$chip/base" && -r "$chip/ngpio" ]]; then
        printf '%s label=%s base=%s ngpio=%s\n' "$chip" \
            "$(cat "$chip/label")" "$(cat "$chip/base")" "$(cat "$chip/ngpio")"
    else
        printf '%s -> %s\n' "$chip" "$(readlink -f "$chip")"
    fi
done
printf 'When available, gpiodetect/gpioinfo show labels and offsets; debugfs gpio shows global bases.\n'

printf '\nPWM controllers (5.10/5.15 legacy ID = pwmchip suffix + channel):\n'
found=0
for chip in /sys/class/pwm/pwmchip*; do
    [[ -e "$chip" ]] || continue
    target=$(readlink -f "$chip")
    printf '%s -> %s (npwm=%s)\n' "$chip" "$target" "$(cat "$chip/npwm")"
    if [[ "$target" == */32e0000.pwm/pwm/pwmchip* ]]; then
        found=1
        pwm_number=${chip##*/pwmchip}
        printf 'Header pin 32, PWM7 channel 0: pwm_id=%s\n' "$pwm_number"
        printf 'After pinmux and wiring verification: sudo insmod drivers/motor/l298n_motor.ko pwm_id=%s\n' "$pwm_number"
    fi
done
if [[ "$found" == 0 ]]; then
    printf '32e0000.pwm was not found. Enable PWM7 in the board configuration first.\n'
fi
printf '\nThis command only reads board information; it does not configure pins or load modules.\n'
