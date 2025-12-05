#!/bin/bash

# agilex_can_activate.sh
set -euo pipefail

DEFAULT_BITRATE="${2:-1000000}"
CAN_PREFIX="can"

log_success() { echo -e "\e[32m[SUCCESS] $1\e[0m"; }
log_error() { echo -e "\e[31m[ERROR] $1\e[0m" >&2; }
log_info() { echo -e "\e[33m[INFO] $1\e[0m"; }


# Check if required utilities are installed
for pkg in ethtool can-utils; do
  if ! dpkg -s "$pkg" &> /dev/null; then
    log_error "Error: $pkg not detected. Please install it: sudo apt update && sudo apt install $pkg"
    exit 1
  fi
done
log_success "All required packages are installed."

# Retrieve CAN interfaces
CAN_INTERFACES=($(ip -br link show type can | awk '{print $1}'))
CAN_COUNT=${#CAN_INTERFACES[@]}

if [ "$CAN_COUNT" -eq 0 ]; then
    log_error "No CAN interfaces detected!"
    exit 1
fi
log_success "Detected $CAN_COUNT CAN interface(s)."

# Configure interfaces
index=0
for iface in "${CAN_INTERFACES[@]}"; do
    NEW_NAME="${CAN_PREFIX}${index}"
    BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')

    log_info "Configuring $iface (USB: $BUS_INFO)..."

    sudo ip link set "$iface" down
    sudo ip link set "$iface" type can bitrate "$DEFAULT_BITRATE"
    sudo ip link set "$iface" up
    log_success "Bitrate set for $iface"

    # Only attempt to rename if the current name doesn't match the desired name
    if [ "$iface" != "$NEW_NAME" ]; then
        log_info "Renaming $iface → $NEW_NAME..."
        sudo ip link set "$iface" down
        if sudo ip link set "$iface" name "$NEW_NAME"; then
            sudo ip link set "$NEW_NAME" up
            log_success "Renamed $iface → $NEW_NAME"
        else
            log_error "Failed to rename $iface to $NEW_NAME"
            sudo ip link set "$iface" up  # Make sure to bring the interface back up
            exit 1
        fi
    else
        log_info "Interface $iface already has the correct name"
    fi

    log_info "Index ++"
    index=$((index + 1))
    log_info "Index incremented"
done

log_success "All CAN interfaces active"
# Return success
exit 0