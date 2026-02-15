#!/usr/bin/env bash
set -euo pipefail

sudo nmcli con add type ethernet ifname eno1 con-name livox ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.gateway "" ipv6.method ignore

sudo nmcli con up livox
