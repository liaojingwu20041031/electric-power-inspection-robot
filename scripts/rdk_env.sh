#!/usr/bin/env bash
# Shared, non-secret RDK X5 runtime defaults.
# Override these variables before running a script when the board layout differs.
# Passwords are intentionally not handled here.

DG_RDK_USER="${DG_RDK_USER:-${RDK_USER:-root}}"
DG_RDK_HOST="${DG_RDK_HOST:-${RDK_HOST:-192.168.43.13}}"
DG_RDK_WORKSPACE="${DG_RDK_WORKSPACE:-${RDK_WORKSPACE:-$HOME/dg202611_rdk}}"
DG_RDK_RECORD_DIR="${DG_RDK_RECORD_DIR:-${RDK_RECORD_DIR:-$HOME/dg202611_rdk_records}}"
DG_RDK_TROS_SETUP="${DG_RDK_TROS_SETUP:-${TROS_SETUP:-/opt/tros/humble/setup.bash}}"
DG_RDK_SSH_TARGET="${DG_RDK_SSH_TARGET:-${RDK_SSH_TARGET:-${DG_RDK_USER}@${DG_RDK_HOST}}}"

export DG_RDK_USER DG_RDK_HOST DG_RDK_WORKSPACE DG_RDK_RECORD_DIR
export DG_RDK_TROS_SETUP DG_RDK_SSH_TARGET
