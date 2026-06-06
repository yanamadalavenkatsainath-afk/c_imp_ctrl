#include "capture_gate.h"
#include <string.h>

void CaptureGate_default(CaptureGateIn *in) {
    memset(in, 0, sizeof(*in));
    in->align_deg = -1.0;
    in->body_clear = 1;
    in->geometry_ok = 1;
    in->soft_capture_range_m = CFG_SOFT_CAPTURE_RANGE_M;
    in->soft_capture_vrel_ms = CFG_SOFT_CAPTURE_VREL_MS;
    in->soft_capture_entry_align_max_deg = CFG_SOFT_CAPTURE_ENTRY_ALIGN_MAX_DEG;
    in->soft_capture_latch_vrel_ms = CFG_SOFT_CAPTURE_LATCH_VREL_MS;
    in->soft_capture_core_align_max_deg = CFG_SOFT_CAPTURE_CORE_ALIGN_MAX_DEG;
    in->hard_capture_range_m = CFG_HARD_CAPTURE_RANGE_M;
    in->hard_capture_vrel_ms = CFG_HARD_CAPTURE_VREL_MS;
    in->dock_align_max_deg = CFG_DOCK_ALIGN_MAX_DEG;
}

void CaptureGate_eval(const CaptureGateIn *in, CaptureGateOut *out) {
    int align_known = (in->align_deg >= 0.0);
    int align_ok_entry =
        (!align_known) ||
        (in->align_deg < in->soft_capture_entry_align_max_deg);

    out->soft_capture_ready =
        (in->port_range_m < in->soft_capture_range_m) &&
        (in->port_vrel_ms < in->soft_capture_vrel_ms) &&
        in->body_clear &&
        align_ok_entry;

    out->hard_capture_ready =
        (in->port_range_m < in->hard_capture_range_m) &&
        (in->port_vrel_ms < in->hard_capture_vrel_ms) &&
        in->geometry_ok &&
        in->body_clear &&
        in->align_ok;

    out->soft_core_ready =
        in->capture_core &&
        in->align_ok;

    out->soft_capture_stable =
        (in->port_range_m < in->soft_capture_range_m) &&
        (in->port_vrel_ms < in->soft_capture_latch_vrel_ms) &&
        in->body_clear;

    out->soft_capture_certified =
        out->soft_capture_stable &&
        out->soft_core_ready;
}
