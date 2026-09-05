#!/usr/bin/env python3
"""Apply the narrowly scoped nRF52840 Bluetooth controller stability fix.

Based on upstream Zephyr commit 355648a69f5137c68d86fe577ddb256e7da74d12.
The script is idempotent and fails if the expected v3.5 code cannot be found.
"""

from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit("usage: apply_zephyr_nrf52840_radio_fix.py <zephyr-root>")

root = Path(sys.argv[1]).resolve()
target = root / "subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/radio/radio.c"
text = target.read_text(encoding="utf-8")

function_start = text.find("void radio_disable(void)")
if function_start < 0:
    raise SystemExit(f"radio_disable() not found in {target}")
function_end = text.find("void radio_status_reset(void)", function_start)
if function_end < 0:
    raise SystemExit(f"radio_disable() end not found in {target}")
section = text[function_start:function_end]

if "radio_tmr_status_reset();" in section:
    print("nRF52840 radio timer reset fix is already present")
    raise SystemExit(0)

needle = """#if !defined(CONFIG_BT_CTLR_TIFS_HW)\n\thal_radio_sw_switch_cleanup();\n#endif /* !CONFIG_BT_CTLR_TIFS_HW */\n\n\tNRF_RADIO->SHORTS = 0;"""
replacement = """#if !defined(CONFIG_BT_CTLR_TIFS_HW)\n\thal_radio_sw_switch_cleanup();\n#endif /* !CONFIG_BT_CTLR_TIFS_HW */\n\n\t/* Reset/disable PPI/DPPI state before disabling the radio. */\n\tradio_tmr_status_reset();\n\n\tNRF_RADIO->SHORTS = 0;"""

if section.count(needle) != 1:
    raise SystemExit(
        "Expected Zephyr v3.5 radio_disable() body was not found exactly once; refusing an unsafe patch"
    )

patched_section = section.replace(needle, replacement, 1)
text = text[:function_start] + patched_section + text[function_end:]
target.write_text(text, encoding="utf-8")
print(f"Applied nRF52840 radio timer reset fix to {target}")
