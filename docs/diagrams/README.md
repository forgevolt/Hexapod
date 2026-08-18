# Diagram sources

The state-machine SVGs in `docs/` are generated from these. The Mermaid versions in
[`../MANUAL.md`](../MANUAL.md) and [`../state-machine-detail.md`](../state-machine-detail.md)
are maintained separately - **change both when a transition changes.**

| File | |
|---|---|
| `sm_operator_body.dot` | Operator diagram, states and transitions |
| `sm_operator_note.dot` | The note that sits under it |
| `sm_detail_body.dot` | Firmware diagram, with guards, gaits and constants |
| `sm_detail_note.dot` | The note that sits under it |
| `compose.py` | Stacks graph and note into one SVG, and adds web-safe font fallbacks |
| `build.sh` | Runs all of the above |

Needs graphviz and python3.
