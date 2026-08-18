#!/bin/sh
# Rebuilds ../state-machine.svg and ../state-machine-detail.svg.
# Each diagram is two graphs: the state machine, and the note that goes under it.
# compose.py stacks them, because dot cannot place a note below a left-to-right graph.
set -e
B=.build
mkdir -p "$B"
for n in sm_operator sm_detail; do
  dot -Tsvg "${n}_body.dot" -o "$B/${n}_body.svg"
  dot -Tsvg "${n}_note.dot" -o "$B/${n}_note.svg"
done
python3 compose.py "$B"
echo "wrote ../state-machine.svg and ../state-machine-detail.svg"
