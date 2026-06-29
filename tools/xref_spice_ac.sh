#!/bin/zsh
# Run a templated XREF spice deck through ngspice .op + .ac and print the
# closed-loop gain (dB) at 100 Hz / 1k / 10k / 100k. Usage:
#   xref_spice_ac.sh <deck.spice> <RF_VALUE> <CF_VALUE>
set -e
DECK="$1"; RF="$2"; CF="$3"
TMP=$(mktemp -d)
sed -e "s/RF_VALUE/$RF/" -e "s/CF_VALUE/$CF/" "$DECK" > "$TMP/core.spice"
cat > "$TMP/run.spice" <<EOF
.TITLE XREF AC
.include $TMP/core.spice
VINAC v_in 0 DC 0 AC 1
.CONTROL
  set filetype=ascii
  op
  print v(nb1) v(nout)
  ac dec 20 10 1meg
  meas ac g100  find vdb(v_out) at=100
  meas ac g1k   find vdb(v_out) at=1000
  meas ac g10k  find vdb(v_out) at=10000
  meas ac g100k find vdb(v_out) at=100000
  quit
.ENDC
.END
EOF
ngspice -b "$TMP/run.spice" 2>&1 | grep -iE "v\(nb1\)|v\(nout\)|g100|g1k|g10k|g100k" | grep -ivE "model"
rm -rf "$TMP"
