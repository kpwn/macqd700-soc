# LiteSDCard native SD core

`litesdcard_core.v` is generated from LiteSDCard 2024.12, upstream commit
`b68dfded8a76c74aea718d9a8914cf8c2bf3f26c`, with LiteX 2024.12 and
Migen 0.10.0. Its source and generated form are covered by the accompanying
BSD-2-Clause `LICENSE`. The generated artifact and CSR map are checked in so
FPGA builds do not depend on Python package resolution or generator behavior.

The upstream `gen.py` targets an older LiteX API. `gen_compat.py` is the
corresponding generator with only its SoC constructor adapted for LiteX
2024.12. Regenerate in an isolated directory with:

```sh
python3 -m venv /tmp/litesdcard-venv
/tmp/litesdcard-venv/bin/pip install 'litesdcard==2024.12' 'litex==2024.12' 'migen==0.10.0'
cd /tmp/litesdcard-output
/tmp/litesdcard-venv/bin/python /path/to/this/dir/gen_compat.py --clk-freq 100e6
```

The checked-in Verilog SHA-256 is
`bd78ba3bccdd9a44f54028b60d897494eb03dc222fa8a25328bb41a405c3d491`;
`csr.csv` SHA-256 is
`8668fe0066592c8dce3b70ce6fd0b6150d2c187116587aeae637835a50e1da3d`.
The probe runs at 100 MHz and its clock divider can deliver 400 kHz init,
25 MHz default-speed, or 50 MHz High Speed on four data wires. The board's
SD supply is fixed at 3.3 V.
