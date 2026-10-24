# hdl-util/hdmi

The SystemVerilog files here (except `serializer.sv`) come from
<https://github.com/hdl-util/hdmi> at commit
`83b1c9543a91b776671a44e68e130f81cae437b7`. Copyright 2019 Sameer Puri.
They are dual licensed under MIT or Apache-2.0; see `LICENSE-MIT` and
`LICENSE-APACHE`. This tree chooses MIT for integration. Trailing whitespace
was removed from the imported source files.

`serializer.sv` is this project's UltraScale+ replacement for the upstream
Series 7 `OSERDESE2` serializer. It implements the same module interface with
five `ODDRE1` transfers per TMDS symbol. The upstream audio packetizer and
TMDS encoder remain intact for a later full SoC integration.
