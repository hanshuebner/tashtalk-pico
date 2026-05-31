ELF = target/thumbv6m-none-eabi/release/tashtalk-pico
UF2 = tashtalk-pico.uf2

.PHONY: all flash clean test

all: $(UF2)

$(UF2): $(ELF)
	elf2uf2-rs $< $@

$(ELF): FORCE
	cd firmware && cargo build --release

flash: $(UF2)
	picotool load -u -v -x -t uf2 $<

test:
	cargo test --target x86_64-unknown-linux-gnu

clean:
	cargo clean
	rm -f $(UF2)

FORCE:
