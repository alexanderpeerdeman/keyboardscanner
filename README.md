# keyboardscanner

Rust project for the _Arduino Mega 2560_. Implementing a keyboardscanner (inspired by [Daniel Mouras keyboardscanner](https://github.com/oxesoft/keyboardscanner)) for a broken Casio Privia PX-330.  
The internal firmware seems corrupted and thus the digital piano was not usable anymore.  
This program produces midi messages which can be turned into a virtual midi device using something like `tty-midi`.

The Arduino C `millis()` function was implemented following https://blog.rahix.de/005-avr-hal-millis/

## Build Instructions

1. Install prerequisites as described in the [`avr-hal` README] (`avr-gcc`, `avr-libc`, `avrdude`, [`ravedude`]).

2. Run `cargo build` to build the firmware.

3. Run `cargo run` to flash the firmware to a connected board. If `ravedude`
   fails to detect your board, check its documentation at
   <https://crates.io/crates/ravedude>.

4. `ravedude` will open a console session after flashing where you can interact
   with the UART console of your board.

[`avr-hal` README]: https://github.com/Rahix/avr-hal#readme
[`ravedude`]: https://crates.io/crates/ravedude

## License

Licensed under either of

- Apache License, Version 2.0
  ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license
  ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

# Todo

- [ ] Implement advertising as [usb-midi device](https://crates.io/crates/usbd-midi) directly (removing the need to use `tty-midi`)
