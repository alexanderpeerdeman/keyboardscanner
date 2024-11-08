// https://crates.io/crates/usbd-midi

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::cell;

use arduino_hal::{
    hal::{
        port::{Dynamic, PD2, PD3, PF0},
        usart::BaudrateArduinoExt,
    },
    pac::USART1,
    port::{
        mode::{Analog, Input, Output, PullUp},
        Pin,
    },
    prelude::_unwrap_infallible_UnwrapInfallible,
    Adc, Usart,
};
use embedded_hal::serial::Read;
use panic_halt as _;

const PRESCALER: u32 = 64;
const TIMER_COUNTS: u32 = 250;
const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;
static MILLIS_COUNTER: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

const KEY_MIN_TIME_MS: u32 = 3;
const KEY_MAX_TIME_MS: u32 = 50;
const KEY_MAX_TIME_MS_N: u32 = KEY_MAX_TIME_MS - KEY_MIN_TIME_MS;

const PITCH_BEND_MAX: u16 = 960;

#[derive(Copy, Clone)]
enum KeyState {
    Off,
    Start,
    Detected,
    On,
    Released,
    Sustain,
}

enum PedalType {
    Damper,
    SoftSostenuto,
}

struct Pedal {
    kind: PedalType,
    pressed: bool,
}

impl Pedal {
    fn new(kind: PedalType) -> Self {
        Self {
            kind,
            pressed: false,
        }
    }

    fn read(&mut self, serial: &mut Console, input: &Pin<Input<PullUp>, Dynamic>) {
        let pressed = input.is_low();
        if self.pressed != pressed {
            self.pressed = pressed;
            let midi_value_1 = match self.kind {
                PedalType::Damper => 64,
                PedalType::SoftSostenuto => 67,
            };
            let midi_value_2 = match self.pressed {
                true => 127,
                false => 0,
            };
            send_midi(serial, MidiEvent::CC, midi_value_1, midi_value_2);
        }
    }
}

struct Wheel {
    bend: u16,
}

impl Wheel {
    fn new() -> Self {
        Wheel { bend: 64 }
    }

    fn read(&mut self, serial: &mut Console, pitchbend: &Pin<Analog, PF0>, adc: &mut Adc) {
        let voltage = pitchbend.analog_read(adc);

        let clamped = clamp(voltage, 0, PITCH_BEND_MAX);
        let mapped = map_range(clamped, 0, PITCH_BEND_MAX, 0, 127);

        if mapped != self.bend {
            self.bend = mapped;
            send_midi(serial, MidiEvent::PitchBend, 0, mapped as u8);
        }
    }
}

struct Key {
    index: u8,
    // name: &'static str,
    state: KeyState,
    time: u32,

    kc: usize,
    ko: usize,

    fi: usize,
    si: usize,
    ki: usize,
}

impl Key {
    fn new(
        index: u8,
        kc: usize,
        ko: usize,
        fi: usize,
        si: usize,
        ki: usize,
        _name: &'static str,
    ) -> Self {
        Self {
            index,
            // name,
            state: KeyState::Off,
            time: 0,
            kc,
            ko,
            fi,
            si,
            ki,
        }
    }

    fn read(
        &mut self,
        serial: &mut Console,
        kc: &mut [Pin<Output, Dynamic>],
        ko: &mut [Pin<Output, Dynamic>],
        fi: &[Pin<Input<PullUp>, Dynamic>],
        si: &[Pin<Input<PullUp>, Dynamic>],
        ki: &[Pin<Input<PullUp>, Dynamic>],
    ) {
        ko[self.ko].set_low();
        let sensor1 = ki[self.ki].is_low();
        ko[self.ko].set_high();

        kc[self.kc].set_low();
        let sensor2 = fi[self.fi].is_low();
        kc[self.kc].set_high();

        kc[self.kc].set_low();
        let sensor3 = si[self.si].is_low();
        kc[self.kc].set_high();

        match self.state {
            KeyState::Off => {
                if sensor1 {
                    self.state = KeyState::Start;
                    return;
                }
            }
            KeyState::Start => {
                if sensor2 {
                    self.state = KeyState::Detected;
                    self.time = millis();
                    return;
                }
                if !sensor1 {
                    self.state = KeyState::Off;
                    let velocity = calc_velocity(millis() - self.time);
                    send_midi(serial, MidiEvent::Off, self.index, velocity);
                    return;
                }
            }
            KeyState::Detected => {
                if sensor3 {
                    self.state = KeyState::On;
                    let velocity = calc_velocity(millis() - self.time);
                    send_midi(serial, MidiEvent::On, self.index, velocity);
                    return;
                }
                if !sensor2 {
                    self.state = KeyState::Start;
                    self.time = millis();
                    return;
                }
            }
            KeyState::On => {
                if !sensor3 {
                    self.state = KeyState::Released;
                    return;
                }
            }
            KeyState::Released => {
                if !sensor2 {
                    self.state = KeyState::Sustain;
                    self.time = millis();
                    return;
                }
            }
            KeyState::Sustain => {
                if sensor2 {
                    self.state = KeyState::Detected;
                    self.time = millis();
                    return;
                }
                if !sensor1 {
                    self.state = KeyState::Off;
                    let velocity = calc_velocity(millis() - self.time);
                    send_midi(serial, MidiEvent::Off, self.index, velocity);
                    return;
                }
            }
        };
    }
}

enum MidiEvent {
    On,
    Off,
    PitchBend,
    CC,
}

#[avr_device::interrupt(atmega2560)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

fn millis_init(tc0: arduino_hal::pac::TC0) {
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
    tc0.tccr0b.write(|w| match PRESCALER {
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        _ => panic!(),
    });
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    // Reset the global millisecond counter
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    })
}

fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);

    let mut button_controller_serial = Usart::new(
        dp.USART1,
        pins.d19,
        pins.d18.into_output(),
        250000.into_baudrate(),
    );

    millis_init(dp.TC0);
    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };

    // 30 pin connector
    let si10 = pins.d24.into_pull_up_input().downgrade();
    let fi10 = pins.d25.into_pull_up_input().downgrade();
    let si9 = pins.d26.into_pull_up_input().downgrade();
    let fi9 = pins.d27.into_pull_up_input().downgrade();
    let si8 = pins.d28.into_pull_up_input().downgrade();
    let fi8 = pins.d29.into_pull_up_input().downgrade();
    let si7 = pins.d30.into_pull_up_input().downgrade();
    let fi7 = pins.d31.into_pull_up_input().downgrade();
    let si6 = pins.d32.into_pull_up_input().downgrade();
    let fi6 = pins.d33.into_pull_up_input().downgrade();
    let si5 = pins.d34.into_pull_up_input().downgrade();
    let fi5 = pins.d35.into_pull_up_input().downgrade();
    let kc6 = pins.d36.into_output().downgrade();
    let kc7 = pins.d37.into_output().downgrade();
    let kc4 = pins.d38.into_output().downgrade();
    let kc5 = pins.d39.into_output().downgrade();
    let kc2 = pins.d40.into_output().downgrade();
    let kc3 = pins.d41.into_output().downgrade();
    let kc0 = pins.d42.into_output().downgrade();
    let kc1 = pins.d43.into_output().downgrade();
    let fi4 = pins.d44.into_pull_up_input().downgrade();
    let si4 = pins.d45.into_pull_up_input().downgrade();
    let fi3 = pins.d46.into_pull_up_input().downgrade();
    let si3 = pins.d47.into_pull_up_input().downgrade();
    let fi2 = pins.d48.into_pull_up_input().downgrade();
    let si2 = pins.d49.into_pull_up_input().downgrade();
    let fi1 = pins.d50.into_pull_up_input().downgrade();
    let si1 = pins.d51.into_pull_up_input().downgrade();
    let fi0 = pins.d52.into_pull_up_input().downgrade();
    let si0 = pins.d53.into_pull_up_input().downgrade();

    let mut kc_pins = [kc0, kc1, kc2, kc3, kc4, kc5, kc6, kc7];
    let fi_pins = [fi0, fi1, fi2, fi3, fi4, fi5, fi6, fi7, fi8, fi9, fi10];
    let si_pins = [si0, si1, si2, si3, si4, si5, si6, si7, si8, si9, si10];

    // 20 pin connector
    let ki10 = pins.d20.into_pull_up_input().downgrade();
    let ki9 = pins.d22.into_pull_up_input().downgrade();
    let ki8 = pins.d23.into_pull_up_input().downgrade();
    let ki7 = pins.d17.into_pull_up_input().downgrade();
    let ki6 = pins.d16.into_pull_up_input().downgrade();
    let ki5 = pins.d15.into_pull_up_input().downgrade();
    let ki4 = pins.d14.into_pull_up_input().downgrade();
    let ko7 = pins.d13.into_output().downgrade();
    let ko6 = pins.d12.into_output().downgrade();
    let ko5 = pins.d11.into_output().downgrade();
    let ko4 = pins.d10.into_output().downgrade();
    let ko3 = pins.d9.into_output().downgrade();
    let ko2 = pins.d8.into_output().downgrade();
    let ko1 = pins.d7.into_output().downgrade();
    let ko0 = pins.d6.into_output().downgrade();
    let ki3 = pins.d5.into_pull_up_input().downgrade();
    let ki2 = pins.d4.into_pull_up_input().downgrade();
    let ki1 = pins.d3.into_pull_up_input().downgrade();
    let ki0 = pins.d2.into_pull_up_input().downgrade();

    let mut ko_pins = [ko0, ko1, ko2, ko3, ko4, ko5, ko6, ko7];
    let ki_pins = [ki0, ki1, ki2, ki3, ki4, ki5, ki6, ki7, ki8, ki9, ki10];

    // Key Matrix
    let mut keys = [
        Key::new(21, 0, 0, 0, 0, 0, "A0"),
        Key::new(22, 1, 1, 0, 0, 0, "A0#"),
        Key::new(23, 2, 2, 0, 0, 0, "B0"),
        Key::new(24, 3, 3, 0, 0, 0, "C1"), //C1
        Key::new(25, 4, 4, 0, 0, 0, "C1#"),
        Key::new(26, 5, 5, 0, 0, 0, "D1"),
        Key::new(27, 6, 6, 0, 0, 0, "D1#"),
        Key::new(28, 7, 7, 0, 0, 0, "E1"),
        Key::new(29, 0, 0, 1, 1, 1, "F1"),
        Key::new(30, 1, 1, 1, 1, 1, "F1#"),
        Key::new(31, 2, 2, 1, 1, 1, "G1"),
        Key::new(32, 3, 3, 1, 1, 1, "G1#"),
        Key::new(33, 4, 4, 1, 1, 1, "A1"),
        Key::new(34, 5, 5, 1, 1, 1, "A1#"),
        Key::new(35, 6, 6, 1, 1, 1, "B1"),
        Key::new(36, 7, 7, 1, 1, 1, "C2"), //C2
        Key::new(37, 0, 0, 2, 2, 2, "C2#"),
        Key::new(38, 1, 1, 2, 2, 2, "D2"),
        Key::new(39, 2, 2, 2, 2, 2, "D2#"),
        Key::new(40, 3, 3, 2, 2, 2, "E2"),
        Key::new(41, 4, 4, 2, 2, 2, "F2"),
        Key::new(42, 5, 5, 2, 2, 2, "F2#"),
        Key::new(43, 6, 6, 2, 2, 2, "G2"),
        Key::new(44, 7, 7, 2, 2, 2, "G2#"),
        Key::new(45, 0, 0, 3, 3, 3, "A2"),
        Key::new(46, 1, 1, 3, 3, 3, "A2#"),
        Key::new(47, 2, 2, 3, 3, 3, "B2"),
        Key::new(48, 3, 3, 3, 3, 3, "C3"), //C3
        Key::new(49, 4, 4, 3, 3, 3, "C3#"),
        Key::new(50, 5, 5, 3, 3, 3, "D3"),
        Key::new(51, 6, 6, 3, 3, 3, "D3#"),
        Key::new(52, 7, 7, 3, 3, 3, "E3"),
        Key::new(53, 0, 0, 4, 4, 4, "F3"),
        Key::new(54, 1, 1, 4, 4, 4, "F3#"),
        Key::new(55, 2, 2, 4, 4, 4, "G3"),
        Key::new(56, 3, 3, 4, 4, 4, "G3#"),
        Key::new(57, 4, 4, 4, 4, 4, "A3"),
        Key::new(58, 5, 5, 4, 4, 4, "A3#"),
        Key::new(59, 6, 6, 4, 4, 4, "B3"),
        Key::new(60, 7, 7, 4, 4, 4, "C4"), //C4
        Key::new(61, 0, 0, 5, 5, 5, "C4#"),
        Key::new(62, 1, 1, 5, 5, 5, "D4"),
        Key::new(63, 2, 2, 5, 5, 5, "D4#"),
        Key::new(64, 3, 3, 5, 5, 5, "E4"),
        Key::new(65, 4, 4, 5, 5, 5, "F4"),
        Key::new(66, 5, 5, 5, 5, 5, "F4#"),
        Key::new(67, 6, 6, 5, 5, 5, "G4"),
        Key::new(68, 7, 7, 5, 5, 5, "G4#"),
        Key::new(69, 0, 0, 6, 6, 6, "A4"),
        Key::new(70, 1, 1, 6, 6, 6, "A4#"),
        Key::new(71, 2, 2, 6, 6, 6, "B4"),
        Key::new(72, 3, 3, 6, 6, 6, "C5"), //C5
        Key::new(73, 4, 4, 6, 6, 6, "C5#"),
        Key::new(74, 5, 5, 6, 6, 6, "D5"),
        Key::new(75, 6, 6, 6, 6, 6, "D5#"),
        Key::new(76, 7, 7, 6, 6, 6, "E5"),
        Key::new(77, 0, 0, 7, 7, 7, "F5"),
        Key::new(78, 1, 1, 7, 7, 7, "F5#"),
        Key::new(79, 2, 2, 7, 7, 7, "G5"),
        Key::new(80, 3, 3, 7, 7, 7, "G5#"),
        Key::new(81, 4, 4, 7, 7, 7, "A5"),
        Key::new(82, 5, 5, 7, 7, 7, "A5#"),
        Key::new(83, 6, 6, 7, 7, 7, "B5"),
        Key::new(84, 7, 7, 7, 7, 7, "C6"), //C6
        Key::new(85, 0, 0, 8, 8, 8, "C6#"),
        Key::new(86, 1, 1, 8, 8, 8, "D6"),
        Key::new(87, 2, 2, 8, 8, 8, "D6#"),
        Key::new(88, 3, 3, 8, 8, 8, "E6"),
        Key::new(89, 4, 4, 8, 8, 8, "F6"),
        Key::new(90, 5, 5, 8, 8, 8, "F6#"),
        Key::new(91, 6, 6, 8, 8, 8, "G6"),
        Key::new(92, 7, 7, 8, 8, 8, "G6#"),
        Key::new(93, 0, 0, 9, 9, 9, "A6"),
        Key::new(94, 1, 1, 9, 9, 9, "A6#"),
        Key::new(95, 2, 2, 9, 9, 9, "B6"),
        Key::new(96, 3, 3, 9, 9, 9, "C7"), //C7
        Key::new(97, 4, 4, 9, 9, 9, "C7#"),
        Key::new(98, 5, 5, 9, 9, 9, "D7"),
        Key::new(99, 6, 6, 9, 9, 9, "D7#"),
        Key::new(100, 7, 7, 9, 9, 9, "E7"),
        Key::new(101, 0, 0, 10, 10, 10, "F7"),
        Key::new(102, 1, 1, 10, 10, 10, "F7#"),
        Key::new(103, 2, 2, 10, 10, 10, "G7"),
        Key::new(104, 3, 3, 10, 10, 10, "G7#"),
        Key::new(105, 4, 4, 10, 10, 10, "A7"),
        Key::new(106, 5, 5, 10, 10, 10, "A7#"),
        Key::new(107, 6, 6, 10, 10, 10, "B7"),
        Key::new(108, 7, 7, 10, 10, 10, "C8"), //C8
    ];

    let mut wheel = Wheel::new();
    let mut pedal_damp = Pedal::new(PedalType::Damper);
    let mut pedal_softsostenuto = Pedal::new(PedalType::SoftSostenuto);

    let pitchbend = pins.a0.into_analog_input(&mut adc);
    let pin_damp = pins.a1.into_pull_up_input().downgrade();
    let pin_sofsos = pins.a2.into_pull_up_input().downgrade();

    loop {
        for key in keys.iter_mut() {
            key.read(
                &mut serial,
                &mut kc_pins,
                &mut ko_pins,
                &fi_pins,
                &si_pins,
                &ki_pins,
            );
        }

        wheel.read(&mut serial, &pitchbend, &mut adc);

        pedal_damp.read(&mut serial, &pin_damp);
        pedal_softsostenuto.read(&mut serial, &pin_sofsos);

        handle_buttons(&mut serial, &mut button_controller_serial);
    }
}

fn handle_buttons(
    serial: &mut Console,
    button_controller_serial: &mut Usart<USART1, Pin<Input, PD2>, Pin<Output, PD3>>,
) {
    match button_controller_serial.read() {
        Ok(button_state) => {
            let button_id = nb::block!(button_controller_serial.read()).unwrap_infallible();

            let button_name = match button_id {
                0 => "Intro",
                1 => "Accomp Chords",
                2 => "Recorder",
                3 => "Latin/World",
                4 => "Modern",
                5 => "Strings/Synth-Pad",
                6 => "Tone",
                7 => "Chorus",
                8 => "Start/Stop",
                9 => "Normal/Fill-In",
                10 => "Pops/Jazz",
                11 => "Metronome",
                12 => "Organ",
                13 => "Classic",
                14 => "Reverb",
                15 => "Function",
                16 => "Variation/Fill-In",
                17 => "Tempo +",
                18 => "Rhythm",
                19 => "User Rhythms",
                20 => "Elec Piano",
                21 => "Various/GM Tones",
                22 => "No",
                23 => "Card/Internal",
                24 => "Tempo -",
                25 => "Synchro/Ending",
                26 => "Ballet/Piano Rhythms",
                28 => "Bass/Guitar",
                29 => "Vibes/Clavi",
                30 => "Split",
                31 => "Yes",
                36 => "Store",
                _ => "not used",
            };

            let midi_controller_id = match button_id {
                0 => 3,
                1 => 9,
                2 => 14,
                3 => 15,
                4 => 20,
                5 => 21,
                6 => 22,
                7 => 23,
                8 => 24,
                9 => 25,
                10 => 26,
                11 => 27,
                12 => 28,
                13 => 29,
                14 => 30,
                15 => 31,
                16 => 85,
                17 => 86,
                18 => 87,
                19 => 88,
                20 => 89,
                21 => 90,
                22 => 102,
                23 => 103,
                24 => 104,
                25 => 105,
                26 => 106,
                28 => 107,
                29 => 108,
                30 => 109,
                31 => 110,
                36 => 111,
                _ => {
                    // unknown button was pressed, do nothing
                    return;
                }
            };

            if button_state == 129 {
                // pressed!
                // ufmt::uwriteln!(serial, "Pressed:  {}", button_name).unwrap_infallible();
                send_midi(serial, MidiEvent::CC, midi_controller_id, 127)
            }
            if button_state == 128 {
                // released!
                // ufmt::uwriteln!(serial, "Released: {}", button_name).unwrap_infallible();
                send_midi(serial, MidiEvent::CC, midi_controller_id, 0)
            }
        }
        _ => {
            // when there is nothing to read from the console, or something bad happend,
            // do nothing
            // ufmt::uwriteln!(serial, "Nothing").unwrap_infallible();
            return;
        }
    };
}

fn clamp(x: u16, min: u16, max: u16) -> u16 {
    if x < min {
        return min;
    };
    if x > max {
        return max;
    };
    return x;
}

fn map_range(x: u16, in_min: u16, in_max: u16, out_min: u16, out_max: u16) -> u16 {
    let a = x - in_min;
    let b = a as u32 * out_max as u32;
    let c = b / in_max as u32;

    return (c + out_min as u32) as u16;
}

type Console = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;

fn calc_velocity(time: u32) -> u8 {
    let mut time = time;

    if time > KEY_MAX_TIME_MS {
        time = KEY_MAX_TIME_MS;
    }
    if time < KEY_MIN_TIME_MS {
        time = KEY_MIN_TIME_MS;
    }

    time -= KEY_MIN_TIME_MS;

    let velocity = 127 - (time * 127 / KEY_MAX_TIME_MS_N);

    let velocity = (((velocity * velocity) >> 7) * velocity) >> 7;
    velocity as u8
}

fn send_midi(serial: &mut Console, event_type: MidiEvent, value_1: u8, value_2: u8) {
    const CHANNEL: u8 = 0;
    match event_type {
        MidiEvent::On => {
            serial.write_byte(0x90 + CHANNEL);
        }
        MidiEvent::Off => {
            serial.write_byte(0x80 + CHANNEL);
        }
        MidiEvent::PitchBend => {
            serial.write_byte(0xE0 + CHANNEL);
        }
        MidiEvent::CC => {
            serial.write_byte(0xB0 + CHANNEL);
        }
    }

    serial.write_byte(value_1);
    serial.write_byte(value_2);

    serial.flush();
}
