//! Simple test for using C bindings in Rust with `bindgen` and `cc`.
//!
//! RAIL SDK sources:
//! - https://docs.silabs.com/rail/latest/rail-api/efr32-main
//! - https://docs.silabs.com/rail/latest/rail-api
//!
//! Relevant resources:
//! - https://docs.rust-embedded.org/book/interoperability/c-with-rust.html
//! - https://rust-lang.github.io/rust-bindgen/tutorial-0.html
//! - https://blog.theembeddedrustacean.com/rust-ffi-and-bindgen-integrating-embedded-c-code-in-rust
//!
//! Infos about the clock configuration:
//! - https://www.silabs.com/documents/public/application-notes/an0004.2-efr32-series2-cmu.pdf
#![no_std]
#![no_main]

use defmt_rtt as _;
use efr32_rail::radio::Radio;
use efr32mg22_pac::{self as _, NVIC, Peripherals, interrupt};

static mut PACKET_RECEIVED: bool = false;
static mut BUTTON_PRESSED: bool = false;

#[cortex_m_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let radio = Radio::new(&peripherals, || unsafe { PACKET_RECEIVED = true });

    setup_led(&peripherals);
    setup_button_for_interrupt(&peripherals);

    let mut packet_sent_counter = 0;
    let mut packet_received_counter = 0;
    let mut led_enabled = false;
    loop {
        unsafe {
            if PACKET_RECEIVED {
                PACKET_RECEIVED = false;
                packet_received_counter += 1;

                let packet = radio.read_received_packet();
                defmt::info!("received packet {}: {:X}", packet_received_counter, packet);

                led_enabled = !led_enabled;
                set_led_state(&peripherals, led_enabled);
            }

            if BUTTON_PRESSED {
                BUTTON_PRESSED = false;
                packet_sent_counter += 1;

                let out_packet: [u8; _] = [
                    0x0F, 0x16, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB,
                    0xCC, 0xDD, 0xEE,
                ];
                radio.send_packet(out_packet);
                defmt::info!("sent packet {}: {:X}", packet_sent_counter, out_packet);

                led_enabled = !led_enabled;
                set_led_state(&peripherals, led_enabled);
            }
        }
    }
}

fn setup_led(peripherals: &Peripherals) {
    peripherals
        .cmu_ns
        .clken0()
        .write(|w| w.gpio().set_bit().gpio().set_bit());

    peripherals
        .gpio_ns
        .porta_model()
        .write(|w| w.mode4().pushpull());
}

fn set_led_state(peripherals: &Peripherals, high: bool) {
    // TODO: support passing pin no parameter to method - don't hardcode to pin 4
    peripherals
        .gpio_ns
        .porta_dout()
        .write(|w| unsafe { w.dout().bits(if high { 0b00010000 } else { 0b0 }) });
}

fn setup_button_for_interrupt(peripherals: &Peripherals) {
    unsafe { NVIC::unmask(interrupt::GPIO_ODD) };

    // configure button as input
    peripherals
        .gpio_ns
        .portc_model()
        .write(|w| w.mode7().input().mode7().input());

    // set dout for the pin in order to enable the "glitch filter"
    peripherals
        .gpio_ns
        .portc_dout()
        .write(|w| unsafe { w.dout().bits(0b10000000) });

    // to target pin 4-7, we have to use one of the external interrupt registers 4-7

    // set interrupt handler for port c, pin 7
    peripherals
        .gpio_ns
        .extipsell()
        .write(|w| w.extipsel5().portc());
    peripherals
        .gpio_ns
        .extipinsell()
        // offset 3 = pin 7
        .write(|w| w.extipinsel5().offset3());

    // trigger interrupt on edge fall
    peripherals
        .gpio_ns
        .extifall()
        // 100_000 = ext interrupt handler 5
        .write(|w| unsafe { w.extifall().bits(0b00_100_000) });

    // finally, enable the interrupt
    peripherals.gpio_ns.ien().write(|w| w.extien5().set_bit());
}

unsafe fn clear_button_interrupt_flag() {
    // clear interrupt flag
    // based on datasheet the GPIO base address and the offset for the IF_CLR register
    let base_addr = efr32mg22_pac::GpioNs::ptr() as u32;
    let clr_addr = base_addr + 0x0000_2420;
    let clr_ptr = clr_addr as *mut u32;

    unsafe {
        // Write a one to the register to clear the bit, it's a CLEAR register
        clr_ptr.write_volatile(0b0010_0000);
    }
}

// interrupts are defined in the Interrupt enum in `lib.rs`
#[interrupt]
fn GPIO_ODD() {
    unsafe {
        clear_button_interrupt_flag();
        BUTTON_PRESSED = true;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panicked");
    loop {
        cortex_m::asm::bkpt();
    }
}
