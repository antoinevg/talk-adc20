#![no_main]
#![no_std]

use panic_semihosting as _;
use daisy_bsp::hal as _;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    cortex_m::asm::bkpt();

    loop {
    }
}
