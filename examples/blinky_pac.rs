#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp::pac;


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // enable gpioc peripheral clock - pac
    let rcc = &dp.RCC;
    rcc.ahb4enr.modify(|_, w| w.gpiocen().set_bit());

    // configure user led pin
    let gpioc = &dp.GPIOC;
    gpioc.moder.modify(|_, w| w.moder7().output());
    gpioc.otyper.modify(|_, w| w.ot7().push_pull());
    gpioc.ospeedr.modify(|_, w| w.ospeedr7().high_speed());

    loop {
        gpioc.odr.modify(|_, w| w.odr7().high());
        cortex_m::asm::delay(100_000_000);

        gpioc.odr.modify(|_, w| w.odr7().low());
        cortex_m::asm::delay(100_000_000);
    }
}
