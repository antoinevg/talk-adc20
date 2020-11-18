#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp::hal;
use hal::{pac, prelude::*};

use daisy_bsp::embedded_hal;
use embedded_hal::digital::v2::OutputPin;


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // configure power & clock
    let vos  = dp.PWR.constrain().freeze();
    let ccdr = dp.RCC.constrain()
                 .sys_ck(100.mhz())
                 .freeze(vos, &dp.SYSCFG);

    // configure pin
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut led_user = gpioc.pc7.into_push_pull_output();

    let one_second = ccdr.clocks.sys_ck().0;

    loop {
        led_user.set_high().unwrap();
        cortex_m::asm::delay(one_second);

        led_user.set_low().unwrap();
        cortex_m::asm::delay(one_second);
    }
}
