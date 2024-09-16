#![no_std]
#![no_main]

use core::ops::Not;

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32mp1::stm32mp157;

mod hal;

// TODO: This is not a full translation.
fn bsp_led_init(rcc: &stm32mp157::RCC, hsem: &stm32mp157::HSEM, gpioh: &stm32mp157::GPIOH) {
    // LED7_GPIO_CLK_ENABLE
    rcc.rcc_mc_ahb4ensetr.write(|w| w.gpiohen().set_bit());
    // BSP_ENTER_CRITICAL_SECTION
    // NOTE: The semaphore id needs to be synchronized between cores. For this application, it
    // doesn't matter at all.
    while hal::hsem_fasttake(hsem, 0).is_err() {}

    hal::gpio_init_h7(gpioh);

    // BSP_EXIT_CRITICAL_SECTION
    hal::hsem_release(hsem, 0);

    // BSP_LED_Off()
    gpioh.gpioh_bsrr.write(|w| w.bs7().clear_bit());
}

fn exti14_irq_handler_config(
    rcc: &stm32mp157::RCC,
    hsem: &stm32mp157::HSEM,
    gpioa: &stm32mp157::GPIOA,
    exti: &stm32mp157::EXTI,
    nvic: &mut stm32mp157::NVIC,
) {
    // __HAL_RCC_GPIOA_CLK_ENABLE
    rcc.rcc_mc_ahb4ensetr.write(|w| w.gpioaen().set_bit());
    while hal::hsem_fasttake(hsem, 0).is_err() {}
    hal::gpio_init_a14(gpioa);
    hal::exti_set_config_line(exti);
    hal::hsem_release(hsem, 0);
    // NOTE: Don't need to set interrupt handler here because it's statically bound at link time.
    unsafe { nvic.set_priority(stm32mp157::Interrupt::EXTI14, 3) };
    // Enable the interrupt
    unsafe { stm32mp157::NVIC::unmask(stm32mp157::Interrupt::EXTI14) };
}

#[entry]
fn main() -> ! {
    let core = cortex_m::Peripherals::take().unwrap();
    let mut scb = core.SCB;

    let p = stm32mp157::Peripherals::take().unwrap();
    let rcc = p.RCC;
    let syscfg = p.SYSCFG;
    let pwr = p.PWR;
    let hsem = p.HSEM;
    let gpioa = p.GPIOA;
    let gpioh = p.GPIOH;
    let exti = p.EXTI;
    let mut nvic = core.NVIC;

    hal::init(&mut scb, &syscfg, &rcc, &pwr);

    // __HAL_RCC_HSEM_CLK_ENABLE
    rcc.rcc_mc_ahb3ensetr.write(|w| w.hsemen().set_bit());

    bsp_led_init(&rcc, &hsem, &gpioh);
    exti14_irq_handler_config(&rcc, &hsem, &gpioa, &exti, &mut nvic);

    // Busy-wait forever.
    loop {}
}

use stm32mp1::stm32mp157::interrupt;
#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn EXTI14() {
    // NOTE: This data needs to be declared at the start of the function, or else the
    // cortex_m_rt::interrupt proc-macro fails to compile.
    static mut STATE: bool = false;

    // Clear the interrupt
    unsafe {
        (*stm32mp157::EXTI::PTR)
            .exti_fpr1
            .write(|w| w.fpif14().set_bit())
    };

    // Toggle the LED
    *STATE = STATE.not();
    if *STATE {
        unsafe {
            (*stm32mp157::GPIOH::PTR)
                .gpioh_bsrr
                .write(|w| w.bs7().set_bit());
        }
    } else {
        unsafe {
            (*stm32mp157::GPIOH::PTR)
                .gpioh_brr
                .write(|w| w.br7().set_bit());
        }
    }
}
