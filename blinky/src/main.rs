#![no_std]
#![no_main]

use byte_strings::c;
use core::cell::Cell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use panic_semihosting as _;
use static_cell::StaticCell;
use stm32mp1::stm32mp157;

mod hal;

const DEMO_STACK_SIZE: usize = 8192;
const DEMO_POOL_SIZE: usize = (DEMO_STACK_SIZE * 2) + 16384;

static LED_STATE: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

const SYSTEM_CLOCK: u32 = 64_000_000;
const SYSTICK_CYCLES: u32 = (SYSTEM_CLOCK / 100) - 1;

fn bsp_led_toggle() {
    let mut state = false;
    critical_section::with(|token| {
        state = LED_STATE.borrow(token).get();
        LED_STATE.borrow(token).set(!state);
    });
    if state {
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

#[no_mangle]
extern "C" fn tx_application_define(_first_unused_memory: *mut core::ffi::c_void) {
    // ThreadX requires a non-const pointer to char for the names, which it
    // wil hold on to in the object, so it must have static lifetime. So we
    // cast-away-const on a static string slice to appease the API.

    let byte_pool = {
        static BYTE_POOL: StaticCell<threadx_sys::TX_BYTE_POOL> = StaticCell::new();
        static BYTE_POOL_STORAGE: StaticCell<[u8; DEMO_POOL_SIZE]> = StaticCell::new();
        let byte_pool = BYTE_POOL.uninit();
        let byte_pool_storage = BYTE_POOL_STORAGE.uninit();
        unsafe {
            threadx_sys::_tx_byte_pool_create(
                byte_pool.as_mut_ptr(),
                c!("byte-pool0").as_ptr() as *mut threadx_sys::CHAR,
                byte_pool_storage.as_mut_ptr() as *mut _,
                DEMO_POOL_SIZE as u32,
            );
            byte_pool.assume_init_mut()
        }
    };

    let _thread = {
        let mut stack_pointer = core::ptr::null_mut();
        unsafe {
            threadx_sys::_tx_byte_allocate(
                byte_pool,
                &mut stack_pointer,
                DEMO_STACK_SIZE as _,
                threadx_sys::TX_NO_WAIT,
            );
        }
        if stack_pointer.is_null() {
            panic!("No space for stack");
        }

        static THREAD_STORAGE: StaticCell<threadx_sys::TX_THREAD> = StaticCell::new();
        let thread = THREAD_STORAGE.uninit();
        unsafe {
            let res = threadx_sys::_tx_thread_create(
                thread.as_mut_ptr(),
                c!("thread0").as_ptr() as *mut threadx_sys::CHAR,
                Some(my_thread),
                0,
                stack_pointer,
                DEMO_STACK_SIZE as _,
                1,
                1,
                threadx_sys::TX_NO_TIME_SLICE,
                threadx_sys::TX_AUTO_START,
            );
            if res != threadx_sys::TX_SUCCESS {
                panic!("Failed to create thread: {}", res);
            }
            thread.assume_init_mut()
        }
    };
}

extern "C" fn my_thread(_entry_value: u32) {
    loop {
        bsp_led_toggle();
        unsafe {
            threadx_sys::_tx_thread_sleep(50);
        }
    }
}

#[entry]
fn main() -> ! {
    let mut core = cortex_m::Peripherals::take().unwrap();
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

    // Enable cycle counter
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();

    // Enable the systick
    core.SYST.set_reload(SYSTICK_CYCLES);
    core.SYST.clear_current();
    core.SYST.enable_interrupt();
    core.SYST
        .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    core.SYST.enable_counter();

    // Busy-wait forever.
    unsafe {
        threadx_sys::_tx_initialize_kernel_enter();
    }
    panic!("Kernel exited!");
}

use stm32mp1::stm32mp157::interrupt;
#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn EXTI14() {
    // NOTE: This data needs to be declared at the start of the function, or else the
    // cortex_m_rt::interrupt proc-macro fails to compile.
    // static mut STATE: bool = false;

    // Clear the interrupt
    unsafe {
        (*stm32mp157::EXTI::PTR)
            .exti_fpr1
            .write(|w| w.fpif14().set_bit())
    };

    bsp_led_toggle();
}

#[no_mangle]
unsafe extern "C" fn __tx_SysTickHandler() {
    extern "C" {
        fn _tx_timer_interrupt();
    }
    // Call into OS function (not in public API)
    _tx_timer_interrupt();
    // Can do any extra work here
}
