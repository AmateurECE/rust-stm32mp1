#![no_std]
#![no_main]

use core::ops::Not;

use cortex_m::peripheral::scb::SystemHandler;
use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32mp1::stm32mp157;

const SCB_AIRCR_VECTKEY_POS: u32 = 16;
const SCB_AIRCR_VECTKEY_MASK: u32 = 0xFFFF << SCB_AIRCR_VECTKEY_POS;
const SCB_AIRCR_PRIGROUP_POS: u32 = 8;
const SCB_AIRCR_PRIGROUP_MASK: u32 = 7 << SCB_AIRCR_PRIGROUP_POS;

const NVIC_PRIORITYGROUP_4: u8 = 0x3;
// TODO: Getting a HardFault in this function
fn nvic_set_priority_grouping(scb: &cortex_m::peripheral::SCB, priority: u8) {
    unsafe {
        scb.aircr.modify(|mut value| {
            value &= !(SCB_AIRCR_VECTKEY_MASK | SCB_AIRCR_PRIGROUP_MASK);
            value
                | (0x5FA << SCB_AIRCR_VECTKEY_POS)
                | (u32::from(priority) << SCB_AIRCR_PRIGROUP_POS)
        })
    };
}

fn is_engineering_boot_mode(syscfg: &stm32mp157::SYSCFG) -> bool {
    let boot0 = syscfg.syscfg_bootr.read().boot0().bit();
    let boot1 = syscfg.syscfg_bootr.read().boot1().bit();
    let boot2 = syscfg.syscfg_bootr.read().boot2().bit();
    !boot0 && !boot1 && boot2
}

fn hal_rcc_deinit(rcc: &stm32mp157::RCC) {
    // Enable HSI oscillator and wait until it's ready.
    rcc.rcc_ocensetr.write(|w| w.hsion().set_bit());
    while !rcc.rcc_ocrdyr.read().hsirdy().bit() {}

    // Reset a bunch of configuration, clock selection, and clock divider registers.
    rcc.rcc_mco1cfgr.reset();
    rcc.rcc_mco2cfgr.reset();
    rcc.rcc_assckselr.reset();
    rcc.rcc_mssckselr.reset();
    rcc.rcc_mpckdivr.reset();
    rcc.rcc_axidivr.reset();
    rcc.rcc_apb4divr.reset();
    rcc.rcc_apb5divr.reset();
    rcc.rcc_mcudivr.reset();
    rcc.rcc_apb1divr.reset();
    rcc.rcc_apb2divr.reset();
    rcc.rcc_apb3divr.reset();

    // Disable PLL1 outputs
    rcc.rcc_pll1cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });

    // Disable PLL1 and wait until it's disabled
    rcc.rcc_pll1cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll1cr.read().pll1rdy().bit() {}
    rcc.rcc_pll1cr.write(|w| w.sscg_ctrl().clear_bit());

    // Disable PLL2 outputs
    rcc.rcc_pll2cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });

    // Disable PLL2 and wait until it's disabled
    rcc.rcc_pll2cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll2cr.read().pll2rdy().bit() {}
    rcc.rcc_pll2cr.write(|w| w.sscg_ctrl().clear_bit());

    // Disable PLL3 outputs
    rcc.rcc_pll3cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });

    // Disable PLL3 and wait until it's disabled
    rcc.rcc_pll3cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll3cr.read().pll3rdy().bit() {}
    rcc.rcc_pll3cr.write(|w| w.sscg_ctrl().clear_bit());

    // Disable PLL4 outputs
    rcc.rcc_pll4cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });

    // Disable PLL4 and wait until it's disabled
    rcc.rcc_pll4cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll4cr.read().pll4rdy().bit() {}
    rcc.rcc_pll4cr.write(|w| w.sscg_ctrl().clear_bit());

    rcc.rcc_rck12selr.reset();
    rcc.rcc_rck3selr.reset();
    rcc.rcc_rck4selr.reset();

    rcc.rcc_pll1cfgr1.reset();
    rcc.rcc_pll1cfgr2.reset();
    rcc.rcc_pll1fracr.reset();
    rcc.rcc_pll1csgr.reset();

    rcc.rcc_pll2cfgr1.reset();
    rcc.rcc_pll2cfgr2.reset();
    rcc.rcc_pll2fracr.reset();
    rcc.rcc_pll2csgr.reset();

    rcc.rcc_pll3cfgr1.reset();
    rcc.rcc_pll3cfgr2.reset();
    rcc.rcc_pll3fracr.reset();
    rcc.rcc_pll3csgr.reset();

    rcc.rcc_pll4cfgr1.reset();
    rcc.rcc_pll4cfgr2.reset();
    rcc.rcc_pll4fracr.reset();
    rcc.rcc_pll4csgr.reset();

    // Reset HSIDIV once PLLs are off
    rcc.rcc_hsicfgr.write(|w| unsafe { w.hsidiv().bits(0) });
    while !rcc.rcc_ocrdyr.read().hsidivrdy().bit() {}

    // Reset all of the Oscillator Enable Control bits (except HSION)
    // NOTE: See RM0422. Writing 0 to these bits has no effect. Writing 1 has the effect of
    // clearing the bit. Explicitly "clear" HSION, because otherwise, the svd2rust generated code
    // will set that bit to it's reset value, which is (you guessed it) 1. If that occurs,
    // the HSI oscillator is disabled and control of the processor is lost.
    rcc.rcc_ocenclrr.write(|w| {
        w.hsikeron()
            .set_bit()
            .csion()
            .set_bit()
            .csikeron()
            .set_bit()
            .digbyp()
            .set_bit()
            .hseon()
            .set_bit()
            .hsekeron()
            .set_bit()
            .hsebyp()
            .set_bit()
            .hsion()
            .clear_bit()
    });
    rcc.rcc_rdlsicr.write(|w| w.lsion().clear_bit());
    rcc.rcc_csicfgr.write(|w| unsafe { w.csitrim().bits(0) });
    rcc.rcc_mc_cier.reset();
    rcc.rcc_mc_cifr.write(|w| {
        w.lsirdyf()
            .set_bit()
            .lserdyf()
            .set_bit()
            .hsirdyf()
            .set_bit()
            .hserdyf()
            .set_bit()
            .csirdyf()
            .set_bit()
            .pll1dyf()
            .set_bit()
            .pll2dyf()
            .set_bit()
            .pll3dyf()
            .set_bit()
            .pll4dyf()
            .set_bit()
            .lsecssf()
            .set_bit()
            .wkupf()
            .set_bit()
    });
    rcc.rcc_mc_rstsclrr.write(|w| {
        w.wwdg1rstf()
            .set_bit()
            .iwdg2rstf()
            .set_bit()
            .iwdg1rstf()
            .set_bit()
            .mcsysrstf()
            .set_bit()
            .mpsysrstf()
            .set_bit()
            .mcurstf()
            .set_bit()
            .vcorerstf()
            .set_bit()
            .hcssrstf()
            .set_bit()
            .padrstf()
            .set_bit()
            .borrstf()
            .set_bit()
            .porrstf()
            .set_bit()
    });
}

// TODO: This is not a direct translation.
fn rcc_lse_config(rcc: &stm32mp157::RCC) {
    rcc.rcc_bdcr.write(|w| w.lseon().clear_bit());
    while rcc.rcc_bdcr.read().lserdy().bit() {}
    rcc.rcc_bdcr.write(|w| w.lsebyp().clear_bit());
    // Have to clear the DIGBYP field. See stm32-rs/stm32-rs#1010.
    // TODO: Double-check this is correct usage
    rcc.rcc_bdcr
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 3)) });

    rcc.rcc_bdcr.write(|w| w.lseon().set_bit());
    while !rcc.rcc_bdcr.read().lserdy().bit() {}
}

// TODO: This is not a direct translation.
fn rcc_pll1_config(rcc: &stm32mp157::RCC) {
    // Disable the post-dividers
    // __HAL_RCC_PLL1CLKOUT_DISABLE
    rcc.rcc_pll1cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });
    // Disable the main PLL
    rcc.rcc_pll1cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll1cr.read().pll1rdy().bit() {}

    const RCC_PLL12SOURCE_HSE: u8 = 1;
    rcc.rcc_rck12selr
        .write(|w| unsafe { w.pll12src().bits(RCC_PLL12SOURCE_HSE) });
    while !rcc.rcc_rck12selr.read().pll12srcrdy().bit() {}

    // NOTE: PLL3/PLL4 differ here:
    // __HAL_RCC_PLL3_IFRANGE(pll3->PLLRGE);

    // __HAL_RCC_PLL1_CONFIG
    // TODO: I have no idea what these magic numbers mean.
    rcc.rcc_pll1cfgr1
        .write(|w| unsafe { w.divn().bits(81).divm1().bits(3) });
    rcc.rcc_pll1cfgr2
        .write(|w| unsafe { w.divp().bits(1).divq().bits(1).divr().bits(1) });

    // Configure the fractional divider
    // __HAL_RCC_PLL1FRACV_DISABLE
    rcc.rcc_pll1fracr.write(|w| w.fracle().clear_bit());
    // __HAL_RCC_PLL1FRACV_CONFIG
    // NOTE: PLL4 differs here slightly:
    // __HAL_RCC_PLL4FRACV_CONFIG(0)
    rcc.rcc_pll1fracr
        .write(|w| unsafe { w.fracv().bits(0x800) });
    // __HAL_RCC_PLL1FRACV_ENABLE
    rcc.rcc_pll1fracr.write(|w| w.fracle().set_bit());

    // __HAL_RCC_PLL1_SSMODE_DISABLE
    rcc.rcc_pll1cr.write(|w| w.sscg_ctrl().clear_bit());

    // __HAL_RCC_PLL1_ENABLE
    rcc.rcc_pll1cr.write(|w| w.pllon().set_bit());
    while !rcc.rcc_pll1cr.read().pll1rdy().bit() {}

    // __HAL_RCC_PLL1CLKOUT_ENABLE
    rcc.rcc_pll1cr
        .write(|w| w.divpen().set_bit().divqen().set_bit().divren().set_bit());
}

// TODO: This is not a direct translation.
fn rcc_pll2_config(rcc: &stm32mp157::RCC) {
    // Disable the post-dividers
    // __HAL_RCC_PLL2CLKOUT_DISABLE
    rcc.rcc_pll2cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });
    // Disable the main PLL
    rcc.rcc_pll2cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll2cr.read().pll2rdy().bit() {}

    const RCC_PLL12SOURCE_HSE: u8 = 1;
    rcc.rcc_rck12selr
        .write(|w| unsafe { w.pll12src().bits(RCC_PLL12SOURCE_HSE) });

    // NOTE: PLL3/PLL4 differ here:
    // __HAL_RCC_PLL3_IFRANGE(pll3->PLLRGE);

    // __HAL_RCC_PLL2_CONFIG
    // TODO: I have no idea what these magic numbers mean.
    rcc.rcc_pll2cfgr1
        .write(|w| unsafe { w.divn().bits(66).divm2().bits(3) });
    rcc.rcc_pll2cfgr2
        .write(|w| unsafe { w.divp().bits(2).divq().bits(1).divr().bits(1) });

    // Configure the fractional divider
    // __HAL_RCC_PLL2FRACV_DISABLE
    rcc.rcc_pll2fracr.write(|w| w.fracle().clear_bit());
    // __HAL_RCC_PLL2FRACV_CONFIG
    // NOTE: PLL4 differs here slightly:
    // __HAL_RCC_PLL4FRACV_CONFIG(0)
    rcc.rcc_pll2fracr
        .write(|w| unsafe { w.fracv().bits(0x1400) });
    // __HAL_RCC_PLL2FRACV_ENABLE
    rcc.rcc_pll2fracr.write(|w| w.fracle().set_bit());

    // __HAL_RCC_PLL2_SSMODE_DISABLE
    rcc.rcc_pll2cr.write(|w| w.sscg_ctrl().clear_bit());

    // __HAL_RCC_PLL2_ENABLE
    rcc.rcc_pll2cr.write(|w| w.pllon().set_bit());
    while !rcc.rcc_pll2cr.read().pll2rdy().bit() {}

    // __HAL_RCC_PLL2CLKOUT_ENABLE
    rcc.rcc_pll2cr
        .write(|w| w.divpen().set_bit().divqen().set_bit().divren().set_bit());
}

// TODO: This is not a direct translation.
fn rcc_pll3_config(rcc: &stm32mp157::RCC) {
    // Disable the post-dividers
    // __HAL_RCC_PLL3CLKOUT_DISABLE
    rcc.rcc_pll3cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });
    // Disable the main PLL
    rcc.rcc_pll3cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll3cr.read().pll3rdy().bit() {}

    const RCC_PLL3SOURCE_HSE: u8 = 1;
    rcc.rcc_rck3selr
        .write(|w| unsafe { w.pll3src().bits(RCC_PLL3SOURCE_HSE) });

    // NOTE: PLL3/PLL4 differ from PLL1/PLL2 here:
    // __HAL_RCC_PLL3_IFRANGE(pll3->PLLRGE);
    rcc.rcc_pll3cfgr1.write(|w| unsafe { w.ifrge().bits(0) });

    // __HAL_RCC_PLL3_CONFIG
    // TODO: I have no idea what these magic numbers mean.
    rcc.rcc_pll3cfgr1
        .write(|w| unsafe { w.divn().bits(34).divm3().bits(2) });
    rcc.rcc_pll3cfgr2
        .write(|w| unsafe { w.divp().bits(2).divq().bits(17).divr().bits(37) });

    // Configure the fractional divider
    // __HAL_RCC_PLL3FRACV_DISABLE
    rcc.rcc_pll3fracr.write(|w| w.fracle().clear_bit());
    // __HAL_RCC_PLL3FRACV_CONFIG
    rcc.rcc_pll3fracr
        .write(|w| unsafe { w.fracv().bits(0x1A04) });
    // __HAL_RCC_PLL3FRACV_ENABLE
    rcc.rcc_pll3fracr.write(|w| w.fracle().set_bit());

    // __HAL_RCC_PLL3_SSMODE_DISABLE
    rcc.rcc_pll3cr.write(|w| w.sscg_ctrl().clear_bit());

    // __HAL_RCC_PLL3_ENABLE
    rcc.rcc_pll3cr.write(|w| w.pllon().set_bit());
    while !rcc.rcc_pll3cr.read().pll3rdy().bit() {}

    // __HAL_RCC_PLL3CLKOUT_ENABLE
    rcc.rcc_pll3cr
        .write(|w| w.divpen().set_bit().divqen().set_bit().divren().set_bit());
}

// TODO: This is not a direct translation.
fn rcc_pll4_config(rcc: &stm32mp157::RCC) {
    // Disable the post-dividers
    // __HAL_RCC_PLL4CLKOUT_DISABLE
    rcc.rcc_pll4cr.write(|w| {
        w.divpen()
            .clear_bit()
            .divqen()
            .clear_bit()
            .divren()
            .clear_bit()
    });
    // Disable the main PLL
    rcc.rcc_pll4cr.write(|w| w.pllon().clear_bit());
    while rcc.rcc_pll4cr.read().pll4rdy().bit() {}

    const RCC_PLL4SOURCE_HSE: u8 = 4;
    rcc.rcc_rck4selr
        .write(|w| unsafe { w.pll4src().bits(RCC_PLL4SOURCE_HSE) });

    // NOTE: PLL3/PLL4 differ from PLL1/PLL2 here:
    // __HAL_RCC_PLL4_IFRANGE(pll4->PLLRGE);
    rcc.rcc_pll4cfgr1.write(|w| unsafe { w.ifrge().bits(0) });

    // __HAL_RCC_PLL4_CONFIG
    // TODO: I have no idea what these magic numbers mean.
    rcc.rcc_pll4cfgr1
        .write(|w| unsafe { w.divn().bits(99).divm4().bits(4) });
    rcc.rcc_pll4cfgr2
        .write(|w| unsafe { w.divp().bits(6).divq().bits(8).divr().bits(8) });

    // Configure the fractional divider
    // __HAL_RCC_PLL4FRACV_DISABLE
    rcc.rcc_pll4fracr.write(|w| w.fracle().clear_bit());
    // __HAL_RCC_PLL4FRACV_CONFIG
    rcc.rcc_pll4fracr.write(|w| unsafe { w.fracv().bits(0) });
    // __HAL_RCC_PLL4FRACV_ENABLE
    rcc.rcc_pll4fracr.write(|w| w.fracle().set_bit());

    // __HAL_RCC_PLL4_SSMODE_DISABLE
    rcc.rcc_pll4cr.write(|w| w.sscg_ctrl().clear_bit());

    // __HAL_RCC_PLL4_ENABLE
    rcc.rcc_pll4cr.write(|w| w.pllon().set_bit());
    while !rcc.rcc_pll4cr.read().pll4rdy().bit() {}

    // __HAL_RCC_PLL4CLKOUT_ENABLE
    rcc.rcc_pll4cr
        .write(|w| w.divpen().set_bit().divqen().set_bit().divren().set_bit());
}

const RCC_BDCR_LSEDRV_1: u8 = 0x2 << 4;

// TODO: This is not a direct translation.
fn rcc_osc_config(rcc: &stm32mp157::RCC, pwr: &stm32mp157::PWR, scb: &mut stm32mp157::SCB) {
    // NOTE: See other note about why we explicitly clear HSION here and below.
    rcc.rcc_ocenclrr
        .write(|w| w.hseon().set_bit().hsion().clear_bit());
    // Wait until HSERDY flag is set.
    while rcc.rcc_ocrdyr.read().hserdy().bit() {}
    rcc.rcc_ocenclrr
        .write(|w| w.digbyp().set_bit().hsebyp().set_bit().hsion().clear_bit());
    rcc.rcc_ocensetr
        .write(|w| w.digbyp().set_bit().hsebyp().set_bit());
    rcc.rcc_ocensetr.write(|w| w.hseon().set_bit());
    // Wait until HSERDY flag is set.
    while !rcc.rcc_ocrdyr.read().hserdy().bit() {}

    // __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST
    let hsi_calibration_value = 16;
    rcc.rcc_hsicfgr
        .write(|w| unsafe { w.hsitrim().bits(hsi_calibration_value) });

    // __HAL_RCC_HSI_DIV
    let hsi_div_value = 0;
    rcc.rcc_hsicfgr
        .write(|w| unsafe { w.hsidiv().bits(hsi_div_value) });
    // __HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIVRDY)
    while !rcc.rcc_ocrdyr.read().hsidivrdy().bit() {}

    // HAL_InitTick(TICK_INT_PRIORITY=15)
    unsafe { scb.set_priority(SystemHandler::SysTick, 15) };

    // Enable write access to backup domain
    pwr.pwr_cr1.write(|w| w.dbp().set_bit());
    // Wait for backup domain write protection to be disabled.
    while !pwr.pwr_cr1.read().dbp().bit() {}

    rcc_lse_config(rcc);
    rcc_pll1_config(rcc);
    rcc_pll2_config(rcc);
    rcc_pll3_config(rcc);
    rcc_pll4_config(rcc);
}

const RCC_MPUSOURCE_PLL1: u8 = 0x2;

fn rcc_mpu_config(rcc: &stm32mp157::RCC) {
    // __HAL_RCC_MPU_SOURCE
    rcc.rcc_mpckselr
        .write(|w| unsafe { w.mpusrc().bits(RCC_MPUSOURCE_PLL1) });
    while !rcc.rcc_mpckselr.read().mpusrcrdy().bit() {}
}

const RCC_AXISSOURCE_PLL2: u8 = 0x2;
const RCC_AXI_DIV1: u8 = 0x0;

fn rcc_axiss_config(rcc: &stm32mp157::RCC) {
    // __HAL_RCC_AXISS_SOURCE()
    rcc.rcc_assckselr
        .write(|w| unsafe { w.axissrc().bits(RCC_AXISSOURCE_PLL2) });
    while !rcc.rcc_assckselr.read().axissrcrdy().bit() {}

    // __HAL_RCC_AXI_DIV()
    rcc.rcc_axidivr
        .write(|w| unsafe { w.axidiv().bits(RCC_AXI_DIV1) });
    while !rcc.rcc_axidivr.read().axidivrdy().bit() {}
}

const RCC_MCUSSOURCE_PLL3: u8 = 0x3;
const RCC_MCU_DIV1: u8 = 0x0;

fn rcc_mcu_config(rcc: &stm32mp157::RCC) {
    // __HAL_RCC_MCU_SOURCE
    rcc.rcc_mssckselr
        .write(|w| unsafe { w.mcussrc().bits(RCC_MCUSSOURCE_PLL3) });
    while !rcc.rcc_mssckselr.read().mcussrcrdy().bit() {}

    // __HAL_RCC_MCU_DIV
    rcc.rcc_mcudivr
        .write(|w| unsafe { w.mcudiv().bits(RCC_MCU_DIV1) });
    while !rcc.rcc_mcudivr.read().mcudivrdy().bit() {}
}

const RCC_APB4_DIV2: u8 = 0x1;
const RCC_APB5_DIV4: u8 = 0x2;
const RCC_APB1_DIV2: u8 = 0x1;
const RCC_APB2_DIV2: u8 = 0x1;
const RCC_APB3_DIV2: u8 = 0x1;

fn rcc_clock_config(rcc: &stm32mp157::RCC) {
    rcc_mpu_config(rcc);
    rcc_axiss_config(rcc);
    rcc_mcu_config(rcc);

    // __HAL_RCC_APB4_DIV
    rcc.rcc_apb4divr
        .write(|w| unsafe { w.apb4div().bits(RCC_APB4_DIV2) });
    while !rcc.rcc_apb4divr.read().apb4divrdy().bit() {}

    // __HAL_RCC_APB5_DIV
    rcc.rcc_apb5divr
        .write(|w| unsafe { w.apb5div().bits(RCC_APB5_DIV4) });
    while !rcc.rcc_apb5divr.read().apb5divrdy().bit() {}

    // __HAL_RCC_APB1_DIV
    rcc.rcc_apb1divr
        .write(|w| unsafe { w.apb1div().bits(RCC_APB1_DIV2) });
    while !rcc.rcc_apb1divr.read().apb1divrdy().bit() {}

    // __HAL_RCC_APB2_DIV
    rcc.rcc_apb2divr
        .write(|w| unsafe { w.apb2div().bits(RCC_APB2_DIV2) });
    while !rcc.rcc_apb2divr.read().apb2divrdy().bit() {}

    // __HAL_RCC_APB3_DIV
    rcc.rcc_apb3divr
        .write(|w| unsafe { w.apb3div().bits(RCC_APB3_DIV2) });
    while !rcc.rcc_apb3divr.read().apb3divrdy().bit() {}
}

fn system_clock_config(rcc: &stm32mp157::RCC, pwr: &stm32mp157::PWR, scb: &mut stm32mp157::SCB) {
    // HAL_PWR_EnableBkUpAccess
    pwr.pwr_cr1.modify(|_, w| w.dbp().set_bit());

    // __HAL_RCC_LSEDRIVE_CONFIG()
    rcc.rcc_bdcr
        .modify(|_, w| unsafe { w.lsedrv().bits(RCC_BDCR_LSEDRV_1) });

    rcc_osc_config(rcc, pwr, scb);
    rcc_clock_config(rcc);

    // __HAL_RCC_RTC_HSEDIV()
    rcc.rcc_rtcdivr.write(|w| unsafe { w.rtcdiv().bits(24) });
}

fn hal_init(
    scb: &mut stm32mp157::SCB,
    syscfg: &stm32mp157::SYSCFG,
    rcc: &stm32mp157::RCC,
    pwr: &stm32mp157::PWR,
) {
    // NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4=3);
    nvic_set_priority_grouping(scb, NVIC_PRIORITYGROUP_4);

    // HAL_InitTick(TICK_INT_PRIORITY=15)
    unsafe { scb.set_priority(SystemHandler::SysTick, 15) };

    if is_engineering_boot_mode(syscfg) {
        hal_rcc_deinit(rcc);
        system_clock_config(rcc, pwr, scb);
    }
}

struct AlreadyLocked;

const HSEM_CPU2_COREID: u8 = 0x2;

fn hal_hsem_fasttake(hsem: &stm32mp157::HSEM, id: u32) -> Result<(), AlreadyLocked> {
    let (locked, core_id) = match id {
        0 => {
            let value = hsem.hsem_rlr0.read();
            (value.lock().bit(), value.coreid().bits())
        }
        _ => panic!("Unsupported semaphore!"),
    };
    if locked && core_id == HSEM_CPU2_COREID {
        Ok(())
    } else {
        Err(AlreadyLocked)
    }
}

fn hal_hsem_release(hsem: &stm32mp157::HSEM, id: u32) {
    match id {
        0 => hsem
            .hsem_r0
            .write(|w| unsafe { w.coreid().bits(HSEM_CPU2_COREID).procid().bits(0) }),
        _ => panic!("Unsupported semaphore!"),
    }
}

fn hal_gpio_init_h7(gpioh: &stm32mp157::GPIOH) {
    // LED7 on the DK is connected to GPIO H7. See RM0422 Rev 6, p.1068.
    // Configure H7 for "General purpose output mode."
    const GPIO_MODE_OUTPUT: u8 = 0x1;
    gpioh
        .gpioh_moder
        .write(|w| unsafe { w.moder7().bits(GPIO_MODE_OUTPUT) });
    // Configure for "very high speed"
    const GPIO_SPEED_FREQ_VERY_HIGH: u8 = 0x3;
    gpioh
        .gpioh_ospeedr
        .write(|w| unsafe { w.ospeedr7().bits(GPIO_SPEED_FREQ_VERY_HIGH) });
    // Configure for pull-up
    const GPIO_PULLUP: u8 = 0x1;
    gpioh
        .gpioh_pupdr
        .write(|w| unsafe { w.pupdr7().bits(GPIO_PULLUP) });
}

fn hal_gpio_init_a14(gpioa: &stm32mp157::GPIOA) {
    const GPIO_MODE_INPUT: u8 = 0;
    gpioa
        .gpioa_moder
        .write(|w| unsafe { w.moder14().bits(GPIO_MODE_INPUT) });
    const GPIO_NOPULL: u8 = 0;
    gpioa
        .gpioa_pupdr
        .write(|w| unsafe { w.pupdr14().bits(GPIO_NOPULL) });
}

// TODO: This is not a full translation.
fn bsp_led_init(rcc: &stm32mp157::RCC, hsem: &stm32mp157::HSEM, gpioh: &stm32mp157::GPIOH) {
    // LED7_GPIO_CLK_ENABLE
    rcc.rcc_mc_ahb4ensetr.write(|w| w.gpiohen().set_bit());
    // BSP_ENTER_CRITICAL_SECTION
    // NOTE: The semaphore id needs to be synchronized between cores. For this application, it
    // doesn't matter at all.
    while hal_hsem_fasttake(hsem, 0).is_err() {}

    hal_gpio_init_h7(gpioh);

    // BSP_EXIT_CRITICAL_SECTION
    hal_hsem_release(hsem, 0);

    // BSP_LED_Off()
    gpioh.gpioh_bsrr.write(|w| w.bs7().clear_bit());
}

fn hal_exti_set_config_line(exti: &stm32mp157::EXTI) {
    // Enable falling trigger for input line
    exti.exti_ftsr1.write(|w| w.ft14().set_bit());
    // Set pin PA[14] as input source for EXTI14 (the default)
    exti.exti_exticr4.write(|w| unsafe { w.exti14().bits(0) });
    // Enable CPU2 wakeup on EXTI14 interrupt
    exti.exti_c2imr1.write(|w| w.im14().set_bit());
    // TODO: What does this really mean?
    // Disable CPU2 wakeup on EXTI14 event (the default)
    exti.exti_c2emr1.write(|w| w.em14().clear_bit());
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
    while hal_hsem_fasttake(hsem, 0).is_err() {}
    hal_gpio_init_a14(gpioa);
    hal_exti_set_config_line(exti);
    hal_hsem_release(hsem, 0);
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

    hal_init(&mut scb, &syscfg, &rcc, &pwr);

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
