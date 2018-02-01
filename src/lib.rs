#![feature(proc_macro, used, lang_items, const_fn, generators, generator_trait, asm, 
           conservative_impl_trait, universal_impl_trait, inclusive_range_syntax,
           generic_associated_types, associated_type_defaults, fmt_internals)]

#![no_std]
#![allow(mutable_transmutes)]
#![allow(dead_code)]

extern crate futures_await as futures;
extern crate async_hal;
extern crate opencm;
extern crate cortex_m;
extern crate cortex_m_rt;

use cortex_m::asm;

pub mod hal;

// use self::hal::gpio;

#[lang="panic_fmt"]
#[no_mangle]
pub fn panic_fmt(_fmt: &core::fmt::Arguments, _file_line: &(&'static str, usize)) -> ! {
    // let red = gpio::get_pin(3, 14);
    // red.make_output();
    // red.set_high();

    loop { }
}

#[allow(private_no_mangle_statics)]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
#[used]
pub static INTERRUPTS: [Option<unsafe extern "C" fn()>; 90] = [
    /* WWDG */          Some(default_handler),
    /* PVD */           Some(default_handler),
    /* TAMP_STAMP */    Some(default_handler),
    /* RTC_WKUP */      Some(default_handler),
    None,
    /* RCC */           Some(default_handler),
    /* EXTI0 */         Some(default_handler /*hal::gpio::exti0_handler*/),
    /* EXTI1 */         Some(default_handler /*hal::gpio::exti1_handler*/),
    /* EXTI2 */         Some(default_handler /*hal::gpio::exti2_handler*/),
    /* EXTI3 */         Some(default_handler /*hal::gpio::exti3_handler*/),
    /* EXTI4 */         Some(default_handler /*hal::gpio::exti4_handler*/),
    /* DMA1_STREAM0 */  Some(default_handler),
    /* DMA1_STREAM1 */  Some(default_handler),
    /* DMA1_STREAM2 */  Some(default_handler),
    /* DMA1_STREAM3 */  Some(default_handler),
    /* DMA1_STREAM4 */  Some(default_handler),
    /* DMA1_STREAM5 */  Some(default_handler),
    /* DMA1_STREAM6 */  Some(default_handler),
    /* ADC */           Some(default_handler),
    /* CAN1_TX */       Some(default_handler),
    /* CAN1_RX0 */      Some(default_handler),
    /* CAN1_RX1 */      Some(default_handler),
    /* CAN1_SCE */      Some(default_handler),
    /* EXTI9_5 */       Some(default_handler /* hal::gpio::exti9_5_handler */),
    /* TIM1_BRK_TIM9 */ Some(default_handler),
    /* TIM1_UP_TIM10 */ Some(default_handler),
    /* TIM1_TRG_COM_TIM11 */ Some(default_handler),
    /* TIM1_CC */       Some(default_handler),
    /* TIM2 */          Some(default_handler),
    /* TIM3 */          Some(default_handler),
    /* TIM4 */          Some(default_handler),
    /* I2C1_EV */       Some(default_handler),
    /* I2C1_ER */       Some(default_handler),
    /* I2C2_EV */       Some(default_handler),
    /* I2C2_ER */       Some(default_handler),
    /* SPI1 */          Some(default_handler),
    /* SPI2 */          Some(default_handler),
    /* USART1 */        Some(default_handler),
    /* USART2 */        Some(default_handler),
    /* USART3 */        Some(default_handler),
    /* EXTI15_10 */     Some(default_handler /* hal::gpio::exti15_10_handler */),
    /* RTC_ALARM */     Some(default_handler),
    /* OTG_FS_WKUP */   Some(default_handler),
    /* TIM8_BRK_TIM12 */ Some(default_handler),
    /* TIM8_UP_TIM13 */ Some(default_handler),
    /* TIM8_TRG_COM_TIM14 */ Some(default_handler),
    /* TIM8_CC */       Some(default_handler),
    /* DMA1_STREAM7 */  Some(default_handler),
    /* FSMC */          Some(default_handler),
    /* SDIO */          Some(default_handler),
    /* TIM5 */          Some(default_handler),
    /* SPI3 */          Some(default_handler),
    /* UART4 */         Some(default_handler),
    /* UART5 */         Some(default_handler),
    /* TIM6_DAC */      Some(hal::timer::tim6_handler),
    /* TIM7 */          Some(hal::timer::tim7_handler),
    /* DMA2_STREAM0 */  Some(default_handler),
    /* DMA2_STREAM1 */  Some(default_handler),
    /* DMA2_STREAM2 */  Some(default_handler),
    /* DMA2_STREAM3 */  Some(default_handler),
    /* DMA2_STREAM4 */  Some(default_handler),
    /* ETH */           Some(default_handler),
    /* ETH_WKUP */      Some(default_handler),
    /* CAN2_TX */       Some(default_handler),
    /* CAN2_RX0 */      Some(default_handler),
    /* CAN2_RX1 */      Some(default_handler),
    /* CAN2_SCE */      Some(default_handler),
    /* OTG_FS */        Some(default_handler),
    /* DMA2_STREAM5 */  Some(default_handler),
    /* DMA2_STREAM6 */  Some(default_handler),
    /* DMA2_STREAM7 */  Some(default_handler),
    /* USART6 */        Some(default_handler),
    /* I2C3_EV */       Some(default_handler),
    /* I2C3_ER */       Some(default_handler),
    /* OTG_HS_EP1_OUT */ Some(default_handler),
    /* OTG_HS_EP1_IN */ Some(default_handler),
    /* OTG_HS_WKUP */   Some(default_handler),
    /* OTG_HS */        Some(default_handler),
    /* DCMI */          Some(default_handler),
    /* CRYP */          Some(default_handler),
    /* HASH_RNG */      Some(default_handler),
    /* FPU */           Some(default_handler),
    None,
    None,
    None,
    None,
    None,
    None,
    /* LCD_TFT */       Some(default_handler),
    /* LCD_TFT_1 */     Some(default_handler),
];

extern "C" fn default_handler() {
    asm::bkpt();
}