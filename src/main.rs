#![no_std]
#![no_main]

use defmt;
use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal as hal;
use hal::{
    prelude::*,
    rcc::RccExt,
    spi::{Mode, Spi},
};

mod bmi088_spi;

use stm32f4xx_hal::gpio::*;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app{
    use cortex_m::asm::delay;
    use stm32f4xx_hal::{gpio, pac};
    use stm32f4xx_hal::pac::SPI1;
    use stm32f4xx_hal::spi::{Pins, TransferModeNormal};
    use super::*;

    #[shared]
    struct Shared{}

    #[local]
    struct Local{
        spi: Spi<SPI1, (Pin<'B', 3, Alternate<5_u8>>,
                        Pin<'B', 4, Alternate<5_u8>>,
                        Pin<'A', 7, Alternate<5_u8>>)>,
        button: PA0<Input>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp: hal::pac::Peripherals = cx.device;
        let mut rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(48_u32.MHz())
            .freeze();

        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();

        let sck = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpioa.pa7.into_alternate();
        let mut cs_acc = gpioa.pa4.into_push_pull_output();
        let mut cs_gyro = gpiob.pb0.into_push_pull_output();

        cs_acc.set_low();
        cs_gyro.set_high();

        let spi_mode = Mode {
            polarity: hal::spi::Polarity::IdleHigh,
            phase: hal::spi::Phase::CaptureOnSecondTransition,
        };

        let mut spi = Spi::new(
            dp.SPI1,
            (sck, miso, mosi),
            spi_mode,
            1_000_000_u32.Hz(),
            &clocks,
        );


        let mut button = gpioa.pa0.into_pull_up_input();
        button.enable_interrupt(&mut dp.EXTI);
        button.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        cs_acc.set_high();

        (
            Shared{},
            Local{
                button,
                spi
            },
            init::Monotonics(),
        )
    }

    #[task(binds = EXTI0, local = [button, spi])]
    fn button_click(cx: button_click::Context) {
        let spi = cx.local.spi;

    }
}
