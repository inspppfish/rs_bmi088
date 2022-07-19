#![no_std]
#![no_main]

use defmt;
use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal as hal;
use hal::{
    prelude::*,
    rcc::RccExt,
};

mod bmi088_spi;

use stm32f4xx_hal::gpio::*;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app{
    use defmt::println;
    use systick_monotonic::fugit::Duration;
    use crate::bmi088_spi::*;
    use systick_monotonic::Systick;
    use super::*;

    #[shared]
    struct Shared{}

    #[local]
    struct Local{
        imu: Bmi088,
        button: PA0<Input>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;


    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp: hal::pac::Peripherals = cx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(48_u32.MHz())
            .freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let sck = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpioa.pa7.into_alternate();
        let cs_acc = gpioa.pa4.into_push_pull_output();
        let cs_gyro = gpiob.pb0.into_push_pull_output();

        let mut imu = Bmi088::new(dp.SPI1,sck, miso, mosi, cs_acc, cs_gyro, clocks);
        imu.init().unwrap();


        let mut button = gpioa.pa0.into_pull_up_input();
        button.enable_interrupt(&mut dp.EXTI);
        button.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        let mono = Systick::new(cx.core.SYST, 48_000_000);
        keep_watch::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();


        (
            Shared{},
            Local{
                button,
                imu,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [button, imu])]
    fn keep_watch(cx: keep_watch::Context) {
        cx.local.button.clear_interrupt_pending_bit();
        let imu:&mut Bmi088 = cx.local.imu;

        imu.enable_acc().unwrap();
        let mut send_msg: [u8; 4] = [0x16|0x80, 0x17|0x80, 0x55, 0x55];
        let res = imu.spi.transfer(send_msg.as_mut_slice()).unwrap();
        println!("{}", res);
        imu.silence().unwrap();
        keep_watch::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }
}

