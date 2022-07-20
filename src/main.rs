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

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM3])]
mod app{
    use defmt::println;
    use systick_monotonic::fugit::Duration;
    use crate::bmi088_spi::*;
    use systick_monotonic::Systick;
    use super::*;

    #[shared]
    struct Shared{
        counter: u32,
    }

    #[local]
    struct Local{
        imu: Bmi088,
        int_acc: PC4<Input>,
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
        let gpioc = dp.GPIOC.split();
        let sck = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpioa.pa7.into_alternate();
        let cs_acc = gpioa.pa4.into_push_pull_output();
        let cs_gyro = gpiob.pb0.into_push_pull_output();

        let mut imu = Bmi088::new(dp.SPI1,sck, miso, mosi, cs_acc, cs_gyro, clocks);
        imu.init().unwrap();


        let mut int_acc = gpioc.pc4.into_pull_up_input();
        int_acc.enable_interrupt(&mut dp.EXTI);
        int_acc.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        let mono = Systick::new(cx.core.SYST, 48_000_000);
        keep_watch::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        show_fps::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();


        (
            Shared{
                counter: 0,
            },
            Local{
                int_acc,
                imu,
            },
            init::Monotonics(mono),
        )
    }

    #[task( local = [int_acc, imu], shared = [counter])]
    fn keep_watch(mut cx: keep_watch::Context) {
        cx.local.int_acc.clear_interrupt_pending_bit();
        let imu:&mut Bmi088 = cx.local.imu;

        imu.enable_acc().unwrap();
        let mut send_msg: [u8; 8] = [0x12|0x80, 0x13|0x80, 0x14|0x80, 0x15|0x80, 0x16|0x80, 0x17|0x80, 0x55, 0x55];
        let res = imu.spi.transfer(send_msg.as_mut_slice()).unwrap();
        cx.shared.counter.lock(|counter| {
            *counter = *counter + 1;
        });
        imu.silence().unwrap();
        keep_watch::spawn_after(Duration::<u64, 1, 1000>::from_ticks(5)).unwrap();
    }

    #[task(shared = [counter])]
    fn show_fps(mut cx: show_fps::Context) {
        defmt::println!("{}", cx.shared.counter.lock(|counter|{*counter}));
        cx.shared.counter.lock(|counter|{*counter = 0});
        show_fps::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }
}

