use cortex_m::asm::delay;
use defmt::println;
#[allow(unused_imports)]
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use stm32f4xx_hal as hal;
use hal::{
    prelude::*,
    spi::{Mode, Spi},
};
use nb::Error;

use stm32f4xx_hal::pac::SPI1;

use stm32f4xx_hal::gpio::*;
use stm32f4xx_hal::rcc::Clocks;
use crate::bmi088_spi::Bmi088ConfError::{AccConfError, AccPwrConfError, AccPwrCtrlError, AccRangeError, Int1ToCtrlError, IntMapDataError};

pub struct Bmi088{
    pub spi: Spi<SPI1, (SCK, MISO, MOSI)>,
    cs_acc: PA4<Output<PushPull>>,
    cs_gyro: PB0<Output<PushPull>>,
}

impl Bmi088 {
    pub fn new(spi1: SPI1,
               sck: SCK, miso: MISO, mosi: MOSI,
               cs_acc: PA4<Output<PushPull>>, cs_gyro: PB0<Output<PushPull>>,
               clocks: Clocks) -> Self {

        //mode for BMI088
        let spi_mode = Mode {
            polarity: hal::spi::Polarity::IdleLow,
            phase: hal::spi::Phase::CaptureOnFirstTransition,
        };

        //new from 1MHz
        let spi = Spi::new(
            spi1,
            (sck, miso, mosi),
            spi_mode,
            328125u32.Hz(),
            &clocks,
        );

        Bmi088 {
            spi,
            cs_acc,
            cs_gyro,
        }
    }

    pub fn init(&mut self) -> Result<(), Error<()>> {
        self.silence()?;

        //acc software reset
        self.acc_write_single_reg(0x7E, 0xB6)?;

        //check communication
        //double the get ID instruction
        { let _res = self.acc_read_single_reg(0x00).unwrap(); }
        { let _res = self.acc_read_single_reg(0x00).unwrap(); }
        { let _res = self.acc_read_single_reg(0x00).unwrap(); }
        let res = self.acc_read_single_reg(0x00).unwrap();
        // if res != 0x1E_u8 {
        //     defmt::panic!("the acc ID is {} ,Cannot communicate with BMI088!", res);
        // }


        let acc_conf_data_error: [([u8; 2], Bmi088ConfError) ;6] = [
            ([0x7D, 0x04,], AccPwrCtrlError),
            ([0x7C, 0x00], AccPwrConfError),
            ([0x40, (0x2 << 0x4)|(0xB << 0x0)|0x80,], AccConfError),
            ([0x41, 0x0<<0x0,], AccRangeError),
            ([0x53, (0x1 << 0x3)|(0x0 << 0x2)|(0x0<<0x1)], Int1ToCtrlError),
            ([0x58, 0x1<<0x2], IntMapDataError),
        ];

        for data_error in acc_conf_data_error {
            let (instruct, error) = data_error;
            while self.acc_read_single_reg(instruct[0]) != Ok(instruct[1]) {
                self.acc_write_single_reg(instruct[0], instruct[1])?;
            }
            match error {
                _other=> {}
            };
        }

        let res = self.acc_read_single_reg(0x00).unwrap();
        defmt::println!("BMI088 init success!");
        Ok(())
    }

    // A method to avoid "no down trigger" in CS is silence all the CS and then pull down,
    // but i consider it of slow in this genetic func
    pub fn acc_read_single_reg(&mut self, reg: u8) -> Result<u8, Error<()>> {
        // self.silence()?;
        self.enable_acc()?;
        let mut send_msg: [u8; 3] = [0x80|reg, 0x55, 0x55];
        let res = self.spi.transfer(send_msg.as_mut_slice());
        self.silence()?;
        Ok(res.unwrap()[2])
    }

    // pub fn gyro_read_single_reg(&mut self, reg: u8) -> Result<u8, Error<()>> {
    //     // self.silence()?;
    //     self.enable_gyro()?;
    //     let mut send_msg: [u8; 3] = [0x80|reg, 0x55, 0x55];
    //     let res = self.spi.transfer(send_msg.as_mut_slice());
    //     self.silence()?;
    //     Ok(res.unwrap()[2])
    // }

    pub fn acc_write_single_reg(&mut self, reg:u8, data: u8) -> Result<(), Error<()>> {
        // self.silence().unwrap();
        self.enable_acc()?;
        let mut send_msg: [u8; 2] = [reg, data];
        let _ = self.spi.transfer(send_msg.as_mut_slice());
        self.silence()?;
        Ok(())
    }


    pub fn enable_acc(&mut self) -> Result<(), Error<()>> {
        self.cs_acc.set_low();
        self.cs_gyro.set_high();
        Ok(())
    }

    pub fn enable_gyro(&mut self) -> Result<(), Error<()>> {
        self.cs_gyro.set_low();
        self.cs_acc.set_high();
        Ok(())
    }

    pub fn silence(&mut self) -> Result<(), Error<()>> {
        self.cs_acc.set_high();
        self.cs_gyro.set_high();
        Ok(())
    }
}


pub(crate) type SCK = Pin<'B', 3, Alternate<5_u8>>;
pub(crate) type MISO = Pin<'B', 4, Alternate<5_u8>>;
pub(crate) type MOSI = Pin<'A', 7, Alternate<5_u8>>;

#[derive(Debug)]
pub enum Bmi088ConfError {
    AccPwrCtrlError,
    AccPwrConfError,
    AccConfError,
    AccRangeError,
    Int1ToCtrlError,
    IntMapDataError,
}