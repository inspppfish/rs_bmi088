use stm32f4xx_hal::gpio::{Alternate, Output, Pin, PushPull};
use stm32f4xx_hal::pac::USART3;
use stm32f4xx_hal::serial::Rx;

pub struct FutabaSbus {
    pub rx: RX,
    vcc: VCC,
    gnd: GND,
}

impl FutabaSbus {
    pub fn new(rx: RX, vcc: VCC, gnd: GND) -> Self{

        FutabaSbus {
            rx,
            vcc,
            gnd,
        }
    }
}

type RX = Rx<USART3>;
type VCC = Pin <'B', 8, Output<PushPull>>;
type GND = Pin <'A', 8, Output<PushPull>>;