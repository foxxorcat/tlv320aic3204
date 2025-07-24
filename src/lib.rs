//! # TLV320AIC3204 I2C Driver
//!
//! 一个基于 `embedded-hal` 的平台无关 Rust 驱动，用于控制 Texas Instruments TLV320AIC3204
//! 超低功耗立体声音频编解码器。
//!
//! 主要功能包括：
//! - 初始化和软件复位 。
//! - 时钟系统配置，包括 PLL 和各种时钟分频器 。
//! - ADC 和 DAC 的电源管理和配置 。
//! - 输入/输出路由选择（例如，将 IN1L 连接到左 MICPGA） 。
//! - 增益和音量控制（数字和模拟） 。
//! - 信号处理块（PRB）的选择 。
//! - 耳机、线路输出驱动器配置，包括 Class-D 模式 。
//! - PowerTune™ 模式配置，用于在功耗和性能之间进行权衡 。
//!
//! ## 使用示例
//!
//! 以下是如何在项目中实例化和使用此驱动程序的示例。
//! 此示例演示了如何初始化编解码器以进行高性能的 48ksps 立体声 DAC 播放，
//! 该示例的配置与应用参考指南中的脚本类似 。
//!
//! ```no_run
//! # use embedded_hal::i2c::I2c;
//! # use tlv320aic3204_driver::{Tlv320aic3204, prelude::*, errors::Error};
//! #
//! # // Placeholder for a concrete I2C implementation from a HAL crate
//! # struct MockI2c;
//! # impl I2c for MockI2c {
//! #     type Error = std::io::Error;
//! #     fn transaction(&mut self, _address: u8, _operations: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), Self::Error> {
//! #         Ok(())
//! #     }
//! # }
//! # let i2c = MockI2c;
//!
//! // 1. 实例化驱动
//! // TLV320AIC3204 的默认 I2C 地址是 0x18
//! let mut codec = Tlv320aic3204::new(i2c, tlv320aic3204_driver::I2C_ADDRESS);
//!
//! // 2. 初始化序列
//! codec.software_reset()?;
//!
//! // 等待复位完成 (datasheet 建议 >1ms)
//! // hal::delay::Delay::delay_ms(2);
//!
//! // --- 时钟配置 ---
//! // 假设 MCLK = 12.288MHz, DAC_FS = 48kHz. PLL 被禁用。
//! // CODEC_CLKIN = MCLK
//! codec.select_codec_clkin(registers::CodecClockInput::MCLK)?;
//! // NDAC = 1, MDAC = 2, DOSR = 128
//! // DAC_FS = MCLK / (NDAC * MDAC * DOSR) = 12.288MHz / (1 * 2 * 128) = 48kHz
//! codec.set_dac_dividers(1, true, 2, true)?;
//! codec.set_dac_oversampling(128)?;
//!
//! // --- 音频接口配置 ---
//! // I2S, 16-bit, 从机模式
//! codec.set_audio_interface(registers::AudioInterface::I2S)?;
//! codec.set_word_length(registers::WordLength::Bits16)?;
//! codec.set_clock_direction(false, false)?; // BCLK 和 WCLK 为输入
//!
//! // --- 信号处理 ---
//! // 选择一个处理块，例如 PRB_P1 (默认)
//! codec.select_dac_processing_block(registers::DacProcessingBlock::PRB_P1)?;
//!
//! // --- 模拟部分配置 ---
//! codec.disable_crude_avdd()?; //
//! codec.enable_master_analog_power()?; //
//! codec.set_ref_charging_time(registers::RefChargingTime::Ms40)?; //
//!
//! // 配置耳机驱动的 Pop-Free 启动
//! let depop_settings = registers::HeadphoneDepopSettings {
//!     step_time: registers::SoftStepTime::Ms100, //
//!     num_time_constants: registers::TimeConstants::N5, //
//!     resistor: registers::RpopValue::K6, //
//! };
//! codec.configure_headphone_depop(depop_settings)?;
//!
//! // 设置共模电压为 0.9V (适用于 AVDD >= 1.8V)
//! codec.set_full_chip_common_mode(registers::CommonModeVoltage::V0_9)?;
//!
//! // --- 路由和输出功率 ---
//! // 将 DAC 信号路由到耳机输出
//! codec.set_hpl_routing(registers::HplInput::from_bits_truncate(registers::HplInput::LEFT_DAC.bits()))?;
//! codec.set_hpr_routing(registers::HprInput::from_bits_truncate(registers::HprInput::RIGHT_DAC.bits()))?;
//!
//! // 设置 DAC PowerTune 模式为 PTM_P3/P4
//! codec.set_dac_ptm_mode(
//!     registers::DacPtmMode::PTM_P3_P4,
//!     registers::DacPtmMode::PTM_P3_P4
//! )?;
//!
//! // 设置耳机驱动增益为 0dB
//! codec.set_headphone_driver_gain(
//!     registers::OutputDriverGain::Db0,
//!     registers::OutputDriverGain::Db0
//! )?;
//!
//! // 打开耳机输出驱动
//! codec.set_output_driver_power(registers::OutputDriverPower::HPL | registers::OutputDriverPower::HPR, true)?;
//!
//! // 等待耳机软启动完成 (重要！)
//! // 实际应用中应轮询标志位 Page 1, Reg 63, D(7:6)
//! // hal::delay::Delay::delay_ms(500); // 简单延时作为替代
//!
//! // --- 启动 DAC ---
//! // 上电并连接 DAC 通道
//! codec.set_dac_channel_power(true, true)?;
//! codec.set_dac_data_paths(
//!     registers::DacDataPath::LeftChanAudio,
//!     registers::DacDataPath::RightChanAudio
//! )?;
//!
//! // 取消 DAC 静音
//! codec.set_dac_mute(false, false)?;
//! # Ok::<(), Error<std::io::Error>>(())
//! ```

pub mod registers;

use core::cell::Cell;

pub use embedded_hal::i2c::{I2c, Operation};

use crate::registers::{
    AdcAnalogVolumeFlags, AdcFineGain, AdcProcessingBlock, AdcPtmMode, AgcConfig,
    AnalogBypassVolume, AnalogInputChargeTime, ButtonDebounceTime, CommonModeVoltage,
    DacAnalogGainFlags, DacAutoMuteDuration, DacDataPath, DacProcessingBlock, DacPtmMode,
    DcMeasurementConfig, DcMeasurementMode, DigitalMicInput, DinMfp1Function, DoutMfp2Function,
    DrcConfig, FloatingInputConfig, GpioFunction, HeadphoneDepopSettings, HeadsetDebounceTime,
    HeadsetDetectionStatus, HeadsetType, HplInput, HprInput, InterruptPulseMode, InterruptSource,
    InterruptStatus, LdoVoltage, LolInput, LorInput, MfpPin, MicPgaLeftNegativeInput,
    MicPgaLeftPositiveInput, MicPgaRightNegativeInput, MicPgaRightPositiveInput, MisoMfp4Function,
    MixerAmpVolume, OutputDriverPower, OutputPowerConfig, RefChargingTime, SclkMfp3Function,
};

use self::registers::{
    AudioInterface, CodecClockInput, PllInput, SampleRatePriority, WordLength, P0_R0,
};

/// 驱动可能返回的错误类型
#[derive(Debug)]
pub enum Error<E> {
    /// 底层 I2C 总线错误
    I2c(E),
    /// 提供了无效的参数值
    InvalidValue,
    /// 提供了无效的时钟或采样率，无法进行计算
    InvalidClockRate,
    /// 未能找到适用于所提供时钟和采样率的有效分频器组合
    ClockConfigNotFound,
}

// 实现 Display trait 以提供人性化的错误信息
impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C bus error: {:?}", e),
            Error::InvalidValue => write!(f, "An invalid value was provided for a parameter"),
            Error::InvalidClockRate => {
                write!(f, "Invalid clock or sample rate provided for calculation")
            }
            Error::ClockConfigNotFound => write!(
                f,
                "Failed to find a valid divider combination for the given clocks"
            ),
        }
    }
}

// 实现标准的 Error trait，使其与 anyhow::Error 和 `?` 兼容。
// 这要求泛型 `E` (I2C错误) 本身也实现了 `std::error::Error`。
impl<E> std::error::Error for Error<E>
where
    E: std::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2c(e) => Some(e),
            _ => None,
        }
    }
}

/// TLV320AIC3204 I2C 默认地址
pub const I2C_ADDRESS: u8 = 0x18;

/// TLV320AIC3204 I2C 驱动结构体
pub struct Tlv320aic3204<I2C> {
    i2c: I2C,
    address: u8,
    last_page: Cell<u8>,
}

impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 创建一个新的驱动实例
    ///
    /// # Arguments
    ///
    /// * `i2c` - 一个实现了 `embedded_hal::i2c::I2c` 特征的 I2C 总线实例
    /// * `address` - TLV320AIC3204 的 7 位 I2C 地址
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            // 初始假设在 Page 0，避免第一次写 Page 0 寄存器时进行不必要的页选择
            last_page: Cell::new(0),
        }
    }

    /// 销毁驱动并返回底层的 I2C 总线
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    // --- 核心寄存器访问方法 ---

    /// 切换到指定的寄存器页。
    /// 内部使用，以确保后续的读写操作在正确的页上进行。
    /// 为了效率，它会缓存上一次选择的页码。
    fn select_page(&mut self, page: u8) -> Result<(), Error<E>> {
        if self.last_page.get() != page {
            self.i2c
                .write(self.address, &[P0_R0::ADDRESS, page])
                .map_err(Error::I2c)?;
            self.last_page.set(page);
        }
        Ok(())
    }

    /// 向指定的寄存器写入一个字节。
    ///
    /// # Arguments
    ///
    /// * `page` - 寄存器所在的页 (0-255)
    /// * `reg` - 寄存器地址 (0-127)
    /// * `value` - 要写入的字节值
    pub fn write_register(&mut self, page: u8, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.select_page(page)?;
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Error::I2c)
    }

    /// 从指定的寄存器读取一个字节。
    ///
    /// # Arguments
    ///
    /// * `page` - 寄存器所在的页 (0-255)
    /// * `reg` - 寄存器地址 (0-127)
    pub fn read_register(&mut self, page: u8, reg: u8) -> Result<u8, Error<E>> {
        self.select_page(page)?;
        let mut buffer = [0u8; 1];
        self.i2c
            .transaction(
                self.address,
                &mut [Operation::Write(&[reg]), Operation::Read(&mut buffer)],
            )
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// 更新寄存器中的特定位。
    ///
    /// 此方法执行一个读-修改-写操作，只改变 `mask` 指定的位。
    ///
    /// # Arguments
    ///
    /// * `page` - 寄存器所在的页
    /// * `reg` - 寄存器地址
    /// * `mask` - 位掩码，指定要修改的位（1 表示修改，0 表示保留）
    /// * `value` - 要写入的值。只有在 `mask` 中对应位为 1 的位才会被写入。
    pub fn update_register(
        &mut self,
        page: u8,
        reg: u8,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<E>> {
        let current_value = self.read_register(page, reg)?;
        let new_value = (current_value & !mask) | (value & mask);
        if new_value != current_value {
            self.write_register(page, reg, new_value)?;
        }
        Ok(())
    }
}

/// --- 时钟配置 ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 执行软件复位。
    /// 这会将所有寄存器重置为其默认值。
    pub fn software_reset(&mut self) -> Result<(), Error<E>> {
        use registers::P0_R1;

        self.write_register(P0_R1::PAGE, P0_R1::ADDRESS, P0_R1::RESET)
    }

    /// 选择 PLL 的输入时钟源。
    pub fn set_pll_input(&mut self, input: PllInput) -> Result<(), Error<E>> {
        use registers::P0_R4;

        self.update_register(
            P0_R4::PAGE,
            P0_R4::ADDRESS,
            P0_R4::PLL_CLKIN_SEL_MASK,
            input as u8,
        )
    }

    /// 选择 CODEC_CLKIN 的来源。
    pub fn select_codec_clkin(&mut self, input: CodecClockInput) -> Result<(), Error<E>> {
        use registers::P0_R4;
        self.update_register(
            P0_R4::PAGE,
            P0_R4::ADDRESS,
            P0_R4::CODEC_CLKIN_SEL_MASK,
            input as u8,
        )
    }

    /// 配置并上电 PLL。
    ///
    /// # Arguments
    /// * `p` - PLL 预分频器 P (1-8)
    /// * `r` - PLL 乘法器 R (1-4)
    /// * `j` - PLL 乘法器 J (4-63)
    /// * `d` - PLL 小数乘法器 D (0-9999)
    pub fn configure_and_power_up_pll(
        &mut self,
        p: u8,
        r: u8,
        j: u8,
        d: u16,
    ) -> Result<(), Error<E>> {
        use registers::{P0_R5, P0_R6, P0_R7, P0_R8};

        if !(1..=8).contains(&p) || !(1..=4).contains(&r) || !(4..=63).contains(&j) || d > 9999 {
            return Err(Error::InvalidValue);
        }
        let p_val = if p == 8 { 0 } else { p };

        // 设置 P 和 R
        let val_p_r = (p_val << 4) | r;
        self.update_register(
            P0_R5::PAGE,
            P0_R5::ADDRESS,
            P0_R5::P_MASK | P0_R5::R_MASK,
            val_p_r,
        )?;

        // 设置 J
        self.write_register(P0_R6::PAGE, P0_R6::ADDRESS, j)?;

        // 设置 D
        let d_msb = (d >> 8) as u8;
        let d_lsb = (d & 0xFF) as u8;
        // 必须先写 MSB，然后立即写 LSB
        self.write_register(P0_R7::PAGE, P0_R7::ADDRESS, d_msb)?;
        self.write_register(P0_R8::PAGE, P0_R8::ADDRESS, d_lsb)?;

        // 上电 PLL
        self.update_register(
            P0_R5::PAGE,
            P0_R5::ADDRESS,
            P0_R5::PLL_POWER_UP,
            P0_R5::PLL_POWER_UP,
        )
    }

    /// 关闭 PLL 电源。
    pub fn power_down_pll(&mut self) -> Result<(), Error<E>> {
        use registers::P0_R5;
        self.update_register(P0_R5::PAGE, P0_R5::ADDRESS, P0_R5::PLL_POWER_UP, 0)
    }

    /// 设置 DAC 时钟分频器 (NDAC, MDAC)。
    pub fn set_dac_dividers(
        &mut self,
        ndac: u8,
        ndac_on: bool,
        mdac: u8,
        mdac_on: bool,
    ) -> Result<(), Error<E>> {
        use registers::{P0_R11, P0_R12};
        if ndac == 0 || ndac > 128 || mdac == 0 || mdac > 128 {
            return Err(Error::InvalidValue);
        }
        let ndac_val = if ndac == 128 { 0 } else { ndac };
        let mdac_val = if mdac == 128 { 0 } else { mdac };
        let ndac_reg = (if ndac_on { P0_R11::POWER_UP } else { 0 }) | ndac_val;
        let mdac_reg = (if mdac_on { P0_R12::POWER_UP } else { 0 }) | mdac_val;
        self.write_register(P0_R11::PAGE, P0_R11::ADDRESS, ndac_reg)?;
        self.write_register(P0_R12::PAGE, P0_R12::ADDRESS, mdac_reg)
    }

    /// 设置 ADC 时钟分频器 (NADC, MADC)。
    pub fn set_adc_dividers(
        &mut self,
        nadc: u8,
        nadc_on: bool,
        madc: u8,
        madc_on: bool,
    ) -> Result<(), Error<E>> {
        use registers::{P0_R18, P0_R19};

        if nadc == 0 || nadc > 128 || madc == 0 || madc > 128 {
            return Err(Error::InvalidValue);
        }
        let nadc_val = if nadc == 128 { 0 } else { nadc };
        let madc_val = if madc == 128 { 0 } else { madc };
        let nadc_reg = (if nadc_on { P0_R18::POWER_UP } else { 0 }) | nadc_val;
        let madc_reg = (if madc_on { P0_R19::POWER_UP } else { 0 }) | madc_val;
        self.write_register(P0_R18::PAGE, P0_R18::ADDRESS, nadc_reg)?;
        self.write_register(P0_R19::PAGE, P0_R19::ADDRESS, madc_reg)
    }

    /// 设置 DAC 过采样率 (DOSR)。
    pub fn set_dac_oversampling(&mut self, dosr: u16) -> Result<(), Error<E>> {
        use registers::{P0_R13, P0_R14};

        if dosr > 1024 || dosr == 0 {
            return Err(Error::InvalidValue);
        }
        let dosr_val = if dosr == 1024 { 0 } else { dosr };
        let dosr_msb = (dosr_val >> 8) as u8 & 0x03;
        let dosr_lsb = (dosr_val & 0xFF) as u8;
        self.write_register(P0_R13::PAGE, P0_R13::ADDRESS, dosr_msb)?;
        self.write_register(P0_R14::PAGE, P0_R14::ADDRESS, dosr_lsb)
    }

    /// 设置 ADC 过采样率 (AOSR)。
    pub fn set_adc_oversampling(&mut self, aosr: u16) -> Result<(), Error<E>> {
        use registers::P0_R20;
        let val = match aosr {
            128 => 0x80,
            64 => 0x40,
            32 => 0x20,
            256 => 0x00,
            _ => return Err(Error::InvalidValue),
        };
        self.write_register(P0_R20::PAGE, P0_R20::ADDRESS, val)
    }
}

/// --- 音频接口 ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 设置数字音频接口格式 (I2S, DSP, RJF, LJF)。
    pub fn set_audio_interface(&mut self, interface: AudioInterface) -> Result<(), Error<E>> {
        use registers::P0_R27;
        self.update_register(
            P0_R27::PAGE,
            P0_R27::ADDRESS,
            P0_R27::INTERFACE_MASK,
            interface as u8,
        )
    }

    /// 设置音频数据字长 (16, 20, 24, 32 bits)。
    pub fn set_word_length(&mut self, length: WordLength) -> Result<(), Error<E>> {
        use registers::P0_R27;
        self.update_register(
            P0_R27::PAGE,
            P0_R27::ADDRESS,
            P0_R27::WORD_LENGTH_MASK,
            length as u8,
        )
    }

    /// 设置 BCLK 和 WCLK 的方向（主/从模式）。
    /// `bclk_is_output`: true 表示主模式, false 表示从模式
    /// `wclk_is_output`: true 表示主模式, false 表示从模式
    pub fn set_clock_direction(
        &mut self,
        bclk_is_output: bool,
        wclk_is_output: bool,
    ) -> Result<(), Error<E>> {
        use registers::P0_R27;

        let val = (if bclk_is_output {
            P0_R27::BCLK_DIR_OUTPUT
        } else {
            0
        }) | (if wclk_is_output {
            P0_R27::WCLK_DIR_OUTPUT
        } else {
            0
        });
        let mask = P0_R27::BCLK_DIR_OUTPUT | P0_R27::WCLK_DIR_OUTPUT;
        self.update_register(P0_R27::PAGE, P0_R27::ADDRESS, mask, val)
    }
}

/// --- 电源和模拟部分 ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 禁用粗糙的 AVDD 生成。
    /// 当外部提供 AVDD 或使用内部 LDO 时，应调用此方法。
    pub fn disable_crude_avdd(&mut self) -> Result<(), Error<E>> {
        use self::registers::P1_R1;
        self.update_register(
            P1_R1::PAGE,
            P1_R1::ADDRESS,
            P1_R1::DISABLE_WEAK_AVDD,
            P1_R1::DISABLE_WEAK_AVDD,
        )
    }

    /// 启用主模拟电源控制。
    pub fn enable_master_analog_power(&mut self) -> Result<(), Error<E>> {
        use self::registers::P1_R2;
        self.update_register(P1_R2::PAGE, P1_R2::ADDRESS, P1_R2::ANALOG_BLOCKS_ENABLE, 0)
    }

    /// 配置输出电源和共模电压。
    ///
    /// 此方法安全地配置 Page 1, Register 10 的所有相关位。
    pub fn configure_output_power(&mut self, config: &OutputPowerConfig) -> Result<(), Error<E>> {
        use self::registers::P1_R10;

        let mut val = 0u8;

        if config.global_cm == CommonModeVoltage::V0_75 {
            val |= P1_R10::FULL_CHIP_CM_MASK;
        }

        val |= (config.headphone_cm as u8) << 4;

        if config.lineout_use_ldoin_and_1_65v_cm {
            val |= P1_R10::LINE_OUT_CM_LDOIN;
        }

        if config.headphone_use_ldoin {
            val |= P1_R10::HP_POWER_SOURCE_LDOIN;
        }

        if config.ldoin_is_high_range {
            val |= P1_R10::LDOIN_HIGH_RANGE;
        }

        // 使用单一的 write 操作来设置所有位
        self.write_register(P1_R10::PAGE, P1_R10::ADDRESS, val)
    }

    /// 配置 LDO 稳压器(模拟低压差线性稳压器)。
    ///
    /// 当AVDD和DVDD未连接外部电源时启用 (此时引脚应有1.7V左右的电压)
    /// 注意: D-LDO 的开关由物理引脚 LDO_SELECT 控制，此函数无法改变。
    ///
    /// # Arguments
    /// * `avdd_on` - `true` 来上电 AVDD LDO, `false` 来下电。
    /// * `avdd_voltage` - AVDD LDO 的输出电压。
    /// * `dvdd_voltage` - DVDD LDO 的输出电压。
    pub fn configure_ldo(
        &mut self,
        avdd_on: bool,
        avdd_voltage: LdoVoltage,
        dvdd_voltage: LdoVoltage,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R2;

        // D7-6: DVDD LDO 电压
        // D5-4: AVDD LDO 电压
        // D0: AVDD LDO 上电
        let mut val = ((dvdd_voltage as u8) << 6) | ((avdd_voltage as u8) << 4);
        if avdd_on {
            val |= 1;
        }

        // 掩码覆盖所有要修改的位
        let mask = 0b1111_0001;

        self.update_register(P1_R2::PAGE, P1_R2::ADDRESS, mask, val)
    }

    /// 设置参考电压的快速充电时间。
    pub fn set_ref_charging_time(&mut self, time: RefChargingTime) -> Result<(), Error<E>> {
        use self::registers::P1_R123;
        self.update_register(
            P1_R123::PAGE,
            P1_R123::ADDRESS,
            P1_R123::REF_POWERUP_MASK,
            time as u8,
        )
    }

    /// 选择 DAC (播放) 信号处理块。
    pub fn select_dac_processing_block(
        &mut self,
        block: DacProcessingBlock,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R60;
        self.write_register(P0_R60::PAGE, P0_R60::ADDRESS, block as u8)
    }

    /// 选择 ADC (录音) 信号处理块。
    pub fn select_adc_processing_block(
        &mut self,
        block: AdcProcessingBlock,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R61;
        self.write_register(P0_R61::PAGE, P0_R61::ADDRESS, block as u8)
    }

    /// 设置 DAC PowerTune™ (PTM) 模式。
    pub fn set_dac_ptm_mode(
        &mut self,
        left: DacPtmMode,
        right: DacPtmMode,
    ) -> Result<(), Error<E>> {
        use self::registers::{P1_R3, P1_R4};
        self.update_register(
            P1_R3::PAGE,
            P1_R3::ADDRESS,
            P1_R3::LEFT_DAC_PTM_MASK,
            (left as u8) << 2,
        )?;
        self.update_register(
            P1_R4::PAGE,
            P1_R4::ADDRESS,
            P1_R4::RIGHT_DAC_PTM_MASK,
            (right as u8) << 2,
        )
    }

    /// 设置 ADC PowerTune™ (PTM) 模式。
    pub fn set_adc_ptm_mode(&mut self, mode: AdcPtmMode) -> Result<(), Error<E>> {
        use self::registers::P1_R61;
        self.write_register(P1_R61::PAGE, P1_R61::ADDRESS, mode as u8)
    }
}

// --- DAC, 输出和路由 ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 设置 DAC 通道的电源状态。
    pub fn set_dac_channel_power(&mut self, left_on: bool, right_on: bool) -> Result<(), Error<E>> {
        use self::registers::P0_R63;
        let val = (if left_on {
            P0_R63::LEFT_DAC_POWER_UP
        } else {
            0
        }) | (if right_on {
            P0_R63::RIGHT_DAC_POWER_UP
        } else {
            0
        });
        let mask = P0_R63::LEFT_DAC_POWER_UP | P0_R63::RIGHT_DAC_POWER_UP;
        self.update_register(P0_R63::PAGE, P0_R63::ADDRESS, mask, val)
    }

    /// 设置 DAC 数据路径。
    pub fn set_dac_data_paths(
        &mut self,
        left_path: DacDataPath,
        right_path: DacDataPath,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R63;

        let val = ((left_path as u8) << 4) | ((right_path as u8) << 2);
        let mask = P0_R63::LEFT_DAC_DATA_PATH_MASK | P0_R63::RIGHT_DAC_DATA_PATH_MASK;
        self.update_register(P0_R63::PAGE, P0_R63::ADDRESS, mask, val)
    }

    /// 设置 DAC 通道的数字静音。
    pub fn set_dac_mute(&mut self, left_mute: bool, right_mute: bool) -> Result<(), Error<E>> {
        use self::registers::P0_R64;

        let mut value = 0;
        if left_mute {
            value |= P0_R64::LEFT_DAC_MUTE;
        }
        if right_mute {
            value |= P0_R64::RIGHT_DAC_MUTE;
        }

        self.update_register(P0_R64::PAGE, P0_R64::ADDRESS, P0_R64::DAC_MUTE_MASK, value)
    }

    /// 设置 HPL (左耳机) 的输入路由。
    pub fn set_hpl_routing(&mut self, inputs: HplInput) -> Result<(), Error<E>> {
        use self::registers::P1_R12;

        self.update_register(
            P1_R12::PAGE,
            P1_R12::ADDRESS,
            HplInput::all().bits(),
            inputs.bits(),
        )
    }

    /// 设置 HPR (右耳机) 的输入路由。
    pub fn set_hpr_routing(&mut self, inputs: HprInput) -> Result<(), Error<E>> {
        use self::registers::P1_R13;

        self.update_register(
            P1_R13::PAGE,
            P1_R13::ADDRESS,
            HprInput::all().bits(),
            inputs.bits(),
        )
    }

    /// 设置 LOL (左线路输出) 的输入路由。
    pub fn set_lol_routing(&mut self, inputs: LolInput) -> Result<(), Error<E>> {
        use self::registers::P1_R14;

        self.update_register(
            P1_R14::PAGE,
            P1_R14::ADDRESS,
            LolInput::all().bits(),
            inputs.bits(),
        )
    }

    /// 设置 LOR (右线路输出) 的输入路由。
    pub fn set_lor_routing(&mut self, inputs: LorInput) -> Result<(), Error<E>> {
        use self::registers::P1_R15;

        self.update_register(
            P1_R15::PAGE,
            P1_R15::ADDRESS,
            LorInput::all().bits(),
            inputs.bits(),
        )
    }

    /// 设置线路输出驱动器 (LOL/LOR) 的增益。
    pub fn set_lineout_driver_gain(
        &mut self,
        left_gain_db: f32,
        right_gain_db: f32,
    ) -> Result<(), Error<E>> {
        use self::registers::{P1_R18, P1_R19};

        /// 将以dB为单位的浮点增益值转换为耳机驱动器的寄存器值。
        ///
        /// # Arguments
        /// * `gain_db` - 期望的增益值, 范围从-6.0到+29.0。
        ///
        /// # Returns
        /// 成功时返回对应的u8寄存器值，如果输入超出范围则返回错误。
        fn convert_gain_db_to_register_value<E>(gain_db: f32) -> Result<u8, Error<E>> {
            // 1. 验证输入是否在有效范围内
            if !((-6.0..=29.0).contains(&gain_db)) {
                // 如果超出范围，返回错误
                return Err(Error::InvalidValue);
            }

            // 2. 将浮点dB值四舍五入到最近的整数，因为硬件步进为1dB
            let rounded_db = gain_db.round() as i16;

            // 3. 根据正负值应用不同的计算规则
            let register_value = if rounded_db >= 0 {
                // 对于正值和0，寄存器值就是dB值
                rounded_db as u8
            } else {
                // 对于负值，使用公式：64 + dB值
                (64 + rounded_db) as u8
            };

            Ok(register_value)
        }

        // 使用辅助函数将dB值转换为寄存器值
        let left_val = convert_gain_db_to_register_value(left_gain_db)?;
        let right_val = convert_gain_db_to_register_value(right_gain_db)?;

        self.update_register(
            P1_R18::PAGE,
            P1_R18::ADDRESS,
            P1_R18::LOL_GAIN_MASK,
            left_val as u8,
        )?;
        self.update_register(
            P1_R19::PAGE,
            P1_R19::ADDRESS,
            P1_R19::LOR_GAIN_MASK,
            right_val as u8,
        )
    }

    /// 设置模拟旁路路径 (IN1L/R 到 HPL/R) 的音量。
    pub fn set_analog_bypass_volume(
        &mut self,
        left: AnalogBypassVolume,
        right: AnalogBypassVolume,
    ) -> Result<(), Error<E>> {
        use self::registers::{P1_R22, P1_R23};
        self.write_register(P1_R22::PAGE, P1_R22::ADDRESS, left as u8)?;
        self.write_register(P1_R23::PAGE, P1_R23::ADDRESS, right as u8)
    }

    /// 设置混音放大器旁路路径 (MAL/R 到 HPL/R) 的音量。
    pub fn set_mixer_amp_volume(
        &mut self,
        left: MixerAmpVolume,
        right: MixerAmpVolume,
    ) -> Result<(), Error<E>> {
        use self::registers::{P1_R24, P1_R25};
        self.write_register(P1_R24::PAGE, P1_R24::ADDRESS, left as u8)?;
        self.write_register(P1_R25::PAGE, P1_R25::ADDRESS, right as u8)
    }

    /// 配置 DAC 数字自动静音功能。
    pub fn set_dac_auto_mute(&mut self, duration: DacAutoMuteDuration) -> Result<(), Error<E>> {
        use self::registers::P0_R64;
        let mask = 0b0111_0000; // D6-D4
        let value = (duration as u8) << 4;
        self.update_register(P0_R64::PAGE, P0_R64::ADDRESS, mask, value)
    }

    /// 设置输出驱动器（HPL, HPR, LOL, LOR, Mixers）的电源状态。
    pub fn set_output_driver_power(
        &mut self,
        drivers: OutputDriverPower,
        on: bool,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R9;

        let mask = drivers.bits();
        let value = if on { mask } else { 0 };
        self.update_register(P1_R9::PAGE, P1_R9::ADDRESS, mask, value)
    }

    /// 设置耳机驱动器的增益（使用浮点dB值）。
    ///
    /// 增益值范围: -6.0 到 +29.0 dB。
    /// 输入值将被四舍五入到最接近的1dB整数步进。
    pub fn set_headphone_driver_gain_db(
        &mut self,
        left_gain_db: f32,
        right_gain_db: f32,
    ) -> Result<(), Error<E>> {
        use self::registers::{P1_R16, P1_R17};

        /// 将以dB为单位的浮点增益值转换为耳机驱动器的寄存器值。
        ///
        /// # Arguments
        /// * `gain_db` - 期望的增益值, 范围从-6.0到+29.0。
        ///
        /// # Returns
        /// 成功时返回对应的u8寄存器值，如果输入超出范围则返回错误。
        fn convert_gain_db_to_register_value<E>(gain_db: f32) -> Result<u8, Error<E>> {
            // 1. 验证输入是否在有效范围内
            if !((-6.0..=29.0).contains(&gain_db)) {
                // 如果超出范围，返回错误
                return Err(Error::InvalidValue);
            }

            // 2. 将浮点dB值四舍五入到最近的整数，因为硬件步进为1dB
            let rounded_db = gain_db.round() as i16;

            // 3. 根据正负值应用不同的计算规则
            let register_value = if rounded_db >= 0 {
                // 对于正值和0，寄存器值就是dB值
                rounded_db as u8
            } else {
                // 对于负值，使用公式：64 + dB值
                (64 + rounded_db) as u8
            };

            Ok(register_value)
        }

        // 使用辅助函数将dB值转换为寄存器值
        let left_val = convert_gain_db_to_register_value(left_gain_db)?;
        let right_val = convert_gain_db_to_register_value(right_gain_db)?;

        // 写入寄存器。
        // 计算出的值 D6 位（静音位）为0，因此会自动解除静音。
        self.update_register(
            P1_R16::PAGE,
            P1_R16::ADDRESS,
            P1_R16::HPL_CONTROL_MASK, // 掩码应为 0x7F
            left_val,
        )?;

        self.update_register(
            P1_R17::PAGE,
            P1_R17::ADDRESS,
            P1_R17::HPR_CONTROL_MASK, // 掩码应为 0x7F
            right_val,
        )
    }

    /// 设置 DAC 通道的数字音量。
    ///
    /// # Arguments
    ///
    /// * `left_volume_db`: 左声道的音量，范围从 -63.5dB 到 +24.0dB，步进 0.5dB 。
    /// * `right_volume_db`: 右声道的音量，范围从 -63.5dB 到 +24.0dB，步进 0.5dB 。
    pub fn set_dac_digital_volume_db(
        &mut self,
        left_volume_db: f32,
        right_volume_db: f32,
    ) -> Result<(), Error<E>> {
        use self::registers::{P0_R65, P0_R66};

        if !(-63.5..=24.0).contains(&left_volume_db) || !(-63.5..=24.0).contains(&right_volume_db) {
            return Err(Error::InvalidValue);
        }

        let left_val = (left_volume_db * 2.0).round() as i8;
        let right_val = (right_volume_db * 2.0).round() as i8;

        self.write_register(P0_R65::PAGE, P0_R65::ADDRESS, left_val as u8)?;
        self.write_register(P0_R66::PAGE, P0_R66::ADDRESS, right_val as u8)
    }

    /// 配置耳机驱动的去爆音（De-pop）/软启动参数。
    pub fn configure_headphone_depop(
        &mut self,
        settings: HeadphoneDepopSettings,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R20;

        let val = (settings.step_time as u8)
            | (settings.num_time_constants as u8)
            | (settings.resistor as u8);
        self.write_register(P1_R20::PAGE, P1_R20::ADDRESS, val)
    }

    /// 配置动态范围压缩 (DRC)。
    /// 只有选择了支持DRC的PRB_Px处理块，此功能才会生效。
    pub fn configure_drc(&mut self, config: DrcConfig) -> Result<(), Error<E>> {
        use self::registers::{P0_R68, P0_R70};

        // 配置使能、阈值和迟滞
        let mut reg68_val = ((config.threshold as u8) << 2) | (config.hysteresis as u8);
        if config.left_drc_enabled {
            reg68_val |= 1 << 6;
        }
        if config.right_drc_enabled {
            reg68_val |= 1 << 5;
        }
        self.write_register(P0_R68::PAGE, P0_R68::ADDRESS, reg68_val)?;

        // 配置攻击和衰减速率
        let reg70_val = (config.attack_rate << 4) | (config.decay_rate);
        self.write_register(P0_R70::PAGE, P0_R70::ADDRESS, reg70_val)
    }

    /// 触发一次蜂鸣声。
    /// 只有选择了 PRB_P25 处理块才能使用此功能。
    /// # Arguments
    /// * `freq_hz`: 蜂鸣声频率 (Hz)
    /// * `sample_rate`: 当前的 DAC 采样率 (Hz)
    /// * `duration_ms`: 持续时间 (ms)
    /// * `volume_db`: 音量, 0 到 -63 dB.
    pub fn generate_beep(
        &mut self,
        freq_hz: f32,
        sample_rate: u32,
        duration_ms: u32,
        volume_db: i8,
    ) -> Result<(), Error<E>> {
        use self::registers::{
            P0_R71, P0_R72, P0_R73, P0_R74, P0_R75, P0_R76, P0_R77, P0_R78, P0_R79,
        };
        use core::f32::consts::PI;

        if !(0..=63).contains(&-volume_db) {
            return Err(Error::InvalidValue);
        }

        // 计算 sin/cos 系数
        let angle = 2.0 * PI * freq_hz / (sample_rate as f32);
        let sin_val = (angle.sin() * 32768.0).round() as i16;
        let cos_val = (angle.cos() * 32768.0).round() as i16;

        // 计算样本长度
        let num_samples = (sample_rate as u64 * duration_ms as u64 / 1000) as u32;

        // 设置音量 (左右声道相同)
        let volume_reg = (-volume_db) as u8;
        self.write_register(P0_R71::PAGE, P0_R71::ADDRESS, volume_reg)?;
        self.write_register(P0_R72::PAGE, P0_R72::ADDRESS, volume_reg)?;

        // 设置持续时间
        self.write_register(P0_R73::PAGE, P0_R73::ADDRESS, (num_samples >> 16) as u8)?;
        self.write_register(P0_R74::PAGE, P0_R74::ADDRESS, (num_samples >> 8) as u8)?;
        self.write_register(P0_R75::PAGE, P0_R75::ADDRESS, num_samples as u8)?;

        // 设置频率系数
        self.write_register(P0_R76::PAGE, P0_R76::ADDRESS, (sin_val >> 8) as u8)?;
        self.write_register(P0_R77::PAGE, P0_R77::ADDRESS, sin_val as u8)?;
        self.write_register(P0_R78::PAGE, P0_R78::ADDRESS, (cos_val >> 8) as u8)?;
        self.write_register(P0_R79::PAGE, P0_R79::ADDRESS, cos_val as u8)?;

        // 触发蜂鸣声 (使能位会自动清除)
        self.update_register(P0_R71::PAGE, P0_R71::ADDRESS, 1 << 7, 1 << 7)
    }

    /// 读取DAC模拟增益控制标志寄存器（Page 1, Register 0x3F）
    /// 功能：获取DAC通道模拟增益（输出放大器、信号路由）的软步进调整状态
    pub fn read_dac_analog_gain_flag(&mut self) -> Result<DacAnalogGainFlags, Error<E>> {
        use self::registers::P1_R63;
        let reg_val = self.read_register(P1_R63::PAGE, P1_R63::ADDRESS)?;

        Ok(DacAnalogGainFlags {
            left_amp_stable: (reg_val & (1 << 7)) != 0, // D7：左DAC输出放大器增益稳定
            left_in1_stable: (reg_val & (1 << 6)) != 0, // D6：左声道IN1→HPL衰减稳定
            left_mix_stable: (reg_val & (1 << 5)) != 0, // D5：左声道混音器→HPL衰减稳定
            reserved_d4: (reg_val & (1 << 4)) != 0,     // D4：预留
            right_amp_stable: (reg_val & (1 << 3)) != 0, // D3：右DAC输出放大器增益稳定
            right_in1_stable: (reg_val & (1 << 2)) != 0, // D2：右声道IN1→HPR衰减稳定
            right_mix_stable: (reg_val & (1 << 1)) != 0, // D1：右声道混音器→HPR衰减稳定
            reserved_d0: (reg_val & (1 << 0)) != 0,     // D0：预留
        })
    }
}

// --- ADC, 输入和路由 ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 设置 ADC 通道的电源状态。
    pub fn set_adc_channel_power(&mut self, left_on: bool, right_on: bool) -> Result<(), Error<E>> {
        use self::registers::P0_R81;

        let val = (if left_on {
            P0_R81::LEFT_ADC_POWER_UP
        } else {
            0
        }) | (if right_on {
            P0_R81::RIGHT_ADC_POWER_UP
        } else {
            0
        });
        let mask = P0_R81::LEFT_ADC_POWER_UP | P0_R81::RIGHT_ADC_POWER_UP;
        self.update_register(P0_R81::PAGE, P0_R81::ADDRESS, mask, val)
    }
    /// 配置麦克风偏置电压 (MICBIAS)。
    ///
    /// # Arguments
    ///
    /// * `enable` - 是否启用MICBIAS。
    /// * `voltage` - 期望的输出电压。
    /// * `use_ldoin_supply` - `true` 表示从 LDOIN 供电, `false` 表示从 AVDD 供电。
    pub fn configure_micbias(
        &mut self,
        enable: bool,
        voltage: registers::MicbiasVoltage,
        use_ldoin_supply: bool,
    ) -> Result<(), Error<E>> {
        use registers::P1_R51;

        if !enable {
            // Power down MICBIAS by clearing the power-up bit (D6)
            return self.update_register(P1_R51::PAGE, P1_R51::ADDRESS, P1_R51::POWER_UP, 0);
        }

        let micbias_val = match voltage {
            registers::MicbiasVoltage::V1_25 => 0b00,
            registers::MicbiasVoltage::V1_7 => 0b01,
            registers::MicbiasVoltage::V2_5 => 0b10,
            registers::MicbiasVoltage::PassThrough => 0b11,
        };

        // D6=1 (上电), D5-4=voltage, D3=supply
        let mut reg_val = (1 << 6) | (micbias_val << 4);
        if use_ldoin_supply {
            reg_val |= 1 << 3;
        }

        self.write_register(P1_R51::PAGE, P1_R51::ADDRESS, reg_val)
    }

    /// 配置数字麦克风。
    /// 注意: 调用此函数后，相关的模拟输入路径可能会被旁路。
    pub fn configure_digital_mic(
        &mut self,
        left_on: bool,
        right_on: bool,
        input: DigitalMicInput,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R81;
        let mut val = (input as u8) << 4;
        if left_on {
            val |= 1 << 3;
        }
        if right_on {
            val |= 1 << 2;
        }
        let mask = 0b0011_1100; // D5-D2
        self.update_register(P0_R81::PAGE, P0_R81::ADDRESS, mask, val)
    }

    /// 设置左路 MICPGA 正端的输入路由和阻抗。
    pub fn set_left_micpga_positive_input(
        &mut self,
        routing: MicPgaLeftPositiveInput,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R52;

        let val = ((routing.in1l as u8) << 6)
            | ((routing.in2l as u8) << 4)
            | ((routing.in3l as u8) << 2)
            | (routing.in1r as u8);
        self.write_register(P1_R52::PAGE, P1_R52::ADDRESS, val)
    }

    /// 设置左路 MICPGA 负端的输入路由和阻抗。
    pub fn set_left_micpga_negative_input(
        &mut self,
        routing: MicPgaLeftNegativeInput,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R54;

        let val = ((routing.cm1l as u8) << 6)
            | ((routing.in2r as u8) << 4)
            | ((routing.in3r as u8) << 2)
            | (routing.cm2l as u8);
        self.write_register(P1_R54::PAGE, P1_R54::ADDRESS, val)
    }

    /// 设置右路 MICPGA 正端的输入路由和阻抗。
    pub fn set_right_micpga_positive_input(
        &mut self,
        routing: MicPgaRightPositiveInput,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R55;

        let val = ((routing.in1r as u8) << 6)
            | ((routing.in2r as u8) << 4)
            | ((routing.in3r as u8) << 2)
            | (routing.in2l as u8);
        self.write_register(P1_R55::PAGE, P1_R55::ADDRESS, val)
    }

    /// 设置右路 MICPGA 负端的输入路由和阻抗。
    pub fn set_right_micpga_negative_input(
        &mut self,
        routing: MicPgaRightNegativeInput,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R57;

        let val = ((routing.cm1r as u8) << 6)
            | ((routing.in1l as u8) << 4)
            | ((routing.in3l as u8) << 2)
            | (routing.cm2r as u8);
        self.write_register(P1_R57::PAGE, P1_R57::ADDRESS, val)
    }

    /// 设置 MICPGA 的模拟增益。
    /// 增益值应为 0.5 的倍数，范围 0.0 到 47.5 dB。
    pub fn set_micpga_gain_db(
        &mut self,
        left_gain_db: f32,
        right_gain_db: f32,
    ) -> Result<(), Error<E>> {
        use self::registers::{P1_R59, P1_R60};

        let left_val = (left_gain_db * 2.0).round() as u8;
        let right_val = (right_gain_db * 2.0).round() as u8;
        if left_val > 0x5F || right_val > 0x5F {
            return Err(Error::InvalidValue);
        }

        // 增益值本身 D7 位为 0，会自动解除静音
        self.update_register(
            P1_R59::PAGE,
            P1_R59::ADDRESS,
            P1_R59::LMIC_CONTROL_MASK,
            left_val,
        )?;
        self.update_register(
            P1_R60::PAGE,
            P1_R60::ADDRESS,
            P1_R60::RMIC_CONTROL_MASK,
            right_val,
        )
    }

    /// 解除 MICPGA 的静音。
    pub fn unmute_micpga(&mut self, unmute_left: bool, unmute_right: bool) -> Result<(), Error<E>> {
        use self::registers::{P1_R59, P1_R60};

        if unmute_left {
            self.update_register(P1_R59::PAGE, P1_R59::ADDRESS, P1_R59::MUTE_ENABLE, 0)?;
        }
        if unmute_right {
            self.update_register(P1_R60::PAGE, P1_R60::ADDRESS, P1_R60::MUTE_ENABLE, 0)?;
        }
        Ok(())
    }

    /// 设置 ADC 的数字音量。
    /// `*_volume_db`: 范围 -12.0 到 +20.0 dB，步进 0.5 dB。
    pub fn set_adc_digital_volume_db(
        &mut self,
        left_volume_db: f32,
        right_volume_db: f32,
    ) -> Result<(), Error<E>> {
        use self::registers::{P0_R83, P0_R84};

        // 音量范围从 -12dB 到 +20dB
        if !(-12.0..=20.0).contains(&left_volume_db) || !(-12.0..=20.0).contains(&right_volume_db) {
            return Err(Error::InvalidValue);
        }

        let left_val = (left_volume_db * 2.0).round() as i8;
        let right_val = (right_volume_db * 2.0).round() as i8;

        self.write_register(P0_R83::PAGE, P0_R83::ADDRESS, left_val as u8)?;
        self.write_register(P0_R84::PAGE, P0_R84::ADDRESS, right_val as u8)
    }

    /// 解除 ADC 数字部分的静音。
    pub fn unmute_adc_digital(
        &mut self,
        unmute_left: bool,
        unmute_right: bool,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R82;

        let mut mask = 0;
        if unmute_left {
            mask |= P0_R82::LEFT_MUTE;
        }
        if unmute_right {
            mask |= P0_R82::RIGHT_MUTE;
        }
        self.update_register(P0_R82::PAGE, P0_R82::ADDRESS, mask, 0)
    }

    /// 配置 ADC 的自动增益控制 (AGC)。
    ///
    /// # Arguments
    /// * `left_config` - 左声道的 AGC 配置。如果为 `None`，则不改变左声道设置。
    /// * `right_config` - 右声道的 AGC 配置。如果为 `None`，则不改变右声道设置。
    pub fn configure_agc(
        &mut self,
        left_config: Option<AgcConfig>,
        right_config: Option<AgcConfig>,
    ) -> Result<(), Error<E>> {
        if let Some(config) = left_config {
            self.configure_agc_channel(&config, true)?;
        }
        if let Some(config) = right_config {
            self.configure_agc_channel(&config, false)?;
        }
        Ok(())
    }

    /// 配置 ADC 的自动增益控制 (AGC)。
    ///
    /// # Arguments
    /// * `left_config` - 左声道的 AGC 配置。如果为 `None`，则不改变左声道设置。
    /// * `is_left` - `true` 配置左声道，`false` 配置右声道
    pub fn configure_agc_channel(
        &mut self,
        config: &AgcConfig,
        is_left: bool,
    ) -> Result<(), Error<E>> {
        use self::registers::{
            P0_R100, P0_R86, P0_R87, P0_R88, P0_R89, P0_R90, P0_R91, P0_R92, P0_R94, P0_R95,
            P0_R96, P0_R97, P0_R98, P0_R99,
        };

        let (r86, r87, r88, r89, r90, r91, r92) = if is_left {
            (
                P0_R86::ADDRESS,
                P0_R87::ADDRESS,
                P0_R88::ADDRESS,
                P0_R89::ADDRESS,
                P0_R90::ADDRESS,
                P0_R91::ADDRESS,
                P0_R92::ADDRESS,
            )
        } else {
            (
                P0_R94::ADDRESS,
                P0_R95::ADDRESS,
                P0_R96::ADDRESS,
                P0_R97::ADDRESS,
                P0_R98::ADDRESS,
                P0_R99::ADDRESS,
                P0_R100::ADDRESS,
            )
        };

        // Enable/Disable and Target Level
        let mut reg86_val = (config.target_level as u8) << 4;
        if config.enabled {
            reg86_val |= 1 << 7;
        }
        self.update_register(P0_R86::PAGE, r86, 0b1111_0000, reg86_val)?;

        // Hysteresis and Noise Threshold
        let reg87_val = ((config.hysteresis as u8) << 6) | ((config.noise_threshold as u8) << 1);
        self.write_register(P0_R87::PAGE, r87, reg87_val)?;

        // Max PGA Gain
        if !(0.0..=58.0).contains(&config.max_pga_gain_db) {
            return Err(Error::InvalidValue);
        }
        let max_gain_val = (config.max_pga_gain_db * 2.0).round() as u8;
        self.write_register(P0_R88::PAGE, r88, max_gain_val)?;

        // Attack Time (approximate mapping)
        let attack_val = ((config.attack_time_ms / 0.032).log2().round() as u8).clamp(0, 31);
        self.write_register(P0_R89::PAGE, r89, attack_val << 3)?;

        // Decay Time (approximate mapping)
        let decay_val = ((config.decay_time_ms / 0.512).log2().round() as u8).clamp(0, 31);
        self.write_register(P0_R90::PAGE, r90, decay_val << 3)?;

        // 噪声防抖时间配置
        let noise_val = ((config.noise_time_ms / 32.0).log2().round() as u8).clamp(0, 15);
        self.write_register(P0_R91::PAGE, r91, noise_val)?;

        // 信号检测防抖时间
        let signal_val = ((config.signal_time_ms / 8.0).log2().round() as u8).clamp(0, 3);
        self.write_register(P0_R92::PAGE, r92, signal_val)?;

        Ok(())
    }

    /// 读取指定声道的AGC当前实际应用增益
    /// # Arguments
    /// * `is_left` - `true` 读取左声道，`false` 读取右声道
    /// # 返回值
    /// 增益值（单位：dB，范围0~47.5dB，步进0.5dB）
    pub fn read_agc_gain(&mut self, is_left: bool) -> Result<f32, Error<E>> {
        use self::registers::{P0_R101, P0_R93};

        // 根据声道选择寄存器地址
        let reg_addr = if is_left {
            P0_R93::ADDRESS // 左声道：Page 0, Register 0x5D
        } else {
            P0_R101::ADDRESS // 右声道：Page 0, Register 0x65
        };

        // 读取寄存器值（8位，只读）
        let reg_val = self.read_register(0, reg_addr)?;

        // 转换为dB：寄存器值 × 0.5dB（每1单位对应0.5dB增益）
        Ok(reg_val as f32 * 0.5)
    }

    /// 设置 ADC 通道的数字精细增益和静音状态。
    pub fn set_adc_fine_gain_and_mute(
        &mut self,
        left_gain: AdcFineGain,
        left_mute: bool,
        right_gain: AdcFineGain,
        right_mute: bool,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R82;
        let mut val = ((left_gain as u8) << 4) | (right_gain as u8);
        if left_mute {
            val |= P0_R82::LEFT_MUTE;
        }
        if right_mute {
            val |= P0_R82::RIGHT_MUTE;
        }
        self.write_register(P0_R82::PAGE, P0_R82::ADDRESS, val)
    }

    /// 设置 ADC 通道间的相位延迟。
    ///
    /// # Arguments
    /// * `delay_val` - 延迟值，具体效果取决于 AOSR 和滤波器类型，请参考数据手册。
    pub fn set_adc_phase_adjust(&mut self, delay_val: u8) -> Result<(), Error<E>> {
        use self::registers::P0_R85;
        self.write_register(P0_R85::PAGE, P0_R85::ADDRESS, delay_val)
    }

    /// 读取ADC模拟音量控制标志寄存器（Page 1, Register 0x3E）
    /// 功能：获取ADC通道模拟增益（PGA）的软步进调整状态
    pub fn read_adc_analog_volume_flag(&mut self) -> Result<AdcAnalogVolumeFlags, Error<E>> {
        use self::registers::P1_R62;
        let reg_val = self.read_register(P1_R62::PAGE, P1_R62::ADDRESS)?;

        Ok(AdcAnalogVolumeFlags {
            left_pga_stable: (reg_val & (1 << 7)) != 0, // D7：左ADC PGA增益稳定
            reserved_d6: (reg_val & (1 << 6)) != 0,     // D6：预留
            reserved_d5: (reg_val & (1 << 5)) != 0,     // D5：预留
            reserved_d4: (reg_val & (1 << 4)) != 0,     // D4：预留
            right_pga_stable: (reg_val & (1 << 3)) != 0, // D3：右ADC PGA增益稳定
            reserved_d2: (reg_val & (1 << 2)) != 0,     // D2：预留
            reserved_d1: (reg_val & (1 << 1)) != 0,     // D1：预留
            reserved_d0: (reg_val & (1 << 0)) != 0,     // D0：预留
        })
    }

    /// 配置并使能 ADC 的 DC 直流测量模式。
    pub fn setup_dc_measurement(
        &mut self,
        left_enable: bool,
        right_enable: bool,
        config: &DcMeasurementConfig,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R102;
        if !(1..=20).contains(&config.d_param) {
            return Err(Error::InvalidValue);
        }

        let mut val = config.d_param;
        if config.mode == DcMeasurementMode::LowPassIIR {
            val |= 1 << 5;
        }
        if left_enable {
            val |= 1 << 7;
        }
        if right_enable {
            val |= 1 << 6;
        }

        self.write_register(P0_R102::PAGE, P0_R102::ADDRESS, val)
    }

    /// 读取 DC 测量结果。
    ///
    /// # Returns
    ///
    /// * `Ok(i32)`: 24位有符号的测量结果。
    pub fn read_dc_measurement(&mut self, is_left_channel: bool) -> Result<i32, Error<E>> {
        use self::registers::{P0_R103, P0_R104, P0_R105, P0_R106, P0_R107, P0_R108, P0_R109};

        // Latch the data
        self.update_register(P0_R103::PAGE, P0_R103::ADDRESS, 1 << 6, 1 << 6)?;

        let (c_msb, c_mid, c_lsb) = if is_left_channel {
            (P0_R106::ADDRESS, P0_R105::ADDRESS, P0_R104::ADDRESS)
        } else {
            // Assuming right channel registers are P0_R107, P0_R108, P0_R109
            (P0_R109::ADDRESS, P0_R108::ADDRESS, P0_R107::ADDRESS)
        };

        let msb = self.read_register(P0_R104::PAGE, c_msb)? as u32;
        let mid = self.read_register(P0_R104::PAGE, c_mid)? as u32;
        let lsb = self.read_register(P0_R104::PAGE, c_lsb)? as u32;

        // Unlatch
        self.update_register(P0_R103::PAGE, P0_R103::ADDRESS, 1 << 6, 0)?;

        let mut result = (msb << 16) | (mid << 8) | lsb;
        // Sign extend if the 24th bit is 1
        if (result & 0x0080_0000) != 0 {
            result |= 0xFF00_0000;
        }

        Ok(result as i32)
    }
}

// --- GPIO
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 设置 GPIO/MFP5 引脚的功能。
    ///
    /// GPIO 引脚是一个多功能引脚 (MFP5)，可以配置为多种用途，
    /// 例如通用输入/输出、时钟输出或中断输出。
    ///
    /// # Arguments
    ///
    /// * `function` - 要为 GPIO 引脚设置的功能，来自 `GpioFunction` 枚举。
    pub fn set_gpio_function(&mut self, function: GpioFunction) -> Result<(), Error<E>> {
        use self::registers::P0_R52;
        self.update_register(
            P0_R52::PAGE,
            P0_R52::ADDRESS,
            P0_R52::GPIO_CTRL_MASK,
            (function as u8) << 2, // 功能控制位在 D5-D2
        )
    }

    /// 配置 DOUT/MFP2 引脚的功能。
    pub fn set_dout_mfp2_function(&mut self, function: DoutMfp2Function) -> Result<(), Error<E>> {
        use self::registers::P0_R53;
        self.update_register(
            P0_R53::PAGE,
            P0_R53::ADDRESS,
            0b0000_1110,
            (function as u8) << 1,
        )
    }

    /// 配置 DIN/MFP1 引脚的功能。
    pub fn set_din_mfp1_function(&mut self, function: DinMfp1Function) -> Result<(), Error<E>> {
        use self::registers::P0_R54;
        self.update_register(
            P0_R54::PAGE,
            P0_R54::ADDRESS,
            0b0000_0110,
            (function as u8) << 1,
        )
    }

    /// 配置 MISO/MFP4 引脚的功能。
    pub fn set_miso_mfp4_function(&mut self, function: MisoMfp4Function) -> Result<(), Error<E>> {
        use self::registers::P0_R55;
        self.update_register(
            P0_R55::PAGE,
            P0_R55::ADDRESS,
            0b0001_1110,
            (function as u8) << 1,
        )
    }

    /// 配置 SCLK/MFP3 引脚的功能。
    pub fn set_sclk_mfp3_function(&mut self, function: SclkMfp3Function) -> Result<(), Error<E>> {
        use self::registers::P0_R56;
        self.update_register(
            P0_R56::PAGE,
            P0_R56::ADDRESS,
            0b0000_0110,
            (function as u8) << 1,
        )
    }

    /// 设置配置为通用输出的引脚的电平。
    ///
    /// **注意**: 此方法仅在引脚已通过 `set_*_function`
    /// 配置为 `GeneralPurposeOutput` 时有效。
    pub fn set_gpio_output(&mut self, pin: MfpPin, high: bool) -> Result<(), Error<E>> {
        use self::registers::{P0_R52, P0_R53, P0_R55};
        let value = if high { 1 } else { 0 };
        match pin {
            MfpPin::GpioMfp5 => self.update_register(P0_R52::PAGE, P0_R52::ADDRESS, 1, value),
            MfpPin::DoutMfp2 => self.update_register(P0_R53::PAGE, P0_R53::ADDRESS, 1, value),
            MfpPin::MisoMfp4 => self.update_register(P0_R55::PAGE, P0_R55::ADDRESS, 1, value),
            _ => Err(Error::InvalidValue), // DIN 和 SCLK 是输入引脚
        }
    }

    /// 读取配置为通用输入的引脚的电平状态。
    ///
    /// **注意**: 此方法仅在引脚已通过 `set_*_function`
    /// 配置为 `GeneralPurposeInput` 时有效。
    pub fn read_gpio_input(&mut self, pin: MfpPin) -> Result<bool, Error<E>> {
        use self::registers::{P0_R52, P0_R54, P0_R56};
        let reg_val = match pin {
            MfpPin::GpioMfp5 => self.read_register(P0_R52::PAGE, P0_R52::ADDRESS)?,
            MfpPin::DinMfp1 => self.read_register(P0_R54::PAGE, P0_R54::ADDRESS)?,
            MfpPin::SclkMfp3 => self.read_register(P0_R56::PAGE, P0_R56::ADDRESS)?,
            _ => return Err(Error::InvalidValue), // DOUT 和 MISO 是输出引脚
        };

        // D1 用于 GPIO/MFP5, D0 用于其他输入引脚
        let mask = if pin == MfpPin::GpioMfp5 { 1 << 1 } else { 1 };
        Ok((reg_val & mask) != 0)
    }
}

// --- 简化配置工具 ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 自动配置 DAC 采样率。
    ///
    /// 此方法会根据给定的主时钟、目标采样率和优先级，自动计算并设置
    /// NDAC, MDAC, 和 DOSR 分频器。
    ///
    /// # Arguments
    /// * `codec_clkin` - 提供给编解码器核心的时钟频率 (Hz)。通常是 MCLK 或 PLL 输出。
    /// * `target_rate` - 目标采样率 (Hz)，例如 48000。
    /// * `priority` - 性能与功耗的权衡策略。
    pub fn configure_dac_rate(
        &mut self,
        codec_clkin: u32,
        target_rate: u32,
        priority: SampleRatePriority,
    ) -> Result<(), Error<E>> {
        // 数据手册中推荐的 DOSR 候选值，从高性能到低功耗排序
        const DOSR_CANDIDATES: &[u16] = &[
            1024, 768, 512, 384, 256, 192, 128, 96, 64, 48, 32, 24, 16, 8,
        ];

        if codec_clkin == 0 || target_rate == 0 || codec_clkin % target_rate != 0 {
            return Err(Error::InvalidClockRate);
        }
        let total_ratio = codec_clkin / target_rate;

        if let Some((n, m, osr)) = self.find_dividers(total_ratio, DOSR_CANDIDATES, priority) {
            self.set_dac_dividers(n, true, m, true)?;
            self.set_dac_oversampling(osr)?;
            Ok(())
        } else {
            Err(Error::ClockConfigNotFound)
        }
    }

    /// 自动配置 ADC 采样率。
    ///
    /// 此方法会根据给定的主时钟、目标采样率和优先级，自动计算并设置
    /// NADC, MADC, 和 AOSR 分频器。
    ///
    /// # Arguments
    /// * `codec_clkin` - 提供给编解码器核心的时钟频率 (Hz)。通常是 MCLK 或 PLL 输出。
    /// * `target_rate` - 目标采样率 (Hz)，例如 48000。
    /// * `priority` - 性能与功耗的权衡策略。
    pub fn configure_adc_rate(
        &mut self,
        codec_clkin: u32,
        target_rate: u32,
        priority: SampleRatePriority,
    ) -> Result<(), Error<E>> {
        // 数据手册中明确支持的 AOSR 值，从高性能到低功耗排序
        const AOSR_CANDIDATES: &[u16] = &[256, 128, 64, 32];

        if codec_clkin == 0 || target_rate == 0 || codec_clkin % target_rate != 0 {
            return Err(Error::InvalidClockRate);
        }
        let total_ratio = codec_clkin / target_rate;

        if let Some((n, m, osr)) = self.find_dividers(total_ratio, AOSR_CANDIDATES, priority) {
            self.set_adc_dividers(n, true, m, true)?;
            self.set_adc_oversampling(osr)?;
            Ok(())
        } else {
            Err(Error::ClockConfigNotFound)
        }
    }

    /// 寻找合适的分频器组合的私有辅助函数。
    fn find_dividers(
        &self,
        total_ratio: u32,
        osr_candidates: &'static [u16],
        priority: SampleRatePriority,
    ) -> Option<(u8, u8, u16)> {
        let mut valid_options: Vec<(u8, u8, u16)> = Vec::new();

        for &osr in osr_candidates {
            if total_ratio % (osr as u32) == 0 {
                let rem_ratio = total_ratio / (osr as u32);
                if rem_ratio == 0 || rem_ratio > (128 * 128) {
                    continue;
                }

                // 寻找 N 和 M 的组合，优先选择较大的 N
                for n in (1..=128).rev() {
                    if rem_ratio % n == 0 {
                        let m = rem_ratio / n;
                        if m > 0 && m <= 128 {
                            // 找到了一个有效的组合 (n, m, osr)
                            valid_options.push((n as u8, m as u8, osr));
                            break; // 找到第一个（N最大的）组合就足够了
                        }
                    }
                }
            }
        }

        if valid_options.is_empty() {
            return None;
        }

        // 根据优先级选择一个组合
        let index = match priority {
            SampleRatePriority::HighestPerformance => 0,
            SampleRatePriority::HighPerformance => valid_options.len() / 4,
            SampleRatePriority::Balanced => valid_options.len() / 2,
            SampleRatePriority::LowPower => (valid_options.len() * 3) / 4,
            SampleRatePriority::LowestPower => valid_options.len() - 1,
        };

        valid_options.get(index).cloned()
    }
}

/// --- 高级功能: 耳机检测, 中断, 抗Thump ---
impl<I2C, E> Tlv320aic3204<I2C>
where
    I2C: I2c<Error = E>,
{
    /// 配置并使能耳机/按键检测。
    pub fn configure_headset_detection(
        &mut self,
        headset_debounce: HeadsetDebounceTime,
        button_debounce: ButtonDebounceTime,
        enable: bool,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R67;
        let mut val = ((headset_debounce as u8) << 2) | (button_debounce as u8);
        if enable {
            val |= 1 << 7;
        }
        self.write_register(P0_R67::PAGE, P0_R67::ADDRESS, val)
    }

    /// 读取耳机检测状态。
    pub fn read_headset_status(&mut self) -> Result<HeadsetDetectionStatus, Error<E>> {
        use self::registers::{P0_R46, P0_R67};
        let status_reg = self.read_register(P0_R46::PAGE, P0_R46::ADDRESS)?;
        let type_reg = self.read_register(P0_R67::PAGE, P0_R67::ADDRESS)?;

        let headset_type = match (type_reg >> 5) & 0b11 {
            0b01 => HeadsetType::Stereo,
            0b11 => HeadsetType::StereoWithMic,
            0b00 => HeadsetType::NotDetected,
            _ => HeadsetType::Reserved,
        };

        Ok(HeadsetDetectionStatus {
            headset_type,
            insertion_detected: (status_reg & (1 << 4)) != 0,
            button_pressed: (status_reg & (1 << 5)) != 0,
        })
    }

    /// 读取所有中断和状态标志。
    ///
    /// 此函数会一次性读取所有相关的状态寄存器，并返回一个包含所有标志的结构体。
    /// 注意：读取粘滞标志寄存器 (P0_R42, P0_R44, P0_R45) 会自动清除其中的标志位。
    pub fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error<E>> {
        use self::registers::{
            HeadsetType, P0_R42, P0_R43, P0_R44, P0_R45, P0_R46, P0_R47, P0_R67,
        };
        let mut status = InterruptStatus::default();

        // 读取粘滞标志
        let r42 = self.read_register(P0_R42::PAGE, P0_R42::ADDRESS)?;
        let r44 = self.read_register(P0_R44::PAGE, P0_R44::ADDRESS)?;
        let r45 = self.read_register(P0_R45::PAGE, P0_R45::ADDRESS)?;

        status.sticky_dac_overflow = (r42 & 0b1100_0000) != 0;
        status.sticky_adc_overflow = (r42 & 0b0000_1100) != 0;
        status.sticky_hpl_over_current = (r44 & (1 << 7)) != 0;
        status.sticky_hpr_over_current = (r44 & (1 << 6)) != 0;
        status.sticky_button_press_detected = (r44 & (1 << 5)) != 0;
        status.sticky_headset_insertion_detected = (r44 & (1 << 4)) != 0;
        status.sticky_dac_drc_threshold_exceeded = (r44 & (1 << 3)) != 0; // 仅左声道
        status.sticky_agc_noise_detected = (r45 & (1 << 6)) != 0; // 仅左声道
        status.sticky_dc_measurement_data_available = (r45 & (1 << 2)) != 0; // 仅左声道

        // 读取实时状态
        let r43 = self.read_register(P0_R43::PAGE, P0_R43::ADDRESS)?;
        let r46 = self.read_register(P0_R46::PAGE, P0_R46::ADDRESS)?;
        let r47 = self.read_register(P0_R47::PAGE, P0_R47::ADDRESS)?;
        let r67 = self.read_register(P0_R67::PAGE, P0_R67::ADDRESS)?;

        status.live_dac_overflow = (r43 & 0b1100_0000) != 0;
        status.live_adc_overflow = (r43 & 0b0000_1100) != 0;
        status.live_headset_inserted = (r46 & (1 << 4)) != 0;
        status.live_button_pressed = (r46 & (1 << 5)) != 0;
        status.live_dac_drc_threshold_exceeded = (r46 & (1 << 3)) != 0;
        status.live_agc_noise_detected = (r47 & (1 << 6)) != 0;
        status.live_dc_measurement_data_available = (r47 & (1 << 2)) != 0;

        // 读取耳机类型
        status.headset_type = match (r67 >> 5) & 0b11 {
            0b01 => HeadsetType::Stereo,
            0b11 => HeadsetType::StereoWithMic,
            0b00 => HeadsetType::NotDetected,
            _ => HeadsetType::Reserved,
        };

        Ok(status)
    }

    /// 配置 INT1 中断。
    pub fn configure_int1(
        &mut self,
        sources: InterruptSource,
        mode: InterruptPulseMode,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R48;
        let val = sources.bits() | (mode as u8);
        self.write_register(P0_R48::PAGE, P0_R48::ADDRESS, val)
    }

    /// 配置 INT2 中断。
    pub fn configure_int2(
        &mut self,
        sources: InterruptSource,
        mode: InterruptPulseMode,
    ) -> Result<(), Error<E>> {
        use self::registers::P0_R49;
        let val = sources.bits() | (mode as u8);
        self.write_register(P0_R49::PAGE, P0_R49::ADDRESS, val)
    }

    /// 配置浮空输入（抗Thump）。
    /// 将未使用的输入引脚弱连接到共模电压。
    pub fn configure_floating_inputs(
        &mut self,
        config: FloatingInputConfig,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R58;
        self.write_register(P1_R58::PAGE, P1_R58::ADDRESS, config.bits())
    }

    /// 设置模拟输入的快速充电时间。
    pub fn set_analog_input_fast_charge(
        &mut self,
        time: AnalogInputChargeTime,
    ) -> Result<(), Error<E>> {
        use self::registers::P1_R71;
        self.write_register(P1_R71::PAGE, P1_R71::ADDRESS, time as u8)
    }
}
