mod gain_enum;
mod page0;
mod page1;

pub use gain_enum::*;
pub use page0::*;
pub use page1::*;

use bitflags::bitflags;

// --- 枚举和位标志定义 ---

/// PLL 输入时钟源选择 (P0, R4, D3-D2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PllInput {
    MCLK = 0b0000_0000,
    BCLK = 0b0000_0100,
    GPIO = 0b0000_1000,
    DIN = 0b0000_1100,
}

/// CODEC_CLKIN 时钟源选择 (P0, R4, D1-D0)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CodecClockInput {
    MCLK = 0b0000_0000,
    BCLK = 0b0000_0001,
    GPIO = 0b0000_0010,
    PLL = 0b0000_0011,
}

/// 数字音频接口格式 (P0, R27, D7-D6)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AudioInterface {
    I2S = 0b0000_0000,
    DSP = 0b0100_0000,
    RJF = 0b1000_0000, // Right-Justified Format
    LJF = 0b1100_0000, // Left-Justified Format
}

/// 音频数据字长 (P0, R27, D5-D4)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WordLength {
    Bits16 = 0b0000_0000,
    Bits20 = 0b0001_0000,
    Bits24 = 0b0010_0000,
    Bits32 = 0b0011_0000,
}

/// DAC (播放) 信号处理块 (PRB) 选择 (P0, R60)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DacProcessingBlock {
    PRB_P1 = 1,
    PRB_P2 = 2,
    PRB_P3 = 3,
    PRB_P4 = 4,
    PRB_P5 = 5,
    PRB_P6 = 6,
    PRB_P7 = 7,
    PRB_P8 = 8,
    PRB_P9 = 9,
    PRB_P10 = 10,
    PRB_P11 = 11,
    PRB_P12 = 12,
    PRB_P13 = 13,
    PRB_P14 = 14,
    PRB_P15 = 15,
    PRB_P16 = 16,
    PRB_P17 = 17,
    PRB_P18 = 18,
    PRB_P19 = 19,
    PRB_P20 = 20,
    PRB_P21 = 21,
    PRB_P22 = 22,
    PRB_P23 = 23,
    PRB_P24 = 24,
    PRB_P25 = 25,
}

/// ADC (录音) 信号处理块 (PRB) 选择 (P0, R61)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcProcessingBlock {
    PRB_R1 = 1,
    PRB_R2 = 2,
    PRB_R3 = 3,
    PRB_R4 = 4,
    PRB_R5 = 5,
    PRB_R6 = 6,
    PRB_R7 = 7,
    PRB_R8 = 8,
    PRB_R9 = 9,
    PRB_R10 = 10,
    PRB_R11 = 11,
    PRB_R12 = 12,
    PRB_R13 = 13,
    PRB_R14 = 14,
    PRB_R15 = 15,
    PRB_R16 = 16,
    PRB_R17 = 17,
    PRB_R18 = 18,
}

/// 参考电压上电配置 (P1, R123)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RefChargingTime {
    /// 默认慢速上电
    Default = 0b000,
    /// 40ms 快速上电
    Ms40 = 0b001,
    /// 80ms 快速上电
    Ms80 = 0b010,
    /// 120ms 快速上电
    Ms120 = 0b011,
    /// 强制开启参考电压，慢速上电
    ForceOnSlow = 0b100,
    /// 强制开启参考电压，40ms 上电
    ForceOnMs40 = 0b101,
    /// 强制开启参考电压，80ms 上电
    ForceOnMs80 = 0b110,
    /// 强制开启参考电压，120ms 上电
    ForceOnMs120 = 0b111,
}

/// 软路由步进时间 (P1, R20, D7-D6)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SoftStepTime {
    Ms0 = 0b0000_0000,
    Ms50 = 0b0100_0000,
    Ms100 = 0b1000_0000,
    Ms200 = 0b1100_0000,
}

/// 慢速充电时间常数 (P1, R20, D5-D2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimeConstants {
    Disabled = 0b0000_0000,
    N0_5 = 0b0000_0100,
    N0_625 = 0b0000_1000,
    N0_75 = 0b0000_1100,
    N0_875 = 0b0001_0000,
    N1_0 = 0b0001_0100,
    N2_0 = 0b0001_1000,
    N3_0 = 0b0001_1100,
    N4_0 = 0b0010_0000,
    N5_0 = 0b0010_0100,
    N6_0 = 0b0010_1000,
    N7_0 = 0b0010_1100,
    N8_0 = 0b0011_0000,
    N16_0 = 0b0011_0100,
    N24_0 = 0b0011_1000,
    N32_0 = 0b0011_1100,
}

/// 去爆音（De-pop）电阻值 (P1, R20, D1-D0)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RpopValue {
    K25 = 0b00,
    K6 = 0b01,
    K2 = 0b10,
}

/// 耳机去爆音（De-pop）配置
pub struct HeadphoneDepopSettings {
    pub step_time: SoftStepTime,
    pub num_time_constants: TimeConstants,
    pub resistor: RpopValue,
}

impl Default for HeadphoneDepopSettings {
    /// 返回基于寄存器复位值的默认配置。
    ///
    /// - `step_time`: 0ms (禁用)
    /// - `num_time_constants`: 禁用
    /// - `resistor`: 25kΩ
    fn default() -> Self {
        Self {
            step_time: SoftStepTime::Ms0,
            num_time_constants: TimeConstants::Disabled,
            resistor: RpopValue::K25,
        }
    }
}

/// 全芯片共模电压设置 (P1, R10, D6)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommonModeVoltage {
    /// 0.9V 共模电压，适用于 AVDD >= 1.8V
    V0_9 = 0b0000_0000,
    /// 0.75V 共模电压，适用于 AVDD < 1.8V 或低功耗模式
    V0_75 = 0b0100_0000,
}

bitflags! {
    /// HPL (左耳机) 输入路由选择 (P1, R12)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct HplInput: u8 {
        const LEFT_DAC = 1 << 3;
        const IN1L = 1 << 2;
        const MAL = 1 << 1; // Left Mixer Amp
        const MAR = 1 << 0; // Right Mixer Amp
    }
}

bitflags! {
    /// HPR (右耳机) 输入路由选择 (P1, R13)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct HprInput: u8 {
        const LEFT_DAC_NEG = 1 << 4;
        const RIGHT_DAC = 1 << 3;
        const IN1R = 1 << 2;
        const MAR = 1 << 1; // Right Mixer Amp
        const HPL_OUT = 1 << 0;
    }
}

bitflags! {
    /// LOL (左线路输出) 输入路由选择 (P1, R14)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct LolInput: u8 {
        const RIGHT_DAC_NEG = 1 << 4;
        const LEFT_DAC = 1 << 3;
        const MAL = 1 << 1; // Left Mixer Amp
        const LOR_OUT = 1 << 0;
    }
}

bitflags! {
    /// LOR (右线路输出) 输入路由选择 (P1, R15)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct LorInput: u8 {
        const RIGHT_DAC = 1 << 3;
        const MAR = 1 << 1; // Right Mixer Amp
    }
}

bitflags! {
    /// 输出驱动器电源控制 (P1, R9)
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct OutputDriverPower: u8 {
        const HPL = 1 << 5;
        const HPR = 1 << 4;
        const LOL = 1 << 3;
        const LOR = 1 << 2;
        const MAL = 1 << 1;
        const MAR = 1 << 0;
    }
}

/// DAC PowerTune (PTM) 模式 (P1, R3/R4, D4-D2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DacPtmMode {
    /// PTM_P3 / PTM_P4 模式
    PTM_P3_P4 = 0b000,
    /// PTM_P2 模式
    PTM_P2 = 0b001,
    /// PTM_P1 模式 (最低功耗)
    PTM_P1 = 0b010,
}

/// ADC PowerTune (PTM) 模式 (P1, R61)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcPtmMode {
    /// PTM_R4 模式 (最高性能)
    PTM_R4 = 0x00,
    /// PTM_R3 模式
    PTM_R3 = 0x64,
    /// PTM_R2 模式
    PTM_R2 = 0xB6,
    /// PTM_R1 模式 (最低功耗)
    PTM_R1 = 0xFF,
}

/// DAC 数据路径控制 (P0, R63, D5-D2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DacDataPath {
    /// 数据被禁用
    Disabled = 0b00,
    /// 采用左声道音频接口数据
    LeftChanAudio = 0b01,
    /// 采用右声道音频接口数据
    RightChanAudio = 0b10,
    /// 采用左右声道混音数据
    MonoMix = 0b11,
}

/// MICPGA 输入路由电阻选择
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InputResistance {
    NotRouted = 0b00,
    K10 = 0b01,
    K20 = 0b10,
    K40 = 0b11,
}

// 为 InputResistance 实现 Default 特征，默认不路由
impl Default for InputResistance {
    fn default() -> Self {
        InputResistance::NotRouted
    }
}

/// 左路 MICPGA 正端输入路由配置 (P1, R52)
#[derive(Default, Debug)]
pub struct MicPgaLeftPositiveInput {
    pub in1l: InputResistance,
    pub in2l: InputResistance,
    pub in3l: InputResistance,
    pub in1r: InputResistance,
}

/// 左路 MICPGA 负端输入路由配置 (P1, R54)
#[derive(Default, Debug)]
pub struct MicPgaLeftNegativeInput {
    pub cm1l: InputResistance,
    pub in2r: InputResistance,
    pub in3r: InputResistance,
    pub cm2l: InputResistance,
}

/// 右路 MICPGA 正端输入路由配置 (P1, R55)
#[derive(Default, Debug)]
pub struct MicPgaRightPositiveInput {
    pub in1r: InputResistance,
    pub in2r: InputResistance,
    pub in3r: InputResistance,
    pub in2l: InputResistance,
}

/// 右路 MICPGA 负端输入路由配置 (P1, R57)
#[derive(Default, Debug)]
pub struct MicPgaRightNegativeInput {
    pub cm1r: InputResistance,
    pub in1l: InputResistance,
    pub in3l: InputResistance,
    pub cm2r: InputResistance,
}

/// 采样率配置的性能/功耗优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleRatePriority {
    /// 性能最高，功耗最高 (使用最高的可用过采样率)
    HighestPerformance,
    /// 性能较高，功耗均衡
    HighPerformance,
    /// 性能与功耗的平衡点
    Balanced,
    /// 功耗较低，性能可能略有下降
    LowPower,
    /// 功耗最低 (使用最低的可用过采样率)
    LowestPower,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioFunction {
    /// 禁用 GPIO 输入/输出功能。
    Disabled = 0b0000,
    /// 作为输入 (用于二级音频接口、数字麦克风或时钟输入)。
    /// 需要配置其他寄存器来选择具体功能 。
    Input = 0b0001,
    /// 通用输入。可以通过 `read_gpio_input()` 读取状态。
    GeneralPurposeInput = 0b0010,
    /// 通用输出。可以通过 `set_gpio_output()` 设置电平。
    GeneralPurposeOutput = 0b0011,
    /// CLKOUT 时钟输出。
    ClockOutput = 0b0100,
    /// INT1 中断输出。
    Int1Output = 0b0101,
    /// INT2 中断输出。
    Int2Output = 0b0110,
    /// ADC_WCLK (ADC 字时钟) 输出。
    AdcWordClockOutput = 0b0111,
    /// 二级接口位时钟 (BCLK) 输出。
    SecondaryBitClockOutput = 0b1000,
    /// 二级接口字时钟 (WCLK) 输出。
    SecondaryWordClockOutput = 0b1001,
    /// 数字麦克风时钟输出。
    DigitalMicrophoneClockOutput = 0b1010,
}

// --- 数字麦克风 ---
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DigitalMicInput {
    Gpio = 0b00,
    Sclk = 0b01,
    Din = 0b10,
}

// --- 动态范围压缩 (DRC) ---
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DrcThreshold {
    DbNeg3 = 0b000,
    DbNeg6 = 0b001,
    DbNeg9 = 0b010,
    DbNeg12 = 0b011,
    DbNeg15 = 0b100,
    DbNeg18 = 0b101,
    DbNeg21 = 0b110,
    DbNeg24 = 0b111,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DrcHysteresis {
    Db0 = 0b00,
    Db1 = 0b01,
    Db2 = 0b10,
    Db3 = 0b11,
}

/// DRC 配置
pub struct DrcConfig {
    pub left_drc_enabled: bool,
    pub right_drc_enabled: bool,
    pub threshold: DrcThreshold,
    pub hysteresis: DrcHysteresis,
    /// 攻击速率, 范围 0-15. 见数据手册 P0_R70 D7-4.
    pub attack_rate: u8,
    /// 衰减速率, 范围 0-15. 见数据手册 P0_R70 D3-0.
    pub decay_rate: u8,
}

impl Default for DrcConfig {
    fn default() -> Self {
        Self {
            left_drc_enabled: true,
            right_drc_enabled: true,
            threshold: DrcThreshold::DbNeg12, // -12dB是音乐处理的常见阈值
            hysteresis: DrcHysteresis::Db2,   // 2dB的滞后是比较平衡的选择
            attack_rate: 4,                   // 中等攻击速率，允许一些峰值通过
            decay_rate: 8,                    // 中等衰减速率，避免声音变"闷"
        }
    }
}

// --- 耳机/按键检测 ---
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HeadsetDebounceTime {
    Ms16 = 0b000,
    Ms32 = 0b001,
    Ms64 = 0b010,
    Ms128 = 0b011,
    Ms256 = 0b100,
    Ms512 = 0b101,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ButtonDebounceTime {
    Disabled = 0b00,
    Ms8 = 0b01,
    Ms16 = 0b10,
    Ms32 = 0b11,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum HeadsetType {
    #[default]
    NotDetected,
    Stereo,
    StereoWithMic,
    Reserved,
}

#[derive(Debug, Clone)]
pub struct HeadsetDetectionStatus {
    pub headset_type: HeadsetType,
    pub insertion_detected: bool,
    pub button_pressed: bool,
}

// --- 中断系统 ---
/// 中断源标志枚举
/// 使用位标志表示不同的中断事件源
bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct InterruptSource: u8 {
        // 耳机插入事件中断
        const HEADSET_INSERTION = 1 << 7;
        // 按钮按下事件中断
        const BUTTON_PRESS      = 1 << 6;
        // DAC 动态范围压缩（DRC）阈值超标中断
        const DAC_DRC_THRESHOLD = 1 << 5;
        // AGC 噪声检测中断
        const AGC_NOISE_DETECT  = 1 << 4;
        // 过流事件中断
        const OVER_CURRENT      = 1 << 3;
        // 数据溢出中断（ADC/DAC 数据溢出）
        const DATA_OVERFLOW     = 1 << 2;
        // 直流测量完成中断
        const DC_MEASUREMENT    = 1 << 1;
    }
}

/// 中断脉冲模式枚举
/// 配置中断输出的脉冲类型（单脉冲或脉冲串）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InterruptPulseMode {
    // 单脉冲模式（持续约 2ms）
    SinglePulse = 0,
    // 脉冲串模式（持续触发直到中断标志被读取）
    PulseTrain = 1,
}

// --- 抗Thump / 浮空输入 ---
bitflags! {
    /// 控制将未使用的模拟输入弱连接到共模电压，以防止噪声。
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct FloatingInputConfig: u8 {
        const IN1L = 1 << 7;
        const IN1R = 1 << 6;
        const IN2L = 1 << 5;
        const IN2R = 1 << 4;
        const IN3L = 1 << 3;
        const IN3R = 1 << 2;
    }
}

/// MICBIAS 输出电压配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MicbiasVoltage {
    /// 1.25V (当 CM=0.9V) 或 1.04V (当 CM=0.75V)
    V1_25,
    /// 1.7V (当 CM=0.9V) 或 1.425V (当 CM=0.75V)
    V1_7,
    /// 2.5V (当 CM=0.9V) 或 2.075V (当 CM=0.75V)
    V2_5,
    /// 直接连接到 AVDD 或 LDOIN
    PassThrough,
}

/// DAC 数字自动静音配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DacAutoMuteDuration {
    Disabled = 0b000,
    /// 100 个连续的 DC 输入样本后静音
    Samples100 = 0b001,
    Samples200 = 0b010,
    Samples400 = 0b011,
    Samples800 = 0b100,
    Samples1600 = 0b101,
    Samples3200 = 0b110,
    Samples6400 = 0b111,
}

/// ADC 数字音量精细调节
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcFineGain {
    Db0_0 = 0b000,
    DbNeg0_1 = 0b111,
    DbNeg0_2 = 0b110,
    DbNeg0_3 = 0b101,
    DbNeg0_4 = 0b100,
}

/// ADC 自动增益控制 (AGC) 目标电平
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AgcTargetLevel {
    DbNeg5_5 = 0b000,
    DbNeg8_0 = 0b001,
    DbNeg10_0 = 0b010,
    DbNeg12_0 = 0b011,
    DbNeg14_0 = 0b100,
    DbNeg17_0 = 0b101,
    DbNeg20_0 = 0b110,
    DbNeg24_0 = 0b111,
}

/// ADC AGC 迟滞设置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AgcHysteresis {
    Db1 = 0b00,
    Db2 = 0b01,
    Db4 = 0b10,
    Disabled = 0b11,
}

/// ADC 自动增益控制 (AGC) 配置
#[derive(Debug, Clone)]
pub struct AgcConfig {
    pub enabled: bool,
    pub target_level: AgcTargetLevel,
    pub hysteresis: AgcHysteresis,
    pub noise_threshold: AgcNoiseThreshold,
    /// 最大 PGA 增益, 0.0 到 58.0 dB, 步进 0.5 dB
    pub max_pga_gain_db: f32,
    /// 攻击时间, 0.032ms 到 2016ms
    pub attack_time_ms: f32,
    /// 衰减时间, 0.512ms 到 32256ms
    pub decay_time_ms: f32,
    /// 噪声检测防抖时间, 32ms 到 512ms
    pub noise_time_ms: f32,
    /// 信号检测防抖时间, 8ms 到 32ms
    pub signal_time_ms: f32,
}

impl Default for AgcConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            target_level: AgcTargetLevel::DbNeg17_0,
            hysteresis: AgcHysteresis::Db2,
            noise_threshold: AgcNoiseThreshold::DbNeg70, // 一个常用的默认值
            max_pga_gain_db: 30.0,
            attack_time_ms: 20.0,
            decay_time_ms: 100.0,
            noise_time_ms: 128.0,
            signal_time_ms: 16.0,
        }
    }
}

/// DC 直流测量模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DcMeasurementMode {
    MovingAverage,
    LowPassIIR,
}

/// DC 直流测量配置
#[derive(Debug, Clone)]
pub struct DcMeasurementConfig {
    pub mode: DcMeasurementMode,
    /// D 参数 (1-20), 用于控制移动平均长度或IIR滤波器带宽
    pub d_param: u8,
}

impl Default for DcMeasurementConfig {
    fn default() -> Self {
        Self {
            mode: DcMeasurementMode::MovingAverage,
            d_param: 10,
        }
    }
}

/// DOUT/MFP2 引脚功能配置枚举
/// 用于配置 DOUT 引脚的复用功能模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DoutMfp2Function {
    // 功能禁用
    Disabled = 0b000,
    // 主数据输出（SPI/I2S 数据输出）
    PrimaryDout = 0b001,
    // 通用输出引脚
    GeneralPurposeOutput = 0b010,
    // 时钟输出（CLKOUT）
    ClkOut = 0b011,
    // 中断 1 输出
    Int1 = 0b100,
    // 中断 2 输出
    Int2 = 0b101,
    // 次级位时钟（Secondary BCLK）
    SecondaryBclk = 0b110,
    // 次级字时钟（Secondary WCLK）
    SecondaryWclk = 0b111,
}

/// DIN/MFP1 引脚功能配置枚举
/// 用于配置 DIN 引脚的复用功能模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DinMfp1Function {
    // 功能禁用
    Disabled = 0b00,
    // 主数据输入或时钟输入（SPI/I2S 数据输入或数字麦克风时钟）
    PrimaryDinOrClock = 0b01,
    // 通用输入引脚
    GeneralPurposeInput = 0b10,
}

/// MISO/MFP4 引脚功能配置枚举
/// 用于配置 MISO 引脚的复用功能模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MisoMfp4Function {
    // 功能禁用
    Disabled = 0b0000,
    // SPI 从机输入（MISO）
    SpiMiso = 0b0001,
    // 通用输出引脚
    GeneralPurposeOutput = 0b0010,
    // 时钟输出（CLKOUT）
    ClkOut = 0b0011,
    // 中断 1 输出
    Int1 = 0b0100,
    // 中断 2 输出
    Int2 = 0b0101,
    // ADC 字时钟输出
    AdcWclk = 0b0110,
    // 数字麦克风时钟输出
    DigitalMicClock = 0b0111,
    // 次级数据输出（Secondary DOUT）
    SecondaryDout = 0b1000,
    // 次级位时钟（Secondary BCLK）
    SecondaryBclk = 0b1001,
    // 次级字时钟（Secondary WCLK）
    SecondaryWclk = 0b1010,
}

/// SCLK/MFP3 引脚功能配置枚举
/// 用于配置 SCLK 引脚的复用功能模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SclkMfp3Function {
    // 功能禁用
    Disabled = 0b00,
    // SPI 时钟或次级接口时钟（SPI SCLK 或次级数据/时钟输入）
    SpiClkOrSecondaryInterface = 0b01,
    // 通用输入引脚
    GeneralPurposeInput = 0b10,
}

/// 模拟输入快速充电时间枚举
/// 配置模拟输入耦合电容的快速充电时间
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AnalogInputChargeTime {
    // 默认充电时间（由寄存器默认值决定）
    Default = 0b00_0000,
    // 约 3.1ms 充电时间
    Ms3_1 = 0b11_0001,
    // 约 6.4ms 充电时间
    Ms6_4 = 0b11_0010,
    // 约 1.6ms 充电时间
    Ms1_6 = 0b11_0011,
}

/// 代表可配置为 GPIO 的多功能引脚 (MFP)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MfpPin {
    GpioMfp5,
    DoutMfp2,
    DinMfp1,
    MisoMfp4,
    SclkMfp3,
}

/// 中断和状态标志的综合结构体
/// 用于汇总编解码器的各类中断事件和状态标志，包含粘滞标志和实时状态
#[derive(Debug, Default, Clone, Copy)]
pub struct InterruptStatus {
    // --- 耳机状态 ---
    /// 已检测到的耳机类型
    pub headset_type: HeadsetType,

    // --- 粘滞标志 (读取后清除) ---
    /// 左耳机输出过流粘滞标志（读取寄存器后清除）
    pub sticky_hpl_over_current: bool,
    /// 右耳机输出过流粘滞标志（读取寄存器后清除）
    pub sticky_hpr_over_current: bool,
    /// 按钮按下事件粘滞标志（读取寄存器后清除）
    pub sticky_button_press_detected: bool,
    /// 耳机插入/拔出事件粘滞标志（读取寄存器后清除）
    pub sticky_headset_insertion_detected: bool,
    /// DAC动态范围压缩阈值超标粘滞标志（读取寄存器后清除）
    pub sticky_dac_drc_threshold_exceeded: bool,
    /// AGC噪声检测粘滞标志（读取寄存器后清除）
    pub sticky_agc_noise_detected: bool,
    /// 直流测量数据就绪粘滞标志（读取寄存器后清除）
    pub sticky_dc_measurement_data_available: bool,
    /// DAC数据溢出粘滞标志（读取寄存器后清除）
    pub sticky_dac_overflow: bool,
    /// ADC数据溢出粘滞标志（读取寄存器后清除）
    pub sticky_adc_overflow: bool,

    // --- 实时状态 ---
    /// 当前耳机插入状态（实时读取寄存器）
    pub live_headset_inserted: bool,
    /// 当前按钮按下状态（实时读取寄存器）
    pub live_button_pressed: bool,
    /// DAC动态范围压缩阈值当前超标状态（实时读取寄存器）
    pub live_dac_drc_threshold_exceeded: bool,
    /// AGC当前噪声检测状态（实时读取寄存器）
    pub live_agc_noise_detected: bool,
    /// 直流测量数据当前就绪状态（实时读取寄存器）
    pub live_dc_measurement_data_available: bool,
    /// DAC当前数据溢出状态（实时读取寄存器）
    pub live_dac_overflow: bool,
    /// ADC当前数据溢出状态（实时读取寄存器）
    pub live_adc_overflow: bool,
}

impl core::fmt::Display for InterruptStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "--- Codec Status ---")?;
        writeln!(f, "Headset Type: {:?}", self.headset_type)?;
        if self.live_headset_inserted {
            writeln!(f, "[Live] 耳机已插入")?;
        }
        if self.live_button_pressed {
            writeln!(f, "[Live] 按钮已按下")?;
        }
        if self.sticky_hpl_over_current || self.sticky_hpr_over_current {
            writeln!(
                f,
                "[!] 粘滞故障: 检测到过流 (左耳机: {}, 右耳机: {})",
                self.sticky_hpl_over_current, self.sticky_hpr_over_current
            )?;
        }
        if self.sticky_button_press_detected {
            writeln!(f, "[!] 粘滞事件: 按钮按下")?;
        }
        if self.sticky_headset_insertion_detected {
            writeln!(f, "[!] 粘滞事件: 耳机插入/拔出")?;
        }
        if self.sticky_dac_overflow || self.sticky_adc_overflow {
            writeln!(
                f,
                "[!] 粘滞故障: 数据溢出 (DAC: {}, ADC: {})",
                self.sticky_dac_overflow, self.sticky_adc_overflow
            )?;
        }
        Ok(())
    }
}
/// LDO 输出电压配置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LdoVoltage {
    V1_72 = 0b00,
    V1_67 = 0b01,
    V1_77 = 0b10,
}

/// 过流保护的去抖时间
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OverCurrentDebounce {
    Ms0 = 0b000,
    Ms8 = 0b001,
    Ms16 = 0b010,
    Ms32 = 0b011,
    Ms64 = 0b100,
    Ms128 = 0b101,
    Ms256 = 0b110,
    Ms512 = 0b111,
}

/// 过流保护的响应动作
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverCurrentAction {
    LimitCurrent,
    PowerDownDriver,
}

/// 耳机输出的共模电压设置
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HeadphoneCm {
    /// 与全局共模电压相同
    SameAsGlobal = 0b00,
    /// 1.25V
    V1_25 = 0b01,
    /// 1.5V
    V1_50 = 0b10,
    /// 1.65V (如果全局CM为0.9V) 或 1.5V (如果全局CM为0.75V)
    V1_65 = 0b11,
}

/// 输出电源和共模电压的综合配置
#[derive(Debug, Clone)]
pub struct OutputPowerConfig {
    /// 全局共模电压
    pub global_cm: CommonModeVoltage,
    /// 耳机输出的共模电压
    pub headphone_cm: HeadphoneCm,
    /// true 表示线路输出由 LDOIN 供电，并使用 1.65V 共模电压
    pub lineout_use_ldoin_and_1_65v_cm: bool,
    /// true 表示耳机驱动由 LDOIN 供电以获得更大摆幅
    pub headphone_use_ldoin: bool,
    /// true 表示 LDOIN 的电压范围为 1.8V-3.6V
    pub ldoin_is_high_range: bool,
}


/// ADC模拟音量控制标志位（Page 1, Register 0x3E）
#[derive(Debug, Clone, Copy)]
pub struct AdcAnalogVolumeFlags {
    pub left_pga_stable: bool,   // D7：左ADC PGA增益软步进完成
    pub reserved_d6: bool,      // D6：预留（未定义）
    pub reserved_d5: bool,      // D5：预留（未定义）
    pub reserved_d4: bool,      // D4：预留（未定义）
    pub right_pga_stable: bool, // D3：右ADC PGA增益软步进完成
    pub reserved_d2: bool,      // D2：预留（未定义）
    pub reserved_d1: bool,      // D1：预留（未定义）
    pub reserved_d0: bool,      // D0：预留（未定义）
}

#[derive(Debug, Clone, Copy)]
pub struct DacAnalogGainFlags {
    pub left_amp_stable: bool,   // D7：左DAC输出放大器（如HPL）增益软步进调整完成
    pub left_in1_stable: bool,  // D6：左声道IN1信号路由至HPL的衰减调整完成
    pub left_mix_stable: bool,  // D5：左声道混音器信号路由至HPL的衰减调整完成
    pub reserved_d4: bool,      // D4：预留（未定义功能）
    pub right_amp_stable: bool,  // D3：右DAC输出放大器（如HPR）增益软步进调整完成
    pub right_in1_stable: bool, // D2：右声道IN1信号路由至HPR的衰减调整完成
    pub right_mix_stable: bool, // D1：右声道混音器信号路由至HPR的衰减调整完成
    pub reserved_d0: bool,      // D0：预留（未定义功能）
}