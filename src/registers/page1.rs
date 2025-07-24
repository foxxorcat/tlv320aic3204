/// Page 1, Register 0x01: 电源配置寄存器
pub mod p1_r1 {
    pub const ADDRESS: u8 = 0x01;
    pub const PAGE: u8 = 1;
    /// 禁用弱AVDD生成位：置1时禁用内部AVDD弱上拉
    pub const DISABLE_WEAK_AVDD: u8 = 0x08;
    // 功能：配置芯片电源模式和内部LDO状态
}
pub use p1_r1 as P1_R1;

/// Page 1, Register 0x02: LDO控制寄存器
pub mod p1_r2 {
    pub const ADDRESS: u8 = 0x02;
    pub const PAGE: u8 = 1;
    /// 模拟模块使能位：置1时启用模拟电路电源
    pub const ANALOG_BLOCKS_ENABLE: u8 = 0x08;
    /// 模拟LDO使能位：置1时启用模拟LDO（AVDD）
    pub const ANALOG_LDO_ENABLE: u8 = 0x01;
    // 功能：控制内部LDO的启用状态和输出电压
}
pub use p1_r2 as P1_R2;

/// Page 1, Register 0x03: 左DAC PowerTune配置寄存器
pub mod p1_r3 {
    pub const ADDRESS: u8 = 0x03;
    pub const PAGE: u8 = 1;
    /// 左DAC PowerTune模式掩码（D2-D4）：
    /// 000=PTM_P4(高性能), 001=PTM_P3, 010=PTM_P2, 011=PTM_P1(低功耗)
    pub const LEFT_DAC_PTM_MASK: u8 = 0b0001_1100;
    // 功能：配置左DAC的PowerTune模式，平衡功耗与性能
}
pub use p1_r3 as P1_R3;

/// Page 1, Register 0x04: 右DAC PowerTune配置寄存器
pub mod p1_r4 {
    pub const ADDRESS: u8 = 0x04;
    pub const PAGE: u8 = 1;
    /// 右DAC PowerTune模式掩码（D2-D4）：配置方式同左DAC
    pub const RIGHT_DAC_PTM_MASK: u8 = 0b0001_1100;
}
pub use p1_r4 as P1_R4;

/// 5-8 0x01 0x05-0x08 Reserved Register

/// Page 1, Register 0x09: 输出驱动电源控制寄存器
pub mod p1_r9 {
    pub const ADDRESS: u8 = 0x09;
    pub const PAGE: u8 = 1;
    // 功能：控制耳机/线路输出驱动的电源状态和驱动模式
}
pub use p1_r9 as P1_R9;

/// P1_R10 寄存器定义：共模控制寄存器
/// 地址：0x0A，页：1
/// 用于配置芯片全局共模电压、耳机/线路输出共模电压及电源选择
pub mod p1_r10 {
    // 寄存器地址与页号定义
    pub const ADDRESS: u8 = 0x0A;
    pub const PAGE: u8 = 1;

    // 寄存器位掩码定义（对应文档第1-157至1-158页）
    /// 全局共模电压配置掩码（D6位）
    /// 0: 0.9V（适用于AVDD≥1.8V）；1: 0.75V（适用于AVDD<1.8V）
    pub const FULL_CHIP_CM_MASK: u8 = 1 << 6;

    /// 耳机输出共模电压配置掩码（D5-D4位）
    /// 00: 与全局共模电压一致；01: 1.25V；10: 1.5V；11: 1.65V（AVDD≥1.8V时）/1.5V（AVDD<1.8V时）
    pub const HP_OUT_CM_MASK: u8 = 0b0011_0000;

    /// 线路输出共模电压选择掩码（D3位）
    /// 0: 与全局共模电压一致；1: 1.65V（由LDOIN供电）
    pub const LINE_OUT_CM_LDOIN: u8 = 1 << 3;

    /// 耳机驱动电源选择掩码（D1位）
    /// 0: 由AVDD供电；1: 由LDOIN供电（支持更高输出电压）
    pub const HP_POWER_SOURCE_LDOIN: u8 = 1 << 1;

    /// LDOIN高电压范围使能掩码（D0位）
    /// 0: LDOIN输入范围1.5-1.95V；1: LDOIN输入范围1.8-3.6V（配合D1=1时使用）
    pub const LDOIN_HIGH_RANGE: u8 = 1 << 0;
}
pub use p1_r10 as P1_R10;

// P1_R11 寄存器定义：过流保护配置寄存器
// 地址：0x0B，页：1
// 用于配置耳机驱动过流保护阈值及使能状态
pub mod p1_r11 {
    pub const ADDRESS: u8 = 0x0B;
    pub const PAGE: u8 = 1;
}
pub use p1_r11 as P1_R11;

/// Page 1, Register 0x0C: 左耳机输出路由选择寄存器
pub mod p1_r12 {
    pub const ADDRESS: u8 = 0x0C;
    pub const PAGE: u8 = 1;
    // 功能：配置左耳机输出的信号源（DAC/ADC/模拟旁路）
}
pub use p1_r12 as P1_R12;

/// Page 1, Register 0x0D: 右耳机输出路由选择寄存器
pub mod p1_r13 {
    pub const ADDRESS: u8 = 0x0D;
    pub const PAGE: u8 = 1;
    // 功能：配置右耳机输出的信号源（DAC/ADC/模拟旁路）
}
pub use p1_r13 as P1_R13;

/// Page 1, Register 0x0E: 左线路输出路由选择寄存器
pub mod p1_r14 {
    pub const ADDRESS: u8 = 0x0E;
    pub const PAGE: u8 = 1;
    // 功能：配置左线路输出的信号源（DAC/ADC/模拟旁路）
}
pub use p1_r14 as P1_R14;

/// Page 1, Register 0x0F: 右线路输出路由选择寄存器
pub mod p1_r15 {
    pub const ADDRESS: u8 = 0x0F;
    pub const PAGE: u8 = 1;
    // 功能：配置右线路输出的信号源（DAC/ADC/模拟旁路）
}
pub use p1_r15 as P1_R15;

/// Page 1, Register 0x10: 左耳机驱动增益设置寄存器
pub mod p1_r16 {
    pub const ADDRESS: u8 = 0x10;
    pub const PAGE: u8 = 1;
    /// 左耳机驱动增益掩码（D0-D5）：-6dB至+29dB（1dB步进）
    pub const HPL_GAIN_MASK: u8 = 0x3F;
    /// 左耳机静音控制位（D6）：置1时左耳机输出静音
    pub const HPL_CONTROL_MASK: u8 = 0x7F; // 包含D6和D0-D5
}
pub use p1_r16 as P1_R16;

/// Page 1, Register 0x11: 右耳机驱动增益设置寄存器
pub mod p1_r17 {
    pub const ADDRESS: u8 = 0x11;
    pub const PAGE: u8 = 1;
    /// 右耳机驱动增益掩码（D0-D5）：-6dB至+29dB（1dB步进）
    pub const HPR_GAIN_MASK: u8 = 0x3F;
    /// 右耳机静音控制位（D6）：置1时右耳机输出静音
    pub const HPR_CONTROL_MASK: u8 = 0x7F; // 包含D6和D0-D5
}
pub use p1_r17 as P1_R17;

/// Page 1, Register 0x12: 左线路输出驱动增益控制寄存器
/// 功能：配置左线路输出（Line Out Left）的增益，范围-6dB至+24dB（1dB步进）
pub mod p1_r18 {
    pub const ADDRESS: u8 = 0x12;
    pub const PAGE: u8 = 1;
    /// 左线路输出增益掩码（D0-D5）：控制输出驱动强度
    pub const LOL_GAIN_MASK: u8 = 0x3F;
}
pub use p1_r18 as P1_R18;

/// Page 1, Register 0x13: 右线路输出驱动增益控制寄存器
/// 功能：配置右线路输出（Line Out Right）的增益，范围-6dB至+24dB（1dB步进）
pub mod p1_r19 {
    pub const ADDRESS: u8 = 0x13;
    pub const PAGE: u8 = 1;
    /// 右线路输出增益掩码（D0-D5）：控制输出驱动强度
    pub const LOR_GAIN_MASK: u8 = 0x3F;
}
pub use p1_r19 as P1_R19;

/// Page 1, Register 0x14: 耳机驱动启动控制寄存器
/// 功能：配置耳机驱动的启动时序、防冲击（Anti-thump）参数
pub mod p1_r20 {
    pub const ADDRESS: u8 = 0x14;
    pub const PAGE: u8 = 1;
}
pub use p1_r20 as P1_R20;

/// 21 0x01 0x15 Reserved Register

/// Page 1, Register 0x16: 音频处理块路由控制寄存器1
/// 功能：配置音频处理路径中PRB_P1-PRB_P4模块的连接关系和数据流向
pub mod p1_r22 {
    pub const ADDRESS: u8 = 0x16;
    pub const PAGE: u8 = 1;
}
pub use p1_r22 as P1_R22;

/// Page 1, Register 0x17: 音频处理块路由控制寄存器2
/// 功能：配置音频处理路径中PRB_P5-PRB_P8模块的连接关系和数据流向
pub mod p1_r23 {
    pub const ADDRESS: u8 = 0x17;
    pub const PAGE: u8 = 1;
}
pub use p1_r23 as P1_R23;

/// Page 1, Register 0x18: 音频处理块路由控制寄存器3
/// 功能：配置音频处理路径中PRB_P9-PRB_P12模块的连接关系和数据流向
pub mod p1_r24 {
    pub const ADDRESS: u8 = 0x18;
    pub const PAGE: u8 = 1;
}
pub use p1_r24 as P1_R24;

/// Page 1, Register 0x19: 音频处理块路由控制寄存器4
/// 功能：配置音频处理路径中PRB_P13-PRB_P16模块的连接关系和数据流向
pub mod p1_r25 {
    pub const ADDRESS: u8 = 0x19;
    pub const PAGE: u8 = 1;
}
pub use p1_r25 as P1_R25;

/// 26-50 0x01 0x1A-0x32 Reserved Register

/// Page 1, Register 0x33: 麦克风偏压配置寄存器
pub mod p1_r51 {
    pub const ADDRESS: u8 = 0x33;
    pub const PAGE: u8 = 1;
    /// 麦克风偏压上电使能位（D6）：置1时启用MICBIAS输出
    pub const POWER_UP: u8 = 1 << 6;
    /// 麦克风偏压电压选择掩码（D4-D5）：
    /// 00=1.25V, 01=1.7V, 10=2.5V, 11=AVDD/LDOIN
    pub const VOLTAGE_MASK: u8 = 0b0011_0000;
    /// 麦克风偏压电源选择位（D3）：
    /// 1=LDOIN供电, 0=AVDD供电（仅当D5-D4=11时有效）
    pub const SUPPLY_LDOIN: u8 = 1 << 3;
}
pub use p1_r51 as P1_R51;

/// Page 1, Register 0x34: 左MICPGA正端输入路由寄存器
pub mod p1_r52 {
    pub const ADDRESS: u8 = 0x34;
    pub const PAGE: u8 = 1;
    // 功能：配置左麦克风PGA正端的输入源（IN1_L/IN2_L/IN3_L）
}
pub use p1_r52 as P1_R52;

/// 53 0x01 0x35 Reserved Register

/// Page 1, Register 0x36: 左MICPGA负端输入路由寄存器
pub mod p1_r54 {
    pub const ADDRESS: u8 = 0x36;
    pub const PAGE: u8 = 1;
    // 功能：配置左麦克风PGA负端的输入源（差分模式时使用）
}
pub use p1_r54 as P1_R54;

/// Page 1, Register 0x37: 右MICPGA正端输入路由寄存器
pub mod p1_r55 {
    pub const ADDRESS: u8 = 0x37;
    pub const PAGE: u8 = 1;
    // 功能：配置右麦克风PGA正端的输入源（IN1_R/IN2_R/IN3_R）
}
pub use p1_r55 as P1_R55;

/// 56 0x01 0x38 Reserved Register

/// Page 1, Register 0x39: 右MICPGA负端输入路由寄存器
pub mod p1_r57 {
    pub const ADDRESS: u8 = 0x39;
    pub const PAGE: u8 = 1;
    // 功能：配置右麦克风PGA负端的输入源（差分模式时使用）
}
pub use p1_r57 as P1_R57;

/// Page 1, Register 0x3A: 浮动输入配置寄存器
pub mod p1_r58 {
    pub const ADDRESS: u8 = 0x3A;
    pub const PAGE: u8 = 1;
    // 功能：配置未使用的模拟输入引脚的处理方式（接地/浮空）
}
pub use p1_r58 as P1_R58;

/// Page 1, Register 0x3B: 左MICPGA音量控制寄存器
pub mod p1_r59 {
    pub const ADDRESS: u8 = 0x3B;
    pub const PAGE: u8 = 1;
    /// 左MICPGA静音控制位（D7）：置1时左MICPGA静音
    pub const MUTE_ENABLE: u8 = 0x80;
    /// 左MICPGA增益掩码（D0-D6）：0dB至+47.5dB（0.5dB步进）
    pub const GAIN_MASK: u8 = 0x7F;

    pub const LMIC_CONTROL_MASK: u8 = 0xFF;
    // 功能：控制左麦克风前置放大器的增益和静音状态
}
pub use p1_r59 as P1_R59;

/// Page 1, Register 0x3C: 右MICPGA音量控制寄存器
pub mod p1_r60 {
    pub const ADDRESS: u8 = 0x3C;
    pub const PAGE: u8 = 1;
    /// 右MICPGA静音控制位（D7）：置1时右MICPGA静音
    pub const MUTE_ENABLE: u8 = 0x80;
    /// 右MICPGA增益掩码（D0-D6）：0dB至+47.5dB（0.5dB步进）
    pub const GAIN_MASK: u8 = 0x7F;

    pub const RMIC_CONTROL_MASK: u8 = 0xFF;
    // 功能：控制右麦克风前置放大器的增益和静音状态
}
pub use p1_r60 as P1_R60;

/// Page 1, Register 0x3D: ADC PowerTune配置寄存器
pub mod p1_r61 {
    pub const ADDRESS: u8 = 0x3D;
    pub const PAGE: u8 = 1;
    // 功能：配置ADC的PowerTune模式（PTM_R1-PTM_R4）
}
pub use p1_r61 as P1_R61;

/// Page 1, Register 0x3E: ADC Analog Volume Control Flag Register（ADC模拟音量控制标志寄存器）
/// 功能：包含ADC通道模拟增益（如PGA）的软步进调整状态标志，指示增益是否稳定
pub mod p1_r62 {
    pub const ADDRESS: u8 = 0x3E;
    pub const PAGE: u8 = 1;
}
pub use p1_r62 as P1_R62;

/// Page 1, Register 0x3F: DAC Analog Gain Control Flag Register（DAC模拟增益控制标志寄存器）
/// 功能：包含DAC通道模拟增益（如输出放大器）的软步进调整状态标志，指示增益是否稳定
pub mod p1_r63 {
    pub const ADDRESS: u8 = 0x3F;
    pub const PAGE: u8 = 1;
}
pub use p1_r63 as P1_R63;

/// 64-70 0x01 0x40-0x46 Reserved Register

// P1_R71 寄存器定义：模拟输入快速充电配置寄存器
// 地址：0x47，页：1
pub mod p1_r71 {
    pub const ADDRESS: u8 = 0x47;
    pub const PAGE: u8 = 1;
}
pub use p1_r71 as P1_R71;

/// 72-122 0x01 0x48-0x7A Reserved Register

/// Page 1, Register 0x7B: 参考电压上电配置寄存器
pub mod p1_r123 {
    pub const ADDRESS: u8 = 0x7B;
    pub const PAGE: u8 = 1;
    /// 参考电压上电模式掩码（D0-D2）：
    /// 000=快速上电, 001=慢速上电（减少冲击噪声）
    pub const REF_POWERUP_MASK: u8 = 0x07;
}
pub use p1_r123 as P1_R123;

// 124-127 0x01 0x7C-0x7F Reserved Register

