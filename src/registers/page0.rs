/// Page 0, Register 0x00: 页面选择寄存器
pub mod p0_r0 {
    pub const ADDRESS: u8 = 0x00;
    pub const PAGE: u8 = 0;
    // 功能：选择当前操作的寄存器页面（0-127）
}
pub use p0_r0 as P0_R0;

/// Page 0, Register 0x01: 软件复位寄存器
pub mod p0_r1 {
    pub const ADDRESS: u8 = 0x01;
    pub const PAGE: u8 = 0;
    /// 软件复位位：置1时触发芯片复位，复位后自动清0
    pub const RESET: u8 = 0x01;
}
pub use p0_r1 as P0_R1;

/// 2-3 0x00 0x02-0x03 Reserved Register

/// Page 0, Register 0x04: 时钟选择寄存器1
pub mod p0_r4 {
    pub const ADDRESS: u8 = 0x04;
    pub const PAGE: u8 = 0;
    /// PLL输入时钟源选择掩码（D2-D3）：
    /// 00=BCLK, 01=MCLK, 10=DIN/MFP1, 11=GPIO/MFP5
    pub const PLL_CLKIN_SEL_MASK: u8 = 0b0000_1100;
    /// Codec输入时钟源选择掩码（D0-D1）：
    /// 00=BCLK, 01=MCLK, 10=PLL输出, 11=GPIO/MFP5
    pub const CODEC_CLKIN_SEL_MASK: u8 = 0b0000_0011;
}
pub use p0_r4 as P0_R4;

/// Page 0, Register 0x05: PLL控制寄存器
pub mod p0_r5 {
    pub const ADDRESS: u8 = 0x05;
    pub const PAGE: u8 = 0;
    /// PLL上电使能位：置1时启动PLL，稳定时间约10ms
    pub const PLL_POWER_UP: u8 = 0b1000_0000;
    /// PLL倍频因子P选择掩码（D4-D6）：P=1-8
    pub const P_MASK: u8 = 0b0111_0000;
    /// PLL参考分频因子R选择掩码（D0-D3）：R=1-4
    pub const R_MASK: u8 = 0b0000_1111;
}
pub use p0_r5 as P0_R5;

/// Page 0, Register 0x06: PLL倍频因子J寄存器（低6位）
pub mod p0_r6 {
    pub const ADDRESS: u8 = 0x06;
    pub const PAGE: u8 = 0;
    // 功能：设置PLL的J因子（J=1-63），配合P、R、D因子计算PLL输出频率
}
pub use p0_r6 as P0_R6;

/// Page 0, Register 0x07: PLL分频因子D寄存器（高6位）
pub mod p0_r7 {
    pub const ADDRESS: u8 = 0x07;
    pub const PAGE: u8 = 0;
    // 功能：设置PLL的D因子高6位（D=0-999），与P0_R8组成12位D因子
}
pub use p0_r7 as P0_R7;

/// Page 0, Register 0x08: PLL分频因子D寄存器（低8位）
pub mod p0_r8 {
    pub const ADDRESS: u8 = 0x08;
    pub const PAGE: u8 = 0;
    // 功能：设置PLL的D因子低8位，与P0_R7组成完整的12位D因子
}
pub use p0_r8 as P0_R8;

/// 9-10 0x00 0x09-0x0A Reserved Register

/// Page 0, Register 0x0B: DAC时钟分频寄存器（NDAC）
pub mod p0_r11 {
    pub const ADDRESS: u8 = 0x0B;
    pub const PAGE: u8 = 0;
    /// NDAC上电使能位：置1时启用NDAC分频器
    pub const POWER_UP: u8 = 0x80;
    // 功能：设置NDAC分频因子（1-128），用于生成DAC主时钟
}
pub use p0_r11 as P0_R11;

/// Page 0, Register 0x0C: DAC时钟分频寄存器（MDAC）
pub mod p0_r12 {
    pub const ADDRESS: u8 = 0x0C;
    pub const PAGE: u8 = 0;
    /// MDAC上电使能位：置1时启用MDAC分频器
    pub const POWER_UP: u8 = 0x80;
    // 功能：设置MDAC分频因子（1-128），配合NDAC生成DAC调制时钟
}
pub use p0_r12 as P0_R12;

/// Page 0, Register 0x0D: DAC过采样率寄存器（高2位）
pub mod p0_r13 {
    pub const ADDRESS: u8 = 0x0D;
    pub const PAGE: u8 = 0;
    // 功能：设置DAC过采样率（DOSR）高2位，与P0_R14组成完整的DOSR（1-1024）
}
pub use p0_r13 as P0_R13;

/// Page 0, Register 0x0E: DAC过采样率寄存器（低8位）
pub mod p0_r14 {
    pub const ADDRESS: u8 = 0x0E;
    pub const PAGE: u8 = 0;
    // 功能：设置DAC过采样率（DOSR）低8位，与P0_R13组成完整的DOSR
}
pub use p0_r14 as P0_R14;

/// 15-17 0x00 0x0F-0x11 Reserved Register

/// Page 0, Register 0x12: ADC时钟分频寄存器（NADC）
pub mod p0_r18 {
    pub const ADDRESS: u8 = 0x12;
    pub const PAGE: u8 = 0;
    /// NADC上电使能位：置1时启用NADC分频器
    pub const POWER_UP: u8 = 0x80;
    // 功能：设置NADC分频因子（1-128），用于生成ADC主时钟
}
pub use p0_r18 as P0_R18;

/// Page 0, Register 0x13: ADC时钟分频寄存器（MADC）
pub mod p0_r19 {
    pub const ADDRESS: u8 = 0x13;
    pub const PAGE: u8 = 0;
    /// MADC上电使能位：置1时启用MADC分频器
    pub const POWER_UP: u8 = 0x80;
    // 功能：设置MADC分频因子（1-128），配合NADC生成ADC调制时钟
}
pub use p0_r19 as P0_R19;

/// Page 0, Register 0x14: ADC过采样率寄存器（AOSR）
pub mod p0_r20 {
    pub const ADDRESS: u8 = 0x14;
    pub const PAGE: u8 = 0;
    // 功能：设置ADC过采样率（AOSR=1-256），影响ADC信噪比和动态范围
}
pub use p0_r20 as P0_R20;

/// 21-24 0x00 0x15-0x18 Reserved Register

/// Page 0, Register 0x1B: 音频接口配置寄存器
pub mod p0_r27 {
    pub const ADDRESS: u8 = 0x1B;
    pub const PAGE: u8 = 0;
    /// 接口模式选择掩码（D6-D7）：
    /// 00=I2S, 01=DSP, 10=右对齐, 11=左对齐
    pub const INTERFACE_MASK: u8 = 0b1100_0000;
    /// 数据字长选择掩码（D4-D5）：
    /// 00=16位, 01=20位, 10=24位, 11=32位
    pub const WORD_LENGTH_MASK: u8 = 0b0011_0000;
    /// BCLK方向控制位（D3）：1=输出模式，0=输入模式
    pub const BCLK_DIR_OUTPUT: u8 = 0b0000_1000;
    /// WCLK方向控制位（D2）：1=输出模式，0=输入模式
    pub const WCLK_DIR_OUTPUT: u8 = 0b0000_0100;
}
pub use p0_r27 as P0_R27;

/// 39-41 0x00 0x27-0x29 Reserved Register

// P0_R42 寄存器定义：粘连标志寄存器1
// 地址：0x2A，页：0
// 用于记录ADC/DAC数据溢出等粘连性中断事件标志
pub mod p0_r42 {
    pub const ADDRESS: u8 = 0x2A;
    pub const PAGE: u8 = 0;
}
pub use p0_r42 as P0_R42;

// P0_R43 寄存器定义：中断标志寄存器1
// 地址：0x2B，页：0
// 用于记录ADC/DAC数据溢出等实时中断事件标志
pub mod p0_r43 {
    pub const ADDRESS: u8 = 0x2B;
    pub const PAGE: u8 = 0;
}
pub use p0_r43 as P0_R43;

/// Page 0, Register 0x2C: 中断标志寄存器1（粘滞标志）
pub mod p0_r44 {
    pub const ADDRESS: u8 = 0x2C;
    pub const PAGE: u8 = 0;
    // 功能：存储ADC/DAC的中断标志，需手动清零（粘滞标志）
}
pub use p0_r44 as P0_R44;

// P0_R45 寄存器定义：粘连标志寄存器3
// 地址：0x2D，页：0
// 用于记录AGC噪声检测、DC测量完成等粘连性事件标志
pub mod p0_r45 {
    pub const ADDRESS: u8 = 0x2D;
    pub const PAGE: u8 = 0;
}
pub use p0_r45 as P0_R45;

/// Page 0, Register 0x2E: 中断标志寄存器2（非粘滞标志）
pub mod p0_r46 {
    pub const ADDRESS: u8 = 0x2E;
    pub const PAGE: u8 = 0;
    // 功能：存储ADC/DAC的中断标志，自动清零（非粘滞标志）
}
pub use p0_r46 as P0_R46;

// P0_R47 寄存器定义：中断标志寄存器3
// 地址：0x2F，页：0
// 用于记录AGC噪声检测、DC测量完成等实时事件标志
pub mod p0_r47 {
    pub const ADDRESS: u8 = 0x2F;
    pub const PAGE: u8 = 0;
}
pub use p0_r47 as P0_R47;

/// Page 0, Register 0x30: INT1中断控制寄存器
/// 功能：配置INT1中断的触发条件和使能状态
pub mod p0_r48 {
    pub const ADDRESS: u8 = 0x30;
    pub const PAGE: u8 = 0;
}
pub use p0_r48 as P0_R48;

/// Page 0, Register 0x31: INT2中断控制寄存器
/// 功能：配置INT2中断的触发条件和使能状态
pub mod p0_r49 {
    pub const ADDRESS: u8 = 0x31;
    pub const PAGE: u8 = 0;
}
pub use p0_r49 as P0_R49;

/// 50-51 0x00 0x32-0x33 Reserved Register

/// Page 0, Register 0x34: GPIO/MFP5控制寄存器
pub mod p0_r52 {
    pub const ADDRESS: u8 = 0x34;
    pub const PAGE: u8 = 0;
    /// GPIO功能控制掩码（D2-D5）：配置GPIO的输入/输出模式及功能复用
    pub const GPIO_CTRL_MASK: u8 = 0b0011_1100;
    /// GPIO输入状态位（D1）：只读，反映GPIO引脚的当前电平
    pub const GPIO_INPUT_STATE_MASK: u8 = 0b0000_0010;
    /// GPIO输出控制位（D0）：置1时GPIO输出高电平，置0时输出低电平
    pub const GPIO_OUTPUT_CTRL_MASK: u8 = 0b0000_0001;
}
pub use p0_r52 as P0_R52;

// P0_R53 寄存器定义：DOUT/MFP2 功能控制寄存器
// 地址：0x35，页：0
pub mod p0_r53 {
    pub const ADDRESS: u8 = 0x35;
    pub const PAGE: u8 = 0;
}
pub use p0_r53 as P0_R53;

// P0_R54 寄存器定义：DIN/MFP1 功能控制寄存器
// 地址：0x36，页：0
pub mod p0_r54 {
    pub const ADDRESS: u8 = 0x36;
    pub const PAGE: u8 = 0;
}
pub use p0_r54 as P0_R54;

// P0_R55 寄存器定义：MISO/MFP4 功能控制寄存器
// 地址：0x37，页：0
pub mod p0_r55 {
    pub const ADDRESS: u8 = 0x37;
    pub const PAGE: u8 = 0;
}
pub use p0_r55 as P0_R55;

// P0_R56 寄存器定义：SCLK/MFP3 功能控制寄存器
// 地址：0x38，页：0
pub mod p0_r56 {
    pub const ADDRESS: u8 = 0x38;
    pub const PAGE: u8 = 0;
}
pub use p0_r56 as P0_R56;

/// 57-59 0x00 0x39-0x3B Reserved Registers

/// Page 0, Register 0x3C: DAC信号处理块控制寄存器
pub mod p0_r60 {
    pub const ADDRESS: u8 = 0x3C;
    pub const PAGE: u8 = 0;
    // 功能：选择DAC的信号处理块（PRB_P1-PRB_P25），影响滤波和音效
}
pub use p0_r60 as P0_R60;

/// Page 0, Register 0x3D: ADC信号处理块控制寄存器
pub mod p0_r61 {
    pub const ADDRESS: u8 = 0x3D;
    pub const PAGE: u8 = 0;
    // 功能：选择ADC的信号处理块（PRB_R1-PRB_R18），影响滤波和增益
}
pub use p0_r61 as P0_R61;

/// 62 0x00 0x3E Reserved Register

/// Page 0, Register 0x3F: DAC通道配置寄存器1
pub mod p0_r63 {
    pub const ADDRESS: u8 = 0x3F;
    pub const PAGE: u8 = 0;
    /// 左DAC上电使能位（D7）：置1时启用左DAC通道
    pub const LEFT_DAC_POWER_UP: u8 = 0x80;
    /// 右DAC上电使能位（D6）：置1时启用右DAC通道
    pub const RIGHT_DAC_POWER_UP: u8 = 0x40;
    /// 左DAC数据路径选择掩码（D4-D5）：配置左DAC输入源
    pub const LEFT_DAC_DATA_PATH_MASK: u8 = 0b0011_0000;
    /// 右DAC数据路径选择掩码（D2-D3）：配置右DAC输入源
    pub const RIGHT_DAC_DATA_PATH_MASK: u8 = 0b0000_1100;
}
pub use p0_r63 as P0_R63;

/// Page 0, Register 0x40: DAC通道配置寄存器2
pub mod p0_r64 {
    pub const ADDRESS: u8 = 0x40;
    pub const PAGE: u8 = 0;
    /// 左DAC静音控制位（D3）：置1时左DAC输出静音
    pub const LEFT_DAC_MUTE: u8 = 0x08;
    /// 右DAC静音控制位（D2）：置1时右DAC输出静音
    pub const RIGHT_DAC_MUTE: u8 = 0x04;
    /// 左DAC/右DAC静音控制位
    pub const DAC_MUTE_MASK: u8 = 0xC;
    // 功能：控制DAC通道的静音状态和输出混合模式
}
pub use p0_r64 as P0_R64;

/// Page 0, Register 0x41: 左DAC通道数字音量控制寄存器
pub mod p0_r65 {
    pub const ADDRESS: u8 = 0x41;
    pub const PAGE: u8 = 0;
    // 功能：设置左DAC通道音量（-63.5dB至+24dB，0.5dB步进）
}
pub use p0_r65 as P0_R65;

/// Page 0, Register 0x42: 右DAC通道数字音量控制寄存器
pub mod p0_r66 {
    pub const ADDRESS: u8 = 0x42;
    pub const PAGE: u8 = 0;
    // 功能：设置右DAC通道音量（-63.5dB至+24dB，0.5dB步进）
}
pub use p0_r66 as P0_R66;

/// Page 0, Register 0x43: 耳机检测配置寄存器
pub mod p0_r67 {
    pub const ADDRESS: u8 = 0x43;
    pub const PAGE: u8 = 0;
    // 功能：配置耳机插入检测的灵敏度和中断触发条件
}
pub use p0_r67 as P0_R67;

/// Page 0, Register 0x44: 动态范围压缩控制寄存器1
pub mod p0_r68 {
    pub const ADDRESS: u8 = 0x44;
    pub const PAGE: u8 = 0;
    // 功能：设置DRC（动态范围压缩）的阈值、滞后和启动时间
}
pub use p0_r68 as P0_R68;

/// Page 0, Register 0x45: 动态范围压缩控制寄存器2
pub mod p0_r69 {
    pub const ADDRESS: u8 = 0x45;
    pub const PAGE: u8 = 0;
    // 功能：设置DRC的衰减时间和释放时间常数
}
pub use p0_r69 as P0_R69;

/// Page 0, Register 0x46: 动态范围压缩控制寄存器3
pub mod p0_r70 {
    pub const ADDRESS: u8 = 0x46;
    pub const PAGE: u8 = 0;
    // 功能：设置DRC的增益变化速率和深度参数
}
pub use p0_r70 as P0_R70;

/// Page 0, Register 0x47: 蜂鸣器控制寄存器1（左通道音量）
pub mod p0_r71 {
    pub const ADDRESS: u8 = 0x47;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器左通道音量（0dB至-63dB，1dB步进）
}
pub use p0_r71 as P0_R71;

/// Page 0, Register 0x48: 蜂鸣器控制寄存器2（右通道音量）
pub mod p0_r72 {
    pub const ADDRESS: u8 = 0x48;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器右通道音量（0dB至-63dB，1dB步进）
}
pub use p0_r72 as P0_R72;

/// Page 0, Register 0x49: 蜂鸣器控制寄存器3（波形长度低8位）
pub mod p0_r73 {
    pub const ADDRESS: u8 = 0x49;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器波形长度低8位（配合P0_R74组成24位长度）
}
pub use p0_r73 as P0_R73;

/// Page 0, Register 0x4A: 蜂鸣器控制寄存器4（波形长度中8位）
pub mod p0_r74 {
    pub const ADDRESS: u8 = 0x4A;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器波形长度中8位
}
pub use p0_r74 as P0_R74;

/// Page 0, Register 0x4B: 蜂鸣器控制寄存器5（波形长度高8位）
pub mod p0_r75 {
    pub const ADDRESS: u8 = 0x4B;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器波形长度高8位，完整长度为24位
}
pub use p0_r75 as P0_R75;

/// Page 0, Register 0x4C: 蜂鸣器控制寄存器6（正弦波系数高6位）
pub mod p0_r76 {
    pub const ADDRESS: u8 = 0x4C;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器正弦波系数高6位，配合P0_R77组成16位系数
}
pub use p0_r76 as P0_R76;

/// Page 0, Register 0x4D: 蜂鸣器控制寄存器7（正弦波系数低10位）
pub mod p0_r77 {
    pub const ADDRESS: u8 = 0x4D;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器正弦波系数低10位，组成完整的16位系数
}
pub use p0_r77 as P0_R77;

/// Page 0, Register 0x4E: 蜂鸣器控制寄存器8（余弦波系数高6位）
pub mod p0_r78 {
    pub const ADDRESS: u8 = 0x4E;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器余弦波系数高6位，配合P0_R79组成16位系数
}
pub use p0_r78 as P0_R78;

/// Page 0, Register 0x4F: 蜂鸣器控制寄存器9（余弦波系数低10位）
pub mod p0_r79 {
    pub const ADDRESS: u8 = 0x4F;
    pub const PAGE: u8 = 0;
    // 功能：设置蜂鸣器余弦波系数低10位，组成完整的16位系数
}
pub use p0_r79 as P0_R79;

// 80 0x00 0x50 Reserved Register

/// Page 0, Register 0x51: ADC通道配置寄存器
pub mod p0_r81 {
    pub const ADDRESS: u8 = 0x51;
    pub const PAGE: u8 = 0;
    /// 左ADC上电使能位（D7）：置1时启用左ADC通道
    pub const LEFT_ADC_POWER_UP: u8 = 0x80;
    /// 右ADC上电使能位（D6）：置1时启用右ADC通道
    pub const RIGHT_ADC_POWER_UP: u8 = 0x40;
    // 功能：配置ADC通道的电源状态和输入路径
}
pub use p0_r81 as P0_R81;

/// Page 0, Register 0x52: ADC精细增益调整与静音控制寄存器
/// 功能：控制ADC左右通道的静音状态及精细增益调整（-0.4dB至0dB，0.1dB步进）
pub mod p0_r82 {
    pub const ADDRESS: u8 = 0x52;
    pub const PAGE: u8 = 0;
    /// 左ADC通道静音控制位：置1时左ADC输入静音（D7）
    pub const LEFT_MUTE: u8 = 1 << 7;
    /// 左ADC精细增益调整掩码：D4-D7控制增益（0000=0dB, 1111=-0.4dB）
    pub const LEFT_FINE_GAIN_MASK: u8 = 0b0111_0000;
    /// 右ADC通道静音控制位：置1时右ADC输入静音（D3）
    pub const RIGHT_MUTE: u8 = 1 << 3;
    /// 右ADC精细增益调整掩码：D0-D2控制增益（000=0dB, 111=-0.4dB）
    pub const RIGHT_FINE_GAIN_MASK: u8 = 0b0000_0111;
}
pub use p0_r82 as P0_R82;

/// Page 0, Register 0x53: 左ADC通道音量控制寄存器
pub mod p0_r83 {
    pub const ADDRESS: u8 = 0x53;
    pub const PAGE: u8 = 0;
    // 功能：设置左ADC通道音量（-12dB至+20dB，0.5dB步进）
}
pub use p0_r83 as P0_R83;

/// Page 0, Register 0x54: 右ADC通道音量控制寄存器
pub mod p0_r84 {
    pub const ADDRESS: u8 = 0x54;
    pub const PAGE: u8 = 0;
    // 功能：设置右ADC通道音量（-12dB至+20dB，0.5dB步进）
}
pub use p0_r84 as P0_R84;

/// Page 0, Register 0x55: ADC通道相位调整寄存器
/// 功能：配置ADC左右通道的相位补偿延迟（用于多麦克风噪声消除等场景）
pub mod p0_r85 {
    pub const ADDRESS: u8 = 0x55;
    pub const PAGE: u8 = 0;
}
pub use p0_r85 as P0_R85;

/// Page 0, Register 0x56: 左ADC通道AGC控制寄存器1
/// 功能：设置左ADC自动增益控制的目标水平、启动模式及噪声阈值
pub mod p0_r86 {
    pub const ADDRESS: u8 = 0x56;
    pub const PAGE: u8 = 0;
}
pub use p0_r86 as P0_R86;

/// Page 0, Register 0x57: 左ADC通道AGC控制寄存器2
/// 功能：设置左ADC自动增益控制的滞后值、噪声阈值及最大PGA增益
pub mod p0_r87 {
    pub const ADDRESS: u8 = 0x57;
    pub const PAGE: u8 = 0;
}
pub use p0_r87 as P0_R87;

/// Page 0, Register 0x58: 左ADC通道AGC控制寄存器3
/// 功能：配置左ADC AGC的最大允许PGA增益（0dB至58dB，0.5dB步进）
pub mod p0_r88 {
    pub const ADDRESS: u8 = 0x58;
    pub const PAGE: u8 = 0;
}
pub use p0_r88 as P0_R88;

/// Page 0, Register 0x59: 左ADC通道AGC控制寄存器4
/// 功能：设置左ADC AGC的攻击时间常数（决定增益减小速度）
pub mod p0_r89 {
    pub const ADDRESS: u8 = 0x59;
    pub const PAGE: u8 = 0;
}
pub use p0_r89 as P0_R89;

/// Page 0, Register 0x5A: 左ADC通道AGC控制寄存器5
/// 功能：设置左ADC AGC的衰减时间常数（决定增益增加速度）
pub mod p0_r90 {
    pub const ADDRESS: u8 = 0x5A;
    pub const PAGE: u8 = 0;
}
pub use p0_r90 as P0_R90;

/// Page 0, Register 0x5B: 左ADC通道AGC控制寄存器6
/// 功能：配置左ADC AGC的噪声检测滞后值及去抖时间
pub mod p0_r91 {
    pub const ADDRESS: u8 = 0x5B;
    pub const PAGE: u8 = 0;
}
pub use p0_r91 as P0_R91;

/// Page 0, Register 0x5C: 左ADC通道AGC控制寄存器7
/// 功能：设置左ADC AGC的信号检测去抖时间（避免噪声触发增益波动）
pub mod p0_r92 {
    pub const ADDRESS: u8 = 0x5C;
    pub const PAGE: u8 = 0;
}
pub use p0_r92 as P0_R92;

/// Page 0, Register 0x5D: 左ADC通道AGC控制寄存器8
/// 功能：设置左ADC AGC实际应用增益读取寄存器（只读）
pub mod p0_r93 {
    pub const ADDRESS: u8 = 0x5D;
    pub const PAGE: u8 = 0;
}
pub use p0_r93 as P0_R93;

/// Page 0, Register 0x5E: 右ADC通道AGC控制寄存器1
/// 功能：设置右ADC自动增益控制的目标水平、启动模式及噪声阈值
pub mod p0_r94 {
    pub const ADDRESS: u8 = 0x5E;
    pub const PAGE: u8 = 0;
}
pub use p0_r94 as P0_R94;

/// Page 0, Register 0x5F: 右ADC通道AGC控制寄存器2
/// 功能：设置右ADC AGC的滞后值、噪声阈值及最大PGA增益
pub mod p0_r95 {
    pub const ADDRESS: u8 = 0x5F;
    pub const PAGE: u8 = 0;
}
pub use p0_r95 as P0_R95;

/// Page 0, Register 0x60: 右ADC通道AGC控制寄存器3
/// 功能：配置右ADC AGC的最大允许PGA增益（0dB至58dB，0.5dB步进）
pub mod p0_r96 {
    pub const ADDRESS: u8 = 0x60;
    pub const PAGE: u8 = 0;
}
pub use p0_r96 as P0_R96;

/// Page 0, Register 0x61: 右ADC通道AGC控制寄存器4
/// 功能：设置右ADC AGC的攻击时间常数（决定增益减小速度）
pub mod p0_r97 {
    pub const ADDRESS: u8 = 0x61;
    pub const PAGE: u8 = 0;
}
pub use p0_r97 as P0_R97;

/// Page 0, Register 0x62: 右ADC通道AGC控制寄存器5
/// 功能：设置右ADC AGC的衰减时间常数（决定增益增加速度）
pub mod p0_r98 {
    pub const ADDRESS: u8 = 0x62;
    pub const PAGE: u8 = 0;
}
pub use p0_r98 as P0_R98;

/// Page 0, Register 0x63: 右ADC通道AGC控制寄存器6
/// 功能：配置右ADC AGC的噪声检测滞后值及去抖时间
pub mod p0_r99 {
    pub const ADDRESS: u8 = 0x63;
    pub const PAGE: u8 = 0;
}
pub use p0_r99 as P0_R99;

/// Page 0, Register 0x64: 右ADC通道AGC控制寄存器7
/// 功能：设置右ADC AGC的信号检测去抖时间（避免噪声触发增益波动）
pub mod p0_r100 {
    pub const ADDRESS: u8 = 0x64;
    pub const PAGE: u8 = 0;
}
pub use p0_r100 as P0_R100;

/// Page 0, Register 0x65: 右ADC通道AGC控制寄存器8
/// 功能：设置右ADC AGC实际应用增益读取寄存器（只读）
pub mod p0_r101 {
    pub const ADDRESS: u8 = 0x65;
    pub const PAGE: u8 = 0;
}
pub use p0_r101 as P0_R101;

/// Page 0, Register 0x66: DC偏移测量控制寄存器
/// 功能：配置ADC通道DC偏移自动测量功能的使能和采样参数
pub mod p0_r102 {
    pub const ADDRESS: u8 = 0x66;
    pub const PAGE: u8 = 0;
}
pub use p0_r102 as P0_R102;

/// Page 0, Register 0x67: DC偏移测量状态寄存器
/// 功能：存储ADC通道DC偏移测量的状态和结果有效性标志
pub mod p0_r103 {
    pub const ADDRESS: u8 = 0x67;
    pub const PAGE: u8 = 0;
}
pub use p0_r103 as P0_R103;

/// Page 0, Register 0x68: 左ADC DC偏移测量结果寄存器（低8位）
/// 功能：存储左ADC通道DC偏移测量结果的低8位（16位有符号整数）
pub mod p0_r104 {
    pub const ADDRESS: u8 = 0x68;
    pub const PAGE: u8 = 0;
}
pub use p0_r104 as P0_R104;

/// Page 0, Register 0x69: 左ADC DC偏移测量结果寄存器（中8位）
/// 功能：存储左ADC通道DC偏移测量结果的中间8位
pub mod p0_r105 {
    pub const ADDRESS: u8 = 0x69;
    pub const PAGE: u8 = 0;
}
pub use p0_r105 as P0_R105;

/// Page 0, Register 0x6A: 左ADC DC偏移测量结果寄存器（高8位）
/// 功能：存储左ADC通道DC偏移测量结果的高8位（包含符号位）
pub mod p0_r106 {
    pub const ADDRESS: u8 = 0x6A;
    pub const PAGE: u8 = 0;
}
pub use p0_r106 as P0_R106;

/// Page 0, Register 0x68: 右ADC DC偏移测量结果寄存器（低8位）
/// 功能：存储右ADC通道DC偏移测量结果的低8位（16位有符号整数）
pub mod p0_r107 {
    pub const ADDRESS: u8 = 0x6B;
    pub const PAGE: u8 = 0;
}
pub use p0_r107 as P0_R107;

/// Page 0, Register 0x69: 右ADC DC偏移测量结果寄存器（中8位）
/// 功能：存储右ADC通道DC偏移测量结果的中间8位
pub mod p0_r108 {
    pub const ADDRESS: u8 = 0x6C;
    pub const PAGE: u8 = 0;
}
pub use p0_r108 as P0_R108;

/// Page 0, Register 0x6A: 右ADC DC偏移测量结果寄存器（高8位）
/// 功能：存储右ADC通道DC偏移测量结果的高8位（包含符号位）
pub mod p0_r109 {
    pub const ADDRESS: u8 = 0x6D;
    pub const PAGE: u8 = 0;
}
pub use p0_r109 as P0_R109;

// 110-127 0x00 0x6E-0x7F Reserved Register