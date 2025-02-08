import time
from machine import Pin, I2C



MAXRLEN       = 18
MIN_STRENGTH  = 228

DEF_FIFO_LENGTH      =       64          


PCD_IDLE            =      0x00          
PCD_AUTHENT         =      0x0E          
PCD_RECEIVE         =      0x08          
PCD_TRANSMIT        =      0x04          
PCD_TRANSCEIVE      =      0x0C          
PCD_RESETPHASE      =      0x0F          
PCD_CALCCRC         =      0x03          

PICC_REQIDL         =      0x26         
PICC_REQALL          =     0x52          
PICC_ANTICOLL1      =      0x93          
PICC_ANTICOLL2      =      0x95          
PICC_AUTHENT1A      =      0x60          
PICC_AUTHENT1B      =      0x61          
PICC_READ           =      0x30          
PICC_WRITE          =      0xA0          
PICC_DECREMENT      =      0xC0          
PICC_INCREMENT       =     0xC1          
PICC_RESTORE        =      0xC2          
PICC_TRANSFER       =      0xB0          
PICC_HALT           =      0x50          

RFU00            =     0x00    
CommandReg       =     0x01    
ComIEnReg        =     0x02    
DivlEnReg        =     0x03    
ComIrqReg        =     0x04    
DivIrqReg        =     0x05
ErrorReg         =     0x06    
Status1Reg       =     0x07    
Status2Reg       =     0x08    
FIFODataReg      =     0x09
FIFOLevelReg      =    0x0A
WaterLevelReg     =    0x0B
ControlReg       =     0x0C
BitFramingReg    =     0x0D
CollReg          =     0x0E
RFU0F             =    0x0F

RFU10             =    0x10
ModeReg          =     0x11
TxModeReg        =     0x12
RxModeReg        =     0x13
TxControlReg     =     0x14
TxASKReg         =     0x15
TxSelReg         =     0x16
RxSelReg         =     0x17
RxThresholdReg   =     0x18
DemodReg         =     0x19
RFU1A            =     0x1A
RFU1B            =     0x1B
MifareReg        =     0x1C
RFU1D            =     0x1D
RFU1E             =    0x1E
SerialSpeedReg   =     0x1F

RFU20            =     0x20  
CRCResultRegM    =     0x21
CRCResultRegL    =     0x22
RFU23            =     0x23
ModWidthReg      =     0x24
RFU25            =     0x25
RFCfgReg         =     0x26
GsNReg           =     0x27
CWGsCfgReg       =     0x28
ModGsCfgReg      =     0x29
TModeReg         =     0x2A
TPrescalerReg    =     0x2B
TReloadRegH      =     0x2C
TReloadRegL      =     0x2D
TCounterValueRegH  =   0x2E
TCounterValueRegL  =   0x2F

RFU30           =      0x30
TestSel1Reg      =     0x31
TestSel2Reg      =     0x32
TestPinEnReg     =     0x33
TestPinValueReg  =     0x34
TestBusReg       =     0x35
AutoTestReg      =     0x36
VersionReg       =     0x37
AnalogTestReg    =     0x38
TestDAC1Reg      =     0x39  
TestDAC2Reg      =     0x3A   
TestADCReg       =     0x3B   
RFU3C            =     0x3C   
RFU3D            =     0x3D   
RFU3E            =     0x3E   
RFU3F		     =     0x3F

MI_OK            =              0 
MI_CHK_OK        =              0 
MI_CRC_ZERO      =              0 

MI_CRC_NOTZERO    =             1 

MI_NOTAGERR       =          0xFF 
MI_CHK_FAILED     =          0xFF 
MI_CRCERR         =          0xFE 
MI_CHK_COMPERR    =          0xFE 
MI_EMPTY          =          0xFD 
MI_AUTHERR        =          0xFC 
MI_PARITYERR      =          0xFB 
MI_CODEERR        =          0xFA 

MI_SERNRERR       =          0xF8 
MI_KEYERR         =          0xF7 
MI_NOTAUTHERR     =          0xF6 
MI_BITCOUNTERR    =          0xF5 
MI_BYTECOUNTERR   =          0xF4 
MI_IDLE           =          0xF3 
MI_TRANSERR       =          0xF2 
MI_WRITEERR       =          0xF1 
MI_INCRERR        =          0xF0 
MI_DECRERR        =          0xEF 
MI_READERR        =          0xEE 
MI_OVFLERR        =          0xED 
MI_POLLING        =          0xEC 
MI_FRAMINGERR     =          0xEB 
MI_ACCESSERR      =          0xEA 
MI_UNKNOWN_COMMAND  =        0xE9 
MI_COLLERR          =        0xE8 
MI_RESETERR         =        0xE7 
MI_INITERR          =        0xE7 
MI_INTERFACEERR     =        0xE7 
MI_ACCESSTIMEOUT    =        0xE5 
MI_NOBITWISEANTICOLL  =      0xE4 
MI_QUIT               =      0xE2 

MI_RECBUF_OVERFLOW    =      0xCF 
MI_SENDBYTENR         =      0xCE 

MI_SENDBUF_OVERFLOW     =    0xCC 
MI_BAUDRATE_NOT_SUPPORTED =  0xCB 
MI_SAME_BAUDRATE_REQUIRED  = 0xCA 

MI_WRONG_PARAMETER_VALUE  =  0xC5 

MI_BREAK                  =  0x9E 
MI_NY_IMPLEMENTED         =  0x9D 
MI_NO_MFRC                =  0x9C 
MI_MFRC_NOTAUTH           =  0x9B 
MI_WRONG_DES_MODE         =  0x9A 
MI_HOST_AUTH_FAILED       =  0x99 

MI_WRONG_LOAD_MODE        =  0x97 
MI_WRONG_DESKEY           =  0x96 
MI_MKLOAD_FAILED          =  0x95 
MI_FIFOERR                =  0x94 
MI_WRONG_ADDR             =  0x93 
MI_DESKEYLOAD_FAILED      =  0x92 

MI_WRONG_SEL_CNT          =  0x8F 
MI_RC531_WRONG_READVALUE  =  0x8E 
MI_WRONG_TEST_MODE        =  0x8C 
MI_TEST_FAILED            =  0x8B 
MI_TOC_ERROR              =  0x8A 
MI_COMM_ABORT             =  0x89 
MI_INVALID_BASE           =  0x88 
MI_MFRC_RESET             =  0x87 
MI_WRONG_VALUE            =  0x86 
MI_VALERR                 =  0x85

_STATUS_OK = 0x00

class Rfid(object):
    password_buffer=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
    RC552_I2C_ADDR = 0x28
    uuid=[0]
    def __init__(self,scl=48,sda=47,bus=0):
        """!
        @brief Module I2C communication init
        @param i2c_addr - I2C communication address
        @param scl - I2C scl pin
        @param sda - I2C sda pin
        @param bus - I2C bus
        """
        self._addr = self.RC552_I2C_ADDR
        self._i2c  = I2C(bus,scl=Pin(scl),sda=Pin(sda),freq=400000)
        self.block_data = [0]*16
    
    def read_raw_rc(self,address):
        """
        @brief 从指定地址读取1个字节数据
        @param address: 寄存器地址
        @return 读取的数据
        """
        try:
            self._i2c.writeto(self._addr, bytes([address]))  # 写入寄存器地址
            read_data = self._i2c.readfrom(self._addr, 1)    # 读取1字节数据
            #print(read_data[0])
            return read_data[0] if read_data else 0          # 返回读取的数据
        except Exception as e:
            print(f"Error in read_raw_rc: {e}")
            return 0

    def write_raw_rc(self,address, value):
        """
        @brief 向指定地址写入1个字节数据
        @param address: 寄存器地址
        @param value: 写入的数据
        """
        try:
            self._i2c.writeto(self._addr, bytes([address, value]))  # 写入地址和数据
        except Exception as e:
            print(f"Error in write_raw_rc: {e}")    
     
    def set_bit_mask(self, reg, mask):
        """
        @brief 设置寄存器中的位
        @param reg: 寄存器地址
        @param mask: 需要设置的位的掩码
        """
        try:
            # 读取寄存器当前值
            tmp = self.read_raw_rc(reg)
            # 设置指定的位
            tmp |= mask
            # 写入新的值回寄存器
            self.write_raw_rc(reg, tmp)
        except Exception as e:
            print(f"Error in set_bit_mask: {e}")

    def clear_bit_mask(self, reg, mask):
        """
        @brief 清除寄存器中的位
        @param reg: 寄存器地址
        @param mask: 需要清除的位的掩码
        """
        try:
            # 读取寄存器当前值
            tmp = self.read_raw_rc(reg)
            # 清除指定的位
            tmp &= ~mask
            # 写入新的值回寄存器
            self.write_raw_rc(reg, tmp)
        except Exception as e:
            print(f"Error in clear_bit_mask: {e}")

    def PcdReset(self):
        """
        PcdReset 函数的 MicroPython 版本，用于初始化复位并配置模块。
        :return: MI_OK (假设 MI_OK 是某个预定义的状态常量，通常为 0)
        """
        # 配置模块寄存器
        self.write_raw_rc(CommandReg, PCD_RESETPHASE)
        time.sleep_ms(100)
        self.write_raw_rc(ModeReg, 0x3D)
        self.write_raw_rc(TReloadRegL, 30)
        self.write_raw_rc(TReloadRegH, 0)
        self.write_raw_rc(TModeReg, 0x8D)
        self.write_raw_rc(TPrescalerReg, 0x3E)
        # WriteRawRC(TxASKReg, 0x40)  # DEBUG 和测试时可启用
        return MI_OK            
            
    def pcd_com_mf522(self, command, p_in_data, in_len_byte, p_out_data, p_out_len_bit):
        """
        @brief 实现 PcdComMF522 的功能，用于与RC522通信。
        @param command: 指令类型 (PCD_AUTHENT 或 PCD_TRANSCEIVE)
        @param p_in_data: 输入数据数组
        @param in_len_byte: 输入数据长度
        @param p_out_data: 输出数据数组（作为传出参数）
        @param p_out_len_bit: 输出数据长度（以位为单位，作为传出参数）
        @return: 状态码（如 MI_OK 或 MI_ERR）
        """
        MI_OK = 0
        MI_ERR = -1
        MI_NOTAGERR = -2
        MAXRLEN = 16  # 最大输出数据长度
    
        irq_en = 0x00
        wait_for = 0x00
    
        # 根据命令类型设置中断使能和等待标志
        if command == PCD_AUTHENT:
            irq_en = 0x12
            wait_for = 0x10
        elif command == PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_for = 0x30
    
        # 初始化寄存器
        self.write_raw_rc(ComIEnReg, irq_en | 0x80)  # 启用中断
        self.clear_bit_mask(ComIrqReg, 0x80)        # 清除所有中断标志
        self.write_raw_rc(CommandReg, PCD_IDLE)     # 停止命令执行
        self.set_bit_mask(FIFOLevelReg, 0x80)       # 清空FIFO缓冲区
    
        # 将输入数据写入FIFO
        for i in range(in_len_byte):
            self.write_raw_rc(FIFODataReg, p_in_data[i])
    
        # 写入命令到CommandReg
        self.write_raw_rc(CommandReg, command)
    
        # 如果是传输命令，设置位帧寄存器以启动传输
        if command == PCD_TRANSCEIVE:
            self.set_bit_mask(BitFramingReg, 0x80)
    
        # 等待响应
        timeout = 6000  # 超时时间，25ms
        while timeout:
            n = self.read_raw_rc(ComIrqReg)
            if n & 0x01 or n & wait_for:  # 检查中断标志
                break
            timeout -= 1
    
        # 清除位帧寄存器
        self.clear_bit_mask(BitFramingReg, 0x80)
    
        # 检查响应结果
        status = MI_ERR
        if timeout:
            if not (self.read_raw_rc(ErrorReg) & 0x1B):  # 检查错误寄存器
                status = MI_OK
                if n & irq_en & 0x01:
                    status = MI_NOTAGERR
                if command == PCD_TRANSCEIVE:
                    n = self.read_raw_rc(FIFOLevelReg)  # 获取接收到的数据字节数
                    last_bits = self.read_raw_rc(ControlReg) & 0x07
                    if last_bits:
                        p_out_len_bit[0] = (n - 1) * 8 + last_bits
                    else:
                        p_out_len_bit[0] = n * 8
    
                    # 限制读取数据长度
                    if n == 0:
                        n = 1
                    if n > MAXRLEN:
                        n = MAXRLEN
    
                    # 读取FIFO中的数据
                    p_out_data.clear()  # 每次循环开始前清空
                    for i in range(n):
                        p_out_data.append(self.read_raw_rc(FIFODataReg))
            else:
                status = MI_ERR
    
        # 停止定时器并进入空闲状态
        self.set_bit_mask(ControlReg, 0x80)
        self.write_raw_rc(CommandReg, PCD_IDLE)
        return status
    
    def pcd_request(self, req_code):
        """
        @brief 实现 PcdRequest 功能，发送请求命令以获取卡片类型。
        @param req_code: 请求代码（REQA 或 WUPA）
        @return: (状态码, p_tag_type)，状态码如 MI_OK 或 MI_ERR，p_tag_type 为卡片类型列表
        """
        MI_OK = 0
        MI_ERR = -1
    
        # 初始化返回的卡片类型
        p_tag_type = [0, 0]
    
        # 清除 Status2Reg 的特定位
        self.clear_bit_mask(Status2Reg, 0x08)
    
        # 设置 BitFramingReg 和 TxControlReg
        self.write_raw_rc(BitFramingReg, 0x07)
        self.set_bit_mask(TxControlReg, 0x03)
    
        # 准备发送缓冲区
        uc_com_mf522_buf = [req_code]
    
        # 调用 PcdComMF522 进行数据交换
        p_out_data = []
        p_out_len_bit = [0]  # 输出长度（位）
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, uc_com_mf522_buf, 1, p_out_data, p_out_len_bit)
    
        # 检查返回状态和长度
        if status == MI_OK and p_out_len_bit[0] == 0x10:
            p_tag_type[0] = p_out_data[0]
            p_tag_type[1] = p_out_data[1]
        else:
            status = MI_ERR
        return status, p_tag_type

    def pcd_anticoll(self):
        """
        @brief 实现 PcdAnticoll 功能，获取卡片的序列号。
        @return: 包含卡片序列号的列表（长度 4，未成功读取则返回全为 0 的列表）
        """
        MI_OK = 0
        PICC_ANTICOLL1 = 0x93  # 防冲突命令
        MAXRLEN = 16           # 最大数据长度
        
        # 初始化返回序列号为全零
        p_snr = [0] * 4
    
        # 清除 Status2Reg 和 CollReg 特定位
        self.clear_bit_mask(Status2Reg, 0x08)
        self.write_raw_rc(BitFramingReg, 0x00)
        self.clear_bit_mask(CollReg, 0x80)
    
        # 准备发送缓冲区
        uc_com_mf522_buf = [PICC_ANTICOLL1, 0x20]
    
        # 调用 PcdComMF522 进行数据交换
        p_out_data = []
        p_out_len_bit = [0]  # 输出长度（位）
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, uc_com_mf522_buf, 2, p_out_data, p_out_len_bit)
        #print(p_snr)
        # 检查返回状态并计算校验
        if status == MI_OK and len(p_out_data) >= 5:
            snr_check = 0
            for i in range(4):
                p_snr[i] = p_out_data[i]
                snr_check ^= p_out_data[i]
            # 校验序列号
            if snr_check != p_out_data[4]:
                p_snr = [0] * 4  # 校验失败，返回全零
    
        # 设置 CollReg 特定位
        self.set_bit_mask(CollReg, 0x80)
        return p_snr

    def pcd_select(self, p_snr):
        """
        @brief 实现 PcdSelect 功能，用于选择特定的 RFID 卡片。
        @param p_snr: 输入参数，卡片的序列号（4字节）。
        @return: 状态码（如 MI_OK 或 MI_ERR）。
        """
        MI_OK = 0
        MI_ERR = -1
        PICC_ANTICOLL1 = 0x93  # 防冲突命令
        MAXRLEN = 18
    
        # 初始化缓冲区
        uc_com_mf522_buf = [0] * MAXRLEN
        uc_com_mf522_buf[0] = PICC_ANTICOLL1
        uc_com_mf522_buf[1] = 0x70
        uc_com_mf522_buf[6] = 0
        crc = [0] * 2
    
        # 将序列号拷贝到缓冲区并计算校验和
        for i in range(4):
            uc_com_mf522_buf[i + 2] = p_snr[i]
            uc_com_mf522_buf[6] ^= p_snr[i]
    
        # 计算 CRC 并附加到缓冲区
        self.calculate_crc(uc_com_mf522_buf, 7, crc)
    
        # 清除 Status2Reg 特定位
        self.clear_bit_mask(Status2Reg, 0x08)
        uc_com_mf522_buf[7]=crc[0]
        uc_com_mf522_buf[8]=crc[1]
        # 执行 PcdComMF522 进行通信
        p_out_data = []
        p_out_len_bit = [0]
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, uc_com_mf522_buf, 9, p_out_data, p_out_len_bit)
    
        # 检查通信结果
        if status == MI_OK and p_out_len_bit[0] == 0x18:
            status = MI_OK
        else:
            status = MI_ERR
        return status
    
    def pcd_auth_state(self, auth_mode, addr, p_key, p_snr):
        """
        @brief 实现 PcdAuthState 功能，用于对卡片的指定块进行认证。
        @param auth_mode: 认证模式（如 KEY_A 或 KEY_B）。
        @param addr: 要认证的块地址。
        @param p_key: 密钥（6字节）。
        @param p_snr: 卡片序列号（4字节）。
        @return: 状态码（如 MI_OK 或 MI_ERR）。
        """
        MI_OK = 0
        MI_ERR = -1
        MAXRLEN = 16
    
        # 初始化缓冲区
        uc_com_mf522_buf = [0] * MAXRLEN
        uc_com_mf522_buf[0] = auth_mode
        uc_com_mf522_buf[1] = addr
    
        # 添加密钥到缓冲区
        for i in range(6):
            uc_com_mf522_buf[i + 2] = p_key[i]
    
        # 添加序列号到缓冲区
        for i in range(4):
            uc_com_mf522_buf[i + 8] = p_snr[i]
    
        # 调用 PcdComMF522 进行认证
        p_out_data = []
        p_out_len_bit = [0]
        status = self.pcd_com_mf522(PCD_AUTHENT, uc_com_mf522_buf, 12, p_out_data, p_out_len_bit)
    
        # 检查认证状态
        if status != MI_OK or not (self.read_raw_rc(Status2Reg) & 0x08):
            status = MI_ERR
        return status
    
    def pcd_read(self, addr):
        """
        @brief 实现 PcdRead 功能，从指定块地址读取数据。
        @param addr: 要读取的块地址。
        @return: (状态码, 数据数组)。
                 状态码可能为 MI_OK 或 MI_ERR。
                 数据数组为读取的 16 字节数据。
        """
        MI_OK = 0
        MI_ERR = -1
        MAXRLEN = 16
    
        # 初始化缓冲区
        uc_com_mf522_buf = [0] * MAXRLEN
        uc_com_mf522_buf[0] = PICC_READ
        uc_com_mf522_buf[1] = addr
        crc = [0] * 2
        # 计算校验码并添加到缓冲区
        self.calculate_crc(uc_com_mf522_buf[:2],2, crc)
    
        # 调用 PcdComMF522 发送命令并获取响应
        uc_com_mf522_buf[2] = crc[0]
        uc_com_mf522_buf[3] = crc[1]
        
        p_out_data = [0] * MAXRLEN
        p_out_len_bit = [0]
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, uc_com_mf522_buf, 4, p_out_data, p_out_len_bit)
        # 检查读取状态
        if status == MI_OK and p_out_len_bit[0] == 0x90:
            # 提取数据
            read_data = p_out_data[:16]
            #print(read_data)
            return MI_OK, read_data
        else:
            return MI_ERR, None
    
    def _read_block(self, block):
        """
        @brief 从指定块地址读取数据，并可选择性解密。
        @param block: 要读取的块地址。
        @return: 读取状态和数据 (成功返回 (0, buf)，失败返回 (error_code, None))。
        """
        MI_OK = 0
        result = self.pcd_auth_state(0x60, block, self.password_buffer, self.uuid)
        if result != MI_OK:
            #print("error1")
            return "error"
        # 调用 PcdRead 读取块数据
        result, data = self.pcd_read(block)
        if result != MI_OK:
            #print("error2")
            return "error"
        # 如果不需要解密，直接返回读取的数据
        return data

    def pcd_write(self,addr, pData):
        MI_OK = 0
        MI_ERR = -1
        status = MI_OK
        unLen = [0]
        ucComMF522Buf = [0] * 18
    
        # 准备写命令的第一部分
        ucComMF522Buf[0] = PICC_WRITE
        ucComMF522Buf[1] = addr
        crc = [0] * 2
        # 计算 CRC
        self.calculate_crc(ucComMF522Buf, 2, crc)
        ucComMF522Buf[2] = crc[0]
        ucComMF522Buf[3] = crc[1]
        # 发送第一部分命令
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, unLen)
    
        if status != MI_OK or unLen[0] != 4 or (ucComMF522Buf[0] & 0x0F) != 0x0A:
            status = MI_ERR
        # 如果第一部分命令成功，发送数据	
        ucComMF522Buf1 = [0] * 18
        #print(ucComMF522Buf)
        if status == MI_OK:
            for i in range(16):
                ucComMF522Buf1[i] = pData[i]
            # 计算数据的 CRC
            self.calculate_crc(ucComMF522Buf1, 16, crc)
            ucComMF522Buf1[16] = crc[0]
            ucComMF522Buf1[17] = crc[1]
            # 发送数据
            status = self.pcd_com_mf522(PCD_TRANSCEIVE, ucComMF522Buf1, 18, ucComMF522Buf1, unLen)
            if status != MI_OK or unLen[0] != 4 or (ucComMF522Buf1[0] & 0x0F) != 0x0A:
                status = MI_ERR
        return status

    def _write_block(self, block, rf_buffer):
        """
        @brief 向指定的 RFID 卡块写入数据。
        @param block: 要写入的块地址。
        @param rf_buffer: 要写入的数据缓冲区 (长度应为 16 字节)。
        @return: 写入操作的状态 (0 表示成功, 非零表示失败)。
        """
        MI_OK = 0
        # 进行认证操作
        result = self.pcd_auth_state(0x60, block, self.password_buffer, self.uuid)
        if result != MI_OK:
            #print("write error")
            return result
        
        # 写入数据到卡片
        #print("rf_buffer")
        #print(rf_buffer)
        result = self.pcd_write(block, rf_buffer)
        return result

    def PcdHalt(self):
        status = MI_OK
        unLen = [0]
        ucComMF522Buf = [0] * MAXRLEN
        # 设置停止命令
        ucComMF522Buf[0] = PICC_HALT
        ucComMF522Buf[1] = 0
        # 计算 CRC
        self.calculate_crc(ucComMF522Buf, 2, ucComMF522Buf[2:4])
        # 发送停止命令
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, unLen)
        return status

    def MIF_Halt(self):
        status = MI_OK
        unLen = [0]
        ucComMF522Buf = [0] * MAXRLEN
        # 设置停止命令
        ucComMF522Buf[0] = PICC_HALT
        ucComMF522Buf[1] = 0
        # 计算 CRC
        self.calculate_crc(ucComMF522Buf, 2, ucComMF522Buf[2:4])
        # 发送停止命令
        status = self.pcd_com_mf522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, unLen)
        return status
 
    def calculate_crc(self, pIndata, length, pOutData):
        i = 0
        n = 0
        self.clear_bit_mask(DivIrqReg, 0x04)  # 清除中断标志位
        self.write_raw_rc(CommandReg, PCD_IDLE)  # 进入空闲模式
        self.set_bit_mask(FIFOLevelReg, 0x80)  # 设置FIFO寄存器
        for i in range(length):
            self.write_raw_rc(FIFODataReg, pIndata[i])  # 将数据写入FIFO
        self.write_raw_rc(CommandReg, PCD_CALCCRC)  # 开始CRC计算
        
        i = 0xFF  # 设置计数器
        while i != 0:
            n = self.read_raw_rc(DivIrqReg)  # 读取中断寄存器
            i -= 1
            if n & 0x04:  # 检查CRC计算完成的标志位
                break
    
        # 读取CRC结果
        pOutData[0] = self.read_raw_rc(CRCResultRegL)  # 低字节
        pOutData[1] = self.read_raw_rc(CRCResultRegM)  # 高字节  

    def PcdAntennaOn(self):
           """
           打开天线的 MicroPython 实现。
           """
           # 设置 TxASKReg 寄存器
           self.write_raw_rc(TxASKReg, 0x40)
           time.sleep_ms(10)  # 延迟 10ms
       
           # 读取 TxControlReg 寄存器
           i = self.read_raw_rc(TxControlReg)
       
           # 检查并设置 TxControlReg 的特定位
           if not (i & 0x03):
               self.set_bit_mask(TxControlReg, 0x03)
           
           # 读取 TxASKReg 用于调试或后续使用（可选）
           i = self.read_raw_rc(TxASKReg)     

    def begin(self):
        """!
        @brief Function begin.
        """
        self.PcdReset()
        self.PcdAntennaOn()   

    def read_block(self, block, index=None):
        """!
        @brief reads data from the specified block of the MIFARE classic NFC smart card/tag.
        @param block - The number of the block to read
        @param index - Data offset (1-16). When index is None, it indicates the size of a block to be read. When index is greater than 0, it indicates a byte to be read
        @return Indicates the read data
        """
        #print("read_block")
        data = self._read_block(block)
        if (
            data == "error"
            or data == "read error!"
            or data == "read timeout!"
            or data == "wake up error!"
            or data == "false"
        ):
            return None
        self.block_data = data
        if index is None:
            
            return data
        else:
            if index>0:
                return self.block_data[index - 1]
            return None

    def write_block(self, block,data,index=0):
        """!
        @brief writes data to MIFARE classic NFC smart card/tag.
        @param block - The number of the block to be written
        @param data - Data to be written, when index is 0, indicates the size of a block to be written, when index is greater than 0, indicates a byte to be written
        @param index - Data offset (1-16)
        @return Boolean type, operation result
        """
        if isinstance(data, str):
            real_val = []
            for i in data:
                real_val.append(int(ord(i)))
            if len(real_val) < 16:
                for i in range(16 - len(real_val)):
                    real_val.append(0)
            elif len(real_val) > 16:
                return False
        if isinstance(data, list):
            real_val = []
            if len(data) < 16:
                for i in range(16 - len(data)):
                    data.append(0)
            elif len(data) > 16:
                return False
            real_val = data
        index = max(min(index, 16), 1)
        if isinstance(data, int):
            #print(self.block_data)
            self.read_block(block)
            self.block_data[index - 1] = data
            self._write_block(block, self.block_data)
        else:
            block_data = [0 for i in range(index - 1)]
            block_data[index:] = real_val
            self._write_block(block, block_data)
        return True        
        
    def read_protocol(self):
        state,self.nfc_protocol = self.pcd_request(0x52)
        if state != MI_OK: 
            return "no card!"
        if self.nfc_protocol[0] == 0x04 and self.nfc_protocol[1] == 0x00:
            return "mifare"
        elif self.nfc_protocol[0] == 0x02 and self.nfc_protocol[1] == 0x00:
            return "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x00:
            return "MF-UltraLight"
        elif self.nfc_protocol[0] == 0x08 and self.nfc_protocol[1] == 0x00:
            return "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x03:
            return "MF Desire"
        else:
            return "Unknown"

    def scan(self,uuid=""):
        """!
        @brief scans to determine if there is an NFC smart card/label.
        @param uuid - When the uuid is set, this method is used to scan the card of the uuid. If the UUID is not set, it detects whether the nfc card exists
        @return Boolean type, operation result
        @retval True: Find the card
        @retval False: No card
        """
        state,self.nfc_protocol = self.pcd_request(0x52)
        ret = True
        if state != MI_OK: 
            self.card_type= "no card!"
            ret = False
        if self.nfc_protocol[0] == 0x04 and self.nfc_protocol[1] == 0x00:
            self.card_type = "mifare"
        elif self.nfc_protocol[0] == 0x02 and self.nfc_protocol[1] == 0x00:
            self.card_type = "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x00:
            self.card_type = "MF-UltraLight"
        elif self.nfc_protocol[0] == 0x08 and self.nfc_protocol[1] == 0x00:
            self.card_type = "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x03:
            self.card_type = "MF Desire"
        else:
            self.card_type = "Unknown"
            ret = False
        if ret:
            self.uuid = self.pcd_anticoll()
            self.scan_serial_num = int.from_bytes(bytes(self.uuid),'big')
            if self.pcd_select(self.uuid)!=0:
              ret = False  
        if uuid!="":
            uid="".join([str(hex(u))[2:] for u in self.uuid])
            uuid = uuid.lower()
            if uuid!=uid:
                ret = False
        return ret

    def read_uid(self):
        """!
        @brief Obtain the UID of the card.
        @return UID of the card.
        """
        return "".join([format(u, "02x") for u in self.uuid])

