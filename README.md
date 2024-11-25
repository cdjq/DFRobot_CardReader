# DFRobot_CardReader

* [中文版](./README_CN.md)


   
   
![正反面svg效果图](https://github.com/cdjq/DFRobot_CardReader/raw/master/resources/images/SEN0245svg4.png)

## Product Link（https://www.dfrobot.com/)
    
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary


Provide an Arduino library to control 

## Installation

1.To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.
2.Use the library also need to download depends on: https://github.com/DFRobot/DFRobot_RTU
## Methods
```C++

    /**
     * @brief 卡片阅读器的构造函数。
     * @param pWire I2C 接口的指针（默认：Wire）。
     * @param addr 卡片阅读器的 I2C 地址（默认：0x28）。
     */
    DFRobot_CardReader(TwoWire *pWire = &Wire, uint8_t addr = I2C_ADDR);
    
    /**
     * @brief 停止当前连接的 RFID 卡。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdHalt(void);
    
    /**
     * @brief 初始化卡片阅读器模块。
     * @param rstPin 模块复位引脚编号（默认：0）。
     * @return 成功返回 0；否则返回错误代码。
     */
    char begin(uint8_t rstPin = 0);
    
    /**
     * @brief 设置与卡片通信的认证密钥。
     * @param key 指向包含 6 字节密钥的数组的指针。
     */
    void setKey(uint8_t *key);
    
    /**
     * @brief 读取卡片的唯一标识符（UUID）。
     * @param UUID 用于存储 UUID 的缓冲区指针。
     * @return UUID 的字节长度。
     */
    uint8_t readUUID(uint8_t *UUID);
    
    /**
     * @brief 激活 RFID 模块的天线。
     * @details 此函数启用与卡片通信所需的射频场。
     */
    void pcdAntennaOn(void);
    
    /**
     * @brief 停用 RFID 模块的天线。
     * @details 此函数禁用射频场以降低功耗。
     */
    void pcdAntennaOff(void);
    
    /**
     * @brief 发送请求以检测卡片。
     * @param req_code 请求类型（如 PICC_REQIDL 用于空闲卡片）。
     * @param pTagType 用于存储检测到的卡片类型的缓冲区指针。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdRequest(unsigned char req_code, unsigned char *pTagType);
    
    /**
     * @brief 执行防碰撞检测以识别场内的多张卡片。
     * @param pSnr 用于存储检测到的卡片序列号的缓冲区指针。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdAnticoll(unsigned char *pSnr);
    
    /**
     * @brief 选择一张卡片进行通信。
     * @param pSnr 包含卡片序列号的数组指针。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdSelect(unsigned char *pSnr);
    
    /**
     * @brief 使用指定密钥验证特定卡片块。
     * @param auth_mode 验证模式（如 PICC_AUTHENT1A 表示密钥 A）。
     * @param addr 要验证的块地址。
     * @param pKey 指向包含 6 字节密钥的数组的指针。
     * @param pSnr 指向卡片序列号数组的指针。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdAuthState(unsigned char auth_mode, unsigned char addr, unsigned char *pKey, unsigned char *pSnr);
    
    /**
     * @brief 从指定卡片块读取数据。
     * @param addr 要读取的块地址。
     * @param pData 用于存储读取数据的缓冲区指针。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdRead(unsigned char addr, unsigned char *pData);
    
    /**
     * @brief 向指定卡片块写入数据。
     * @param addr 要写入的块地址。
     * @param pData 指向包含要写入数据的数组的指针。
     * @return 成功返回 0；否则返回错误代码。
     */
    char pcdWrite(unsigned char addr, unsigned char *pData);
    
    /**
     * @brief 计算给定数据的 CRC 校验值。
     * @param pIndata 输入数据数组的指针。
     * @param len 输入数据数组的长度。
     * @param pOutData 用于存储 CRC 校验值的缓冲区指针。
     */
    void calulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData);
    
    /**
     * @brief 直接写入卡片阅读器寄存器。
     * @param Address 寄存器地址。
     * @param value 要写入的值。
     */
    void writeRawRC(unsigned char Address, unsigned char value);
    
    /**
     * @brief 直接读取卡片阅读器寄存器。
     * @param Address 寄存器地址。
     * @return 从寄存器读取的值。
     */
    unsigned char readRawRC(unsigned char Address);
    
    /**
     * @brief 设置卡片阅读器寄存器中的特定位。
     * @param reg 寄存器地址。
     * @param mask 要设置的位的掩码。
     */
    void setBitMask(unsigned char reg, unsigned char mask);
    
    /**
     * @brief 清除卡片阅读器寄存器中的特定位。
     * @param reg 寄存器地址。
     * @param mask 要清除的位的掩码。
     */
    void clearBitMask(unsigned char reg, unsigned char mask);
    
    /**
     * @brief 启用天线测试模式，用于调试或开发。
     */
    void pcdAntennaTestOn(void);
    
    /**
     * @brief 将卡片阅读器置于低功耗模式以节省能量。
     */
    void softLowPower();
    
    /**
     * @brief 与卡片建立连接。
     * @return 如果成功连接卡片返回 true，否则返回 false。
     */
    bool connect();
    
    /**
     * @brief 向连接的卡片发送停止命令。
     * @return 成功返回 0；否则返回错误代码。
     */
    char MIFHalt(void);
    
    /**
     * @brief 从指定卡片内存块读取一块数据。
     * @param Block 卡片上的内存块地址（Mifare 卡为 0-63）。
     * @param Buf 用于存储 16 字节块数据的数组指针。
     * @return 成功返回 MI_OK（0x00）；否则返回错误代码。
     * @note 调用此函数前必须通过 pcdAuthState 函数验证该块。
     *       Buf 提供的缓冲区必须至少有 16 字节以存储完整块数据。
     */
    char readBlock(unsigned char Block, unsigned char *Buf);
    
    /**
     * @brief 将数据写入指定卡片内存块。
     * @param Block 卡片上的内存块地址（Mifare 卡为 0-63）。
     * @param data 包含 16 字节写入数据的数组指针。
     * @return 成功返回 MI_OK（0x00）；否则返回错误代码。
     * @note 调用此函数前必须通过 pcdAuthState 函数验证该块。
     */
    char writeBlock(unsigned char Block, uint8_t *data);
    
    /**
     * @brief 向 RC522 发送命令并处理响应。
     * @param Command 要发送到 RC522 的命令（如 PCD_TRANSCEIVE、PCD_AUTHENT）。
     * @param pInData 指向发送到 RC522 的输入数据数组的指针。
     * @param InLenByte 输入数据数组的长度（以字节为单位）。
     * @param pOutData 用于存储接收到数据的缓冲区指针。
     * @param pOutLenBit 用于存储接收到数据长度（以位为单位）的变量指针。
     * @return 成功返回 MI_OK（0x00）；否则返回错误代码。
     * @details 此函数处理与 RC522 模块的低级通信，包括数据传输和响应接收。
     */
    char pcdComMF522(unsigned char Command, unsigned char *pInData,
                     unsigned char InLenByte, unsigned char *pOutData,
                     unsigned int *pOutLenBit);
    
    /**
     * @brief 对卡片内存块执行算术操作（增量、减量、恢复）。
     * @param dd_mode 操作模式（如 PICC_INCREMENT、PICC_DECREMENT、PICC_RESTORE）。
     * @param addr 要执行操作的内存块地址。
     * @param pValue 指向包含操作值的 4 字节数组的指针。
     * @return 成功返回 MI_OK（0x00）；否则返回错误代码。
     * @note 操作结果暂时存储在卡片的缓冲区中，
     *       必须使用 PICC_TRANSFER 命令将其提交到内存块。
     */
    char pcdValue(unsigned char dd_mode, unsigned char addr, unsigned char *pValue);
    
    /**
     * @brief 将一个内存块的值复制到卡片上的另一个内存块。
     * @param sourceaddr 源内存块地址。
     * @param goaladdr 目标内存块地址。
     * @return 成功返回 MI_OK（0x00）；否则返回错误代码。
     * @note 此函数直接在块之间传输值，不涉及主机。
     */
    char pcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);


```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32        |      √       |              |             | 


## History

- Date 2024-11-25
- Version V0.1
## Credits
Written by fengli(li.feng@dfrobot.com), 2024.7.22 (Welcome to our [website](https://www.dfrobot.com/))
## History