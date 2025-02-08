# DFRobot_CardReader

* [English Version](./README.md)



![正反面svg效果图](https://github.com/cdjq/DFRobot_CardReader/raw/master/resources/images/SEN0245svg4.png)

## 产品链接（ https://www.dfrobot.com.cn/ ）



## 目录

* [简介](#简介)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介



## 安装

1.要使用此库，首先下载库文件，将其粘贴到 `\Arduino\libraries` 目录中，然后打开示例文件夹并运行其中的示例。

## 方法
```c++
  /**
   * @brief 卡片阅读器的构造函数。
   * @param pWire I2C 接口的指针（默认：Wire）。
   * @param addr 卡片阅读器的 I2C 地址（默认：0x28）。
   */
  DFRobot_CardReader(TwoWire *pWire = &Wire, uint8_t addr = I2C_ADDR);
    
  /**
   * @brief Initializes the card reader module.
   * @param rstPin Pin number for resetting the module (default: 0).
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char begin();

  /**
   * @brief Sets the authentication key for communication with the card.
   * @param key Pointer to an array containing the 6-byte key.
   */
  void setKey(uint8_t *key);
  
  /*!
   * @fn scan(String nfcuid)
   * @brief Scan to determine whether there is a NFC smart card/tag with the specified UID.
   * @param nfcuid UID of the NFC card.
   * @return Boolean type, the result of operation
   * @retval true Finds a card with a specific UID
   * @retval false The card with a specific UID was not found
   */
  bool scan(String nfcuid);

  /*!
   * @fn scan(void)
   * @brief Scan to determine whether there is a NFC smart card/tag.
   * @return Boolean type, the result of operation
   * @retval true means find out a MIFARE Classic card.
   * @retval false no card
   */
  bool  scan(void);

  /*!
   * @fn readUid
   * @brief Obtain the UID of the card .
   * @return UID of the card.
   */
  String  readUid();

   /*!
    * @fn readData(int block, uint8_t offset)
    * @brief Read a byte from a specified block of a MIFARE Classic NFC smart card/tag.
    * @param block The number of the block to read from.
    * @param offset The offset of the block.
    * @return A byte read from the card.
    */
  uint8_t readData(uint8_t block, uint8_t offset);

  /*!
   * @fn readData(uint8_t* buffer, uint8_t block)
   * @brief Read a block from a MIFARE Classic NFC smart card/tag (16 bytes each block).
   * @param buffer The buffer of the read data.
   * @param block The number of the block to read from.
   * @return Status code.
   * @retval 1 successfully read data
   * @retval -1 Failed to read data
   */
  String readData(uint8_t block);

  /*!
   * @fn writeData(int block, uint8_t num, uint8_t data)
   * @brief Write a byte to a MIFARE Classic NFC smart card/tag.
   * @param block The number of pages you want to writes the data.
   * @param num The offset of the data.
   * @param data The byte to be written.
   */
  char writeData(uint8_t block, uint8_t num, uint8_t data);

  /*!
   * @fn writeData(int block, uint8_t data[])
   * @brief Write a block to a MIFARE Classic NFC smart card/tag..
   * @param block The number of the block to write to.
   * @param data The buffer of the data to be written.
   */
  char  writeData(uint8_t block, uint8_t data[]);

```


## 兼容性

MCU                | 表现良好	|表现异常	|未测试	|备注 |
------------------ | :----------: | :----------: | :---------: | -----

FireBeetle-ESP32        |      √       |              |             | 


## 历史

- Date 2025-02-08
- Version V0.1


## 贡献者

Written by fary(feng.yang@dfrobot.com), 2025.02.08 (Welcome to our [website](https://www.dfrobot.com/))
