/* Arduino UDisk Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino Sd2Card Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#define USE_SPI_LIB
#include <Arduino.h>
#include "2Card.h"
//------------------------------------------------------------------------------


#define SDCARD_SPI SPI
#include <SPI.h>
#include <W25Q.h>
#define UDISK W25Q
static SPISettings settings;

#define CS 32
#define CS_H  digitalWrite(CS, HIGH)
#define CS_L  digitalWrite(CS, LOW)
static void ReadUniqueID(uint8_t *recData){
  CS_L;
  SPI.transfer(0x4B);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  for(int i = 0; i < 8; i++){
    *recData = SPI.transfer(0xFF);
    recData++;
  }
}
static uint8_t W25Q16_BUSY(void) //判断W25Q16是否繁忙函数 繁忙则返回1
{
    uint8_t flag;
    CS_L;
    SPI.transfer(0x05);
    flag=SPI.transfer(0xFF);
    CS_H;
    flag&=0x01;
    return flag;
}

static void W25Q16_Read(uint32_t address,uint8_t *data,uint16_t j)//从W25Q16中的address地址上读取 j个字节的数据保存到 以data为首地址的内存中
{
    uint16_t i;
    while(W25Q16_BUSY());
    CS_L;
    SPI.transfer(0x03);
    SPI.transfer(address>>16);
    SPI.transfer(address>>8);
    SPI.transfer(address);
    for(i=0;i<j;i++)
    {
        *(data+i)=SPI.transfer(0xFF);
    }
    CS_H;
}
static void Write_Enable(void) //写使能函数 对W25Q16进行写操作之前要进行这一步操作
{
    CS_L;
    SPI.transfer(0x06);
    CS_H;
}

static void EraseSector(uint32_t address)
{
    while(W25Q16_BUSY());
    Write_Enable();                              
    CS_L;                                              //置cs低选中
    SPI.transfer(0x20);
    SPI.transfer(address>>16);
    SPI.transfer(address>>8);
    SPI.transfer(address);
    CS_H;
  
}


static void W25Q16_Write(uint32_t address,const uint8_t *data,uint16_t left)
{
    uint16_t max;
    while(left) {
      while(W25Q16_BUSY());//如果芯片繁忙就等在这里
  //    EraseSector(address);
      Write_Enable();//要先写入允许命令
      CS_L;
      SPI.transfer(0x02);
      SPI.transfer(address>>16);
      SPI.transfer(address>>8);
      SPI.transfer(address);
      if(left>256)
        max = 256;
      else 
        max = left;
      for(uint16_t i=0;i<max;i++)
      {
        SPI.transfer(*(data+i));
      }
      CS_H;
      address += 256;
      data += 256;
      left -= 256;
    }  
}


/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t Sd2Card::diskSizeByte(void) {
  return 12*1024*1024;
}
//------------------------------------------------------------------------------
static uint8_t chip_select_asserted = 0;
/** Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 *
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
 
uint8_t Sd2Card::flashtemp[4096] = {0};

uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock) {
 // memset(flashtemp,0xFF,4096);
  uint32_t firstS = (firstBlock/8)*8;
  uint32_t lastS = (lastBlock/8)*8;
  while(lastS >= firstS) {
    if(lastS == firstS) { 
        if (firstBlock-firstS) 
          W25Q.readFlash(firstS<<9, flashtemp, (firstBlock-firstS)<<9);
        if ((firstS+8)-(lastBlock+1))
          W25Q.readFlash((lastBlock+1)<<9, flashtemp+((firstBlock-firstS)<<9), ((firstS+8)-(lastBlock+1))<<9);
    } else {
        if (firstBlock-firstS)
          W25Q.readFlash(firstS<<9, flashtemp, (firstBlock-firstS)<<9);
    }     
    W25Q.eraseSector(firstS<<9,false);
    if (lastS == firstS) { 
        if (firstBlock-firstS) 
          W25Q.writeFlash(firstS<<9, flashtemp, (firstBlock-firstS)<<9);       
        if ((firstS+8)-(lastBlock+1)) 
          W25Q.writeFlash((lastBlock+1)<<9, flashtemp+((firstBlock-firstS)<<9), ((firstS+8)-(lastBlock+1))<<9);        
    } else {
        if (firstBlock-firstS)
          W25Q.writeFlash(firstS<<9, flashtemp, (firstBlock-firstS)<<9);
    }  
    firstS += 8;
    firstBlock = firstS;
  }
  return true;
}
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  The reason for failure
 * can be determined by calling errorCode() and errorData().
 */
uint8_t Sd2Card::init(uint8_t chipSelectPin,uint32_t freq) {
  W25Q.begin(chipSelectPin,freq);
  return true;
}

uint8_t Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
  return readData(block, 0, 512, dst);
}
//------------------------------------------------------------------------------
/**
 * Read part of a 512 byte block from an SD card.
 *
 * \param[in] block Logical block to be read.
 * \param[in] offset Number of bytes to skip at start of block
 * \param[out] dst Pointer to the location that will receive the data.
 * \param[in] count Number of bytes to read
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::readData(uint32_t block,
        uint16_t offset, uint16_t count, uint8_t* dst) {
  if (count == 0) return true;
  if ((count + offset) > 512) {
    return false;
  }
  if (!inBlock_ || block != block_ || offset < offset_) {
    block_ = block;
    offset_ = 0;
    inBlock_ = 1;
  }
  W25Q.readFlash((block<<9)+offset, (uint8_t *)dst, count);
  offset_ += count;
  return true;
}

//------------------------------------------------------------------------------
// wait for card to go not busy
uint8_t Sd2Card::waitNotBusy(unsigned int timeoutMillis) {
  unsigned int t0 = millis();
  unsigned int d;
  do {
    if(!W25Q.waitBusy()) return true;
    d = millis() - t0;
  }while (d < timeoutMillis);
  return false;
}
/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
#if UD_PROTECT_BLOCK_ZERO
  // don't allow write to first block
  if (blockNumber == 0) {
    error(UD_DISK_ERROR_WRITE_BLOCK_ZERO);
    return false;
  }
#endif  // UD_PROTECT_BLOCK_ZERO
  W25Q.readFlash(blockNumber<<9, flashtemp, 512);
  for(uint16_t i=0;i<512;i++) {
      if(flashtemp[i]!=0xFF) {
          W25Q.readFlash((blockNumber/8*8)<<9,flashtemp,4096);
          memset(flashtemp+((blockNumber%8)<<9),0xFF,512);
          W25Q.eraseSector((blockNumber/8*8)<<9,false);
          W25Q.writeFlash((blockNumber/8*8)<<9,flashtemp,4096);
          break;
      }
  }
  W25Q.writeFlash(blockNumber<<9, (uint8_t *)src, 512);
  return true;
}

