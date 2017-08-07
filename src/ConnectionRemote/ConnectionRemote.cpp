/**
    @file ConnectionRemote.cpp
    @author Lime Microsystems
    @brief Implementation of EVB7 connection of serial COM port.
*/

#include "ConnectionRemote.h"
#include "ErrorReporting.h"
#include <string>
#include "string.h"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "FPGA_common.h"
#ifdef __unix__

#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>

#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#else
#include <Winsock.h>
#endif // LINUX

using namespace std;
using namespace lime;

ConnectionRemote::ConnectionRemote(const char *comName)
{
#ifndef __unix__
    WSADATA wsaData;
    if( int err = WSAStartup(0x0202, &wsaData))
        printf("WSAStartup error %i\n", err);
#endif
    if (this->Open(comName) != 0)
    {
        this->Close();
        fprintf(stderr, "ConnectionRemote(%s) - %s\n", comName, GetLastErrorMessage());
    }
}

ConnectionRemote::~ConnectionRemote(void)
{
    this->Close();
#ifndef __unix__
    WSACleanup();
#endif
}

void ConnectionRemote::Close(void)
{
    if(socketFd)
    {
#ifndef __unix__
        //shutdown(socketFd, SD_BOTH);
        closesocket(socketFd);
#else
        shutdown(socketFd, SHUT_RDWR);
        close(socketFd);
#endif
    }
    socketFd = -1;
}

bool ConnectionRemote::IsOpen(void)
{
    return socketFd >= 0;
}

int ConnectionRemote::Open(const char *comName)
{
    string ip = "127.0.0.1";
    //cout << "Input IP: ";
    //cin >> ip;
	return Connect(ip.c_str(), 5000);
}

int ConnectionRemote::Connect(const char* ip, uint16_t port)
{
    struct sockaddr_in clientService;
    clientService.sin_family = AF_INET;
    clientService.sin_addr.s_addr = inet_addr(ip);
    clientService.sin_port = htons(port);

    socketFd = socket(AF_INET, SOCK_STREAM, 0);
    if(socketFd < 0)
    {
#ifndef __unix__
        printf("socket failed with error: %d\n", WSAGetLastError());
#else
        printf("socket failed with error %i", socketFd);
#endif

        return FAIL_CREATE_SOCKET;
    }

    if(int result = connect(socketFd, (struct sockaddr*)&clientService, sizeof(clientService)) )
    {
#ifndef __unix__
        printf("connect failed with error: %d\n", WSAGetLastError());
        closesocket(socketFd);
#else
        printf("connect failed with error: %d\n", result);
        close(socketFd);
#endif
        return FAIL_CONNECT;
    }
    printf("Connected to %s : %i\n", ip, port);
    return REMOTE_NO_ERROR;
}

int ConnectionRemote::TransferData(const uint8_t* data, uint32_t len, uint8_t* response)
{
    int bytesWritten = 0;
    while(bytesWritten < len)
    {
        int wrBytes = send(socketFd, (char*)data+bytesWritten, len-bytesWritten, 0);
        if(wrBytes < 0)
        {
            printf("Write failed\n");
            return FAIL_WRITE;
        }

        bytesWritten += wrBytes;
    }
    int bytesRead = 0;
    while(bytesRead < len)
    {
        int rdBytes = recv(socketFd, (char*)response+bytesRead, len-bytesRead, 0);
        if(rdBytes < 0)
        {
            printf("Read failed\n");
            return FAIL_READ;
        }
        bytesRead += rdBytes;
    }
    return REMOTE_NO_ERROR;
}
/*
int ConnectionRemote::TransactSPI(const int addr, const uint32_t *writeData, uint32_t *readData, const size_t size)
{
    for(int i=0; i<size; ++i)
    {
        printf("Wr: 0x%04X 0x%04X\n", (writeData[i]>>16) & 0xFFFF, (writeData[i]) & 0xFFFF);
        uint8_t wr[4];
        wr[0] = (writeData[i] >> 24)&0xFF;
        wr[1] = (writeData[i] >> 16)&0xFF;
        wr[2] = (writeData[i] >> 8) &0xFF;
        wr[3] = (writeData[i] >> 0) &0xFF;
        if(readData == nullptr)
            wr[0] |= 0x80;
        uint8_t rd[4];
        TransferData(wr, 4, rd);
        if(readData)
        {
            readData[i] = rd[0]<<24 | rd[1]<<16 | rd[2]<<8 | rd[3];
            printf("RD: 0x%04X\n", (readData[i]) & 0xFFFF);
        }
    }
    return 0;
}*/

//! virtual write function to be implemented by the base class
int ConnectionRemote::Write(const unsigned char *data, int len, int timeout_ms)
{
    int bytesWritten = 0;
    while(bytesWritten < len)
    {
        int wrBytes = send(socketFd, (char*)data+bytesWritten, len-bytesWritten, 0);
        printf("wr %i  %i/%i\n", wrBytes, bytesWritten, len);
        if(wrBytes < 0)
        {
            printf("Write failed\n");
            return FAIL_WRITE;
        }
        bytesWritten += wrBytes;
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }
    return 0;
};

//! virtual read function to be implemented by the base class
int ConnectionRemote::Read(unsigned char *response, int len, int timeout_ms)
{
    int bytesRead = 0;
    while(bytesRead < len)
    {
        int rdBytes = recv(socketFd, (char*)response+bytesRead, len-bytesRead, 0);
        printf("rd %i  %i/%i\n", rdBytes, bytesRead, len);
        if(rdBytes < 0)
        {
            printf("Read failed\n");
            return FAIL_READ;
        }
        bytesRead += rdBytes;
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }
    return 0;
}

int ConnectionRemote::WriteLMS7002MSPI(const uint32_t *writeData, size_t size,unsigned periphID)
{
    Packet msg;
    Packet response;
    for(int i=0; i<size; ++i)
    {
        msg.cmd = CMD_LMS7002_WR;
        memcpy(msg.data, &writeData[i], sizeof(uint32_t));
        if( Write((uint8_t*)&msg, sizeof(msg), 0) == REMOTE_NO_ERROR)
            Read((uint8_t*)&response, sizeof(response));
        else
            return -1;
    }
    return 0;
}

int ConnectionRemote::ReadLMS7002MSPI(const uint32_t *writeData, uint32_t *readData, size_t size, unsigned periphID)
{
    Packet msg;
    Packet response;
    for(int i=0; i<size; ++i)
    {
        msg.cmd = CMD_LMS7002_RD;
        memcpy(msg.data, &writeData[i], sizeof(uint32_t));
        if(Write((uint8_t*)&msg, sizeof(msg), 0) == REMOTE_NO_ERROR)
            Read((uint8_t*)&response, sizeof(response));
        else
            return -1;
        memcpy(&readData[i], &response.data, sizeof(uint32_t));
    }
    return 0;
}

int ConnectionRemote::WriteRegisters(const uint32_t *Addr, const uint32_t *Data, size_t size)
{
    Packet msg;
    Packet response;
    for(int i=0; i<size; ++i)
    {
        msg.cmd = CMD_BRDSPI_WR;
        memcpy(msg.data, &Addr[i], sizeof(uint32_t));
        memcpy(&((uint32_t*)msg.data)[1], &Data[i], sizeof(uint32_t));
        if( Write((uint8_t*)&msg, sizeof(msg), 0) == REMOTE_NO_ERROR)
            Read((uint8_t*)&response, sizeof(response));
        else
            return -1;
    }
    return 0;
}

int ConnectionRemote::ReadRegisters(const uint32_t *addrs, uint32_t *readData, size_t size)
{
    Packet msg;
    Packet response;
    for(int i=0; i<size; ++i)
    {
        msg.cmd = CMD_BRDSPI_RD;
        memcpy(msg.data, &addrs[i], sizeof(uint32_t));
        if(Write((uint8_t*)&msg, sizeof(msg), 0) == REMOTE_NO_ERROR)
            Read((uint8_t*)&response, sizeof(response));
        else
            return -1;
        memcpy(&readData[i], &response.data, sizeof(uint32_t));
    }
    return 0;
}


int ConnectionRemote::ProgramMCU(const uint8_t *buffer, const size_t length, const MCU_PROG_MODE mode, ProgrammingCallback callback)
{
    #ifndef NDEBUG
    auto timeStart = std::chrono::high_resolution_clock::now();
#endif // NDEBUG
    const auto timeout = std::chrono::milliseconds(1000);
    uint16_t i;
    const uint32_t controlAddr = 0x0002;
    const uint32_t statusReg = 0x0003;
    const uint32_t addrDTM = 0x0004; //data to MCU
    const uint16_t EMTPY_WRITE_BUFF = 1 << 0;
    const uint16_t PROGRAMMED = 1 << 6;
    const uint8_t fifoLen = 32;

    //reset MCU, set mode
    SPI_write(controlAddr, 0);
    SPI_write(controlAddr, 2 & 0x3); //SRAM

    for(i=0; i<length; i+=fifoLen)
    {
        //wait till EMPTY_WRITE_BUFF = 1
        bool fifoEmpty = false;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t2 = t1;
        do{
            fifoEmpty = SPI_read(statusReg) & EMTPY_WRITE_BUFF;
            t2 = std::chrono::high_resolution_clock::now();
        }while( !fifoEmpty && (t2-t1)<timeout);

        if(!fifoEmpty)
            return ReportError(ETIMEDOUT, "MCU FIFO full");

        //write 32 bytes into FIFO
        {
            uint8_t j;
            uint16_t addr[fifoLen];
            uint16_t data[fifoLen];
            for(j=0; j<fifoLen; ++j)
            {
                addr[j] = addrDTM;
                data[j] = buffer[i+j];
                SPI_write(addr[j], data[j]);
            }
        }
#ifndef NDEBUG
        printf("MCU programming : %4i/%4i\r", i+fifoLen, length);
#endif
    }

    //wait until programmed flag
    {
        bool programmed = false;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t2 = t1;
        do{
            programmed = SPI_read(statusReg) & PROGRAMMED;
            t2 = std::chrono::high_resolution_clock::now();
        }while( !programmed && (t2-t1)<timeout);

        if(!programmed)
            return ReportError(ETIMEDOUT, "MCU not programmed");
    }
#ifndef NDEBUG
    auto timeEnd = std::chrono::high_resolution_clock::now();
    printf("\nMCU Programming finished, %li ms\n",
            std::chrono::duration_cast<std::chrono::milliseconds>
            (timeEnd-timeStart).count());
#endif //NDEBUG
    return 0;
}

int ConnectionRemote::UpdateExternalDataRate(const size_t channel, const double txRate_Hz, const double rxRate_Hz, const double txPhase, const double rxPhase)
{
    lime::fpga::FPGA_PLL_clock clocks[2];

    if (channel == 2)
    {
        clocks[0].index = 0;
        clocks[0].outFrequency = rxRate_Hz;
        clocks[1].index = 1;
        clocks[1].outFrequency = txRate_Hz;
        return lime::fpga::SetPllFrequency(this, 4, 30.72e6, clocks, 2);
    }

    const float txInterfaceClk = 2 * txRate_Hz;
    const float rxInterfaceClk = 2 * rxRate_Hz;
    //mExpectedSampleRate = rxRate_Hz;
    const int pll_ind = (channel == 1) ? 2 : 0;

    clocks[0].index = 0;
    clocks[0].outFrequency = rxInterfaceClk;
    clocks[1].index = 1;
    clocks[1].outFrequency = rxInterfaceClk;
    clocks[1].phaseShift_deg = rxPhase;
    if (lime::fpga::SetPllFrequency(this, pll_ind+1, rxInterfaceClk, clocks, 2)!=0)
        return -1;

    clocks[0].index = 0;
    clocks[0].outFrequency = txInterfaceClk;
    clocks[1].index = 1;
    clocks[1].outFrequency = txInterfaceClk;
    clocks[1].phaseShift_deg = txPhase;
    if (lime::fpga::SetPllFrequency(this, pll_ind, txInterfaceClk, clocks, 2)!=0)
        return -1;

    return 0;
}

/** @brief Configures FPGA PLLs to LimeLight interface frequency
*/
int ConnectionRemote::UpdateExternalDataRate(const size_t channel, const double txRate_Hz, const double rxRate_Hz)
{
    const float txInterfaceClk = 2 * txRate_Hz;
    const float rxInterfaceClk = 2 * rxRate_Hz;
    const int pll_ind = (channel == 1) ? 2 : 0;
    int status = 0;
    uint32_t reg20;
    const double rxPhC1[] = { 91.08, 89.46 };
    const double rxPhC2[] = { -1 / 6e6, 1.24e-6 };
    const double txPhC1[] = { 89.75, 89.61 };
    const double txPhC2[] = { -3.0e-7, 2.71e-7 };

    const std::vector<uint32_t> spiAddr = { 0x021, 0x022, 0x023, 0x024, 0x027, 0x02A,
                                            0x400, 0x40C, 0x40B, 0x400, 0x40B, 0x400};
    const int bakRegCnt = spiAddr.size() - 4;
    auto info = GetDeviceInfo();
    bool phaseSearch = false;
    /*if (!(mStreamers.size() > channel && (mStreamers[channel]->rxRunning || mStreamers[channel]->txRunning)))
        if (this->chipVersion == 0x3841) //0x3840 LMS7002Mr2, 0x3841 LMS7002Mr3
            if(rxInterfaceClk >= 5e6 || txInterfaceClk >= 5e6)
                phaseSearch = true;
*/
    //mExpectedSampleRate = rxRate_Hz;
    std::vector<uint32_t> dataWr;
    std::vector<uint32_t> dataRd;

    if (phaseSearch)
    {
        dataWr.resize(spiAddr.size());
        dataRd.resize(spiAddr.size());
        //backup registers
        dataWr[0] = (uint32_t(0x0020) << 16);
        ReadLMS7002MSPI(dataWr.data(), &reg20, 1, channel);

        dataWr[0] = (1 << 31) | (uint32_t(0x0020) << 16) | 0xFFFD; //msbit 1=SPI write
        WriteLMS7002MSPI(dataWr.data(), 1, channel);

        for (int i = 0; i < bakRegCnt; ++i)
            dataWr[i] = (spiAddr[i] << 16);
        ReadLMS7002MSPI(dataWr.data(),dataRd.data(), bakRegCnt, channel);
    }

    if(rxInterfaceClk >= 5e6)
    {
        if (phaseSearch)
        {
            const std::vector<uint32_t> spiData = { 0x0E9F, 0x07FF, 0x5550, 0xE4E4, 0xE4E4, 0x0086,
                                                    0x028D, 0x00FF, 0x5555, 0x02CD, 0xAAAA, 0x02ED};
            //Load test config
            const int setRegCnt = spiData.size();
            for (int i = 0; i < setRegCnt; ++i)
                dataWr[i] = (1 << 31) | (uint32_t(spiAddr[i]) << 16) | spiData[i]; //msbit 1=SPI write
            WriteLMS7002MSPI(dataWr.data(), setRegCnt, channel);
        }
        lime::fpga::FPGA_PLL_clock clocks[2];
        clocks[0].index = 0;
        clocks[0].outFrequency = rxInterfaceClk;
        clocks[1].index = 1;
        clocks[1].outFrequency = rxInterfaceClk;
        //if (this->chipVersion == 0x3841)
            clocks[1].phaseShift_deg = rxPhC1[1] + rxPhC2[1] * rxInterfaceClk;
        //else
            //clocks[1].phaseShift_deg = rxPhC1[0] + rxPhC2[0] * rxInterfaceClk;
        if (phaseSearch)
            clocks[1].findPhase = true;
        status = lime::fpga::SetPllFrequency(this, pll_ind+1, rxInterfaceClk, clocks, 2);
    }
    else
        status = lime::fpga::SetDirectClocking(this, pll_ind+1, rxInterfaceClk, 90);

    if(txInterfaceClk >= 5e6)
    {
        if (phaseSearch)
        {
            const std::vector<uint32_t> spiData = {0x0E9F, 0x07FF, 0x5550, 0xE4E4, 0xE4E4, 0x0484};
            WriteRegister(0x000A, 0x0000);
            //Load test config
            const int setRegCnt = spiData.size();
            for (int i = 0; i < setRegCnt; ++i)
                dataWr[i] = (1 << 31) | (uint32_t(spiAddr[i]) << 16) | spiData[i]; //msbit 1=SPI write
            WriteLMS7002MSPI(dataWr.data(), setRegCnt, channel);
        }

        lime::fpga::FPGA_PLL_clock clocks[2];
        clocks[0].index = 0;
        clocks[0].outFrequency = txInterfaceClk;
        clocks[0].phaseShift_deg = 0;
        clocks[1].index = 1;
        clocks[1].outFrequency = txInterfaceClk;
        //if (this->chipVersion == 0x3841)
            clocks[1].phaseShift_deg = txPhC1[1] + txPhC2[1] * txInterfaceClk;
        //else
            //clocks[1].phaseShift_deg = txPhC1[0] + txPhC2[0] * txInterfaceClk;

        if (phaseSearch)
        {
            clocks[1].findPhase = true;
            WriteRegister(0x000A, 0x0200);
        }
        status = lime::fpga::SetPllFrequency(this, pll_ind, txInterfaceClk, clocks, 2);
    }
    else
        status = lime::fpga::SetDirectClocking(this, pll_ind, txInterfaceClk, 90);

    if (phaseSearch)
    {
        //Restore registers
        for (int i = 0; i < bakRegCnt; ++i)
            dataWr[i] = (1 << 31) | (uint32_t(spiAddr[i]) << 16) | dataRd[i]; //msbit 1=SPI write
        WriteLMS7002MSPI(dataWr.data(), bakRegCnt, channel);
        dataWr[0] = (1 << 31) | (uint32_t(0x0020) << 16) | reg20; //msbit 1=SPI write
        WriteLMS7002MSPI(dataWr.data(), 1, channel);
        WriteRegister(0x000A, 0);
    }
    return status;
}

int ConnectionRemote::WriteRegister(const uint32_t addr, const uint32_t data)
{
    return this->WriteRegisters(&addr, &data, 1);
}

int ConnectionRemote::SPI_write(uint16_t addr, uint16_t value)
{
    const uint32_t dataWr = addr << 16 | value;
    return WriteLMS7002MSPI(&dataWr, 1, 0);
}

uint16_t ConnectionRemote::SPI_read(uint16_t addr)
{
    const uint32_t dataWr = addr << 16;
    uint32_t dataRd = 0;
    return ReadLMS7002MSPI(&dataWr, &dataRd, 1, 0);
    return dataRd & 0xFFFF;
}
