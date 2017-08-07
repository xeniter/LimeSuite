/**
    @file ConnectionRemote.h
    @author Lime Microsystems
    @brief Implementation of EVB7 connection of serial COM port.
*/

#pragma once
#include <ConnectionRegistry.h>
#include <IConnection.h>
#include <LMS64CProtocol.h>
#include <vector>
#include <string>
#include "host.h"
#include "IConnection.h"

namespace lime{

class ConnectionRemote : public IConnection
{
public:
    struct Packet
    {
        uint8_t cmd;
        uint8_t data[64];
    };

    ConnectionRemote(const char *comName);
    ~ConnectionRemote(void);
    bool IsOpen(void);

    //int TransactSPI(const int addr, const uint32_t *writeData, uint32_t *readData, const size_t size) override;

protected:
    enum
    {
        REMOTE_NO_ERROR = 0,
        REMOTE_ERROR,
        ALREADY_LISTENING,
        FAIL_CREATE_SOCKET,
        FAIL_BIND,
        FAIL_LISTEN,
        FAIL_THREAD_CREATE,
        FAIL_CONNECT,
        FAIL_WRITE,
        FAIL_READ,
    };
    int socketFd;
    int TransferData(const uint8_t* data, uint32_t len, uint8_t* response);
    int Connect(const char* ip, uint16_t port);

    /*virtual lime::IConnection::eConnectionType GetType(void)
    {
        return COM_PORT;
    };*/
    int WriteLMS7002MSPI(const uint32_t *writeData, size_t size,unsigned periphID = 0);
    int ReadLMS7002MSPI(const uint32_t *writeData, uint32_t *readData, size_t size, unsigned periphID = 0);
    int ProgramMCU(const uint8_t *buffer, const size_t length, const MCU_PROG_MODE mode, ProgrammingCallback callback = 0);

    //! virtual write function to be implemented by the base class
    virtual int Write(const unsigned char *buffer, int length, int timeout_ms = 100);

    //! virtual read function to be implemented by the base class
    virtual int Read(unsigned char *buffer, int length, int timeout_ms = 100);

    virtual int WriteRegisters(const uint32_t *addrs, const uint32_t *data, const size_t size);

    int WriteRegister(const uint32_t addr, const uint32_t data);

    virtual int ReadRegisters(const uint32_t *addrs, uint32_t *data, const size_t size);
    virtual int UpdateExternalDataRate(const size_t channel, const double txRate_Hz, const double rxRate_Hz);
    int UpdateExternalDataRate(const size_t channel, const double txRate_Hz, const double rxRate_Hz, const double txPhase, const double rxPhase);

private:
    int SPI_write(uint16_t addr, uint16_t value);
    uint16_t SPI_read(uint16_t addr);
    int Open(const char *comName);
    void Close(void);

};

class ConnectionRemoteEntry : public ConnectionRegistryEntry
{
public:
    ConnectionRemoteEntry(void);
    std::vector<ConnectionHandle> enumerate(const ConnectionHandle &hint);
    IConnection *make(const ConnectionHandle &handle);
};

}
