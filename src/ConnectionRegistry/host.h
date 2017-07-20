#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <thread>
#include <atomic>

#include <vector>
#include <mutex>

namespace lime
{

class Controller
{
public:
    enum
    {
        REMOTE_NO_ERROR = 0,
        REMOTE_ERROR,
        ALREADY_LISTENING,
        FAIL_CREATE_SOCKET,
        FAIL_BIND,
        FAIL_LISTEN,
        FAIL_THREAD_CREATE
    };

    struct Packet
    {
        uint8_t cmd;
        uint8_t data[64];
    };

    Controller();
    virtual ~Controller();

    int Listen(uint16_t port, bool localhostOnly);
    void Halt();

    std::function<void(const Packet pkt, Packet* response)> spiOp;

protected:
    int socketFd;
    void CloseSocket();
    std::atomic<bool> mRunning;
    std::thread workerThread;
    int ProcessConnections();

};

}
#endif // CONTROLLER_H
