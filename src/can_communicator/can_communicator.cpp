#include <unistd.h>

#include "can_communicator.hpp"

STATUS CanCommunicator::open()
{
    if (this->isOpen)
        return SUCCESS;

    this->canHandler = LINUX_CAN_Open(this->devicePath.c_str(), this->flag);
    if (!this->canHandler)
    {
        std::cerr << "Error opening CAN device" << std::endl;
        return FAIL;
    }

    this->isOpen = true;
    return SUCCESS;
}

STATUS CanCommunicator::close()
{
    STATUS status = 0;

    if (this->isOpen)
    {
        status = CAN_Close(this->canHandler);
        if (status != 0)
            return FAIL;

        this->isOpen = false;
        this->canHandler = nullptr;
    }

    return SUCCESS;
}

STATUS CanCommunicator::init()
{
    if (!this->isOpen)
        return FAIL;

    STATUS status = CAN_Init(this->canHandler, this->btr0btr1, this->canMsgType);
    if (status != 0)
        return FAIL;

    return SUCCESS;
}

STATUS CanCommunicator::get_version(std::string *info)
{
    if (!this->isOpen)
        return FAIL;

    char version_info[64];
    STATUS status = CAN_VersionInfo(this->canHandler, version_info);

    if (status != 0)
        return FAIL;

    *info = version_info;

    return SUCCESS;
}

STATUS CanCommunicator::get_status(STATUS status)
{
    if (!this->isOpen)
        return FAIL;

    status = CAN_Status(this->canHandler);

    return SUCCESS;
}

STATUS CanCommunicator::send(DWORD id, std::vector<BYTE> &msg)
{
    if (!this->isOpen)
        return FAIL;
    if (msg.size() > 8)
        return FAIL;

    // construct the msg
    TPCANMsg send_msg;
    send_msg.ID = id;

    int indx = 0;
    for (auto item : msg)
    {
        send_msg.DATA[indx++] = item;
    }

    send_msg.LEN = indx;
    send_msg.MSGTYPE = this->canMsgType;

    // send the msg
    STATUS status = CAN_Write(this->canHandler, &send_msg);

    usleep((__useconds_t)(400));

    if (status)
        return FAIL;

    return SUCCESS;
}

STATUS CanCommunicator::receive(TPCANMsg &msg)
{
    if (!this->isOpen)
        return FAIL;

    TPCANRdMsg read_msg;
    STATUS status = LINUX_CAN_Read(this->canHandler, &read_msg);

    if (status)
        return FAIL;

    msg = read_msg.Msg;

    return SUCCESS;
}

STATUS CanCommunicator::receive_timeout(TPCANMsg &msg, int ms)
{
    if (!this->isOpen)
        return FAIL;

    TPCANRdMsg read_msg;
    STATUS status = LINUX_CAN_Read_Timeout(this->canHandler, &read_msg, ms);

    if (status)
        return FAIL;

    msg = read_msg.Msg;

    return SUCCESS;
}