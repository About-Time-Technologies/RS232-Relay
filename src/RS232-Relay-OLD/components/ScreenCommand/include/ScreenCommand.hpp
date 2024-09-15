#pragma once

#include "stdint.h"
#include "string.h"

#include ""

class ScreenCommand {
public:
    enum class Command { DOWN, UP, QUERY };


    ScreenCommand();
    ~ScreenCommand();

    bool parseCommand(char *_commandData, size_t _length); // Takes command bytes and configures this class based on the incoming data
    bool toString(char *_commandString, size_t _maxLength, bool isResponse);

    uint8_t id = 0;
    Command command;
private:
    const char* TAG = "ScreenCommand";
};