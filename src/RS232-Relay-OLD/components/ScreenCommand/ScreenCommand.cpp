#include "ScreenCommand.hpp"

ScreenCommand::ScreenCommand() {};
ScreenCommand::~ScreenCommand() {};

bool ScreenCommand::parseCommand(char *_commandData, size_t _length) {
    if (_length != 6) {
        return false;
        // Incorrect length of data
    }

    // Check end of string
    if (_commandData[4] != "\\" || _commandData[5] != "r") { 
        return false;
        // Termination incorrect
    }

    if (_commandData[2] != "=" && _commandData[2] != "=") {
        return false;
        // Instruction/response bytes invalid
    }

    // Get ID

    switch (_commandData[3])
    {
    case "+":
        command = Command::DOWN;
        break;

    case "-":
        command = Command::UP;
        break;

    case "?":
        command = Command::QUERY;
        break;
    
    default:
        return false;
        // Command bytes invalid
        break;
    }

    // FIgure out the ID mapping into an int
}

bool ScreenCommand::toString(char *_commandString, size_t _maxLength, bool _isResponse) {
    if (_maxLength < 6) {
        return false;
        // We need at least 8 characters of space in the command string in order to send the command
    }

    if (id < 10) {
        _commandString[0] = "0";
        _commandString[1] = id;
    } else {
        _commandString[0] = "1";
        _commandString[1] = id - 10;
    }

    if (_isResponse) {
        _commandString[2] = "=";
    } else {
        _commandString[2] = ":";
    }
    
    switch (command)
    {
    case Command::DOWN:
        _commandString[3] = "+";
        break;

    case Command::UP:
        _commandString[3] = "-";
        break;

    case Command::QUERY:
        _commandString[3] = "?";
        break;
    
    default:
        break;
    }

    _commandString[4] = "\\";
    _commandString[5] = "r";

    return true;
}