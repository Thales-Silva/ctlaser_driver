#ifndef CTLASER_WORDS_HPP
#define CTLASER_WORDS_HPP

enum CtlaserCommand
{
    READ_TEMP_PROCESS = 0x01,
    READ_TEMP_HEAD = 0x02,
    READ_TEMP_BOX = 0x03,
    READ_TEMP_ACT = 0x81,
    READ_EPSILON = 0x04,
    SET_EPSILON = 0x84,
    READ_TRANSMISSION = 0x05,
    SET_TRANSMISSION = 0x85,
    READ_SPOT_LASER = 0x25,
    SET_SPOT_LASER = 0xA5
};

#endif // CTLASER_WORDS_HPP
