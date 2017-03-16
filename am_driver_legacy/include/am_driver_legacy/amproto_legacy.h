/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef __AM_PROTO_LEGACY_H__
#define __AM_PROTO_LEGACY_H__

namespace Husqvarna
{

/////////////////////////////////////////////////////////////////
// Defines / types
/////////////////////////////////////////////////////////////////
#define SWAP_16(x) ((x >> 8) | (x << 8))

#define SWAP_32(x) (((x>>24)&0xff) |    \
                    ((x<<8)&0xff0000) | \
                    ((x>>8)&0xff00) |   \
                    ((x<<24)&0xff000000)

typedef unsigned int UInt32;
typedef unsigned short UInt16;

#define AM_PROTO_MAX_LENGTH 260

/////////////////////////////////////////////////////////////////
// Message
/////////////////////////////////////////////////////////////////
class Message
{
public:
    Message(unsigned char type, unsigned char len);
    ~Message();

    int getMessage(unsigned char* msg);
    void printMsg();

protected:
    void setLength(unsigned char len);
    void calculateCRC();
    void compoundMessage();
    virtual void putDataIntoMsg() = 0;

protected:
    unsigned char Data[255];

private:
    unsigned char msg[AM_PROTO_MAX_LENGTH]; // each message consists of up to 260 bytes
    unsigned char MessageType;
    unsigned char CRC;
    unsigned char Length;

    unsigned char totalBytes;
};

/////////////////////////////////////////////////////////////////
// Remote Navigation
/////////////////////////////////////////////////////////////////
class NavigationSetup : public Message
{
public:
    NavigationSetup();
    ~NavigationSetup();

protected:
    virtual void putDataIntoMsg();

private:
    unsigned char sub; // Subcommand, 0 = set speed, 1 = setup
};

class Navigation : public Message
{
public:
    Navigation();
    ~Navigation();

    //	void setMode(unsigned char mode); //ok, event, but mode sounds better..
    void requestControl();
    void releaseControl();
    void setSpeed(signed short left, signed short right);
    void setStatus(unsigned short newStatus);
    unsigned short getStatus();

protected:
    virtual void putDataIntoMsg();

private:
    unsigned char sub;        // Subcommand, 0 = set speed, 1 = setup
    unsigned char no;         // Serial number
    signed short left_speed;  // Desired wheel speed in mm/s (-1000 to +1000)
    signed short right_speed; // Desired wheel speed in mm/s (-1000 to +1000)
    unsigned char event;      // SCCS.RB Event, 0 = Request Control, 1 = Release Control, 2 = Follow boundary wireyy
    unsigned short status;    // SCCS.RB Status.;
    unsigned char left_pwr;
    unsigned char right_pwr;
};

/////////////////////////////////////////////////////////////////
// CUTTING HEIGHT (Using SetSettings)
/////////////////////////////////////////////////////////////////
class CuttingHeight : public Message
{
public:
    CuttingHeight();
    ~CuttingHeight();

    void setCuttingHeight(unsigned char height);

protected:
    void putDataIntoMsg();

private:
    unsigned char targetHeight;
    unsigned char initialHeight;
    unsigned char action;
};
}

#endif //__AM_PROTO_LEGACY_H__
