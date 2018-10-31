/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <pwd.h>


#include "am_driver_safe/amproto.h"

namespace Husqvarna
{

/////////////////////////////////////////////////////////////////
// Message
/////////////////////////////////////////////////////////////////

static unsigned char MowerCRC[] = 
{ 0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253,  31, 65,
 157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,  130, 220,
 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128, 222, 60,  98,
 190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158, 29,  67,  161, 255,
 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,  102, 229, 187, 89,  7,
 219, 133, 103, 57,  186, 228, 6,   88,  25,  71,  165, 251, 120, 38,  196, 154,
 101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152, 122, 36,
 248, 166, 68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,   231, 185,
 140, 210, 48,  110, 237, 179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205,
 17,  79,  173, 243, 112, 46,  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,
 175, 241, 19,  77,  206, 144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238,
 50,  108, 142, 208, 83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115,
 202, 148, 118, 40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139,
 87,  9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,
 233, 183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
 116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107, 53};


Message::Message(unsigned char type, unsigned char len)
: Length(len), MessageType(type)
{

}

Message::~Message()
{

}

int Message::getMessage(unsigned char *msg)
{
    compoundMessage();
    memcpy(msg, this->msg, totalBytes);
    return totalBytes;
}

void Message::setLength(unsigned char len)
{
	Length = len;	
}

void Message::compoundMessage()
{
    putDataIntoMsg();

    totalBytes = 0;

    *(msg + totalBytes) = 0x02; //STX
    totalBytes += 1;

    memcpy(&msg[totalBytes], &MessageType, 1);
    totalBytes += 1;

    memcpy(&msg[totalBytes], &Length, 1);
    totalBytes += 1;

    memcpy(&msg[totalBytes], &Data, Length);
    totalBytes += Length;

    calculateCRC();
    memcpy(&msg[totalBytes], &CRC, 1);
    totalBytes += 1;

    *(msg + totalBytes) = 0x03; //ETX
    totalBytes += 1;
}

void Message::printMsg()
{
    compoundMessage();
    int i = 0;
    for(; i < totalBytes; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

void Message::calculateCRC()
{
    CRC = 0;
    int i = 0;
    unsigned char *pBuffer = msg + 1; //Start calculate CRC on MessageType
    int len = Length + 2; //DataLength + MessageType + Length

    for (i = 0; i < len; i++) {
        CRC = MowerCRC[ (CRC ^ *pBuffer) ];
        pBuffer++;
    }
}

/////////////////////////////////////////////////////////////////
// SetSimulationData
/////////////////////////////////////////////////////////////////
SetSimulationData::SetSimulationData()
: Message(0x26, 1){
    
    this->group = 9; 
    this->numParams = 0;
}

SetSimulationData::~SetSimulationData(){

}

bool SetSimulationData::addValue(unsigned char type, signed short value)
{
	if (this->numParams >= maxParams)
	{
		return false;
	}
	
	parameterBuffer[this->numParams*3 + 0] = type;
	memcpy((void*)(&parameterBuffer[this->numParams*3 + 1]), (void*)&value, sizeof(value));
	this->numParams++;
	
	return true;
}

void SetSimulationData::putDataIntoMsg(){

	// Copy the "group" into data array
    memcpy((void*)(&Data[0]), (void*)&group, sizeof(group));

	// Copy the params
	int paramSize = 3*numParams;
	memcpy((void*)(&Data[1]), (void*)&parameterBuffer, paramSize);
	
	setLength(1 + paramSize);
	
}


/////////////////////////////////////////////////////////////////
// REMOTE NAVIGATION
/////////////////////////////////////////////////////////////////


// NAVIGATION SETUP

NavigationSetup::NavigationSetup()
: Message(0x38, 1)
{
	sub = 1; // Setup
}

NavigationSetup::~NavigationSetup()
{

}

void NavigationSetup::putDataIntoMsg()
{
    memcpy((void*)(&Data[0]), (void*)&sub, sizeof(sub));
}

// NAVIGATION
Navigation::Navigation()
: Message(0x38, 11)
{
	sub = 3; // Set speed
	no = 0; // sequent number starting from 0
	event = 0; // request control
	status = 0; // cutter disk etc
	left_pwr = 100;
	right_pwr = 100;
}

Navigation::~Navigation()
{

}

void Navigation::requestControl() 
{
		event = 0;
}

void Navigation::releaseControl() 
{
		event = 1;
}

void Navigation::setSpeed(signed short left, signed short right)
{
	left_speed = left;
	right_speed = right;
}

void Navigation::setStatus(unsigned short newStatus)
{
	status = newStatus;
}

unsigned short Navigation::getStatus()
{
	return status;
}

void Navigation::putDataIntoMsg()
{
    memcpy((void*)(&Data[0]), (void*)&sub, sizeof(sub));
    memcpy((void*)(&Data[1]), (void*)&no, sizeof(no));
	memcpy((void*)(&Data[2]), (void*)&left_speed, sizeof(left_speed));
	memcpy((void*)(&Data[4]), (void*)&right_speed, sizeof(right_speed));
    memcpy((void*)(&Data[6]), (void*)&event, sizeof(event));
    memcpy((void*)(&Data[7]), (void*)&status, sizeof(status));
    memcpy((void*)(&Data[9]), (void*)&left_pwr, sizeof(left_pwr));
    memcpy((void*)(&Data[10]), (void*)&right_pwr, sizeof(right_pwr));

	// Increment no for each message
	no++;
}


/////////////////////////////////////////////////////////////////
// LOOP DETECTION (Using SetSettings)
LoopDetection::LoopDetection(unsigned char loopdet)
: Message(0x02, 2){
    
    //Do not change baud
    this->loopdet = loopdet;
}

LoopDetection::~LoopDetection(){
}


void LoopDetection::setLoopDetection(unsigned char loopdet) { 
	this->loopdet = loopdet;
}

void LoopDetection::putDataIntoMsg(){
    Data[0] = 0x88;
    memcpy((void*)(&Data[1]), (void*)&loopdet, sizeof(loopdet));
}

/////////////////////////////////////////////////////////////////
// CUTTING HEIGHT (Using SetSettings)
CuttingHeight::CuttingHeight()
: Message(0x02, 4){

	targetHeight = 20;
    initialHeight = 20;
    action = 0;
}

CuttingHeight::~CuttingHeight(){
}


void CuttingHeight::setCuttingHeight(unsigned char height) { 
	targetHeight = height;
}

void CuttingHeight::putDataIntoMsg(){

    Data[0] = 0x90;
    memcpy((void*)(&Data[1]), (void*)&targetHeight, sizeof(targetHeight));
    memcpy((void*)(&Data[2]), (void*)&initialHeight, sizeof(initialHeight));
    memcpy((void*)(&Data[3]), (void*)&action, sizeof(action));
    
}


/////////////////////////////////////////////////////////////////
// MOWER COMMAND
/////////////////////////////////////////////////////////////////

MowerCommand::MowerCommand()
: Message(0x0e, 1){
	state = 1;
}

MowerCommand::~MowerCommand(){
}

void MowerCommand::stop(){
	state = 0;
}

void MowerCommand::start(){
	state = 1;
}

void MowerCommand::home(){
	state = 2;
}

void MowerCommand::manual(){
	state = 3;
}

void MowerCommand::putDataIntoMsg(){
	Data[0] = state;
}

/////////////////////////////////////////////////////////////////
// PARAMMETERS
/////////////////////////////////////////////////////////////////
Parameter::Parameter(unsigned char id, unsigned char group, unsigned char type)
: Message(id, 2){
    
    this->group = group; 
	this->type = type;
}

Parameter::~Parameter(){

}

void Parameter::setGroupType(unsigned char group, unsigned char type)
{
	this->group = group;
	this->type = type;
}

void Parameter::putDataIntoMsg(){
    memcpy((void*)(&Data[0]), (void*)&group, sizeof(group));
    memcpy((void*)(&Data[1]), (void*)&type, sizeof(type));
}


// GET PARAMETERS
GetParameter::GetParameter(unsigned char group, unsigned char type)
: Parameter(0x08, group, type ){
}

GetParameter::~GetParameter(){
}


// SET PARAMETERS
SetParameter::SetParameter(unsigned char group, unsigned char type, signed short value)
: Parameter(0x0A, group, type ){
    
	setLength(4);
	this->value = value;
}

SetParameter::~SetParameter(){
}

void SetParameter::setValue(signed short value) {
	this->value = value;
}


void SetParameter::putDataIntoMsg(){
	//Call base class function
	Parameter::putDataIntoMsg();

    memcpy((void*)(&Data[2]), (void*)&value, sizeof(value));
}

} // End namespace
