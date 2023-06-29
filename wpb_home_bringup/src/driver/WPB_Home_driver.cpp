/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <driver/WPB_Home_driver.h>
#include <math.h>

static bool bFirstQuart = true;

CWPB_Home_driver::CWPB_Home_driver()
{
   	m_SendBuf = new unsigned char[1024];
	memset(m_SendBuf, 0, 1024);
	memset(m_ParseBuf, 0, 128);
	m_nRecvIndex = 0;
	m_lastRecv = 0;
	m_bFrameStart = false;
	m_nFrameLength = 14;

	for (int i = 0; i < 15;i++)
	{
		arValAD[i] = 0;
	}
	for (int i = 0; i < 6;i++)
	{
		arMotorCurrent[i] = 0;
		arMotorPos[i] = 0;
	}
	arMotorPos[5] = 47998;
	nParseCount = 0;
	fQuatW = 0;
	fQuatX = 0;
	fQuatY = 0;
	fQuatZ = 0;
	
	fGyroX = 0;
	fGyroY = 0;
	fGyroZ = 0;
	
	fAccX = 0;
	fAccY = 0;
	fAccZ = 0;

	fCurYaw = 0;
	fFirstYaw = 0;
	bCalFirstYaw = false; 

	fLinearAccLimit = 0.2;
	fAngularAccLimit = 0.1;
	nSndSrcAngle = 0;
	bSndSrcUpdated = false;

	//mani gripper
	arManiGripperValue[0] = 0;
	arManiGripperPos[0] = 47998;
	arManiGripperValue[1] = 0.034;
	arManiGripperPos[1] = 40000;
	arManiGripperValue[2] = 0.07;
	arManiGripperPos[2] = 30000;
	arManiGripperValue[3] = 0.102;
	arManiGripperPos[3] = 20000;
	arManiGripperValue[4] = 0.133;
	arManiGripperPos[4] = 10000;
	arManiGripperValue[5] = 0.16;
	arManiGripperPos[5] = 0;

	nLastCmdLiftPos = 0;
	nLastCmdGripperPos = 0;
}
    
CWPB_Home_driver::~CWPB_Home_driver()
{
	delete []m_SendBuf;
}


void CWPB_Home_driver::Parse(unsigned char inData)
{
	m_ParseBuf[m_nRecvIndex] = inData;

	if (m_lastRecv == 0x55 && inData == 0xAA && m_bFrameStart == 0)
	{
		m_bFrameStart = 1;
		m_ParseBuf[0] = m_lastRecv;
		m_ParseBuf[1] = inData;
		m_nRecvIndex = 2;
		m_lastRecv = 0x00;
		return;
	}

	if (m_bFrameStart)
	{
		if (m_nRecvIndex == 3)
		{
			m_nFrameLength = inData + 8;
		}

		//put received data into buffer
		m_ParseBuf[m_nRecvIndex] = inData;
		m_nRecvIndex++;

		//receive one frame, invoke ParseFrame to parse
		if (m_nRecvIndex == m_nFrameLength)
		{
			m_DisRecv();
			m_ParseFrame();
			m_bFrameStart = false;
		}

		//receive buffer overflow
		if (m_nRecvIndex >= 128)
		{
			//m_ResetRcvBuf();
			m_bFrameStart = 0;
		}
	}
	else
		m_lastRecv = inData;
}


void CWPB_Home_driver::m_CalSendSum(unsigned char* pNewCmdBuf)
{
	int nLen = pNewCmdBuf[3] + 7;

	pNewCmdBuf[nLen - 1] = 0x00;
	for (int i = 0; i < nLen - 1; i++)
	{
		pNewCmdBuf[nLen - 1] += pNewCmdBuf[i];
	}
}


void CWPB_Home_driver::m_ParseFrame()
{
	nParseCount = 0;
	if (m_ParseBuf[4] == 0x06)	//IO_Input
	{
		unsigned char bIOFlag = 0x01;
		for(int i=0;i<4;i++)
		{
			if((m_ParseBuf[8] & bIOFlag) > 0)
			{
				arValIOInput[i] = 1;
			}
			else
			{
				arValIOInput[i] = 0;
			}
			bIOFlag = bIOFlag <<1;
		}
	}

	if (m_ParseBuf[4] == 0x07)	//AD
	{
		if (m_ParseBuf[8] == 0x01) //AD 0~4
		{
			for (int i = 0; i < 5; i++)
			{
				arValAD[i] = m_USFromChar(&m_ParseBuf[9 + i * 2]);
			}
		}
		if (m_ParseBuf[8] == 0x02) //AD 5~9
		{
			for (int i = 0; i < 5; i++)
			{
				arValAD[i+5] = m_USFromChar(&m_ParseBuf[9 + i * 2]);
			}
		}
		if (m_ParseBuf[8] == 0x03) //AD 10~14
		{
			for (int i = 0; i < 5; i++)
			{
				arValAD[i+10] = m_USFromChar(&m_ParseBuf[9 + i * 2]);
			}
		}
	}

	if (m_ParseBuf[4] == 0x08)	//Motor
	{
		int nCurMotorID = m_ParseBuf[8];
		if (nCurMotorID > 0 && nCurMotorID < 10)
		{
			int nMotorIndex = nCurMotorID - 1;
			arMotorCurrent[nMotorIndex] = m_IntFromChar(&m_ParseBuf[9]);
			arMotorPos[nMotorIndex] = m_IntFromChar(&m_ParseBuf[13]);
		}
	}

	if (m_ParseBuf[4] == 0x09)	//IMU
	{
		if(m_ParseBuf[6] == 0x01)	//GYRO
		{
			fGyroX = (float)m_Piece2int(&m_ParseBuf[7]);
			fGyroY = (float)m_Piece2int(&m_ParseBuf[11]);
			fGyroZ = (float)m_Piece2int(&m_ParseBuf[15]);
		}
		if(m_ParseBuf[6] == 0x02)	//ACC
		{
			fAccX = (float)m_Piece2int(&m_ParseBuf[7]);
			fAccY = (float)m_Piece2int(&m_ParseBuf[11]);
			fAccZ = (float)m_Piece2int(&m_ParseBuf[15]);
		}
		if(m_ParseBuf[6] == 0x03)	//QUAT-W-X
		{
			fQuatW = (float)m_Piece2int(&m_ParseBuf[7]);
			fQuatX = (float)m_Piece2int(&m_ParseBuf[11]);
		}
		if(m_ParseBuf[6] == 0x04)	//QUAT-Y-Z
		{
			fQuatY = (float)m_Piece2int(&m_ParseBuf[7]);
			fQuatZ = (float)m_Piece2int(&m_ParseBuf[11]);
			// yaw: (about Z axis)
    		//fCurYaw = atan2(2*fQuatX*fQuatY - 2*fQuatW*fQuatZ, 2*fQuatW*fQuatW + 2*fQuatX*fQuatX - 1);
			//printf("[CWPB_Home_driver] fYaw = %.2f\n",fCurYaw);
			if(bFirstQuart == true)
			{
				//fFirstYaw = fCurYaw;
				bCalFirstYaw = true;
				bFirstQuart = false;
			}
		}
	}
	if (m_ParseBuf[4] == 0x0a)	//声源定位模块
	{
		nSndSrcAngle = m_USFromChar(&m_ParseBuf[8]);
		bSndSrcUpdated = true;
		//printf("[CWPB_Home_driver]Sound source angle = %d\n",nSndSrcAngle);
	}
}


void CWPB_Home_driver::m_DisRecv()
{
	
}



int CWPB_Home_driver::GenCmd(int inBuffOffset, int inDevID, int inModule, int inMethod, unsigned char* inData, int inDataLen)
{
	int nCmdLen = 0;

	int nTailIndex = inBuffOffset + 7 + inDataLen;
	if (nTailIndex >= 1024)
	{
		return nCmdLen;
	}

	unsigned char* pNewCmd = m_SendBuf + inBuffOffset;
	pNewCmd[0] = 0x55;
	pNewCmd[1] = 0xaa;
	pNewCmd[2] = (unsigned char)inDevID;
	pNewCmd[3] = (unsigned char)inDataLen;
	pNewCmd[4] = (unsigned char)inModule;
	pNewCmd[5] = (unsigned char)inMethod;
	memcpy(&pNewCmd[6], inData, inDataLen);

	m_CalSendSum(pNewCmd);

	nCmdLen = inDataLen + 7;
	return nCmdLen;
}


void CWPB_Home_driver::SendMotors(int inMotor1, int inMotor2, int inMotor3, int inMotor4)
{
	static unsigned char arMotorSpeedData[12];
	arMotorSpeedData[0] = 0;
	arMotorSpeedData[1] = 0x01;
	m_Split4Bytes(arMotorSpeedData + 2, inMotor1);
	arMotorSpeedData[6] = 0;
	arMotorSpeedData[7] = 0x02;
	m_Split4Bytes(arMotorSpeedData + 8, inMotor2);
	int nFirstCmdLenght = GenCmd(0, 0x40, 0x08, 0x60, arMotorSpeedData, 12);

	arMotorSpeedData[0] = 0;
	arMotorSpeedData[1] = 0x03;
	m_Split4Bytes(arMotorSpeedData + 2, inMotor3);
	arMotorSpeedData[6] = 0;
	arMotorSpeedData[7] = 0x04;
	m_Split4Bytes(arMotorSpeedData + 8, inMotor4);
	int nSecondCmdLenght = GenCmd(nFirstCmdLenght, 0x40, 0x08, 0x60, arMotorSpeedData, 12);

	int nTotalLen = nFirstCmdLenght + nSecondCmdLenght;
	Send(m_SendBuf, nTotalLen);
}

static float vkx = (float)sqrt(3.0f)*0.5;
static float fKLinearMotorK = 1953.125;
static float fKAngularMotorK = 371.875;
void CWPB_Home_driver::Velocity(float inX, float inY, float inAngular)
{
	//upward backward
	int nVectorX = inX * fKLinearMotorK;

	//shift left right
	int nVectorY = inY * fKLinearMotorK;

	//Turning 
	int nVectorTurn = inAngular * fKAngularMotorK;

	//Speed Value
	int nMotorToSend[4];
	nMotorToSend[0] = 0;	//left front
	nMotorToSend[0] = -vkx*nVectorX + nVectorY*0.5 + nVectorTurn;

	nMotorToSend[1] = 0;	//right front
	nMotorToSend[1] = vkx*nVectorX + nVectorY*0.5 + nVectorTurn;

	nMotorToSend[2] = 0;	//back 
	nMotorToSend[2] = -nVectorY + nVectorTurn;

	nMotorToSend[3] = 0;	//NC

	SendMotors(nMotorToSend[0],nMotorToSend[1],nMotorToSend[2],nMotorToSend[3]);
}

float CWPB_Home_driver::GetYaw()
{
	float diffYaw = fCurYaw - fFirstYaw;
	return diffYaw;
}


void CWPB_Home_driver::MotorCmd(int inMethod, int inID1, int inValue1, int inID2, int inValue2)
{
	static unsigned char arMotorSpeedData[12];
	m_Split2Bytes(arMotorSpeedData,inID1);
	m_Split4Bytes(arMotorSpeedData + 2, inValue1);

	m_Split2Bytes(arMotorSpeedData + 6, inID2);
	m_Split4Bytes(arMotorSpeedData + 8, inValue2);
	int nCmdLenght = GenCmd(0, 0x40, 0x08, inMethod, arMotorSpeedData, 12);
	Send(m_SendBuf, nCmdLenght);
}


void CWPB_Home_driver::MotorCmd2(int inMethod, int inID1, int inValue1_1, int inValue1_2, int inID2, int inValue2_1, int inValue2_2)
{
	static unsigned char arMotorCmdData[20];
	m_Split2Bytes(arMotorCmdData, inID1);
	m_Split4Bytes(arMotorCmdData + 2, inValue1_1);
	m_Split4Bytes(arMotorCmdData + 6, inValue1_2);

	m_Split2Bytes(arMotorCmdData + 10, inID2);
	m_Split4Bytes(arMotorCmdData + 12, inValue2_1);
	m_Split4Bytes(arMotorCmdData + 16, inValue2_2);
	int nCmdLenght = GenCmd(0, 0x40, 0x08, inMethod, arMotorCmdData, 20);
	Send(m_SendBuf, nCmdLenght);
}

void CWPB_Home_driver::ManiPos(float inHeight, int inRaiseSpeed, float inGripper, int inGripperSpeed)
{
	if(inHeight > 0.01)
	{
		MotorCmd2(0x63, 5, inRaiseSpeed, inHeight, 6, inGripperSpeed, inGripper);
	}
	else
	{
		// 折叠
		MotorCmd2(0x64, 5, inRaiseSpeed, inHeight, 6, inGripperSpeed, inGripper);
	}
}

static float nMinHeight = 0.493;
static float nMaxHeight = 1.036;
void CWPB_Home_driver::ManiCmd(float inManiLift, float inLiftSpeed, float inManiGripper, float inGripperSpeed)
{
	int nLiftPos = nLastCmdLiftPos;
	float tmpLift = inManiLift;
	unsigned char ctrl_code = 0x63;
	if (inManiLift < 0)
	{
		nLiftPos = nLastCmdLiftPos;
		if(nLiftPos < 11201)
		{
			ctrl_code = 0x64;
		}
	}
	else
	{
		if (tmpLift < nMinHeight)
		{
			tmpLift = 0;
			ctrl_code = 0x64;
		}
		if (tmpLift > nMaxHeight)
		{
			tmpLift = nMaxHeight;
		}
		nLiftPos = (tmpLift - nMinHeight) * 76660 + 11201;
	}
	int nLiftSpeed = inLiftSpeed * 1960;
	if(nLiftSpeed > 4000)
	{
		nLiftSpeed = 4000;
	}

	int nGripperPos = nLastCmdGripperPos;
	if (inManiGripper >= 0)
	{
		float tmpGripper = inManiGripper;
		nGripperPos = arManiGripperPos[0];
		if (tmpGripper < arManiGripperValue[0])
		{
			tmpGripper = arManiGripperValue[0];
			nGripperPos = arManiGripperPos[0];
		}
		if (tmpGripper > arManiGripperValue[5])
		{
			tmpGripper = arManiGripperValue[5];
			nGripperPos = arManiGripperPos[5];
		}
		for (int i = 0; i < 5; i++)
		{
			if (tmpGripper == arManiGripperValue[i + 1])
			{
				nGripperPos = arManiGripperPos[i + 1];
				break;
			}
			if (tmpGripper > arManiGripperValue[i] && tmpGripper < arManiGripperValue[i+1])
			{
				float fKG = (arManiGripperPos[i] - arManiGripperPos[i + 1]) / (arManiGripperValue[i] - arManiGripperValue[i + 1]);
				nGripperPos = arManiGripperPos[i] + (tmpGripper - arManiGripperValue[i])*fKG;
				break;
			}
		}
	}

	int nGripperSpeed = inGripperSpeed * 178;

	nLastCmdLiftPos = nLiftPos;
	nLastCmdGripperPos = nGripperPos;
	
	MotorCmd2(ctrl_code, 5, nLiftSpeed, nLiftPos, 6, nGripperSpeed, nGripperPos);
}

bool CWPB_Home_driver::ManiArrived()
{
	bool bArrived = true;
	if(abs (arMotorPos[4] - nLastCmdLiftPos) > 100 )
	{
		bArrived = false;
	}
	if(abs (arMotorPos[5] - nLastCmdGripperPos) > 100 )
	{
		bArrived = false;
	}
	return bArrived;
}

void CWPB_Home_driver::Output(int* inValue)
{
	//55 AA 40 01 06 70 输出IO 校验和
	unsigned char cOutputData = 0;
	for(int i=7;i>=0;i--)
	{
		if(inValue[i] > 0)
		{
			cOutputData |= 0x01;
		}
		if(i > 0)
			cOutputData = cOutputData << 1;
	}
	int nCmdLenght = GenCmd(0, 0x40, 0x06, 0x70, &cOutputData, 1);
	Send(m_SendBuf, nCmdLenght);
}

void CWPB_Home_driver::QuerySoundLocal()
{
	//55 AA 40 00 0A 09 51
	int nCmdLenght = GenCmd(0, 0x40, 0x0a, 0x09, NULL, 0);
	Send(m_SendBuf, nCmdLenght);
}