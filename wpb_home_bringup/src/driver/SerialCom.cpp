/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Zhang Wanjie.
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

#include <driver/SerialCom.h>

CSerialCom::CSerialCom()
{
    buffer = new char[1024*1024];

}

CSerialCom::~CSerialCom()
{
    delete[] buffer;
}

void CSerialCom::Open(const char* inDev, int inBaudRate)
{
    printf("[SerialCom]InitSerialCom (%s)...\n",inDev);
    /*以读写方式打开串口*/
    fdCom = open( inDev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fdCom)
    {
        /* 不能打开串口 */
        perror("打开串口错误！");
        return;
    }

	int status = 0;
    struct termios Opt;
	memset(&Opt,0,sizeof(Opt));

	unsigned int nBaudRateVal = GetBaudRate(inBaudRate);

    cfsetispeed(&Opt, nBaudRateVal);
	cfsetospeed(&Opt, nBaudRateVal);
    Opt.c_cflag |= (CLOCAL | CREAD);
	
    /*设置端口参数*/
    Opt.c_cflag &= ~CSIZE;
    Opt.c_cflag |= CS8;
    Opt.c_cflag &= ~PARENB;   /* Clear parity enable */
    Opt.c_iflag &= ~INPCK;     /* Enable parity checking */
	Opt.c_cflag &= ~CSTOPB;
	status = tcsetattr(fdCom, TCSANOW, &Opt);
	if(status != 0)
	{
		perror("设置波特率错误！");
		return;
	}

	//printf("pthread_create(&hComThread)\n");
    int res;
    //res = pthread_create(&hComThread, NULL, threadSerialCom_func, this);
}
 
#define ITOU( _val) case _val:  return B##_val
unsigned int CSerialCom::GetBaudRate(int inBaudRate)
{
	switch (inBaudRate) 
	{
        ITOU(1200);
        ITOU(1800);
        ITOU(2400);
        ITOU(4800);
        ITOU(9600);
        ITOU(19200);
        ITOU(38400);
        ITOU(57600);
        ITOU(115200);
        ITOU(230400);
        ITOU(460800);
        ITOU(500000);
        ITOU(576000);
        ITOU(921600);
        ITOU(1000000);
        ITOU(1152000);
        ITOU(1500000);
        ITOU(2000000);
    }
    return -1;
}

void* CSerialCom::threadSerialCom_func(void* inParam)
{
    printf ("[SerialCom]threadSerialCom_func start...\n");
    CSerialCom* pCom = (CSerialCom*)inParam;
    int nRead = 0;
    while (1) //循环读取数据
    {
 //       printf ("[SerialCom]Ready to Read data...\n");
        while((nRead = read(pCom->fdCom, pCom->buffer, 1024))>0)
        {
			int i =0;
//          printf("[Com]Read %d bytes\n",nRead);
// 
// 			printf("%d [COM_RecvRaw] ",nRead);
// 
    		// for(i=0;i<nRead;i++)
    		// {
    		//     printf("%.2X ",pCom->buffer[i]);
    		// }
    		// printf("\n");

            for(i=0;i<nRead;i++)
            {
                pCom->Parse((unsigned char)pCom->buffer[i]);
            }
        }
	//printf("while 1\n");
    }
	close(pCom->fdCom);

    printf("[SerialCom]threadSerialCom_func exit\n");
    pthread_exit(NULL);
}

int CSerialCom::m_Piece2int(unsigned char *inTarg)
{
	int ret;
	ret = inTarg[0];
	ret <<= 8;
	ret |= ((int)inTarg[1]&0x00ff);
	ret <<= 8;
	ret |= ((int)inTarg[2]&0x00ff);
	ret <<= 8;
	ret |= ((int)inTarg[3]&0x00ff);
	return ret;
}

int CSerialCom::m_Piece2short(unsigned char *inTarg)
{
	int ret;
	ret = inTarg[0];
	ret <<= 8;
	ret |= ((int)inTarg[1]&0x00ff);
	return ret;
}

void CSerialCom::m_Split4Bytes(unsigned char *inTarg, int inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned int temp;
	memcpy(&temp, &inSrc, sizeof(int));
	inTarg[3] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[2] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[1] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[0] = (unsigned char)temp & 0x00ff;
}

unsigned short CSerialCom::m_USFromChar(unsigned char *inBuf)
{
	unsigned short wtemp;
	wtemp = 0;
	wtemp = *(inBuf);

	wtemp <<= 8;
	wtemp |= *(inBuf + 1);

	return wtemp;
}


int CSerialCom::m_IntFromChar(unsigned char *inBuf)
{
	static int itemp;
	itemp = 0;
	itemp = *(inBuf);

	itemp <<= 8;
	itemp |= *(inBuf + 1);

	itemp <<= 8;
	itemp |= *(inBuf + 2);

	itemp <<= 8;
	itemp |= *(inBuf + 3);

	return itemp;
}

void CSerialCom::m_Split2Bytes(unsigned char *inTarg, short inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	unsigned short temp;
	memcpy(&temp, &inSrc, sizeof(short));
	inTarg[1] = (unsigned char)temp & 0x00ff;

	temp >>= 8;

	inTarg[0] = (unsigned char)temp & 0x00ff;
}

int CSerialCom::ReadNewData()
{
	int nRead = read(fdCom, buffer, 1024);
	for(int i=0;i<nRead;i++)
	{
		Parse((unsigned char)buffer[i]);
	}
	return nRead;
}

int CSerialCom::Send(unsigned char* inBuf,int inLen)
{
    /*printf("[COM_Send] ");
    int i =0;
    for(i=0;i<inLen;i++)
    {
        printf("%.2X ",inBuf[i]);
    }
    printf("\n");*/
    int nWrite = write(fdCom, inBuf ,inLen);
	return nWrite;
}
