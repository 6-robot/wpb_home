#include "wpbh_local_planner/CLidarAC.h"

static double y_sin[180];
static double x_cos[180];
static float max_vy = 0.5;
CLidarAC::CLidarAC()
{
    for(int i=0;i<180;i++) 
    {
        arFrontRanges[i] = i;
		x_cos[i] = cos(i*3.14159/180);
		y_sin[i] = sin(i*3.14159/180);
        arPnt_x[i] = arFrontRanges[i] * x_cos[i];
		arPnt_y[i] = arFrontRanges[i] * y_sin[i];
        arACPnt[i] = false;
    }
	kx = 0.005;
	ky = 0.1;
	fVx = 0;
	fVy = 0;
    nLeftIndex = nRightIndex = -1;
    nBlobDist = 5;
    nIgnDist = 22;
    nACWidth = 30;
	nACDist = 100;
}

CLidarAC::~CLidarAC()
{
}

static int CalDist(int inX1, int inY1, int inX2, int inY2)
{
    int res = sqrt((inX1-inX2)*(inX1-inX2) + (inY1-inY2)*(inY1-inY2));
    return res;
}

int CLidarAC::GetMinIndex(int inBeginIndex, int inEndIndex)
{
    int res = inBeginIndex;
    int nMinDist = arFrontRanges[inBeginIndex];
    for(int i=inBeginIndex+1;i<inEndIndex;i++) 
    {
        if(arACPnt[i] == true && arFrontRanges[i] < nMinDist)
        {
            nMinDist = arFrontRanges[i];
            res = i;
        }
    }
    return res;
}

void CLidarAC::SetRanges(float* inData)
{
    for(int i=0;i<180;i++)
    {
        arFrontRanges[i] = inData[270-i];
        arPnt_x[i] = arFrontRanges[i] * x_cos[i];
		arPnt_y[i] = arFrontRanges[i] * y_sin[i];
    }
}

bool CLidarAC::OutLine()
{
    bool res = true;
    // 有效障碍点
    for(int i=0;i<180;i++) 
    {
        arACPnt[i] = false;
        if(arFrontRanges[i] > nIgnDist )
        {
            if(
                arPnt_x[i] > -nACWidth &&
                arPnt_x[i] < nACWidth &&
                arPnt_y[i] > 0 &&
                arPnt_y[i] < nACDist
            )
            {
                arACPnt[i] = true;
            }
        }
    }

    // 连通检测
    bool bBlobStart = false;
    int nBeginIndex = 0;
    int nEndIndex = 0;
    int nLastRange = 0;
    arBlobIndex.clear();
    for(int i=0;i<180;i++) 
    {
        if(arACPnt[i] == true)
        {
            if(bBlobStart == false)
            {
                nBeginIndex = i;
                bBlobStart = true;
            }
            else
            {
                if(abs(arFrontRanges[i] - nLastRange) > nBlobDist)
                {
                    nEndIndex = i-1;
                    if(nEndIndex > nBeginIndex)
                    {
                        int nMinIndex = GetMinIndex(nBeginIndex , nEndIndex);
                        arBlobIndex.push_back(nMinIndex);
                    }
                    nBeginIndex = i;
                }
            }
            nLastRange = arFrontRanges[i];
        }
        else
        {
            if(bBlobStart == true)
            {
                bBlobStart= false;
                nEndIndex = i-1;
                if(nEndIndex > nBeginIndex)
                {
                    int nMinIndex = GetMinIndex(nBeginIndex , nEndIndex);
                    arBlobIndex.push_back(nMinIndex);
                }
            }
        }
    }

    //左右
    nLeftIndex = nRightIndex = -1;
    int nMinLeft = 1000;
    int nMinRight = 1000;
    int nBlobNum = arBlobIndex.size();
    for(int i=0;i<nBlobNum;i++)
    {
        if(arBlobIndex[i] < 90)
        {
            if(arFrontRanges[arBlobIndex[i]] < nMinLeft)
            {
                nMinLeft = arFrontRanges[arBlobIndex[i]];
                nLeftIndex = arBlobIndex[i];
            }
        }
        else
        {
            if(arFrontRanges[arBlobIndex[i]] < nMinRight)
            {
                nMinRight = arFrontRanges[arBlobIndex[i]];
                nRightIndex = arBlobIndex[i];
            }
        }
    }

    // 速度
    if(nLeftIndex < 0 && nRightIndex < 0)
    {
        fVx = fVy = 0;
    }
    else
    {
        int nMinX = nACDist;
        if(nLeftIndex >= 0 )
        {
            if(arPnt_y[nLeftIndex] < nMinX)
            {
                nMinX = arPnt_y[nLeftIndex];
            }
        }
         if(nRightIndex >= 0 )
        {
            if(arPnt_y[nRightIndex] < nMinX)
            {
                nMinX = arPnt_y[nRightIndex];
            }
        }
        fVx = -(nACDist - nMinX)*kx;
        if(nMinX < 40)
        {
            res = false;
        }

        float tmpLeftV = 0;
        float tmpRightV = 0;
        if(nLeftIndex >= 0 )
        {
            tmpLeftV = (arPnt_x[nLeftIndex] + nACWidth) * ky / arPnt_y[nLeftIndex];
        }
        if(nRightIndex >= 0 )
        {
            tmpRightV = -(nACWidth - arPnt_x[nRightIndex]) * ky / arPnt_y[nRightIndex];
        }
        fVy = tmpLeftV + tmpRightV;
        if(fVy < -max_vy)
        {
            fVy = -max_vy;
        }
        if(fVy > max_vy)
        {
            fVy = max_vy;
        }
    }

    return res;
}