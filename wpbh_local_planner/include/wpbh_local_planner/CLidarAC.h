
#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace std;

class CLidarAC
{
public:
	CLidarAC();
	virtual ~CLidarAC();
	void SetRanges(float* inData);
	bool OutLine();
	int GetMinIndex(int inBeginIndex, int inEndIndex);
	vector<int> arBlobIndex;

	int arFrontRanges[180];
	int arPnt_x[180];
	int arPnt_y[180];
	bool arACPnt[180];
	int nACWidth;
	int nACDist;
	int nIgnDist;
	int nBlobDist;
	int nLeftIndex;
	int nRightIndex;
	float kx;
	float ky;
	float fVx;
	float fVy;
};