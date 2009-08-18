#ifndef _CHESSBOARD_TRACKER_H_
#define _CHESSBOARD_TRACKER_H_

#define DLLEXPORT __declspec(dllexport)
#define DLLIMPORT __declspec(dllimport)

#include <cv.h>
#include "Tracker.h"

namespace windage
{
	class DLLEXPORT ChessboardTracker:public Tracker
	{
	private:
		CvPoint2D32f* chessboardPoints;

		int chessboardWidth;
		int chessboardHeight;
		double fieldSize;

		void Release();

		inline void SetChessboard(int width, int height, double size){this->chessboardWidth=width, this->chessboardHeight=height; this->fieldSize=size;};
		inline int GetPointCount(){return (this->chessboardWidth-1) * (this->chessboardHeight-1);};

		int FindChessBoardCorner(IplImage* inputImage);
		bool UpdateExtrinsicParams();

	public:
		ChessboardTracker();
		virtual ~ChessboardTracker();

		void Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, int chessboardWidth, int chessboardHeight, double fieldSize);

		int UpdateCameraPose(IplImage* grayImage);
		void DrawDebugInfo(IplImage* colorImage);
	};
}

#endif