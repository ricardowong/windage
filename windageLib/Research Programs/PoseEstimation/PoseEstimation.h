#include <cv.h>

class HomographyLMmethod
{
private:
	int size;
	int iteration;
	int maxIteration;

	int lambdaLg10;

	// estimation prameter
	CvMat* JtJ;
    CvMat* JtJN;
    CvMat* JtJV;
    CvMat* JtJW;
	CvMat* JtErr;

	CvMat* prevModel;
	CvMat* model;

	void Step();
	double CalcHomographyError(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* h);
public:
	HomographyLMmethod()
	{
		// const
		size = 8;

		JtJ = cvCreateMat( size, size, CV_64F );
		JtJN = cvCreateMat( size, size, CV_64F );
		JtJV = cvCreateMat( size, size, CV_64F );
		JtJW = cvCreateMat( size, 1, CV_64F );
		JtErr = cvCreateMat( size, 1, CV_64F );

		prevModel = cvCreateMat( size, 1, CV_64F );
		model = cvCreateMat( size, 1, CV_64F );

		lambdaLg10 = -3;
		iteration = 0;
		maxIteration = 30;
	}

	~HomographyLMmethod()
	{
		cvReleaseMat(&JtJ);
		cvReleaseMat(&JtJN);
		cvReleaseMat(&JtJV);
		cvReleaseMat(&JtJW);
		cvReleaseMat(&JtErr);

		cvReleaseMat(&model);
	}

	inline void SetMaxIteration(int iteration){this->maxIteration = iteration;};

	void CalcHomography(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* homography);
};

double HomographyLMmethod::CalcHomographyError(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* h)
{
	const int count = refPoints->size();
	const CvPoint2D32f* M = (const CvPoint2D32f*)&(*refPoints)[0];
    const CvPoint2D32f* m = (const CvPoint2D32f*)&(*scePoints)[0];

	double _errNorm = 0;
	for(int i=0; i<count; i++)
	{
		double Mx = M[i].x;
		double My = M[i].y;

		double ww = 1./(h[6]*Mx + h[7]*My + 1.);
		double _xi = (h[0]*Mx + h[1]*My + h[2])*ww;
		double _yi = (h[3]*Mx + h[4]*My + h[5])*ww;
		double err[] = { _xi - m[i].x, _yi - m[i].y };

		double J[][8] =
		{
			{ Mx*ww, My*ww, ww, 0, 0, 0, -Mx*ww*_xi, -My*ww*_xi },
			{ 0, 0, 0, Mx*ww, My*ww, ww, -Mx*ww*_yi, -My*ww*_yi }
		};

		for(int j=0; j<8; j++)
		{
			for(int k=j; k<8; k++ )
				JtJ->data.db[j*8+k] += J[0][j]*J[0][k] + J[1][j]*J[1][k];
			JtErr->data.db[j] += J[0][j]*err[0] + J[1][j]*err[1];
		}
		_errNorm += err[0]*err[0] + err[1]*err[1];
	}
	return _errNorm;
}

void HomographyLMmethod::CalcHomography(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* homography)
{
	const int count = refPoints->size();
	const CvPoint2D32f* M = (const CvPoint2D32f*)&(*refPoints)[0];
    const CvPoint2D32f* m = (const CvPoint2D32f*)&(*scePoints)[0];

	// final result model
	const CvMat _model = cvMat(this->size, 1, CV_64F, homography);
	double prevErrNorm = DBL_MAX;
	double errNorm = 0;

	// initialize
	cvZero( JtJ );
	cvZero( JtErr );
	errNorm = CalcHomographyError(refPoints, scePoints, homography);
    
	for(iteration=0; iteration<maxIteration; iteration++)
	{
		// calc Jacobian
		cvCopy(model, prevModel);
        Step();
		prevErrNorm = errNorm;

		// check error
		errNorm = CalcHomographyError(refPoints, scePoints, homography);
		while(errNorm > prevErrNorm)
		{
			lambdaLg10++;
			Step();
			errNorm = CalcHomographyError(refPoints, scePoints, homography);
		}

		lambdaLg10 = MAX(lambdaLg10-1, -16);
		prevErrNorm = errNorm;
		cvZero( JtJ );
		cvZero( JtErr );

		// terminate condition
		if(cvNorm(model, prevModel, CV_RELATIVE_L2) < DBL_EPSILON)
		{
			iteration = maxIteration;
		}
	}

	// set data
}

void HomographyLMmethod::Step()
{
    const double LOG10 = log(10.);
    double lambda = exp(lambdaLg10*LOG10);

	cvCopy(JtJ, JtJN);
    for(int i=0; i<this->size; i++)
        JtJN->data.db[(this->size+1)*i] *= 1. + lambda;

    cvSVD( JtJN, JtJW, 0, JtJV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
    cvSVBkSb( JtJW, JtJV, JtJV, JtErr, model, CV_SVD_U_T + CV_SVD_V_T );
    for(int i=0; i<this->size; i++ )
        model->data.db[i] = prevModel->data.db[i] - model->data.db[i];
}