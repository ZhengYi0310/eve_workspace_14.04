/*
File: HMMFunctional.h
Date: 11/04/2016
Author: Kenneth Gutierrez
*/

#ifndef HMMFunctional
#define HMMFunctional

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include <Eigen/Dense>
//#include <Eigen/Core>
//#include <Eigen/Cholesky>
//#include <Eigen/LU>

using namespace Eigen;
using namespace std;

// HMM Information: Based on MATLAB SIMULATION
int nHS(3); // Number of Hidden States
int nBins(12); // Number of Bins used
int d(40); // Number of Input
int ts(25); // Length of Data stream (Time Steps)

void importMatricesfromMATLAB(int bin, VectorXf& pi, MatrixXf& A, MatrixXf& sigma, 
	MatrixXf& mu)
{
	/*
	Matrices that need to be imported:
	- model.pi (1 x nHS)
	- model.A (nHSxnHS) - Transition Matrix
	- model.emission.mu (d x nHS)
	- model.emission.Sigma (dxdxnHS) 
	*/

	string mainDir("/home/robot/HMM_MODELS_12/");
	string piext("/pi.csv");
	string aext("/A.csv");
	string sigmaext("/sigma.csv");
	string muext("/mu.csv");

	char BinDir[100];
	char piDir[100];
	char aDir[100];
	char sigmaDir[100];
	char muDir[100];

	int n;
	n = sprintf(BinDir,"%s%02d",mainDir.c_str(),bin);
	// Name all directories
	n = sprintf(piDir,"%s%s",BinDir,piext.c_str());
	n = sprintf(aDir,"%s%s",BinDir,aext.c_str());
	n = sprintf(sigmaDir,"%s%s",BinDir,sigmaext.c_str());
	n = sprintf(muDir,"%s%s",BinDir,muext.c_str());

	//ROS_INFO("Opening: %s",piDir);

	// Load Pi
	ifstream pifile(piDir);
	string line;
	string value;
	int counter = 0;
	while (pifile.good())
	{
		while(getline(pifile,line))
		{
			istringstream s(line);
			while (getline(s,value,','))
			{
				//ROS_INFO("%s",value.c_str());
				//ROS_INFO("%f",atof(value.c_str()));
				pi(counter) = atof(value.c_str());
				counter++;
			}
		}

	}
	pifile.close();

	//cout<<pi<<"\n";

	// Load A
	int i = 0;
	int j = 0;
	ifstream afile(aDir);
	while (afile.good())
	{
		while(getline(afile,line))
		{
			j = 0;
			istringstream s(line);
			while(getline( s, value, ',' ))
			{
				// ROS_INFO("%s",value.c_str());
				//ROS_INFO("%f",atof(value.c_str()));
				A(i,j) = atof(value.c_str());
				j++;
			}
			i++;
		}
	}
	//cout<< A << "\n";
	afile.close();


	// Load Sigma
	i = 0;
	j = 0;
	ifstream sigmafile(sigmaDir);
	while (sigmafile.good())
	{
		while(getline(sigmafile,line))
		{
			j = 0;
			istringstream s(line);
			while(getline( s, value, ',' ))
			{
				// ROS_INFO("%s",value.c_str());
				//ROS_INFO("%f",atof(value.c_str()));
				sigma(i,j) = atof(value.c_str());
				j++;
			}
			i++;
		}
	}
	//cout<< sigma << "\n";
	sigmafile.close();

	// Load mu
	i = 0;
	j = 0;
	ifstream mufile(muDir);
	while (mufile.good())
	{
		while(getline(mufile,line))
		{
			j = 0;
			istringstream s(line);
			while(getline( s, value, ',' ))
			{
				// ROS_INFO("%s",value.c_str());
				//ROS_INFO("%f",atof(value.c_str()));
				mu(i,j) = atof(value.c_str());
				j++;
			}
			i++;
		}
	}
	//cout<< mu << "\n";
	mufile.close();
}

void normalizeLogspace(MatrixXf& logB, VectorXf& scale)
{
	VectorXf Y(nHS);
	float m(0);
	MatrixXf y(ts,nHS);
	MatrixXf a(ts,nHS);


	y = MatrixXf::Zero(ts,nHS);

	for (int t = 0; t<ts; t++)
	{
		Y = VectorXf::Zero(nHS);
		for(int n = 0; n<nHS; n++)
			Y(n) = logB.coeff(t,n);
		m = Y.maxCoeff();
		for(int n = 0; n<nHS; n++)
			y(t,n) = m;
	}

	a = logB - y;


	// Apply exp to all terms in a
	for(int i = 0; i<a.rows(); i++)
		for(int j = 0; j<a.cols(); j++)
			a(i,j) = exp(a.coeff(i,j));

	// sum up the rows
	float sumCount(0);
	for (int t = 0; t<a.rows(); t++)
	{
		sumCount = 0;
		for(int n = 0; n<nHS; n++)
			sumCount = a(t,n) + sumCount;
		scale(t) = y.coeff(t,0)+log(sumCount);
	}


	// Subtract normalization constant from each row
	for (int i = 0; i<logB.rows(); i++)
	{
		for (int j = 0; j<logB.cols(); j++)
			logB(i,j) = logB.coeff(i,j)-scale.coeff(i);
	}

}

void normalize(VectorXf& A, float& z)
{
	z = A.sum();
	for(int t = 0; t<A.size(); t++)
		A(t) = A(t)/z;
}

float hmmFilter(VectorXf pi, MatrixXf A, MatrixXf B)
{
	MatrixXf AT;
	VectorXf scale(ts);
	VectorXf alpha(nHS);
	float z;
	float loglik;

	AT = A.transpose();


	for (int t = 0; t<ts; t++)
	{
	//Multiply Coeffients of PI with Columns of B
		if (t==0)
		{
			//Initialize alpha term
			for (int n = 0; n<nHS;n++)
				alpha(n) = pi(n)*B(n,t);
			normalize(alpha,z);
			scale(t) = z;
		}
		else
		{
			// Update Alpha with Transition Matrix
			z = 0;
			alpha = AT*alpha;
			for(int n =0; n<nHS; n++)
				alpha(n) = alpha(n)*B(n,t);
			normalize(alpha,z);
			scale(t) = z;
		}
	}

	//Calculate logLikelihood
	loglik = 0;
	for(int i =0; i<scale.size(); i++)
		loglik = log(scale(i)+FLT_EPSILON)+loglik;

	return loglik;
}

void findSigmaMu(int n, MatrixXf sigma_0,MatrixXf mu_0, MatrixXf& sigma_HS, 
	MatrixXf& mu_HS)
{
	n = n+1;
	int shift = d*(n-1);
	// Find sigma of current HS
	for(int j = 0; j<sigma_HS.cols(); j++)
		for(int i = 0; i<sigma_HS.rows(); i++)
			sigma_HS(i,j) = sigma_0.coeff(i,j+shift);

	/*
	if (n==2)
	{
		for(int i=0; i<sigma_HS.rows(); i++)
		{
			for(int j=0; j<sigma_HS.cols();j++)
			{
				ROS_INFO("SIG(%i,%i) = %f",i,j,sigma_HS.coeff(i,j));
			}
			sleep(60.0);
		}
	}
	*/

	n = n-1; //Reset n

	// Find mu for current HS
	for(int i = 0; i<mu_HS.rows(); i++)
	{
		for(int j=0; j<mu_HS.cols(); j++)
		{
			mu_HS(i,j) = mu_0.coeff(j,n);
			//ROS_INFO("MU(%i) = %f", j, mu_HS(i,j));
		}
	}
}

float calcHMMLikelihood(int bin, VectorXf BT_Input, MatrixXf PI, MatrixXf A,
	MatrixXf MU_0, MatrixXf SIGMA)
{
	// Paramters for bin
	VectorXf pi(nHS); // model.pi
	MatrixXf transmat(nHS,nHS); // model.A
	MatrixXf sigma(d,d*nHS);// model.emission.Sigma
	MatrixXf mu(d,nHS); //model.emission.mu

	//importMatricesfromMATLAB(bin, pi, transmat, sigma, mu);

	//Find the parameters for the current bin
	int b = bin-1; //Adjusts bin number for loop
	for(int n=0; n<nHS; n++)
	{
		//pi
		pi(n) = PI.coeff(b,n);
		//transmat
		for(int k=0; k<nHS; k++)
			transmat(n,k) = A.coeff(nHS*b+n,k);
		//mu
		for (int i = 0; i < d; i++)
			mu(i,n) = MU_0.coeff(i,nHS*b+n);
		//sigma
		for (int i = 0; i < d; i++)
			for(int j=0; j<d; j++)
				sigma(i,n*d+j) = SIGMA.coeff(i,b*nHS*d+n*d+j);
	}

	MatrixXf bt(d,ts); //Initialize Observation Matrix
	VectorXf scale(ts); //Scaling for Normalization
	MatrixXf logB(nHS,ts);
	MatrixXf B(nHS,ts);

	// Transfer BioTac input into Eigen Matrix
	int VecCounter = 0;
	for(int j=0; j<bt.cols(); j++ )
	{
		for(int i=0; i<bt.rows(); i++)
		{
			bt(i,j) = BT_Input.coeff(VecCounter);
			VecCounter++;
		}
	}

	//ROS_INFO("I made it here!");
	// Transpose data matrix
	MatrixXf bt_T = bt.transpose();

	// Initialize Manipulation Matrices and Vectors
	MatrixXf X2(ts,d);
	MatrixXf R_Inv;
	VectorXf logP(ts); //Stores logP for current HS
	VectorXf logZ;

	//Emission Matrix Properties
	MatrixXf SIG(d,d);
	MatrixXf MU(ts,d); // Vector turned into Matrix in order to make operations easier

	for (int n = 0; n<nHS; n++)
	{
		//Find sigma and mu for the current HS
		findSigmaMu(n,sigma,mu,SIG,MU);

		/*
		for(int j = 0; j<SIG.cols(); j++)
		{
			if (n==1)
				ROS_INFO("SIG(0,%i) = %f", j, SIG.coeff(0,j));
		}

		if (n==1)
			sleep(60.0);
			*/
		

		X2 = bt_T-MU; // Subtract Mu from each row in transposed input matrix
		//ROS_INFO("ROWS: %i, COLS: %i",SIG.rows(),SIG.cols());

		LLT<MatrixXf> lltofSigma(SIG);
		MatrixXf R_temp = lltofSigma.matrixL();

		// Added line to be compatible with Eigen2
		// Transposing seems to be the have the same result of matrixU()
		MatrixXf R = R_temp.transpose();

		R_Inv = R.inverse();

		MatrixXf S;
		S = X2*R_Inv;

		/*
		for(int j = 0; j<S.cols(); j++)
			ROS_INFO("S(0,%i) = %f", j, S.coeff(0,j));
			*/

		// Square all elements in matrix S
		for(int j = 0; j<S.cols(); j++)
			for(int i = 0; i<S.rows();i++)
				S(i,j) = S(i,j)*S(i,j);

		VectorXf p(ts);
		float sumCount;
		// Sum up the rows of Matrix S
		for(int i= 0; i<S.rows(); i++)
		{
			sumCount = 0;
			for(int j = 0; j<S.cols(); j++)
				sumCount = S(i,j) + sumCount;
			p(i) = -0.5*sumCount;

			//ROS_INFO("p(%i)= %f",i,p(i));
		}

		//log Z Component of Algorithim

		// Sum Log Diagonal of R
		float logCount = 0;
		for(int j=0; j<R.cols(); j++)
			logCount = log(R.coeff(j,j))+logCount;

		VectorXf ones;
		ones = VectorXf::Ones(ts);

		logZ = (0.5*d*log(2*M_PI)+logCount)*ones;
		logP = p - logZ;

		for (int j=0; j<logB.cols();j++)
		{
			logB(n,j) = logP.coeff(j);
			//ROS_INFO("log P: %f", logB.coeff(n,j));
		}
	}

	//Normalize logB
	MatrixXf logB_T(ts,nHS);
	logB_T = logB.transpose();

	normalizeLogspace(logB_T,scale);
	

	logB = logB_T.transpose();

	//Apply exponential term
	for (int j = 0; j<logB.cols(); j++)
		for(int i = 0; i<logB.rows();i++)
			B(i,j) = exp(logB.coeff(i,j));

	

	//HMM Filter
	float loglik;
	loglik = hmmFilter(pi,transmat,B);

	loglik = loglik + scale.sum();
	//ROS_INFO("loglik = %f",loglik);
	return loglik;
}
#endif
