
/* Driver for routine simplx */


#define NRANSI
#include "nr.h"
#include "nrutil.h"
#include "nrutil2.h"
#include "simp1.h"
#include "simp2.h"
#include "simp3.h"
#include "simplx.h"

#define alpha_TTD 65
#define alpha_TTD_M alpha_TTD*5

float Mins(float a,float b)
{
	if(a<=b)
		return a;
	else
		return b;
}
float Maxs(float a,float b)
{
	if(a>=b)
		return a;
	else
		return b;
}


#define N (NumOnRamp)*Np
#define M1 (NumOnRamp)*Np*2        /* M1+M2+M3 = M */
#define M2 0
#define M3 0
#define NM1M2 (N+M1+M2)
#define M (M1+M2+M3)
#define NP N+1        /* NP >= N+1 */
#define MP M+2        /* MP >= M+2 */

int set_coef(float c[MP][NP],int* index1,int* index2,double Qm)
{//a_w0 not OK
		char str[len_str];
//	AKIPrintString("coeff");
	float w_const[SecSize][Np]={0.0};
	float p_const[SecSize][Np]={0.0};
	float w_r_coeff[SecSize][Np][(NumOnRamp)*Np]={0.0};
	float p_r_coeff[SecSize][Np][(NumOnRamp)*Np]={0.0};
	float f[(NumOnRamp)*Np]={0.0};
	float A[(NumOnRamp)*Np][(NumOnRamp)*Np]={0.0};
	float b[(NumOnRamp)*Np]={0.0};
	float b_u[(NumOnRamp)*Np]={0.0}; // upper bound of r

	float c1[MP][NP]={0.0};
	float c2[MP][NP]={0.0};
	int num1=0,num2=0;

	int m,j,a;
	static unsigned short memset_sw=1;

	if (memset_sw==1)
	{
	memset(&w_const,0,sizeof(w_const));
	memset(&p_const,0,sizeof(p_const));
	memset(&w_r_coeff,0,sizeof(w_r_coeff));
	memset(&p_r_coeff,0,sizeof(p_r_coeff));
	memset(&f,0,sizeof(f));
	memset(&A,0,sizeof(A));
	memset(&b,0,sizeof(b));
	memset(&b_u,0,sizeof(b_u));
	memset(&c1,0,sizeof(c1));
	memset(&c2,0,sizeof(c2));
	memset_sw=0;
	}

	
	
	float a_w0[SecSize]={0.0};

	float Q_o[SecSize];  // onramp capacity
	for(int i=0;i<SecSize;i++)
		Q_o[i]=1500.0;

	float Q_Ops=float(Qm);

	for(m=1;m<=NumOnRamp;m++)
	{
		if(m==1)
		{
			for(j=0;j<Np;j++)
			{
				// coeff
				if(j==0)
				{
					for(a=0;a<(NumOnRamp)*Np;a++)
					{
						p_r_coeff[m][j][a]=float((1-T/L[m]*u2[m])*0.0+T/(L[m]*lambda[m])*lambda[m-1]*u2[m-1]*0.0); // // this all 0??  08_29_13
						w_r_coeff[m][j][a]=0.0;
						if(a==j*(NumOnRamp)+m-1)	//may be wrong
						{
							p_r_coeff[m][j][a]=float(p_r_coeff[m][j][a]+T/(L[m]*lambda[m]));	
							w_r_coeff[m][j][a]=float(w_r_coeff[m][j][a]-T);
						}
					}
				}
				else
				{
					for(a=0;a<(NumOnRamp)*Np;a++)
					{
						p_r_coeff[m][j][a]=float((1-T/L[m]*u2[m])*p_r_coeff[m][j-1][a]+T/(L[m]*lambda[m])*lambda[m-1]*u2[m-1]*0.0); // this all 0??  08_29_13
						w_r_coeff[m][j][a]=w_r_coeff[m][j-1][a];
						if(a==j*(NumOnRamp)+m-1)
						{
							p_r_coeff[m][j][a]=float(p_r_coeff[m][j][a]+T/(L[m]*lambda[m]));	
							w_r_coeff[m][j][a]=float(w_r_coeff[m][j][a]-T);
						}
					}
				}
				//const

				if(j==0)
				{
					p_const[m][j]=float((1-T/L[m]*u2[m])*pre_rho[m]+T/(L[m]*lambda[m])*(lambda[m-1]*u2[m-1])*pre_rho[m-1]-T/(L[m]*lambda[m])*ss[m][j]);	//pre_rho[m=0]=up_rho[j=0]
					w_const[m][j]=float(pre_w[m]+T*dd[m][j]);
				}
				else
				{
					p_const[m][j]=float((1-T/L[m]*u2[m])*p_const[m][j-1]+T/(L[m]*lambda[m])*(lambda[m-1]*u2[m-1])*up_rho[j]-T/(L[m]*lambda[m])*ss[m][j]);
					w_const[m][j]=float(w_const[m][j-1]+T*dd[m][j]);
				}
				
			}
		}
		else  // for m>1
		{
			for(j=0;j<Np;j++)
			{
				// coeff
				if(j==0)
				{
					for(a=0;a<(NumOnRamp)*Np;a++)
					{
						p_r_coeff[m][j][a]=float((1-T/L[m]*u2[m])*0.0+T/(L[m]*lambda[m])*lambda[m-1]*u2[m-1]*0.0);  // this all 0??  08_29_13
						w_r_coeff[m][j][a]=0.0;
						if(a==j*(NumOnRamp)+m-1)	//may be wrong
						{
							p_r_coeff[m][j][a]=float(p_r_coeff[m][j][a]+T/(L[m]*lambda[m]));	
							w_r_coeff[m][j][a]=float(w_r_coeff[m][j][a]-T);
						}
					}
				}
				else
				{
					for(a=0;a<(NumOnRamp)*Np;a++)
					{
						p_r_coeff[m][j][a]=float((1-T/L[m]*u2[m])*p_r_coeff[m][j-1][a]+T/(L[m]*lambda[m])*lambda[m-1]*u2[m-1]*p_r_coeff[m-1][j-1][a]);
						w_r_coeff[m][j][a]=w_r_coeff[m][j-1][a];
						if(a==j*(NumOnRamp)+m-1)
						{
							p_r_coeff[m][j][a]=float(p_r_coeff[m][j][a]+T/(L[m]*lambda[m]));	
							w_r_coeff[m][j][a]=float(w_r_coeff[m][j][a]-T);
						}
					}
				}
				//const
				if(j==0)
				{
					p_const[m][j]=float((1-T/L[m]*u2[m])*pre_rho[m]+T/(L[m]*lambda[m])*(lambda[m-1]*u2[m-1])*pre_rho[m-1]-T/(L[m]*lambda[m])*ss[m][j]);
					w_const[m][j]=float(pre_w[m]+T*dd[m][j]);
				}
				else
				{
					p_const[m][j]=float((1-T/L[m]*u2[m])*p_const[m][j-1]+T/(L[m]*lambda[m])*(lambda[m-1]*u2[m-1])*p_const[m-1][j-1]-T/(L[m]*lambda[m])*ss[m][j]);
					w_const[m][j]=float(w_const[m][j-1]+T*dd[m][j]);
				}
			}
		}
	}

//obj fnc
	for(m=1;m<=(NumOnRamp);m++)
	{
		if(m<NumOnRamp)
		{
			for(j=0;j<Np;j++)
			{
				for(a=0;a<(NumOnRamp)*Np;a++)
					f[a]=float(f[a]+a_tts*T*lambda[m]*L[m]*p_r_coeff[m][j][a]+a_w*a_w0[m]*w_r_coeff[m][j][a]-a_ttd*T*lambda[m]*L[m]*u2[m]*p_r_coeff[m][j][a]);
			}
		}
		else
		{
			for(j=0;j<Np;j++)
			{
				for(a=0;a<(NumOnRamp)*Np;a++)
					f[a]=float(f[a]+a_tts*T*lambda[m]*L[m]*p_r_coeff[m][j][a]+a_w*a_w0[m]*w_r_coeff[m][j][a]-a_ttdM*T*lambda[m]*L[m]*u2[m]*p_r_coeff[m][j][a]);
			}
		}
	}
// lower and upper bounds
	/*for(m=1;m<=NumOnRamp;m++)
	{
		//if(W[m][k-1]<=Onramp_Q_Thresh)	//I do not know what is this
		if(W[m][k-1]<=500)
		{
			for(j=1;j<=Np;j++)
				b_u[(m)*Np+j]=Mins( Q_o[m],D[m][k-1] );
		}
		else
		{
			for(j=1;j<=Np;j++)
				b_u[(m)*Np+j]=Mins( D[m][k-1], Mins (Q_o[m],lambda[m]*(Lane_cap-Q2[m][k-1-1])));
		}
	}*/
	for(m=1;m<=(NumOnRamp);m++)
	{
		for(j=0;j<Np;j++)
		{
			a=j*(NumOnRamp)+m-1;
			b_u[a]=Mins(dd[m][0],Mins( Mins(Q_o[m],float(1.5/lambda[m]*(qc[m]-q[m-1]))),float(1.5/lambda[m]*Omega*(Rou_J*lambda[m]-detection_s[m]->data[Np-1].density))  ));
			
			//b_u[a]=Mins(dd[m][0],Mins(Mins(Q_o[m],float(1.5/lambda[m]*(qc[m]-q[m-1]))),float(1.5/lambda[m]*u2[m]*(Rou_J*lambda[m]-detection_s[m]->data[Np-1].density))));
			//b_u[a]=Mins(dd[m][0],Mins(Mins(Q_o[m],float(1.5/lambda[m]*(qc[m]-q[m-1]))),float(1.5/lambda[m]*u2[m]*(Rou_J*lambda[m]-120.0))));
			//b_u[a]=Mins(dd[m][0],Mins(Mins(Q_o[m],float(1.5/lambda[m]*(qc[m]-q[m-1]))),float(1.5/lambda[m]*u2[m]*(Rou_J*lambda[m]-detection[m]->data[Np-1].density))));
			
//			b_u[a]=700; // for DBG
		}
	}

//A and b
	for(m=1;m<=(NumOnRamp);m++)
	{
		for(j=0;j<Np;j++)
		{
			for(a=0;a<(NumOnRamp)*Np;a++)
			{
				A[j*(NumOnRamp)+m-1][a]=p_r_coeff[m][j][a];
				//b[j*NumOnRamp+m-1]=Q_Ops*lambda[m]/Maxs(float(u2[m]),1.0)-p_const[m][j];
//				b[j*NumOnRamp+m-1]=Maxs(Mins(Rou_J,Q_Ops/Maxs(float(u2[m]),1.0)),float(detection_s[m]->data[Np-1].density/lambda[m]))-p_const[m][j];//Maxs(Mins(Rou_J,Q_Ops/Maxs(float(u2[m]),1.0)),float(detection_s[m]->data[Np-1].density/lambda[m]))-p_const[m][j];
//wrong				b[j*NumOnRamp+m-1]=Mins(Rou_J,Q_Ops/Maxs(float(u2[m]),1.0))-p_const[m][j];
				b[j*(NumOnRamp)+m-1]=Q_Ops*lambda[m]/Maxs(float(u2[m]),1.0)-p_const[m][j];
//				if(m<=3 || b[j*NumOnRamp+m-1]<100-p_const[m][j])//need to be 2
//					b[j*NumOnRamp+m-1]=100-p_const[m][j];
//				sprintf_s(str,len_str,"A[%d][%d]=%10.8f",j*NumOnRamp+m-1,a,A[j*NumOnRamp+m-1][a]);
//				AKIPrintString(str);
			}
		}
	}
	if(false)//(count==int(1300/20))
	{
		sprintf_s(str,len_str,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t",Q_Ops/Maxs(float(u2[1]),1.0),Q_Ops/Maxs(float(u2[2]),1.0),Q_Ops/Maxs(float(u2[3]),1.0),Q_Ops/Maxs(float(u2[4]),1.0),Q_Ops/Maxs(float(u2[5]),1.0),Q_Ops/Maxs(float(u2[6]),1.0),Q_Ops/Maxs(float(u2[7]),1.0));
		AKIPrintString(str);
		sprintf_s(str,len_str,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t",float(detection_s[1]->data[Np-1].density/lambda[1]),float(detection_s[2]->data[Np-1].density/lambda[2]),float(detection_s[3]->data[Np-1].density/lambda[3]),float(detection_s[4]->data[Np-1].density/lambda[4]),float(detection_s[5]->data[Np-1].density/lambda[5]),float(detection_s[6]->data[Np-1].density/lambda[6]),float(detection_s[7]->data[Np-1].density/lambda[7]));
		AKIPrintString(str);
	}
//put value to c matrix and get <= , >= euation numbers
	for(m=0;m<(NumOnRamp)*Np;m++)
		c[0][m+1]=-1*f[m];
	for(m=0;m<(NumOnRamp)*Np;m++)
	{
		if(b_u[m]>=0.0)
		{
			c1[num1][m+1]=-1.0;
			c1[num1][0]=b_u[m];
			num1++;
		}
		else
		{
			c2[num2][m+1]=1.0;
			c2[num2][0]=float(-1.0*b_u[m]);
			num2++;
		}
	}
	for(m=0;m<(NumOnRamp)*Np;m++)
	{
		if(b[m]>=0.0)
		{
			c1[num1][0]=b[m];
			for(a=0;a<(NumOnRamp)*Np;a++)
			{
				if(A[m][a]!=0)
					c1[num1][a+1]=float(-1.0*A[m][a]);
			}
			num1++;
		}
		else
		{
			c2[num2][0]=float(-1.0*b[m]);
			for(a=0;a<(NumOnRamp)*Np;a++)
			{
				if(A[m][a]!=0)
					c2[num2][a+1]=A[m][a];
			}
			num2++;
		}
	}
	m=1;
	for(j=0;j<num1;j++)
	{
		for(a=0;a<1+(NumOnRamp)*Np;a++)
		{
			c[m][a]=c1[j][a];
			
		}
		m++;
	}
	for(j=0;j<num2;j++)
	{
		for(a=0;a<1+(NumOnRamp)*Np;a++)
		{
			c[m][a]=c2[j][a];
			
			
		}
		m++;
	}
	*index1=num1;
	*index2=num2;
	
	
	return 0;
}

int opt_metering(void)
{
	int i,icase,*izrov,*iposv;
	static float c[MP][NP]={0.0};
  //float **c, **cc;
	int row,col;
//	char str[len_str];
	float cc[MP][NP]={0.0};
	int index1=0,index2=0;

	//c=matrix(0,(long)(MP-1),0,(long)(NP-1));
	//cc=matrix(0,(long)(MP-1),0,(long)(NP-1));

	memset(&cc,0,sizeof(cc));
	memset(&c,0,sizeof(c));

	set_coef(cc,&index1,&index2,1800.0);
	for(row=0;row<MP;row++)
		for(col=0;col<NP;col++)
			c[row][col]=cc[row][col];

	float **a;
	izrov=ivector(1,N);
	iposv=ivector(1,M);
	a=convert_matrix(&c[0][0],1,MP,1,NP);
	simplx(a,M,N,index1,index2,M3,&icase,izrov,iposv);
	if (icase == 1)
		AKIPrintString("\nunbounded objective function\n");
	else if (icase == -1)
	{
		set_coef(cc,&index1,&index2,2500.0);
		for(row=0;row<MP;row++)
			for(col=0;col<NP;col++)
				c[row][col]=cc[row][col];
		a=convert_matrix(&c[0][0],1,MP,1,NP);
		simplx(a,M,N,index1,index2,M3,&icase,izrov,iposv);
		if(icase!=1 && icase!=-1)
		{
			for(row=0;row<SecSize;row++)	//reset opt_r[][], otherwise, it keeps the original values
				for(col=0;col<Np;col++)
					opt_r[row][col]=0.0;
			for(i=1;i<=M;i++)
				if(iposv[i]<=N)
				{
					col=(iposv[i]-1)/NumOnRamp;
					row=iposv[i]-col*NumOnRamp;
					opt_r[row][col]=a[i+1][1];
	//				sprintf_s(str,len_str,"i=%d, iposv=%d, row=%d, col=%d, r=%10.2f",i,iposv[i],row,col, opt_r[row][col]);
	//				AKIPrintString(str);
				}
		}
//		sprintf_s(str,len_str,"time=%lf",AKIGetCurrentSimulationTime());
//		AKIPrintString(str);
//		AKIPrintString("\nno solutions satisfy constraints given\n");
	}	
	else
	{
		for(row=0;row<SecSize;row++)	//reset opt_r[][], otherwise, it keeps the original values
		{
			for(col=0;col<Np;col++)
				opt_r[row][col]=0.0;
		}
		for(i=1;i<=M;i++)
		{
			if(iposv[i]<=N)
			{
				col=(iposv[i]-1)/NumOnRamp;
				row=iposv[i]-col*NumOnRamp;
				opt_r[row][col]=a[i+1][1];
//				sprintf_s(str,len_str,"i=%d, iposv=%d, row=%d, col=%d, r=%10.2f",i,iposv[i],row,col, opt_r[row][col]);
//				AKIPrintString(str);
			}
		}
//		AKIPrintString("solution found");
		//for (i=0;i<NumOnRamp-2;i++)
			//fprintf(dbg_f,"%5.2f\t ", opt_r[i][0]);
		//fprintf(dbg_f,"%5.2f\n", opt_r[NumOnRamp-1][0]);
		
		/*fprintf(dbg_f,"%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\n",
			opt_r[0][0],opt_r[1][0],opt_r[2][0],opt_r[3][0],opt_r[4][0],opt_r[5][0],opt_r[6][0],opt_r[7][0],opt_r[8][0],
			opt_r[9][0],opt_r[10][0],opt_r[11][0],opt_r[12][0], opt_r[13][0],opt_r[14][0],opt_r[15][0]);*/

		/*sprintf_s(str,len_str,"opt_r= %5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t%5.2f\t",
			opt_r[0][0],opt_r[1][0],opt_r[2][0],opt_r[3][0],opt_r[4][0],opt_r[5][0],opt_r[6][0],opt_r[7][0],opt_r[8][0],
			opt_r[9][0],opt_r[10][0],opt_r[11][0],opt_r[12][0], opt_r[13][0],opt_r[14][0],opt_r[15][0]);
		AKIPrintString(str);*/

	}
	free_convert_matrix(a,1,MP,1,NP);
	free_ivector(iposv,1,M);
	free_ivector(izrov,1,N);
	//free_matrix(c,0,MP,0,NP);
	//free_matrix(cc,0,MP,0,NP);

	return 0;
}

#undef NRANSI
