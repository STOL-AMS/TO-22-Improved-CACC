/******************************************************************

                Version from "combine"
******************************************************************/

#include <math.h>
#include <string.h>
#include "readdetectordata.h"
#include "CIProxie.h"

double Mind(double a,double b)
{
	if(a<=b)
		return a;
	else
		return b;
}
double Maxd(double a,double b)
{
	if(a>=b)
		return a;
	else
		return b;
}
double get_min(double a, double b)
{
	if(a<=b)
		return a;
	else
		return b;
}

bool get_u_for_opt()
{
	int i;
//	char str[len_str];
	for(i=0;i<SecSize;i++)
		//u2[i]=90.0;
		u2[i]=detection_s[i]->data[Np-1].speed;   // direct use measurement
		//u2[i]=get_min(u[i],detection_s[i]->data[Np-1].speed);
//		u2[i]=get_min(300,detection_s[i]->data[Np-1].speed);
	//sprintf_s(str,len_str,"measued speed: %f\t%f\t%f\t%f\t%f\t%f\t%f\n",u2[0],u2[1],u2[2],u2[3],u2[4],u2[5],u2[6]);
	//AKIPrintString(str);
	return true;
}

bool det_data_4_contr(double time) // not used anymore
{
	int i,j;
	//char str[len_str];
//	for(i=0;i<SecSize;i++)		// this part need to be changed
//		Lr[i]=2.0;
	update_queue(time);
	for(i=0;i<SecSize;i++)
	{
		pre_w[i]=float(queue[i]);
	}

	for(i=0;i<NumOnRamp;i++)
		for(j=0;j<Np;j++)
		{
			dd[i][j]=float(detection_onramp[i]->data[j].flow);
//			sprintf_s(str,len_str,"flow=%lf, dd=%f",dataProfileOnRamp[i]->data[j].flow,dd[i][j]);
//			AKIPrintString(str);
		}

	//for(i=0;i<SecSize;i++)
for(i=0;i<NumOnRamp;i++)
{
	for(j=0;j<Np;j++)
	{
		if (detection_offramp[i]->detId > 0)		
			ss[i][j]=(float)(exp_flt*(detection_offramp[i]->data[j].flow)+(1-exp_flt)*ss[i][j]); 
			//ss[i][j]=(float)detection_offramp[i]->data[j].flow; 	
		else
			ss[i][j]=0.0;
	}	
}
	up_rho[j]=(float)((detection_s[0]->data[Np-1].density)/lambda[0]);
	for(j=1;j<Np;j++)
		up_rho[j]=(float)((detection_s[0]->data[j].density)/lambda[0]);
		//up_rho[j]=float(dataProfileUp->data[j].density)/lambda[0];     //08_28_13
	//for(i=0;i<SecSize;i++)
	for(i=0;i<NumOnRamp;i++)
		pre_rho[i]=(float)((detection_s[i]->data[Np-1].density)/lambda[i]);

	
//	sprintf(str,"d=");
//	AKIPrintString(str);
	//sprintf_s(str,len_str,"dd= %f\t%f\t%f\t%f\t%f\t%f\t%f\n",dd[0][0],dd[1][0],dd[2][0],dd[3][0],dd[4][0],dd[5][0],dd[6][0]);
	//AKIPrintString(str);
	
	return true;
}



bool update_alpha()  // update parameter for VSL calculation
{
	int i;
	double den=0;
	for(i=1;i<=NumOnRamp;i++)
		den=den+pow((qc[i]-q[i-1]),2.0);
	for(i=1;i<=NumOnRamp;i++)
		alpha[i]=pow((qc[i]-q[i]),2.0)/den;

	return true;
}
bool update_beta()  // update parameter for VSL calculation
{
	int i,k;
	double product_L[SecSize];
	double den=0.0;
	for(i=1;i<=NumOnRamp;i++)
	{
		product_L[i]=1.0;
	}
	for(i=1;i<=NumOnRamp;i++)
		for(k=1;k<=NumOnRamp;k++)
		{
			if(k!=i)
				product_L[i]=product_L[i]*pow(onrampL[k],2);
		}
	for(i=1;i<=NumOnRamp;i++)
		den=den+product_L[i];
	for(i=1;i<=NumOnRamp;i++)
		beta_c[i]=product_L[i]/den;

	return true;
}
bool get_s()     // get off-ramp flow
{
	int i;	
	for(i=0;i<NumOnRamp;i++)
	{
		if (detection_offramp[i]->detId > 0)		
			//s[i]=exp_flt*(detection_offramp[i]->data[0].flow)+(1-exp_flt)*s[i]; 
			s[i]=exp_flt*(detection_offramp[i]->data[Np-1].flow)+(1-exp_flt)*s[i]; // changed on 03/03/14
			//s[i]=detection_offramp[i]->data[Np-1].flow;
		else
			s[i]=0.0;
	}

	return true;
}
bool get_q_main()      // get flow for each cell
{
	int i;
	for(i=0;i<SecSize;i++)
		q_main[i]=exp_flt*(detection_s[i]->data[Np-1].flow)+(1.0-exp_flt)*q_main[i]; // use upastream section flow for MPC
		//q_main[i]=detection_s[i]->data[Np-1].flow;

	return true;
}

bool update_q_R()       // update flow for each cell
{
	int i;
	q[0]=q_main[0];
	for(i=1;i<=NumOnRamp;i++)
	{
		//R[i-1]=get_min(d[i-1],get_min(Q[i],qc[i]-q[i-1]));  // space left in the current cell
		//R[i-1]=get_min(detection_onramp[i]->practical_flow,get_min(Q[i],qc[i]-q[i-1]));  // space left in the current cell	
		
		//R[i-1]=get_min(detection_onramp[i]->practical_flow,get_min(Q[i],qc[i]-q[i-1]));  // index of R changed; 01/03/13	
		R[i-1]=get_min(dmd[i-1],get_min(Q[i],qc[i]-q[i-1]));                                     // use filtered onramp flow
		q[i]=q_main[i]+R[i-1]-s[i-1];                       // flow of currect cell          // index of R changed;  01/03/13
	}
	return true;
}
/*bool update_R()      // update feasible onramp flow
{
	int i;
	for(i=1;i<=NumOnRamp;i++)
		R[i-1]=get_min(d[i],get_min(Q[i],qc[i]-q[i-1]));  // space left in the current cell

	return true;
}*/
/*bool get_occ3(double *occ1,double *occ2,double *occ3) // occ estimation
{
	int i;
	double occ=0;
	for(i=0;i<5;i++)
		occ=occ+AKIDetGetTimeOccupedAggregatedbyId(free_sec_detId1[i],0);
	*occ1=occ/5;
	occ=0;
	for(i=0;i<5;i++)
		occ=occ+AKIDetGetTimeOccupedAggregatedbyId(free_sec_detId2[i],0);
	*occ2=occ/5;
	occ=0;
	for(i=0;i<5;i++)
		occ=occ+AKIDetGetTimeOccupedAggregatedbyId(free_sec_detId3[i],0);
	*occ3=occ/5;
	return true;
}

bool GetOptSpeed(double time) // occ based; calculate VSL for eahc cell; not used for I-66
{

	int i;
	double Occ1,Occ2,Occ3;
	get_occ3(&Occ1,&Occ2,&Occ3);
	if((Occ1<15 || Occ2<15 || Occ3<15) && u[0]<50 )
	{
		for(i=1;i<=CellSize;i++)
			u[i]=u[NumOnRamp];
		return true;
	}
	
	for(i=1;i<=CellSize;i++)
		u[i]=u[i-1]+Maxd(-20.0,get_min((eta*alpha[i]+(1-eta)*beta_c[i])*(u[NumOnRamp]-u[0]),0.0));

	return true;
}*/




bool get_dmd()
{
	int i;
	//char str[len_str];

	for(i=0;i<NumOnRamp;i++);	
	{		
		//dmd[i]=exp_flt*(detection_onramp[i]->practical_flow)+(1-exp_flt)*dmd[i]; // used in update_R()		
		dmd[i]=double(AKIDetGetCounterAggregatedbyId(OnRampDetectionEndId[i],0))*3600.0/Maxd(detInterval,30.0);
	//sprintf_s(str,len_str,"%lf", dmd[i]);
	//AKIPrintString(str);
	}


	return true;
}



bool get_meas(double T)
{
	int i;


	for(i=0;i<SecSize;i++)
	{
		//q[i]=exp_flt*(detection_s[i]->data[Np-1].flow)+(1.0-exp_flt)*q[i];
		v[i]=exp_flt*(detection_s[i]->data[Np-1].speed)+(1.0-exp_flt)*v[i];
		o[i]=exp_flt*(detection_s[i]->data[Np-1].occupancy)+(1.0-exp_flt)*o[i];
		//q[i]=detection_s[i]->data[Np-1].flow;
		//v[i]=detection_s[i]->data[Np-1].speed;
		//o[i]=detection_s[i]->data[Np-1].occupancy;
		//fprintf(dbg_f, "%lf %lf %lf %lf\n", T, q[i], v[i], o[i]);
	}
	
//	char str[128];
//	sprintf(str,"q= %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
//	AKIPrintString(str);
	return true;
}
#ifdef U1_UM
bool get_u1_uM(double time)
{

	u[0]=detection_s[0]->data[Np-1].speed;
	if(u[0]>VSL_max)     // most upstream
		u[0]=VSL_max;
	char str[128];
	double Occ1,Occ2,Occ3;
	get_occ3(&Occ1,&Occ2,&Occ3);
	sprintf(str,"occ1 =%lf, occ2=%lf, occ3=%lf",Occ1,Occ2,Occ3);
//	AKIPrintString(str);
//	if(Occ1>25 || Occ2>18 || Occ3>18)
/*	if( (Occ1+Occ2+Occ3)/3>18 )
	{
		if(Occ1>18)
			u[NumOnRamp]=u[NumOnRamp]-5*Occ1/100;
		if(Occ2>18)
			u[NumOnRamp]=u[NumOnRamp]-5*Occ2/100;
		if(Occ3>18)
			u[NumOnRamp]=u[NumOnRamp]-5*Occ2/100;
	}*/
	
	if( (Occ1+Occ2+Occ3)/3>Occ_thresh )	//threshold
	{
		u[NumOnRamp]=u[NumOnRamp]-vsl_gain*((Occ1+Occ3+Occ3)/3-Occ_thresh)/100;	//threshold and gain
	}
	else
//		if(Occ1<15 && Occ2<15 & Occ3<15)
//			u[NumOnRamp]=u[NumOnRamp]+3;
		u[NumOnRamp]=u[NumOnRamp]-vsl_gain*((Occ1+Occ3+Occ3)/3-Occ_thresh)/100;	//threshold and gain
	if(u[NumOnRamp]>VSL_max)
		u[NumOnRamp]=VSL_max;
	if(u[NumOnRamp]<VSL_min)  //10  // 20 & 30 did not work; 15 seems to be good; better than 10
		u[NumOnRamp]=VSL_min;
	if(u[0]<u[NumOnRamp])
		u[0]=u[NumOnRamp];
	return true;
}

#endif

double round_speed(double a)
{
	a=a*2/10;
	int b=int(a+0.5);
	a=b*10/2;
	if(a>VSL_max)
		return VSL_max;
	else
		return a;

}

#ifdef USE_VSL
bool Cal_cr_vsl(double time, int vsl_id)  // based on measured occ of previous approach
{

	char str[len_str];
	double ave_occ=0;

	
	if (vsl_id ==3)
	{
		//ave_occ=o[vsl_id]/(N_Ln[vsl_id]);
		//ave_occ=(o[vsl_id]+o[vsl_id+1]+o[vsl_id+2])/(3.0*N_Ln[vsl_id]);
		ave_occ=(o[vsl_id]+o[vsl_id+1])/(2.0*N_Ln[vsl_id]);
		
	sprintf_s(str,len_str,"occ =%lf", ave_occ);
	AKIPrintString(str);
	//fprintf(vsl_crm_f, "%3.2lf  ", ave_occ);

	if (ave_occ > Occ_thresh)		
		u[vsl_id]=u[vsl_id]-vsl_p_gain*60*(ave_occ-Occ_thresh)/100-vsl_gain*(ave_occ-Occ_thresh)/100;	//threshold and gain	
	else
		u[vsl_id]=VSL_max;
	}

	if (vsl_id ==4)
	{
		//ave_occ=o[vsl_id-1]/(N_Ln[vsl_id]);
		//ave_occ=(o[vsl_id-1]+o[vsl_id]+o[vsl_id-1])/(3.0*N_Ln[vsl_id-1]);
		ave_occ=(o[vsl_id-1]+o[vsl_id])/(2.0*N_Ln[vsl_id-1]);
		
	sprintf_s(str,len_str,"occ =%lf", ave_occ);
	AKIPrintString(str);
	//fprintf(vsl_crm_f, "%3.2lf  ", ave_occ);

	if (ave_occ > Occ_thresh)		
		u[vsl_id]=u[vsl_id]-vsl_p_gain*60*(ave_occ-Occ_thresh)/100-vsl_gain*(ave_occ-Occ_thresh)/100;	//threshold and gain	
	else
		u[vsl_id]=VSL_max;
	}
		
	if(u[vsl_id]>VSL_max)
		u[vsl_id]=VSL_max;
	if(u[vsl_id]<VSL_min)  
		u[vsl_id]=VSL_min;

	u[vsl_id]=round_speed(u[vsl_id]);

	return true;
}





bool SetOptSpeed(double time)
{
	//int i; //k;
///	errno_t err;

	Cal_cr_vsl(time, 3);
	Cal_cr_vsl(time, 4);
	//Cal_cr_vsl(time, 11);
	
	
	if(ISUPDATE>=VSL_Update_Step)
	{
		//if(secIDS[3]!=0)		
		//	AKIActionAddSpeedAction(secIDS[3],u[3],0,Compliance_Level);    // Sycamore merge
		//if(secIDS[4]!=0)		
		//	AKIActionAddSpeedAction(secIDS[4],u[4],0,Compliance_Level);    // Sycamore merge
		//if(secIDS[11]!=0)		
		//	AKIActionAddSpeedAction(secIDS[11],u[11],0,Compliance_Level);  // I66-Rt267 merge
		
		ISUPDATE=1;
//	char str[len_str];
	//sprintf_s(str,len_str, "u= %lf, %lf", u[3], u[11]);
	//sprintf_s(str,len_str, "u3= %lf u4= %lf ", u[3], u[4]);
	//AKIPrintString(str);
	//fprintf(vsl_crm_f, "%3.2lf, %3.2lf\n",u[3], u[11]);
	//fprintf(vsl_crm_f, "%3.2lf %3.2lf\n",u[3], u[4]);
	/*
	sprintf_s(str,len_str, "u= %lf, %lf, %lf, %lf, %lf, %lf, %lf,%lf %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf %lf, %lf, %lf, %lf, %lf",
		u[0],u[1],u[2],u[3],u[4],u[5],u[6],u[7],u[8],u[9],u[10],u[11],u[12],u[13],u[14],u[15],u[16],u[17],u[18],u[19],u[20]);
	AKIPrintString(str);

	fprintf(vsl_crm_f, "%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\t%3.2lf\n",
		time,u[0],u[1],u[2],u[3],u[4],u[5],u[6],u[7],u[8],u[9],u[10],u[11],u[12],u[13],u[14],u[15],u[16],u[17],u[18],u[19],u[20]);	
	*/
	}
	else
		ISUPDATE++;
	
	return true;
}

#endif 

int ln_rm_distrib()
{
	int i,j;
	double tt_flw;
//	char str[len_str];

	for (i=0;i<NumOnRamp;i++)
	{
		tt_flw=0.0;
		for(j=0;j<max_onramp_ln;j++)
		{
			if (detection_onramp[i]->detId_ln[j]>0)
				tt_flw+=detection_onramp[i]->ln_flow[j];
		}
		for(j=0;j<max_onramp_ln;j++)
		{
			if (detection_onramp[i]->detId_ln[j]>0)
			{
				ln_metering_rate[i][j]=((detection_onramp[i]->ln_flow[j])/Maxd(tt_flw,1.0))*opt_r[i][0];
				//fprintf(vsl_crm_f, "%10.2lf ", detection_onramp[i]->ln_flow[j]);
			}
			else
				ln_metering_rate[i][j]=0.0;
			//fprintf(vsl_crm_f, "%10.2lf ", ln_meteringId[i][j]);
			
		}
		
	}
	//fprintf(vsl_crm_f, "\n");
	/*sprintf_s(str,len_str,"%10.2lf %10.2lf %10.2lf %10.2lf %10.2lf %10.2lf %10.2lf",
						//tt_flw, detection_onramp[1]->ln_flow[0], detection_onramp[1]->ln_flow[1],detection_onramp[1]->ln_flow[2],  
						tt_flw, ln_metering_rate[1][0], ln_metering_rate[1][1], ln_metering_rate[1][2],
						ln_metering_rate[2][0], ln_metering_rate[2][1], ln_metering_rate[2][2]);
					AKIPrintString(str);*/

	return 1;
}

/**************************************************

	Should apply to all 16 onramps

***************************************************/
bool Set_Default_Meter(double time,double time2,double timeSta) // this implemenmtation is correct 03_05_14; changed from 11 onarmps to 16 onramps 11_28_14
{
//	AKIPrintString("metering");
	int i,j,k, tmp_err, tmp_err1;
	double flow=0.0;
	double amax=0.0;
	double amin=0.0;
	//double tmp_occ=0.0;
	//int tmp_deno=0;
//	char str[len_str];
	//static int release_cycle=0;

	tmp_err=0; tmp_err1=0; 
	if(ISUPDATE2>=0)  // set every step
	{
		for(i=0;i<NumOnRamp+5;i++)
		{	
			//tmp_deno=1;
			for (j=0;j<max_mainline_ln;j++)
			{
				if (Mainline_Ln_Det_Id_All[i][j]>0)
				{
					//RM_occ[i]=((tmp_deno-1)/tmp_deno)*RM_occ[i]+(1/tmp_deno)*AKIDetGetTimeOccupedAggregatedbyId(Mainline_Ln_Det_Id[i][j],0);	
					//tmp_deno++;
					RM_occ_All[i]+=AKIDetGetTimeOccupedAggregatedbyId(Mainline_Ln_Det_Id_All[i][j],0);											
					//fprintf(dbg_f,"%10.2f ", AKIDetGetTimeOccupedAggregatedbyId(Mainline_Ln_Det_Id[i][j],0));
				}
			}
			RM_occ_All[i]=RM_occ_All[i]/N_Mainline_Ln_RM_All[i];
			
			//fprintf(dbg_f,"%10.2f ", RM_occ_All[i]);
			//fprintf(dbg_f,"%10.2f %10.2f ", SR99_RM_rate_tbl[0][i], SR99_RM_occ_tbl[0][i]);
			
			for (j=0;j<N_interv-1;j++)
			{
				if (RM_occ_All[i]<SR99_RM_occ_tbl[0][i])
				{
					for (k=0;k<max_onramp_ln;k++)
					{
						if (ln_meteringId[i][k]>0)
						{
							tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][k],timeSta,&amax,&flow,&amin);
							if (k<2)
								actual_r[i][k]=SR99_RM_rate_tbl[0][i];
							else  // HOV lane								
								actual_r[i][k]=amax;							
						}
						else
							actual_r[i][k]=0.0;
					}
				}
				else if (RM_occ_All[i]>=SR99_RM_occ_tbl[j][i] && RM_occ_All[i]<=SR99_RM_occ_tbl[j+1][i])
				{					
					for (k=0;k<max_onramp_ln;k++)
					{
						if (ln_meteringId[i][k]>0)
						{
							tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][k],timeSta,&amax,&flow,&amin);
							if (k<2)
								actual_r[i][k]=SR99_RM_rate_tbl[j][i];								
							else  // HOV lane														
								actual_r[i][k]=amax;							
						}
						else
							actual_r[i][k]=0.0;
					}
				}
				else
				{
					for (k=0;k<max_onramp_ln;k++)
					{
						if (ln_meteringId[i][k]>0)
						{
							tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][k],timeSta,&amax,&flow,&amin);
							if (k<2)
								actual_r[i][k]=SR99_RM_rate_tbl[N_interv-1][i];
							else  // HOV lane							{								
								actual_r[i][k]=amax;							
						}
						else
							actual_r[i][k]=0.0;
					}
				}			
			} // for j-loop
			for (k=0;k<max_onramp_ln;k++)
			{
				if (actual_r[i][k]>max_RM_rt)
					actual_r[i][k]=max_RM_rt;
			}
		} // for i-loop
		
		//fprintf(dbg_f,"\n");

		// activation of Feild RM
		for(i=0;i<NumOnRamp+5;i++)
		{	
			for (j=0;j<max_onramp_ln;j++)
			{				
				if(ln_meteringId[i][j]>0)				
					tmp_err1 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, max_RM_rt, actual_r[i][j], amin, time, AKIGetSimulationStepTime());
			} // for j-loop end
			//opt_r[i][0]=(float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]);
		}// for i-loop end	

		//fprintf(vsl_crm_f, "\n");

		ISUPDATE2=0;

		//fprintf(vsl_crm_f,"%10.2f\t", timeSta);

		/*for(i=0;i<NumOnRamp+5;i++)
			 fprintf(vsl_crm_f,"%10.2f\t", (float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]));
		fprintf(vsl_crm_f,"\n");*/

		//sprintf_s(str,len_str,"%d %d %d %d %d\n", tmp_err, tmp_err1, tmp_err2, tmp_err3, tmp_err4);
		//AKIPrintString(str);

	} // if ISUPDATE2 loop 
	else
		ISUPDATE2++;	
	return true;
}

/*******************************************************
	Upstream 5 meters using default local responsive;
	downstream 11 meters are coordinated
********************************************************/
bool Set_Hybrid_Meter(double time,double time2,double timeSta)
{
//	AKIPrintString("metering");
	int i,j,k, tmp_err, tmp_err1, tmp_err2, tmp_err3, tmp_err4;
	double flow=0.0;
	double amax=0.0;
	double amin=0.0;
	double new_rate=0.0;
//	char str[len_str];

	tmp_err=0; tmp_err1=0; tmp_err2=0; tmp_err3=0; tmp_err4=0; 
	if(ISUPDATE2>=0)  // set every step
	{
		
		for(i=0;i<5;i++)  // Upstream 5 ramps using Field Local Adaptive Metering rate
		{				
			for (j=0;j<max_mainline_ln;j++)
			{
				if (Mainline_Ln_Det_Id_All[i][j]>0)	
					RM_occ[i]+=AKIDetGetTimeOccupedAggregatedbyId(Mainline_Ln_Det_Id_All[i][j],0);																				
			}
			RM_occ_All[i]=RM_occ_All[i]/N_Mainline_Ln_RM_All[i];

			//fprintf(dbg_f,"%10.2f ", RM_occ[i]);			
			
			for (j=0;j<N_interv-1;j++)
			{
				if (RM_occ_All[i]<SR99_RM_occ_tbl[0][i])
				{
					for (k=0;k<max_onramp_ln;k++)
					{
						if (ln_meteringId[i][k]>0)
						{
							tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][k],timeSta,&amax,&flow,&amin);
							if (k<2)
								actual_r[i][k]=SR99_RM_rate_tbl[0][i];
							else  // HOV lane								
								actual_r[i][k]=amax;							
						}
						else
							actual_r[i][k]=0.0;
					}
				}
				else if (RM_occ_All[i]>=SR99_RM_occ_tbl[j][i] && RM_occ_All[i]<=SR99_RM_occ_tbl[j+1][i])
				{					
					for (k=0;k<max_onramp_ln;k++)
					{
						if (ln_meteringId[i][k]>0)
						{
							tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][k],timeSta,&amax,&flow,&amin);
							if (k<2)
								actual_r[i][k]=SR99_RM_rate_tbl[j][i];								
							else  // HOV lane														
								actual_r[i][k]=amax;							
						}
						else
							actual_r[i][k]=0.0;
					}
				}
				else
				{
					for (k=0;k<max_onramp_ln;k++)
					{
						if (ln_meteringId[i][k]>0)
						{
							tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][k],timeSta,&amax,&flow,&amin);
							if (k<2)
								actual_r[i][k]=SR99_RM_rate_tbl[N_interv-1][i];
							else  // HOV lane							{								
								actual_r[i][k]=amax;							
						}
						else
							actual_r[i][k]=0.0;
					}
				}				
			} // for j-loop
		}// for i-loop end; upstream use field RM
		
		// activate upstream 5 meter with Caltrsn D3 default meter only
		for(i=0;i<5;i++)  // Upstream using Field Adaptive Metering rate
		{				
			for (j=0;j<max_onramp_ln;j++)
			{
				if(ln_meteringId[i][j]>0)
				{					
					if (actual_r[i][j]>max_RM_rt)
						actual_r[i][j]=max_RM_rt;
					tmp_err1 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, max_RM_rt, actual_r[i][j], amin, time, AKIGetSimulationStepTime());

					// Queue flush for all onramps				
					/*if( (AKIDetGetTimeOccupedAggregatedbyId(OnRamp_Ln_AdvanceDetEndId[i][j],0)> RELEASE_OCC)  )  // 80.0 is too high							
						release_cycle[i][j]=0;			
					if (release_cycle[i][j]<= 2)
					{															
						tmp_err4=ECIChangeParametersFlowMeteringById(ln_meteringId[i][j],timeSta,max_RM_rt,max_RM_rt,dyna_min_r[i]);															
						actual_r[i][j]=max_RM_rt;
						release_cycle[i][j]++;
					}*/
				}
			} // for j-loop end
		} // for i-loop end
		////// Upstream 5 default metering ends


		////////////////////////////////////////////////////////
		// Coorfdinated Opt Metering for downstream 11 Onramps
		////////////////////////////////////////////////////////

		for(i=0;i<NumOnRamp;i++)  
		{
			#ifdef DYNAMIC_BOUNBDS		
			if (detection_s[i+1]->data[Np-1].density < 30)  
				dyna_min_r[i]=700.0;
			else if (detection_s[i+1]->data[Np-1].density < 45)
				dyna_min_r[i]=675.0;
			else if (detection_s[i+1]->data[Np-1].density < 60)
				dyna_min_r[i]=650.0;
			else if (detection_s[i+1]->data[Np-1].density < 75)
				dyna_min_r[i]=625.0;
			else if (detection_s[i+1]->data[Np-1].density < 90)
				dyna_min_r[i]=600.0;
			else if (detection_s[i+1]->data[Np-1].density < 105)
				dyna_min_r[i]=575.0;
			else if (detection_s[i+1]->data[Np-1].density < 120)
				dyna_min_r[i]=550.0;
			else if (detection_s[i+1]->data[Np-1].density < 135)
				dyna_min_r[i]=525.0;
			else 
				dyna_min_r[i]=500.0;
		
			dyna_min_r[i]=dyna_min_r[i]*1.25;								
			dyna_max_r[i]=dyna_min_r[i]+rm_band;
			if (dyna_max_r[i]>max_RM_rt)
				dyna_max_r[i]=max_RM_rt;
			#endif	

			//fprintf(dbg_f,"%10.2f %10.2f ", detection_s[i]->data[Np-1].density, (detection_s[i]->data[Np-1].occupancy));
			
			for (j=0;j<max_onramp_ln;j++)
			{				
				if(ln_meteringId[i+5][j]>0)    //Shuld be   "i+5"
				{
						tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i+5][j],timeSta,&amax,&flow,&amin);  // 
						new_rate=ln_metering_rate[i+5][j];

						// further modify RM rate
											
						if (i==(5-5) || i==(7-5)) // 
						{
							dyna_min_r[i]=dyna_min_r[i]*(1.0-0.35);
							dyna_max_r[i]=dyna_max_r[i]*(1.0-0.35);
							new_rate=new_rate*(1.0-0.35);
						}
						if (i==(6-5) || i==(8-5) || i==(11-5)) // 7: Calvine WB: 
						{
							dyna_min_r[i]=dyna_min_r[i]*(1.0+0.25);
							dyna_max_r[i]=dyna_max_r[i]*(1.0+0.25);
							new_rate=new_rate*(1.0+0.25); 
						}				

						if (dyna_max_r[i]>max_RM_rt)                            // added on 01_08_14
							dyna_max_r[i]=max_RM_rt;						
										
						if (new_rate < dyna_min_r[i])
						{
							//tmp_err3=ECIChangeParametersFlowMeteringById(ln_meteringId[i][j],timeSta,dyna_max_r[i],dyna_min_r[i],dyna_min_r[i-5]);								
							if (j<2)
								actual_r[i+5][j]=dyna_min_r[i];	
							else
								actual_r[i+5][j]=max_RM_rt;  //HOV
						}
						else if(new_rate<=dyna_max_r[i] && new_rate>=dyna_min_r[i])	
						{							
							//tmp_err1=ECIChangeParametersFlowMeteringById(ln_meteringId[i][j],timeSta,dyna_max_r[i],new_rate,dyna_min_r[i-5]);	
							if (j<2)
								actual_r[i+5][j]=new_rate;
							else
								actual_r[i+5][j]=max_RM_rt;  //HOV
						}
						else
						{										
							//tmp_err2=ECIChangeParametersFlowMeteringById(ln_meteringId[i][j],timeSta,dyna_max_r[i],dyna_max_r[i],dyna_min_r[i-5]);	
							if (j<2)
								actual_r[i+5][j]=dyna_max_r[i];
							else
								actual_r[i+5][j]=max_RM_rt;  //HOV
						}
																				
				} //ln_meteringId
				else
					actual_r[i+5][j]=0.0;
				
				
			} // for j-loop end

			//fprintf(dbg_f,"\n");							
		}  // i-llop end downstream

		

		// Activate downstream metering only with queue over-write
		for(i=0;i<NumOnRamp;i++) 
		{
			for (j=0;j<max_onramp_ln;j++)
			{				
				if(ln_meteringId[i+5][j]>0)
				{
					if (actual_r[i+5][j]>max_RM_rt)
						actual_r[i+5][j]=max_RM_rt;
				}
				else
					actual_r[i+5][j]=0.0;

				if(ln_meteringId[i+5][j]>0)
				{
					tmp_err1 = ECIChangeParametersFlowMeteringById(ln_meteringId[i + 5][j], timeSta, max_RM_rt, actual_r[i + 5][j], dyna_min_r[i], time, AKIGetSimulationStepTime());

					// Queue flush for all onramps				
					if( (AKIDetGetTimeOccupedAggregatedbyId(OnRamp_Ln_AdvanceDetEndId[i][j],0)> RELEASE_OCC)  )  // 80.0 is too high							
						release_cycle[i+5][j]=0;			
					if (release_cycle[i+5][j]<= RM_RELEASE_CYCLE)
					{															
						tmp_err4 = ECIChangeParametersFlowMeteringById(ln_meteringId[i + 5][j], timeSta, max_RM_rt, max_RM_rt, dyna_min_r[i], time, AKIGetSimulationStepTime());
						actual_r[i+5][j]=max_RM_rt;
						release_cycle[i+5][j]++;
					}
				}
			}  // for j-loop
			ISUPDATE2=0;
		}  // for i-loop end

		// Output time and RM rate
		//fprintf(vsl_crm_f,"%10.2f\t", timeSta);

		//for(i=0;i<NumOnRamp+5;i++)
		//	fprintf(vsl_crm_f,"%10.2f\t", (float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]));
		//fprintf(vsl_crm_f,"\n");
			
	} // if ISUPDATE2 loop 
	else
		ISUPDATE2++;	
	return true;
}


// This is not used anymore, 11_27_14
bool Set_Opt_Meter(double time,double time2,double timeSta)
{
//	AKIPrintString("metering");
	int i,j, tmp_err, tmp_err1, tmp_err2, tmp_err3, tmp_err4;
	double flow=0.0;
	double amax=0.0;
	double amin=0.0;
	double tmp_occ=0.0;
	static double new_rate=0.0;	
//	char str[len_str];
	//static int release_cycle=0;

	tmp_err=0; tmp_err1=0; tmp_err2=0; tmp_err3=0; tmp_err4=0;

	if(ISUPDATE2>=0)  // set every step
	{
		for(i=0;i<NumOnRamp;i++)
		{
			//for (j=0;j<max_onramp_ln;j++)
			//	Green[i][j]=false;

			#ifdef DYNAMIC_BOUNBDS
			#ifndef USE_FIELD_BOUND
			if (detection_s[i+1]->data[Np-1].density < 30)  
				dyna_min_r[i]=700.0;
			else if (detection_s[i+1]->data[Np-1].density < 45)
				dyna_min_r[i]=675.0;
			else if (detection_s[i+1]->data[Np-1].density < 60)
				dyna_min_r[i]=650.0;
			else if (detection_s[i+1]->data[Np-1].density < 75)
				dyna_min_r[i]=625.0;
			else if (detection_s[i+1]->data[Np-1].density < 90)
				dyna_min_r[i]=600.0;
			else if (detection_s[i+1]->data[Np-1].density < 105)
				dyna_min_r[i]=575.0;
			else if (detection_s[i+1]->data[Np-1].density < 120)
				dyna_min_r[i]=550.0;
			else if (detection_s[i+1]->data[Np-1].density < 135)
				dyna_min_r[i]=525.0;
			else //if (detection_s[i+1]->data[Np-1].density < 150)
				dyna_min_r[i]=500.0;
			//else
			//	dyna_min_r[i]=100.0;

			dyna_min_r[i]=dyna_min_r[i]*1.25;
			#endif
			#ifdef USE_FIELD_BOUND
				tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][0],timeSta,&amax,&flow,&amin);
				dyna_min_r[i]=amin;
			#endif

			
			dyna_max_r[i]=dyna_min_r[i]+rm_band;
			//if (dyna_max_r[i]>max_RM_rt)                            // added on 01_08_14
			//	dyna_max_r[i]=max_RM_rt;

			#endif
			
			//fprintf(dbg_f,"%10.2f %10.2f ", detection_s[i]->data[Np-1].density, (detection_s[i]->data[Np-1].occupancy));
			//fprintf(dbg_f,"%10.2f ", detection_onramp[i]->data[Np-1].occupancy);
		}
		//fprintf(dbg_f,"\n");

		for(i=0;i<NumOnRamp;i++)
		{	
			for (j=0;j<max_onramp_ln;j++)
			{
				
				if(ln_meteringId[i][j]>0)
				{
						tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][j],timeSta,&amax,&flow,&amin);  // this reading is incorrect; causeerrors; 08_30_13													
						new_rate=ln_metering_rate[i][j];

						// further modify RM rate
						/*if (i==5 || i==7) // run multi-day and reps on 2/11~2/12
						{
							dyna_min_r[i]=dyna_min_r[i]*(1.0-0.25);
							dyna_max_r[i]=dyna_max_r[i]*(1.0-0.25);
							new_rate=new_rate*(1.0-0.25);
						}
						if (i==6) // 7: Calvine WB: 
						{
							dyna_min_r[i]=dyna_min_r[i]*(1.0+0.1);
							dyna_max_r[i]=dyna_max_r[i]*(1.0+0.1);
							new_rate=new_rate*(1.0+0.1); 
						}*/
					#ifndef USE_FIELD_BOUND
						if (i==5 || i==7) // 
						{
							dyna_min_r[i]=dyna_min_r[i]*(1.0-0.35);
							dyna_max_r[i]=dyna_max_r[i]*(1.0-0.35);
							new_rate=new_rate*(1.0-0.35);
						}
						if (i==6 || i==8 || i==11) // 7: Calvine WB: 
						{
							dyna_min_r[i]=dyna_min_r[i]*(1.0+0.25);
							dyna_max_r[i]=dyna_max_r[i]*(1.0+0.25);
							new_rate=new_rate*(1.0+0.25); 
						}				

						if (dyna_max_r[i]>max_RM_rt)                            // added on 01_08_14
							dyna_max_r[i]=max_RM_rt;
					#endif	
					
					//#ifdef DYNAMIC_BOUNBDS
						if (new_rate < dyna_min_r[i])
						{
							tmp_err3 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, dyna_max_r[i], dyna_min_r[i], dyna_min_r[i], time, AKIGetSimulationStepTime());
							if (j<2)
								actual_r[i][j]=dyna_min_r[i];	
							else
								actual_r[i][j]=max_RM_rt;  //HOV
						}
						else if(new_rate<=dyna_max_r[i] && new_rate>=dyna_min_r[i])	
						{							
							tmp_err1 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, dyna_max_r[i], new_rate, dyna_min_r[i], time, AKIGetSimulationStepTime());
							if (j<2)
								actual_r[i][j]=new_rate;
							else
								actual_r[i][j]=max_RM_rt;  //HOV
						}
						else
						{										
							tmp_err2 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, dyna_max_r[i], dyna_max_r[i], dyna_min_r[i], time, AKIGetSimulationStepTime());
							if (j<2)
								actual_r[i][j]=dyna_max_r[i];
							else
								actual_r[i][j]=max_RM_rt;  //HOV
						}
						// Queue flush				
						if( (AKIDetGetTimeOccupedAggregatedbyId(OnRamp_Ln_AdvanceDetEndId[i][j],0)> RELEASE_OCC)  )  // 80.0 is too high							
							release_cycle[i][j]=0;			
						if (release_cycle[i][j]<= 2)
							{															
								tmp_err4 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, max_RM_rt, max_RM_rt, dyna_min_r[i], time, AKIGetSimulationStepTime());
				
								actual_r[i][j]=max_RM_rt;
								release_cycle[i][j]++;
							}														
																			
					//#endif
						
						

						//sprintf_s(str,len_str,"Err=%d; time=%10.2lf; time2=%10.2lf; timeSta=%10.2lf;  amin= %10.2lf; amax= %10.2lf; flow=%10.2lf; new_rate=%10.2lf; meter_Id=%5d ",
						//tmp_err, time, time2, timeSta, amin, amax, flow, new_rate, ln_meteringId[i][j]);
						//AKIPrintString(str);

						//fprintf(vsl_crm_f, "%10.2lf ", ln_metering_rate[i][j]);					
				} //ln_meteringId
				else
					actual_r[i][j]=0.0;
			} // for j-loop end

			opt_r[i][0]=(float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]);
		}// for i-loop end	
		//fprintf(vsl_crm_f, "\n");
		ISUPDATE2=0;

		//fprintf(vsl_crm_f,"%10.2f\t", timeSta);
		//for(i=0;i<NumOnRamp;i++)
		//	 fprintf(vsl_crm_f,"%10.2f\t", (float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]));
		//fprintf(vsl_crm_f,"\n");

		/*fprintf(vsl_crm_f,"%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\n",
			timeSta,opt_r[0][0],opt_r[1][0],opt_r[2][0],opt_r[3][0],opt_r[4][0],opt_r[5][0],opt_r[6][0],opt_r[7][0],opt_r[8][0],
			opt_r[9][0],opt_r[10][0],opt_r[11][0],opt_r[12][0], opt_r[13][0],opt_r[14][0],opt_r[15][0]);*/


		/*sprintf_s(str,len_str,"%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f\t%10.2f",
			opt_r[0][0],opt_r[1][0],opt_r[2][0],opt_r[3][0],opt_r[4][0],opt_r[5][0],opt_r[6][0],opt_r[7][0],opt_r[8][0],
			opt_r[9][0],opt_r[10][0],opt_r[11][0],opt_r[12][0], opt_r[13][0],opt_r[14][0],opt_r[15][0]);*/
		//sprintf_s(str,len_str,"%d %d %d %d %d\n", tmp_err, tmp_err1, tmp_err2, tmp_err3, tmp_err4);
		//AKIPrintString(str);

	} // if ISUPDATE2 loop 
	else
		ISUPDATE2++;
	
	return true;
}

bool Set_Coord_ALINEA(double time,double time2,double timeSta)
{
//	AKIPrintString("metering");
	int i,j,tmp_err, tmp_err1, tmp_err2;
	double flow=0.0;
	double amax=0.0;
	double amin=0.0;
	double corridor_mean_occ=0.0, tmp_occ=0.0;
	//int tmp_deno=0;
	char str[len_str];
	//static int release_cycle=0;

	tmp_err=0; tmp_err1=0; 
	if(ISUPDATE2>=0)  // set every step
	{
		for(i=0;i<NumOnRamp;i++)
		{	
			
			for (j=0;j<max_mainline_ln;j++)
			{
				if (Mainline_Ln_Det_Id[i][j]>0)
				{					
					RM_occ[i]+=AKIDetGetTimeOccupedAggregatedbyId(Mainline_Ln_Det_Id[i][j],0);																
				}
			}
			RM_occ[i]=RM_occ[i]/N_Mainline_Ln_RM[i];
			//fprintf(dbg_f,"%10.2f ", RM_occ[i]);
			//fprintf(dbg_f,"%10.2f %10.2f ", SR99_RM_rate_tbl[0][i], SR99_RM_occ_tbl[0][i]);
		}
		corridor_mean_occ=0.0;
		for(i=0;i<NumOnRamp;i++)
			corridor_mean_occ+=RM_occ[i];
		corridor_mean_occ=Maxd(corridor_mean_occ/NumOnRamp, 30.0);

		sprintf_s(str,len_str,"%lf \n", corridor_mean_occ);
		AKIPrintString(str);
		
		for(i=0;i<NumOnRamp;i++)
		{	
			Ramp_rt[i]=0.0;
			for (j=0;j<max_onramp_ln;j++)
			{
				if (ln_meteringId[i][j]>0)
				{
					tmp_err=ECIGetParametersFlowMeteringById(ln_meteringId[i][j],timeSta,&amax,&flow,&amin);
					if (j<2)
					{
						tmp_occ=corridor_mean_occ*(1+(8.0-i)*0.025);  // Upstream operate at higher OCC
						if (RM_occ[i] <= tmp_occ)
							Ramp_rt[i]+=Gain_Up*(tmp_occ-RM_occ[i]);
						if (RM_occ[i] > tmp_occ)
							Ramp_rt[i]+=Gain_Dn*(tmp_occ-RM_occ[i]);

						//actual_r[i][j]=(AKIDetGetTimeOccupedAggregatedbyId(Mainline_Ln_Det_Id[i][j],0)/Maxd(RM_occ[i],1.0))*Ramp_rt[i];
						actual_r[i][j]=Ramp_rt[i]*1.35;
						
					}
					else  // HOV lane														
						actual_r[i][j]=max_RM_rt;							
				}
				else
					actual_r[i][j]=0.0;

				
			} // for j-loop
			
			if (actual_r[i][j]>max_RM_rt)
				actual_r[i][j]=max_RM_rt;
			if (actual_r[i][j]<amin)
				actual_r[i][j]=amin;
		} // for i-loop
		//fprintf(dbg_f,"\n");

		for(i=0;i<NumOnRamp;i++)
		{	
			for (j=0;j<max_onramp_ln;j++)
			{				
				if(ln_meteringId[i][j]>0)
				{
					tmp_err1 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, max_RM_rt, actual_r[i][j], amin, time, AKIGetSimulationStepTime());

				// Queue flush				
					if( (AKIDetGetTimeOccupedAggregatedbyId(OnRamp_Ln_AdvanceDetEndId[i][j],0)> RELEASE_OCC)  )  // 80.0 is too high							
						release_cycle[i][j]=0;			
					if (release_cycle[i][j]<= 2)
					{															
						tmp_err2 = ECIChangeParametersFlowMeteringById(ln_meteringId[i][j], timeSta, max_RM_rt, max_RM_rt, dyna_min_r[i], time, AKIGetSimulationStepTime());
						actual_r[i][j]=max_RM_rt;
						release_cycle[i][j]++;
					}	
				}
			} // for j-loop end
			//opt_r[i][0]=(float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]);
		}// for i-loop end	
		//fprintf(vsl_crm_f, "\n");
		ISUPDATE2=0;

	/*	fprintf(vsl_crm_f,"%10.2f\t", timeSta);
		for(i=0;i<NumOnRamp;i++)
			 fprintf(vsl_crm_f,"%10.2f\t", (float)(actual_r[i][0]+actual_r[i][1]+actual_r[i][2]));
		fprintf(vsl_crm_f,"\n");*/

		//sprintf_s(str,len_str,"%d %d %d %d %d\n", tmp_err, tmp_err1, tmp_err2, tmp_err3, tmp_err4);
		//AKIPrintString(str);

	} // if ISUPDATE2 loop 
	else
		ISUPDATE2++;	

	return true;
}


bool get_state(double time)   // a Major function
{
	get_dmd();          // onramp flow not considered	
	get_meas(time);		
	get_s();
	get_q_main();
	update_q_R();
	
	return true;
}

bool optSpeed(double time)   // a Major function for VSL; not used for CRM; 10_21_13
{
	
	//update_beta();        // for VSL
	//update_alpha();       // for VSL
//	GetOptSpeed(time);	//Occ based I-80 method	
	//SetOptSpeed(time);	//feedback to network
	return true;
}

bool optMeter(double time,double time2,double timeSta)
{
	det_data_4_contr(time);
	get_u_for_opt();  // this is OK
	opt_metering();
	ln_rm_distrib();
	
	if (use_CRM == 1)
		Set_Default_Meter(time,time2,timeSta); 
	if (use_CRM == 2)
	{
		if (OPT_RM == 1)
			Set_Opt_Meter(time,time2,timeSta);  // original Opt RM strategy fro all 16 onramps
		else if (OPT_RM == 2)
			Set_Coord_ALINEA(time,time2,timeSta);
		else if (OPT_RM == 3)
			Set_Hybrid_Meter(time,time2,timeSta);  // upstream use default; downstream 11 onramps use CRM
		else;
	}
	       

	return true;
}

bool readProfile()
{
	/*int k;
	if(count==Np)
	{
		for(k=0;k<Np;k++)
			call_moveprofile(dataProfileUp,1);	//read the data profile for the offramp, the boundary	
		for(k=0;k<Np;k++)
			call_moveprofile(dataProfileUp,1);
	}
	else
	{				
		call_moveprofile(dataProfileUp,1);
	}

	if(count==Np)
	{
		for(k=0;k<Np;k++)
			call_moveprofile(dataProfileDown,1);	//read the data profile for the offramp, the boundary	
		for(k=0;k<Np;k++)
			call_moveprofile(dataProfileDown,1);
	}
	else
	{				
		call_moveprofile(dataProfileDown,1);
	}

	

	for(i=0;i<SecSize;i++)
	{
		if(dataProfileOnRamp[i]->detId!=-1)
		{
			if(count==Np)
			{
				for(k=0;k<Np;k++)
					call_moveprofile(dataProfileOnRamp[i],1);	//read the data profile for the onramp, the boundary	
				for(k=0;k<Np;k++)
					call_moveprofile(dataProfileOnRamp[i],1);
			}
			else
			{				
				call_moveprofile(dataProfileOnRamp[i],1);
			}
		}
		
	}

	for(i=0;i<SecSize;i++)
	{
		if(dataProfileOffRamp[i]->detId!=-1)
		{
			if(count==Np)
			{
				for(k=0;k<Np;k++)
					call_moveprofile(dataProfileOffRamp[i],1);	//read the data profile for the offramp, the boundary	
				for(k=0;k<Np;k++)
					call_moveprofile(dataProfileOffRamp[i],1);
			}
			else
			{				
				call_moveprofile(dataProfileOffRamp[i],1);
			}
		}
		
	}*/
	return true;
}

bool optControl(double time,double time2,double timeSta)
{
	if(count>=Np)
	{
		//readProfile();	    //   read history or boundar data ??? 08_28_13

		get_state(time);
		
		if (use_CRM > 0)
			optMeter(time,time2,timeSta);
	}
	return true;
}
bool Finish()	//should be all the fclose()
{
	delete detection;
	delete section;
	delete detection_s;
	delete detection_onramp;
	delete detection_offramp;
	//delete dataProfileUp;
	//delete dataProfileDown;
	//delete dataProfileOnRamp;
	//delete dataProfileOffRamp;

	return true;	
}