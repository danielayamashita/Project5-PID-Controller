#include "PID.h"
#include <vector>
#include <iostream>
#include <numeric>
#include <math.h>
using namespace std;
#define EPS 0.01
#define NUM_MAX 20




/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::InitParamsOptimizer() 
{
    #ifdef PRINT_PARAMS_OPTIMIZER
    cout << "------PARAMS_OPTIMIZER-----"<<endl;
    #endif

    Kp = 0.1;
    Ki = 0.0001;
    Kd = 1;

    //small parameters
    dKp = Kp*0.1;
    dKi = Ki*0.1;
    dKd = Kd*0.1;
    maq_Twiddle = TWIDDLE_INIT;
    num_samples = 0;
    num_max = 20;
}

double PID::Twiddle(double  cte,double speed) 
{

    std::vector<double> p;
    p.push_back(Kp);
    p.push_back(Ki);
    p.push_back(Kd);

    static double sumSmallParams;
    static double steer;
    static double err;
    static double best_err;
    static double current_err;
    static bool isFirst = true;
    static int Idx;

    // Read parameters and save into vectors
    std::vector<double> dp;
    dp.push_back(dKp);
    dp.push_back(dKi);
    dp.push_back(dKd);


    #ifdef PRINT_PARAMS_OPTIMIZER
    for (int i = 0 ; i < dp.size(); i++)
    {
        cout << "dp["<<i<<"]: "<< dp[i]<< endl;
    }
    #endif

    num_samples ++;
    switch (maq_Twiddle)
    {
        case TWIDDLE_INIT:
            #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "------TWIDDLE_INIT-----"<<endl;
                cout << "num_samples:"<< num_samples<< endl;
            #endif
            err += pow(cte,2);
            if (num_samples >= num_max)
            { 
                num_samples = 0;
                best_err = err;
                current_err =err;
                err = 0;
                maq_Twiddle = TWIDDLE_EVALUATE;
                #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "best_err:"<<best_err<<endl;
                cout << "current_err:"<<current_err<< endl;
                #endif
            }

        break;

        case TWIDDLE_RUN:
            #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "------TWIDDLE_RUN-----"<<endl;
                cout << "num_samples:"<< num_samples<< endl;
            #endif
            err += pow(cte,2);
            if (num_samples >= num_max)
            { 
                //num_samples = 0;
                current_err =err;
                err = 0;
                #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "maq_Twiddle_previous:"<< maq_Twiddle_previous<< endl;
                cout << "best_err:"<<best_err<<endl;
                cout << "current_err:"<<current_err<< endl;
                #endif
                #ifdef PRINT_PARAMS_OPTIMIZER2
                    cout << "------TWIDDLE_RUN-----"<<endl;
                     cout << "num_samples:"<< num_samples<< endl;
                    cout << "maq_Twiddle_previous:"<< maq_Twiddle_previous<< endl;
                #endif
                switch (maq_Twiddle_previous)
                {
                    case TWIDDLE_UPDATE_PARAMS_STEP1:
                        maq_Twiddle = TWIDDLE_UPDATE_PARAMS_STEP2;
                    break;
                    case TWIDDLE_UPDATE_PARAMS_STEP2:
                        if (current_err < best_err)
                        {
                            best_err =current_err;
                            dp[Idx]*=1.1;
                            #ifdef PRINT_PARAMS_OPTIMIZER2
                            cout << "CASE 1 dp:"<< dp[Idx]<< endl;
                            #endif
                        }
                        else
                        {
                            p[Idx] +=dp[Idx];
                            dp[Idx]*=0.9;
                            #ifdef PRINT_PARAMS_OPTIMIZER2
                            cout << "CASE 2 dp:"<< dp[Idx]<< endl;
                            #endif
                        }
                        maq_Twiddle = TWIDDLE_UPDATE_PARAMS_STEP1;
                    break;
                } 
            }

        break;

        case TWIDDLE_UPDATE_PARAMS_STEP1:
            #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "------TWIDDLE_UPDATE_PARAMS_STEP1-----"<<endl;
            #endif
            Idx = paramCount % dp.size();
            //cout << "Idx"<< Idx<<endl;

            p[Idx] += dp[Idx];
            paramCount++;
            maq_Twiddle = TWIDDLE_RUN;
            num_samples = 0;
            maq_Twiddle_previous = TWIDDLE_UPDATE_PARAMS_STEP1;

            if ((Idx == 0) && (isFirst == false))
            {
                //cout << "EVALUATION"<< Idx<<endl;

                if (num_max>=NUM_MAX)
                {
                    num_max = NUM_MAX;
                }
                else
                {
                    num_max +=1;
                }
                sumSmallParams = dp[0]+dp[1]+dp[2];
                 if(sumSmallParams < EPS)
                    {
                        maq_Twiddle = TWIDDLE_END;

                    }
            }

            if (isFirst == true)
            {
                isFirst = false;
            }


            

            
        break;

        case TWIDDLE_UPDATE_PARAMS_STEP2:
            #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "------TWIDDLE_UPDATE_PARAMS_STEP2-----"<<endl;
                cout << "best_err: " << best_err<< endl;
                cout << "current_err: " << best_err<< endl;
            #endif
            if (current_err < best_err)
            {
                best_err = current_err;
                dp[Idx] *= 1.1;
                maq_Twiddle = TWIDDLE_UPDATE_PARAMS_STEP1;
            }
            else
            {
                p[Idx] -= 2* dp[Idx];
                maq_Twiddle = TWIDDLE_RUN;
                num_samples = 0;
            }
            maq_Twiddle_previous = TWIDDLE_UPDATE_PARAMS_STEP2;
        break;

        case TWIDDLE_EVALUATE:
            sumSmallParams  =  dp[0] + dp[1] + dp[2];
            #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "------TWIDDLE_EVALUATE-----"<<endl;
                cout << "sumSmallParams" << sumSmallParams << endl;
            #endif
           
            if(sumSmallParams > EPS)
            {
                maq_Twiddle = TWIDDLE_UPDATE_PARAMS_STEP1;
            }
            else
            {
                maq_Twiddle = TWIDDLE_END;
            }

        break;

        case TWIDDLE_END:
            #ifdef PRINT_PARAMS_OPTIMIZER
                cout << "------END PARAMS_OPTIMIZER-----"<<endl;
            #endif
            #ifdef PRINT_FINAL_PARAMS
            cout << "p:"<<p[0]<<","<<p[1]<<","<<p[2] <<endl;
            #endif
            maq_Twiddle = TWIDDLE_STOP;
        break;
        case TWIDDLE_STOP:
        break;


    }
    

    Kp = p[0];
    Ki = p[1];
    Kd = p[2];

    

    dKp = dp[0];
    dKi = dp[1];
    dKd = dp[2];

    #ifdef PRINT_PARAMS_OPTIMIZER
    cout << "p:"<<p[0]<<","<<p[1]<<","<<p[2] <<endl;
    #endif 
    
    UpdateError(cte);
    steer = TotalError();

    



    return steer;
}



void PID::Init(double Kp, double Ki, double Kd) {
    #ifdef PRINT_INIT
    cout << "------PID INIT-----"<<endl;
    #endif
    
    //parameters
    PID::Kd = Kd;
    PID::Ki = Ki;
    PID::Kp = Kp;
    
    

    //previous CTE
    previous_cte = 0;

}

void PID::UpdateError(double cte) {
    #ifdef PRINT_UPDATE_ERROR
    cout << "------UPDATE ERROR-----"<<endl;
    #endif
    p_error = cte;
    i_error += cte;
    d_error = cte - previous_cte;
    previous_cte = cte;

}

double PID::TotalError() {
    double TotalErr  = -Kp*p_error - Kd*d_error - Ki*i_error;
 return TotalErr;
}

