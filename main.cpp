#include <iostream>
#include<cmath>
#include<fstream>
using namespace std;


float *time_series(float t_start,float t_r,float t_end,int n);

float **model(float x0,float y0,float theta0,float v,float w,float dt,int n_t);

int main()
{   int n_t=100;
    float t_s,dt,t_e;
    float x0,y0,theta0;
    float v,w;
    float *T=new float [n_t];

//initialization
    t_s = 0,dt=0.1,t_e=10;//time simulation values
    cout<<"start="<<t_s<<",\trange="<<dt<<",\tEnd="<<t_e<<endl;
    cout<<endl;
    cout<<"initial state values:"<<endl;
    x0 = 0,y0 = 0, theta0=0; //initial state values
    cout<<"x0="<<x0<<"\n y0="<<y0<<"\ntheta="<<theta0<<endl;
    x[0]=x0,y[0]=y0,theta[0]=theta0;
    v = 0.5,w = 1;
// time series generation
    T = time_series(t_s,dt,t_e,n_t);
//calculation of the system's states according to the time series T
    float **state = model(x0,y0,theta0,v,w,dt,n_t);

//creation a data.csv file
    ofstream file("C:\\Users\\mkhaleghi\\Documents\\C++\\EKF_pioneerp3dx\\data.csv");
//saving the data in the data.csv file
    if (file.is_open()) {
        file << "Time,x,y,theta\n";
        for (int i = 0; i <= n_t; i++) {
            file << T[i] << "," << state[0][i] << "," << state[1][i] << "," << state[2][i] << "\n";
        }
        file.close();
            cout << "CSV file created and the data was saved successfully.\n";
    } else {
            cout << "Unable to open file.\n";
    }

    //Extended Kalman Filter
    float x_pri = x0,y_pri = y0, theta_pri = theta0; //initilizing of the system states
    float x_pred,y_pred,theta_pred;

    //Prediction
    x_pred = x_pri + v*cos(theta_pri)*dt;
    y_pred = y_pri + v*sin(theta_pri)*dt;
    theta_pred = theta_pri + w*dt;
    //F Jacobian
    float F[3][3] = {{1,0,-v*sin(theta_pri)*dt},
                     {0,1,v*cos(theta_pri)*dt},
                     {0,0,1}}
    //Covariance matrix P

    //correction

    //H Jacobian

    //Kalman Gain updating

    //state updating

    //Covariance matrix P updating

    delete[] T;
    delete[] state[0];
    delete[] state[1];
    delete[] state[2];
    delete[] state;
    return 0;
}

//model of the system (pioneer_p3dx robot)
float **model(float x0,float y0,float theta0,float v,float w,float dt,int n_t){
    //definition of pointer "state"
    float **state = new float *[3];
    state[0] = new float[n_t+1];
    state[1] = new float[n_t+1];
    state[2] = new float[n_t+1];

    //initialization of the state
    state[0][0] = x0,state[1][0] = y0;state[2][0] = theta0;
    //nonlinear equation of the system
    for(int j=0;j<=n_t;j++){
        state[0][j+1] = state[0][j]+v*cos(state[2][j])*dt;
        state[1][j+1] = state[1][j]+v*sin(state[2][j])*dt;
        state[2][j+1] = state[2][j]+w*dt;
    }
    return state;
}

//time series generation
float *time_series(float t_start,float t_r,float t_end,int n){
    float c1=t_start;
    float *t=new float [n];
    for(int i=0;i<=n;i++){
        *(t+i) = c1;
        c1+=t_r;
    }
    return t;
}
