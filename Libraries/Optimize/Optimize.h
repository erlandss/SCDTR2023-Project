#ifndef OPTIMIZE_H
#define OPTIMIZEH
#include "Arduino.h"
#include <cstring>
float dot_product(float v1[],float v2[],int length){
    int i;
    float product=0;


        for(int i=0;i<length;i++){
            product += v1[i] * v2[i];
        }
            
        return product;

}
float* add_arrays(float v1[],float v2[],int length){
    float* ans =new float[3];
    int i;

        for(int i=0;i<length;i++){
            ans[i]=v1[i]+v2[i];
        }
        return ans;
        


}
float* scalarProductArray(float arr[], float k,int length)
{   
    float* ans=new float[3];

    // scalar element is multiplied by the array
    for (int i = 0; i < length; i++){
        ans[i]=k*arr[i];
    }
    return ans;
}


class Optimizer{
    private:
    const int N=3;
    float* d =new float[3];
    float* d_bar =new float[3];
    float* c;
    float* y=new float[3];
    float rho;
    float* K;//coupling gains for node i
    float* e=new float[3];// should be set to delta_i
    float o; //external disturbance
    float L; //reference in LUX
    int index=0;
    public:

    explicit Optimizer(float _rho,float _o,float l,float* _K,float* _c):
    rho{_rho},o {_o},L {l}, K {_K}, c {_c}
    {

    }
    ~Optimizer(){
        delete[] d_bar;
        delete[] e;
        delete[] y;
        delete[] d;
    };

    void initOptimizer(){
        for (int k=0;k<3;k++){
            d_bar[k]=0;
            y[k]=0;
            d[k]=100;
            // d[k]=1;
            if(k==index){
                e[k]=1;
            }else{
                e[k]=0;
            }
            

        }
    }
    void solveQP(){

        float* z=computeZ();
        //Array containing the solutions for each constraint(s) and global solution
        // float* solutions[6];
        //Global solution:
        float* sol0Temp;//dealloc
        float* sol1Temp;//dealloc
        float k_squared = dot_product(K,K,N);
        float kz = dot_product(K,z,N);
        float rhoInv =(float)(1/rho);
        float* sol0;
        float lowestVal1=10000000;
        sol0 =scalarProductArray(z,(1/rho),N);
        // solutions[0]=sol0;

        sol0Temp=scalarProductArray(K,(float)-1,N);
        sol1Temp=scalarProductArray(e,(float)-1,N);

        if(dot_product(sol0Temp,sol0,N)<=(o-L) && dot_product(sol1Temp,sol0,N)<=0 && dot_product(e,sol0,N)<=100){
            //Global solution is feasible
            d=sol0;
        }else{

        for(int i=1;i<6;i++){
            float* d_opt = new float[3];

            float* temp1;
            float* temp2;
            switch (i)
            {
            case 1:
                temp1=scalarProductArray(z,rhoInv,N);
                temp2=scalarProductArray(K,-1*(o-L+rhoInv*kz)/(k_squared),N);
                d_opt =add_arrays(temp1,temp2,N);
                delete[] temp1;
                delete[] temp2;

                break;
            case 2:
                temp1=scalarProductArray(z,rhoInv,N);
                temp2=scalarProductArray(e,-1*rhoInv*z[index],N);
                d_opt=add_arrays(temp1,temp2,N);
                delete[] temp1;
                delete[] temp2;
                
                break;
            case 3:
                temp1=scalarProductArray(z,rhoInv,N);
                temp2=scalarProductArray(e,(100-rhoInv*z[index]),N);
                d_opt=add_arrays(temp1,temp2,N);
                delete[] temp1;
                delete[] temp2;
                break;
            case 4:
                for (int j=0;j<3;j++){
                    if(j==index){
                        d_opt[j]=0;
                    }else{
                       d_opt[j]=rhoInv*z[j]-K[j]*(o-L)/(k_squared-K[index] * K[index])-rhoInv*(-1*K[j])*(-1*kz+K[index]*z[index])/(k_squared-K[index] * K[index]);
                    }
                }


                break;
            case 5:
                for (int j=0;j<3;j++){
                    if(j==index){
                        d_opt[j]=100;
                    }else{
                       d_opt[j]=rhoInv*z[j]-(K[j]*(o-L)+100*K[j]*K[index])/(k_squared-K[index] * K[index])-rhoInv*(-1*K[j])*(-1*kz+K[index]*z[index])/(k_squared-K[index] * K[index]);
                    }
                }

                break;

            default:
                break;
            }
            // for (int k=0;k<3;k++){
            //     Serial.print(d_opt[k]);
            //     Serial.print(" ");
            // }
            // Serial.print("COST1:\n");
            // Serial.print(costFunction(d_opt));
            // Serial.print("\n");

            // solutions[i]=d_opt;

            // for (int k=0;k<3;k++){
            //     Serial.print(solutions[i][k]);
            //     Serial.print(" ");
            // }
            // Serial.print("COST2:\n");
            // Serial.print(costFunction(d_opt));
            // Serial.print("\n");
        //SJEKK OM NEW ER BRUKT RIKTIG! 
            
            // Serial.print("COST2:\n");
            // Serial.print(costFunction(solutions[i]));
            // Serial.print("\n");

            // &&(dot_product(sol0Temp,d_opt,N)<=(o-L) && dot_product(sol1Temp,d_opt,N)<0 && dot_product(e,d_opt,N)<=100)

            if(costFunction(d_opt)<costFunction(d) && (dot_product(sol0Temp,d_opt,N)<=(o-L) && dot_product(sol1Temp,d_opt,N)<0 && dot_product(e,d_opt,N)<=100) ){
                // Serial.print("Current CF:");

                // Serial.print(costFunction(d_opt));
                // Serial.print(" ");
                // Serial.print(costFunction(d));
                // Serial.print("\n");
                
                // for (int k=0;k<3;k++)
                // {
                //     Serial.print(d_opt[k]);
                //     Serial.print(" ");
                // }
                // Serial.print("\n");
                // for (int k=0;k<3;k++)
                // {
                //     Serial.print(d[k]);
                //     Serial.print(" ");
                // }
                // Serial.print("\n");
                d=d_opt;
                // lowestVal1=costFunction(d);

            }
            // for (int k=0;k<3;k++){
            //     Serial.print(d[k]);
            //     Serial.print(" ");
            // }
            delete[] d_opt;
        }

        // float lowestVal=10000000;
        // int ind=0;
        // for (int i =1; i<6;i++){
        //     for (int k=0;k<3;k++){
        //         Serial.print(solutions[i][k]);
        //         Serial.print(" ");
        //     }
        //     Serial.print("\n");

        //     // if(dot_product(scalarProductArray(K,(float)-1,N),solutions[0],N)<=(o-L) && dot_product(scalarProductArray(e,(float)-1,N),solutions[0],N)<=0 && dot_product(e,solutions[0],N)<=100){
        //     if(costFunction(solutions[i])<lowestVal){
        //         ind =i;
        //         lowestVal=costFunction(solutions[i]);
        //     }
        // }
        // }
        // d=solutions[ind];


        }
        // for (int i=0;i<6;i++){
        //     delete[] solutions[i];
        // }
        delete[] z;
        delete[] sol0;
        delete[] sol0Temp;
        delete[] sol1Temp;
    }

    // float* computeZ(){
    //     float* z;
    //     z = add_arrays((scalarProductArray(d_bar,rho,N)),scalarProductArray(c,(float)(-1),N),N);
    //     z=add_arrays(z,scalarProductArray(y,(float)(-1),N),N);
    //     return z;
    // }
    float* computeZ(){
    float* z = new float[N];
    for(int i=0;i<N;i++){
        z[i] = d_bar[i]*rho - c[i] - y[i];
    }
    return z;
}
    // float costFunction(float di[]){
    //     return 0.5*rho*dot_product(di,di,N) - dot_product(di,computeZ(),N);
    // }
    float costFunction(float di[]) {
        float di_copy[3];
        memcpy(di_copy, di, 3 * sizeof(float)); 

        float* z = computeZ();
        float cost = 0.5 * rho * dot_product(di_copy, di_copy, N) - dot_product(di_copy, z, N);

        delete[] z;
    return cost;
}
    void update_dbar(float d1[],float d2[],float d3[]){
        float* temp1;
        float* temp2;
        temp1=add_arrays(d1,d2,N);
        temp2 = add_arrays(temp1,d3,N);
        d_bar = scalarProductArray(temp2,(float)(1/N),N);
        delete[] temp1;
        delete[] temp2;
    }
    void update_y(){
        float* temp1;
        float* temp2;
        float* temp3;
        temp1=scalarProductArray(d_bar,(float)(-1),N);
        temp2=add_arrays(d,temp1,N);
        temp3=scalarProductArray(temp2,rho,N);
        y = add_arrays(y,temp3,N);
        delete[] temp1;
        delete[] temp2;
        delete[] temp3;
    }
    float* get_d(){
      return d;
    }
    float* get_dbar(){
      return d_bar;
    }
    float* get_y(){
      return y;
    }


};


#endif