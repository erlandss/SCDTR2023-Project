#ifndef FILTER_H
#define FILTER_H

class Filter{
    float alpha, beta;
    float data_filtered[4];
    float last_data;
    int n;
public:
    Filter(float _alpha = 0.60);
    ~Filter(){};
    float filter_data(float _data);
};

#endif