#include "Filter.h"

Filter::Filter(float _alpha)
        : alpha{_alpha}
        {data_filtered[0], data_filtered[1] = 0, data_filtered[2] = 0, data_filtered[3];
         last_data = 0;
         beta = (1-alpha)/2;
         n = 3;}

float Filter::filter_data(float _data){
  // Low Pass Filter
  data_filtered[n] = (alpha/3)*data_filtered[n-1] + (alpha/3)*data_filtered[n-2] 
                    + (alpha/3)*data_filtered[n-3] + beta*_data + beta*last_data;
  // Store the last filtered data and data
  data_filtered[n-3] = data_filtered[n-2];
  data_filtered[n-2] = data_filtered[n-1];
  data_filtered[n-1] = data_filtered[n];
  last_data = _data;
  return data_filtered[n];
}