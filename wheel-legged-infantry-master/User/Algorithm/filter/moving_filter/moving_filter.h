#ifndef MOVING_FILTER_H
#define MOVING_FILTER_H

#define WINDOW_SIZE 20

typedef struct{
  float window[WINDOW_SIZE];
  int index;
  float sum;
} MovingAverageFilter;

void moving_average_filter_init(MovingAverageFilter *filter);
void update_moving_average_filter(MovingAverageFilter *filter, float newValue);
float get_moving_average_filtered_value(const MovingAverageFilter *filter);

#endif
