#include "moving_filter.h"

void moving_average_filter_init(MovingAverageFilter *filter) {
  // �����������е�Ԫ�س�ʼ��Ϊ0
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filter->window[i] = 0;
  }

  // ��ʼ����������
  filter->index = 0;
  filter->sum = 0;
}

// �����˲���״̬
void update_moving_average_filter(MovingAverageFilter *filter, float newValue) {
  // ��ȥ��ֵ
  filter->sum -= filter->window[filter->index];

  // �����ֵ
  filter->window[filter->index] = newValue;
  filter->sum += newValue;

  // ��������
  filter->index = (filter->index + 1) % WINDOW_SIZE;
}

float get_moving_average_filtered_value(const MovingAverageFilter *filter) {
  // ����ƽ��ֵ
  return filter->sum / WINDOW_SIZE;
}