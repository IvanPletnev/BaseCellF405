/*
 * utilites.h
 *
 *  Created on: 30 июн. 2021 г.
 *      Author: ivan
 */

#ifndef INC_UTILITES_H_
#define INC_UTILITES_H_

#include "main.h"

#define FILTER_LEN 10
#define CHANNELS 3

typedef struct _filterType
{
    int16_t filterData[FILTER_LEN];    // данные фильтра
    int32_t sum;                        // текущая сумма
    int16_t top;                        // указатель на текущую выборку
} filterType;      // упаковать данные

// определяем масcив данных фильтра
extern filterType currentFilter[CHANNELS];        // как внешний
extern filterType voltageFilter[CHANNELS];

int16_t filtering(int16_t input_data, filterType * flt);


#endif /* INC_UTILITES_H_ */
