/**
 * @file simple_timer.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-12-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Arduino.h>

class SimpleTimer
{
public:
    SimpleTimer() { memset(this, 0, sizeof(SimpleTimer)); }
    void begin()
    {
        t_begin = micros();
    }
    const unsigned long &elapsed()
    {
        t_duration = micros() - t_begin;
        return t_duration;
    }
    const unsigned long &get_time_duration()
    {
        return t_duration;
    }

protected:
    unsigned long t_begin;
    unsigned long t_duration;
};