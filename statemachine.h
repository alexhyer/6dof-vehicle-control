#ifndef STATEMACHINE_H
#define STATEMACHINE_H

int state = 0;

void check_state()
{
    if (state == 0)
    {
        liftoff_time_micros = time_micros;
        if (time_sec >= 10)
        {
            // if (y_pos_gnss < .5 && y_pos_gnss > -.5)
            // {
            // if (z_pos_gnss < .5 && z_pos_gnss > -.5)
            //  {
            state++;
            //  }
            // }
        }
    }

    if (state == 1)
    {
        liftoff_time_micros = time_micros;
        if ((x_accel - 9.81) >= liftoff_threshold)
        {
            state++;
        }
    }

    if (state == 2)
    {
        if (flight_time_sec >= 180)
        {
            state++;
        }
    }
}

#endif