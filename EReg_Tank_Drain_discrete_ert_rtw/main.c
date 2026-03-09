#include "EReg_Tank_Drain_discrete.h"

int main(void)
{
    /* initialize model */
    EReg_Tank_Drain_discrete_initialize();

    while (1)
    {
        /* run one timestep */
        EReg_Tank_Drain_discrete_step();
    }

    return 0;
}