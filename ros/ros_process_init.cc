
#include "helper.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "normal/packet_4_init.h"
#include "normal/packet_100_init.h"
#include "normal/packet_1000_init.h"
#include "normal/packet_10000_init.h"
#include "normal/packet_100000_init.h"
#include "normal/packet_1000000_init.h"
#include "normal/packet_10000000_init.h"
#include "normal/packet_100000000_init.h"
#include "ros_common.h"

ClInfo_t cli;
timeacc_t acc;



int main(int argc, char* argv[])
{
    if (!parse_cl(argc, argv, &cli))
        exit(0);

    if (cli.is_server)
    {
        set_priority(cli.master_prio);
        ros::init(argc, argv, "server");

        if      (cli.pktsize == 4)          run_template<normal::packet_4_init>();
        else if (cli.pktsize == 100)        run_template<normal::packet_100_init>();
        else if (cli.pktsize == 1000)       run_template<normal::packet_1000_init>();
        else if (cli.pktsize == 10000)      run_template<normal::packet_10000_init>();
        else if (cli.pktsize == 100000)     run_template<normal::packet_100000_init>();
        else if (cli.pktsize == 1000000)    run_template<normal::packet_1000000_init>();
        else if (cli.pktsize == 10000000)   run_template<normal::packet_10000000_init>();
        else if (cli.pktsize == 100000000)  run_template<normal::packet_100000000_init>();
        else close_and_exit("unsupported pktsize");
    }

    return 0;
}

