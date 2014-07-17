
#include "helper.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "normal/packet_4.h"
#include "normal/packet_100.h"
#include "normal/packet_1000.h"
#include "normal/packet_10000.h"
#include "normal/packet_100000.h"
#include "normal/packet_1000000.h"
#include "normal/packet_10000000.h"
#include "normal/packet_100000000.h"
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

        if      (cli.pktsize == 4)          run_template<normal::packet_4>();
        else if (cli.pktsize == 100)        run_template<normal::packet_100>();
        else if (cli.pktsize == 1000)       run_template<normal::packet_1000>();
        else if (cli.pktsize == 10000)      run_template<normal::packet_10000>();
        else if (cli.pktsize == 100000)     run_template<normal::packet_100000>();
        else if (cli.pktsize == 1000000)    run_template<normal::packet_1000000>();
        else if (cli.pktsize == 10000000)   run_template<normal::packet_10000000>();
        else if (cli.pktsize == 100000000)  run_template<normal::packet_100000000>();
        else close_and_exit("unsupported pktsize");
    }

    return 0;
}


