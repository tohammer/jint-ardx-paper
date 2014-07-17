



#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>
#include <rtt/types/SequenceTypeInfo.hpp>

using namespace RTT::corba;
using namespace RTT;
using namespace OCL;
using namespace RTT::types;


#include "orocos_common.h"
#include "helper.h"

ClInfo_t cli;


MAKE_PLUGINS()

int ORO_main(int argc, char** argv)
{
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    int clients = cli.clients;
    if (cli.client_prio > 0)
        set_priority(cli.client_prio-2);

    register_packet();

    TaskContextServer::InitOrb(argc, argv);

    // clients
    PerfClient *cl[clients];
    char buffer[100];
    for (int i = 0; i < clients; ++i)
    {
        sprintf(buffer, "client_%d", i);
        cli.clientnum = i;
        cl[i] = new PerfClient(buffer, cli);
        cl[i]->start();
        TaskContextServer::Create(cl[i], true, true);
    }

    PerfMaster the_master("perf_master", cli, 0);
    the_master.start();

    TaskContextServer::Create(&the_master, true, true);

    // Wait for requests
    TaskContextServer::RunOrb();

    // Cleanup Corba:
    TaskContextServer::DestroyOrb();
    return 0;
}


