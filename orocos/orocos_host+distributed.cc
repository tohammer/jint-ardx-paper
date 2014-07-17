



#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <ocl/TaskBrowser.hpp>
#include <rtt/os/main.h>

using namespace RTT::corba;
using namespace RTT;
using namespace OCL;


#include "orocos_common.h"
#include "helper.h"

ClInfo_t cli;

MAKE_PLUGINS()


int ORO_main(int argc, char** argv)
{
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    register_packet();

    TaskContextServer::InitOrb(argc, argv);

    if (cli.is_server)
    {
        PerfMaster the_master("perf_master", cli, 1);
        the_master.start();

        TaskContextServer::Create(&the_master, true, true);
        TaskContextServer::RunOrb();
        TaskContextServer::DestroyOrb();
    }
    else
    {
        char buffer[100];
        sprintf(buffer, "client_%d", cli.clientnum);
        PerfClient the_client(buffer, cli);
        the_client.start();

        TaskContextServer::Create(&the_client, true, true);
        TaskContextServer::RunOrb();
        TaskContextServer::DestroyOrb();
    }

    return 0;
}
