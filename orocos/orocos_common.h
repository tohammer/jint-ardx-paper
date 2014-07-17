#ifndef OROCOS_FUN_H
#define OROCOS_FUN_H


#include <rtt/transports/corba/TaskContextServer.hpp>

#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/main.h>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/transports/mqueue/MQSerializationProtocol.hpp>
#include <rtt/transports/corba/CorbaTemplateProtocol.hpp>
#include <vector>

using namespace RTT;
using namespace RTT::corba;
using namespace RTT::types;
using namespace RTT::mqueue;

#include "helper.h"


extern ClInfo_t cli;



// *** the type
#include <rtt/types/TypekitRepository.hpp>
#include <rtt/os/ThreadInterface.hpp>
#include "typekit/Plugin.hpp"
#include "typekit/transports/corba/TransportPlugin.hpp"
#include "typekit/transports/mqueue/TransportPlugin.hpp"
#include "typekit/transports/typelib/TransportPlugin.hpp"
#include "orocos_packet.h"

#define MAKE_PLUGINS() \
    namespace orogen_typekits { \
    commtestTypekitPlugin commtestTypekit; \
    commtestCorbaTransportPlugin commtestCorbaTransport; \
    commtestMQueueTransportPlugin commtestMQueueTransport; \
    commtestTypelibTransportPlugin commtestTypelibTransport; \
    }

void register_packet()
{
    RTT::types::TypekitRepository::Import(&orogen_typekits::commtestTypekit);
    RTT::types::TypekitRepository::Import(&orogen_typekits::commtestCorbaTransport);
    RTT::types::TypekitRepository::Import(&orogen_typekits::commtestMQueueTransport);
    RTT::types::TypekitRepository::Import(&orogen_typekits::commtestTypelibTransport);
}


// *** callback functor
template <typename T>
class Fun
{
    T *m_p;
public:
    Fun(T* p)
        : m_p(p)
    {
    }

    void operator()(base::PortInterface* p)
    {
        m_p->event((InputPort<Packet>*)p);
    }
};


// *** exit function
void close_and_exit(const char* msg = NULL)
{
    if (msg)
        printf("exit msg: %s\n", msg);
    else
        printf("exit\n");
    exit(0);
}


// *** helper
#include <rtt/os/ThreadInterface.hpp>

void set_oro_prio(TaskContext *tc, int prio, const char* user)
{
    os::ThreadInterface *ti = tc->engine()->getThread();
    if (prio != -1)
    {
        if (!ti->setScheduler(ORO_SCHED_RT) ||
            !ti->setPriority(prio))
        {
            fprintf(stderr, "ERROR setting prio for %s to %d\n", user, prio);
            exit(1);
        }
        else
            fprintf(stderr, "set prio for %s to %d\n", user, prio);
    }
}


// *** the client
class PerfClient
    : public RTT::TaskContext
{
    InputPort<Packet> input;
    OutputPort<Packet> output;
    Fun<PerfClient> ff;
    ClInfo_t cli;
    Packet pkt, sample_pkt;

public:
    PerfClient(string const& name, const ClInfo_t& cc)
        : TaskContext(name),
          input("get"),
          output("put", false),
          ff(this),
          cli(cc)
    {
        this->ports()->addEventPort( input, ff );
        this->ports()->addPort( output );
        std::cerr << "client " << cli.clientnum <<std::endl;
        sample_pkt.data.resize(cli.pktsize);
        output.setDataSample(sample_pkt);
        set_priority(cli.client_prio);
        set_oro_prio(this, cli.client_prio, "client");

        CorbaDispatcher * disp = CorbaDispatcher::Instance(this->ports());
        if (cli.client_prio > 0)
        {
            disp->setScheduler(ORO_SCHED_RT);
            disp->setPriority(cli.client_prio);
        }
    }

    void event(InputPort<Packet>* p)
    {
        input.read(pkt);
        *((int*)&pkt.data[0]) += 1;
        output.write(pkt);
    }

    virtual void exceptionHook ()
    {
        exit(1);
    }
};



#include <rtt/transports/corba/CorbaDispatcher.hpp>

// *** the master
class PerfMaster : public TaskContext
{
    InputPort<Packet> input;
    OutputPort<Packet> output;
    Fun<PerfMaster> ff;
    ClInfo_t cli;
    int r, resp_cnt;
    timeacc_t acc;
    Packet send_pkt, recv_pkt, sample_pkt;
    int transport;

public:
    PerfMaster(string const& name, const ClInfo_t& cc, int transport_)
        : TaskContext(name),
          input("get"),
          output("put", false),
          ff(this),
          r(0),
          cli(cc),
          resp_cnt(0),
          transport(transport_)
    {
        this->ports()->addEventPort( input, ff );
        this->ports()->addPort( output );
        send_pkt.data.resize(cli.pktsize);
        sample_pkt.data.resize(cli.pktsize);
        output.setDataSample(sample_pkt);
    }


    bool startHook() {
        std::cerr << "connect" <<std::endl;
        ConnPolicy cp = ConnPolicy::data();
        if (transport == 0)  // local
            cp.transport = 0;
        else if (transport == 1)  // hostlocal
        {
            cp.transport = 0; //ORO_MQUEUE_PROTOCOL_ID;
        }
        else if (transport == 2) // global
        {
            cp.transport = 0; //ORO_CORBA_PROTOCOL_ID;
        }

        char buffer[100];
        for (int i = 0; i < cli.clients; ++i)
        {
            sprintf(buffer, "client_%d", i);
            TaskContext* cl = TaskContextProxy::Create( buffer );

            this->ports()->getPort("get")->connectTo( cl->ports()->getPort("put"), cp);
            cl->ports()->getPort("get")->connectTo( this->ports()->getPort("put"), cp);
        }


        CorbaDispatcher * disp = CorbaDispatcher::Instance(this->ports());
        if (cli.master_prio > 0)
        {
            disp->setScheduler(ORO_SCHED_RT);
            disp->setPriority(cli.master_prio);
        }

        msleep(2000);

        set_priority(cli.master_prio);
        set_oro_prio(this, cli.master_prio, "master");

        std::cerr << "run" <<std::endl;

        r = 0;
        resp_cnt = 0;
        timeacc_init(acc, cli.runs);

        timeacc_start_round(acc);
        *((int*)&send_pkt.data[0]) = r;
        output.write(send_pkt);
        return true;
    }

    void event(InputPort<Packet>* p)
    {
        p->read(recv_pkt);

        if ( *((int*)&recv_pkt.data[0]) != r+1)
            close_and_exit("data mismatch");

        resp_cnt++;

        if (resp_cnt == cli.clients)
        {
            timeacc_end_round(acc);

            r++;
            resp_cnt = 0;

            if (r < cli.runs)
            {
                timeacc_start_round(acc);
                *((int*)&send_pkt.data[0]) = r;
                output.write(send_pkt);
            }
            else
            {
                timeacc_print(acc);
                exit(0);
            }
        }
    }

    virtual void exceptionHook ()
    {
        exit(1);
    }
};




#endif //OROCOS_FUN_H
