
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "helper.h"

// disable unsed error for yarp + keep for file
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

ClInfo_t cli;
typedef VectorOf<uint8_t> MyVector;

void close_and_exit(const char* msg = NULL)
{
    if (msg)
        printf("exit msg: %s\n", msg);
    else
        printf("exit\n");
    exit(0);
}



pthread_mutex_t nw_mutex = PTHREAD_MUTEX_INITIALIZER;
BufferedPort<MyVector> *inputs, *outputs;

void* client(void* n)
{
    size_t num = (size_t)n;
    set_priority(cli.client_prio);

    BufferedPort<MyVector> &input = inputs[num],
            &output = outputs[num];
    input.setStrict();

    for (int i = 0; i < cli.runs; ++i)
    {
        MyVector *vin = input.read();
        if (!vin)
            close_and_exit("client read");

        MyVector& vout = output.prepare();
        vout.resize(cli.pktsize);
        vout[0] = (*vin)[0]+1;
        output.writeStrict();
    }

    // prevent pure virtual method calls or read errors in yarp ...
    while(1) {
        msleep(1000);
    }

    return NULL;
}




int main(int argc, char* argv[])
{
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    int clients = cli.clients;
    pthread_t th[clients];

    if (cli.is_server)
    {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 1024*1024);

        set_priority(cli.master_prio);

        Network yarp;
        Network::setLocalMode(true);

        BufferedPort<MyVector> input, output;
        if (!input.open("/server_in") ||
                !output.open("/server_out"))
            close_and_exit("server connect");
        input.setStrict();

        BufferedPort<MyVector> l_inputs[clients], l_outputs[clients];
        inputs = l_inputs;
        outputs = l_outputs;

        char client_in[100],
                client_out[100];
        for (int i = 0; i < clients; ++i)
        {
            sprintf(client_in, "/client_in_%d", i);
            sprintf(client_out, "/client_out_%d", i);

            if (!l_inputs[i].open(client_in) ||
                !l_outputs[i].open(client_out))
                close_and_exit("client");
        }

        msleep(500);
        for (int i = 0; i < clients; ++i)
        {
            sprintf(client_in, "/client_in_%d", i);
            sprintf(client_out, "/client_out_%d", i);

            if (!Network::connect("/server_out", client_in, "local", true) ||
                    !Network::connect(client_out, "/server_in", "local", true))
                close_and_exit("connect");
        }

        for (int i = 0; i < cli.clients; ++i)
        {
            if (pthread_create( &th[i], &attr, client, (void*)i ) != 0)
                close_and_exit("thread");
        }

        msleep(1000);
        MyVector *inpkt;

        timeacc_t acc;
        timeacc_init(acc, cli.runs);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);

            MyVector& vout = output.prepare();
            vout.resize(cli.pktsize);
            vout[0] = i;
            output.writeStrict();

            for (int j = 0; j < clients; ++j)
            {
                if (!(inpkt = input.read()))
                    close_and_exit("client rec");
                if ((*inpkt)[0] != (uint8_t)(i+1))
                    close_and_exit("mismatch");
            }
            timeacc_end_round(acc);
        }
        timeacc_print(acc);

        input.close();
        output.close();

    }

    close_and_exit();

    return 0;
}

