
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include "helper.h"

// disable unsed error for yarp + keep for file
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

typedef VectorOf<uint8_t> MyVector;

void close_and_exit(const char* msg = NULL)
{
    if (msg)
        fprintf(stderr, "exit msg: %s\n", msg);
    else
        fprintf(stderr, "exit\n");
    exit(0);
}


void sig_handler(int)
{
    close_and_exit("sig");
}


int main(int argc, char* argv[])
{
    ClInfo_t cli;
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    int clients = cli.clients;


    if (cli.is_server)
    {
        set_priority(cli.master_prio);

        Network yarp;

        // server ports
        BufferedPort<MyVector> input, output;
        if (!input.open("/server_in") ||
            !output.open("/server_out"))
            close_and_exit("server connect");
        input.setStrict();
        msleep(300);

        // connect to clients
        char client_in[100],
                client_out[100];
        const char *transport = "tcp";
        fprintf(stderr, "transport: %s\n", transport);
        for (int i = 0; i < clients; ++i)
        {
            fprintf(stderr, "connect %d\n", i);
            sprintf(client_in, "/client_in_%d", i);
            sprintf(client_out, "/client_out_%d", i);

            fprintf(stderr, "** /server_out -> %s\n", client_in);
            if (!Network::connect("/server_out", client_in, transport, true))
                close_and_exit("connect");
            fprintf(stderr, "** %s -> /server_in\n", client_out);
            if(!Network::connect(client_out, "/server_in", transport, true))
                close_and_exit("connect");
            fprintf(stderr, "** done\n");
        }

        msleep(1000);

        sleep(3);
        MyVector *inpkt;

        fprintf(stderr, "run\n");
        timeacc_t acc;
        timeacc_init(acc, cli.runs);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);
            MyVector& vout = output.prepare();
            vout.resize(cli.pktsize);
            vout[1] = i;
            output.writeStrict();

            for (int j = 0; j < clients; ++j)
            {
                if (!(inpkt = input.read()))
                    close_and_exit("client rec");
                if ((*inpkt)[1] != (uint8_t)(i+1))
                  close_and_exit("mismatch");
            }
            timeacc_end_round(acc);
        }
        timeacc_print(acc);
        fprintf(stderr, "done\n");

        for (int i = 0; i < clients; ++i)
        {
            fprintf(stderr, "disconnect %d\n", i);
            sprintf(client_in, "/client_in_%d", i);
            sprintf(client_out, "/client_out_%d", i);

            if (!Network::disconnect("/server_out", client_in, false))
                close_and_exit("connect");
            if(!Network::disconnect(client_out, "/server_in", false))
                close_and_exit("connect");
        }
    }
    else
    {
        set_priority(cli.client_prio);

        Network yarp;

        char client_in[100],
                client_out[100];
        sprintf(client_in, "/client_in_%d", cli.clientnum);
        sprintf(client_out, "/client_out_%d", cli.clientnum);

        msleep(200);
        BufferedPort<MyVector> input, output;
        if (!input.open(client_in) ||
            !output.open(client_out))
            close_and_exit("client");
        input.setStrict();
        fprintf(stderr, "client %d up and running\n", cli.clientnum);

        while(1)
        {
            MyVector *vin = input.read();
            if (!vin)
                close_and_exit("client read");

            MyVector& vout = output.prepare();
            vout.resize(cli.pktsize);
            vout[0] = cli.clientnum;
            vout[1] = (*vin)[1]+1;
            output.writeStrict();
        }

    }
    close_and_exit();

    return 0;
}



