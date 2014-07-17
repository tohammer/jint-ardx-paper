
#include "helper.h"
#include "common_ard.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>


//#define BOUND

ClInfo_t cli;


void close_and_exit(const char* msg = NULL)
{
    if (msg)
        printf("exit msg: %s\n", msg);
    else
        printf("exit\n");
    exit(0);
}


void* client(void* n)
{
    size_t num = (size_t)n;
#ifdef BOUND
    set_cpu_affinity((num+1)%4);
#endif
    ArdComIn *in = connect_shm_in("local_server2client", num, 0, cli.pktsize);
    ArdComOut *out = connect_shm_out("local_client2server", num, 0, cli.pktsize);
    if (!in||!out)
        close_and_exit("client");

    set_priority(cli.client_prio);
    uint8_t *pkt = (uint8_t*)malloc(cli.pktsize);

    while (1)
    {
        if (in->rec(pkt, cli.pktsize) <= 0)
            close_and_exit("client rec");
        pkt[0]++;
        out->send(pkt, cli.pktsize);
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
        ArdComIn *in[clients];
        ArdComOut *out[clients];

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 1024*1024);

        for (int i = 0; i < cli.clients; ++i)
        {
            in[i] = connect_shm_in("local_client2server", i, 1, cli.pktsize);
            out[i] = connect_shm_out("local_server2client", i, 1, cli.pktsize);
            if (!in[i]||!out[i])
                close_and_exit("server");

            if (pthread_create( &th[i], &attr, client, (void*)i ) != 0)
                close_and_exit("thread");
        }

        uint8_t *pkt = (uint8_t*)malloc(cli.pktsize);

#ifdef BOUND
        set_cpu_affinity(0);
#endif
        set_priority(cli.client_prio);
        msleep(500);

        timeacc_t acc;
        timeacc_init(acc, cli.runs);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);

            pkt[0] = i;
            for (int j = 0; j < clients; ++j)
                if (out[j]->send(pkt, cli.pktsize) < 0)
                    close_and_exit("send server");

            for (int j = 0; j < clients; ++j)
            {
                if (in[j]->rec(pkt, cli.pktsize) <= 0)
                    close_and_exit("client rec");
                if (pkt[0] != (uint8_t)(i+1))
                    close_and_exit("mismatch");
            }

            timeacc_end_round(acc);
        }

        timeacc_print(acc);

        msleep(100);
    }

    close_and_exit();

    return 0;
}

