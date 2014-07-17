
#include "helper.h"
#include "common_ard.h"
#include <stdio.h>
#include <stdlib.h>


ClInfo_t cli;


void close_and_exit(const char* msg = NULL)
{
    if (msg)
        printf("exit msg: %s\n", msg);
    else
        printf("exit\n");
    exit(0);
}


void init_connection( ArdComIn** in, ArdComOut** out)
{
    uint8_t *pkt = (uint8_t*)malloc(cli.pktsize);
    int ready_num = 0;
    do
    {
        ready_num = 0;
        for (int j = 0; j < cli.clients; ++j)
            out[j]->send(pkt, cli.pktsize);
        for (int j = 0; j < cli.clients; ++j)
            if (in[j]->rec(pkt, cli.pktsize) >= 0)
                ready_num++;

        fprintf(stderr, "ready: %d / %d\n", ready_num, cli.clients);
    } while (ready_num < cli.clients);

    msleep(200);
    for (int j = 0; j < cli.clients; ++j)
        while (in[j]->tryrec(pkt, cli.pktsize) > 0)
            ;
    free(pkt);
}


int main(int argc, char* argv[])
{
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    int clients = cli.clients;

    if (cli.is_server)
    {
        ArdComIn *in[clients];
        ArdComOut *out[clients];
        memset(in, 0, sizeof(void*)*clients);
        memset(out, 0, sizeof(void*)*clients);

        for (int i = 0; i < cli.clients; ++i)
        {
            in[i] = connect_network_in(argc, argv, cli.pktsize);
            out[i] = connect_network_out(argc, argv, cli.pktsize);
            if (!in[i]||!out[i])
                close_and_exit("server");
        }

        uint8_t *pkt = (uint8_t*)malloc(cli.pktsize);
        set_priority(cli.master_prio);

        fprintf(stderr, "init\n");
        init_connection(in, out);
        fprintf(stderr, "run\n");

        timeacc_t acc;
        timeacc_init(acc, cli.runs);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);
            pkt[0] = i;
            for (int j = 0; j < clients; ++j)
                if (out[j]->send(pkt, cli.pktsize) <= 0)
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
    }
    else
    {
        ArdComIn *in = connect_network_in(argc, argv, cli.pktsize);
        ArdComOut *out = connect_network_out(argc, argv, cli.pktsize);
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
    }

    close_and_exit();

    return 0;
}

