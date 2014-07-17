
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


int main(int argc, char* argv[])
{
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    int clients = cli.clients;

    if (cli.is_server)
    {
        ArdComIn *in[clients];
        ArdComOut *out[clients];

        for (int i = 0; i < cli.clients; ++i)
        {
            in[i] = connect_shm_in("hostlocal_client2server", i, 0, cli.pktsize);
            out[i] = connect_shm_out("hostlocal_server2client", i, 0, cli.pktsize);
            if (!in[i]||!out[i])
                close_and_exit("server");
        }

        uint8_t *pkt = (uint8_t*)malloc(cli.pktsize);

        for (int j = 0; j < clients; ++j)
            while (in[j]->tryrec(pkt, cli.pktsize, NULL) > 0)
                ;

        set_priority(cli.master_prio);
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
    else
    {
        int num = cli.clientnum;
        ArdComIn *in = connect_shm_in("hostlocal_server2client", num, 1, cli.pktsize);
        ArdComOut *out = connect_shm_out("hostlocal_client2server", num, 1, cli.pktsize);
        if (!in||!out)
            close_and_exit("client");

        set_priority(cli.client_prio);
        uint8_t *pkt = (uint8_t*)malloc(cli.pktsize);

        for (int j = 0; j < clients; ++j)
            while (in->tryrec(pkt, cli.pktsize, NULL) > 0)
                ;

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

