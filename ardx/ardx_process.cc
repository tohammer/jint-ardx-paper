
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>

#include <ardx/ardx.h>
#include "helper.h"

#define BOUND

int clients;
ClInfo_t cli;
ardx_channel_id_t cid_s2c;
ardx_channel_id_t *cid_c2s;
ardx_port_options_t popts = {0, 1};


void close_and_exit(const char* msg = NULL, const char* extra = NULL)
{
    if (msg)
        fprintf(stderr, "exit msg (%d): %s  %s\n", (cli.is_server ? -1 : cli.clientnum), msg, extra ? extra : "");
    if (ardx_get_errno() != ARDERR_NO_ERROR)
        fprintf(stderr, "ardx error %s\n", ardx_error_msg());
    exit(0);
}


void sig_handler(int)
{
    close_and_exit("sig");
}


void* client(void* n)
{
    size_t num = (size_t)n;
#ifdef BOUND
    set_cpu_affinity((num+1)%4);
#endif

    ardx_channel_port_t *put_port, *get_port;    
    put_port = ardx_connect_channel(&cid_c2s[num], ARDX_CHANNEL_MODE_PUT, cli.pktsize, NULL, &popts);
    get_port = ardx_connect_channel(&cid_s2c, ARDX_CHANNEL_MODE_GET, cli.pktsize, NULL, &popts);

    if (!put_port || !get_port)
        close_and_exit("client");

    set_priority(cli.client_prio);
    while (1)
    {
        ardx_packet_t *rec_pkt = ardx_channel_get(get_port);
        if (!rec_pkt)
            close_and_exit("client get");

        ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
        if (!send_pkt)
            close_and_exit("client malloc");

        *((int*)ardx_packet_data(send_pkt)) = *((int*)ardx_packet_data(rec_pkt)) + 100;

        int res_rel = ardx_release_packet(get_port, rec_pkt);
        if (res_rel != 0)
            close_and_exit("client release");

        int res_put = ardx_channel_put(put_port, send_pkt);
        if (res_put != 0)
            close_and_exit("client put");
    }

    return NULL;
}


int main(int argc, char* argv[])
{
    if (!parse_cl(argc, argv, &cli))
        exit(1);

    signal(SIGINT, sig_handler);

    clients = cli.clients;

    if (cli.is_server)
    {
        ardx_acquire_free_channel_id(ARDX_CHANNEL_DOMAIN_LOCAL,&cid_s2c);
        cid_c2s = new ardx_channel_id_t[clients];

        pthread_t th[clients];

        ardx_channel_port_t *put_port, **get_port_ar;
        get_port_ar = (ardx_channel_port_t**)malloc(sizeof(ardx_channel_port_t*)*clients);

        if (cli.is_multi_server)
        {
            ardx_channel_options_t copt_c2s;
            copt_c2s.ring_buffer_length = 1;
            copt_c2s.packet_heap_length = 3;

            for (int i = 0; i < clients; ++i)
            {
                ardx_acquire_free_channel_id(ARDX_CHANNEL_DOMAIN_LOCAL, &cid_c2s[i]);
                get_port_ar[i] = ardx_connect_channel(&cid_c2s[i], ARDX_CHANNEL_MODE_GET, cli.pktsize, &copt_c2s, &popts);
            }
        }
        else
        {
            ardx_channel_options_t copt_c2s;
            copt_c2s.ring_buffer_length = clients;
            copt_c2s.packet_heap_length = clients + clients + 1;

            ardx_channel_id_t tmp;
            ardx_acquire_free_channel_id(ARDX_CHANNEL_DOMAIN_LOCAL, &tmp);
            ardx_channel_port_t *tmp_port = ardx_connect_channel(&tmp, ARDX_CHANNEL_MODE_GET, cli.pktsize, &copt_c2s, &popts);

            for (int i = 0; i < clients; ++i)
            {
                cid_c2s[i] = tmp;
                get_port_ar[i] = tmp_port;
            }
        }
        ardx_channel_options_t copt_s2c;
        copt_s2c.ring_buffer_length = 1;
        copt_s2c.packet_heap_length = 1 + clients + 1;

        put_port = ardx_connect_channel(&cid_s2c, ARDX_CHANNEL_MODE_PUT, cli.pktsize, &copt_s2c, &popts);

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 1024*1024);

        for (int i = 0; i < cli.clients; ++i)
            if (pthread_create( &th[i], &attr, client, (void*)i ) != 0)
                close_and_exit("thread");

#ifdef BOUND
        set_cpu_affinity(0);
#endif
        set_priority(cli.master_prio);
        msleep(500);

        timeacc_t acc;
        timeacc_init(acc, cli.runs);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);

            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            *((int*)ardx_packet_data(send_pkt)) = i+1;
            int res_send = ardx_channel_put(put_port, send_pkt);
            if (!send_pkt || res_send != 0)
            {
                printf("%p %d\n", send_pkt, res_send);
                close_and_exit("error send");
            }

            for (int j = 0; j < clients; ++j)
            {
                ardx_packet_t *recv_pkt = ardx_channel_get(get_port_ar[j]);
                int res = ardx_release_packet(get_port_ar[j], recv_pkt);
                if (!recv_pkt || res != 0)
                    close_and_exit("error rec");

                if ( *((int*)ardx_packet_data(recv_pkt)) != i+101 )
                    close_and_exit("recv mismatch");
            }
            timeacc_end_round(acc);
        }

        timeacc_print(acc);

        msleep(100);
    }

    close_and_exit();

    return 0;
}




