
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>

#include <ardx/ardx.h>
#include "helper.h"


//#define BOUND


int clients;
ClInfo_t cli;
ardx_port_options_t popts = {0, 1};
int sg_type = 0;
int high_prio = -1, low_prio = -1;

ardx_channel_id_t cid_s2c;
ardx_channel_id_t *cid_c2s;

#define SINGLE_SG 1
#define HIGH_PRIO 2
#define MULTI_SG  3


void close_and_exit(const char* msg = NULL, const char* extra = NULL)
{
    if (msg)
        fprintf(stderr, "exit msg (%d): %s  %s\n", (cli.is_server ? -1 : cli.clientnum), msg, extra ? extra : "");
    if (ardx_get_errno() != ARDERR_NO_ERROR)
        ardx_perror("ardx error");
    exit(0);
}


void sig_handler(int)
{
    close_and_exit("sig");
}


void ardx_error_exit(const char* msg)
{
    ardx_perror(msg);
    exit(1);
}


void* client(void* n_void)
{
    size_t num = (size_t)n_void;
    ardx_channel_port_t *put_port, *get_port;
    int cid_c2s_num = (cli.is_multi_server ? num : 0);
    put_port = ardx_connect_channel(&cid_c2s[cid_c2s_num], ARDX_CHANNEL_MODE_PUT, cli.pktsize, NULL, &popts);
    get_port = ardx_connect_channel(&cid_s2c, ARDX_CHANNEL_MODE_GET, cli.pktsize, NULL, &popts);
    if (ardx_get_errno() != 0)
        close_and_exit("client");

#ifdef BOUND
    set_cpu_affinity((num+1)%4);
#endif

    ardx_sync_group_t *sg = NULL;
    if (sg_type == HIGH_PRIO || sg_type == MULTI_SG)
    {
        sg = ardx_create_sync_group(&get_port, 1);
        if (!sg)
            ardx_error_exit("create sg cl");
    }

    if (num == 0 || sg_type != HIGH_PRIO)
    {
        if (sg_type == HIGH_PRIO)
            set_priority(high_prio);
        else
            set_priority(low_prio);

        while (1)
        {
            if (ardx_get_errno()!=0)
                ardx_error_exit("client: unknown");

            ardx_channel_port_t *local_get_port = NULL;
            if (sg)
                local_get_port = ardx_sync(sg);
            else
                local_get_port = get_port;
            ardx_packet_t *rec_pkt = ardx_channel_get(local_get_port);
            if (!rec_pkt)
                ardx_error_exit("client error: get");

            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            if (!send_pkt)
                ardx_error_exit("client error: malloc");

            *((int*)ardx_packet_data(send_pkt)) = *((int*)ardx_packet_data(rec_pkt)) + 100;

            int res_rel = ardx_release_packet(local_get_port, rec_pkt);
            if (res_rel != 0)
                ardx_error_exit("client error: release");

            int res_put = ardx_channel_put(put_port, send_pkt);
            if (res_put != 0)
                ardx_error_exit("client error: put");
        }
    }
    else // num != 0 && sg_type == 2
    {
        set_priority(low_prio);

        while (1)
        {
            ardx_channel_port_t *p = ardx_sync(sg);
            ardx_packet_t *rec_pkt = ardx_channel_get(p);
            int res_rel = ardx_release_packet(p, rec_pkt);
            if (!rec_pkt || res_rel != 0)
                close_and_exit("client error");
        }
    }

    return NULL;
}


int main(int argc, char* argv[])
{
    set_extra_usage("{-single-sg | -high-prio | -multi-sg} <master-prio> <high-prio> <low-prio>", NULL);
    if (!parse_cl(argc, argv, &cli, 4))
        exit(1);

    signal(SIGINT, sig_handler);

    int first = first_user_argc();
    int master_prio = -1;
    master_prio = atoi(argv[first+1]);
    high_prio = atoi(argv[first+2]);
    low_prio = atoi(argv[first+3]);

    if (ARGEQ(first+0, "-single-sg"))
        sg_type = SINGLE_SG;
    else if (ARGEQ(first+0, "-high-prio") && argc >= 9)
        sg_type = HIGH_PRIO;
    else if (ARGEQ(first+0, "-multi-sg"))
        sg_type = MULTI_SG;
    else
    {
        usage();
        exit(1);
    }
    clients = cli.clients;

#ifdef BOUND
    fprintf(stderr, "BOUND!\n");
#endif

    if (cli.is_server)
    {
        ardx_acquire_free_channel_id(ARDX_CHANNEL_DOMAIN_LOCAL, &cid_s2c);
        cid_c2s = (ardx_channel_id_t*)malloc(sizeof(ardx_channel_id_t)*clients);
        for (int i=0; i < clients; ++i)
            ardx_acquire_free_channel_id(ARDX_CHANNEL_DOMAIN_LOCAL, &cid_c2s[i]);

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
                get_port_ar[i] = ardx_connect_channel(&cid_c2s[i], ARDX_CHANNEL_MODE_GET, cli.pktsize, &copt_c2s, &popts);
                if (!get_port_ar[i])
                    close_and_exit("connect multi");
            }
        }
        else
        {
            ardx_channel_options_t copt_c2s;
            copt_c2s.ring_buffer_length = clients;
            copt_c2s.packet_heap_length = clients + clients + 1;

            ardx_channel_port_t *tmp_port = ardx_connect_channel(&cid_c2s[0], ARDX_CHANNEL_MODE_GET, cli.pktsize, &copt_c2s, &popts);

            if (!tmp_port)
                close_and_exit("connect");

            for (int i = 0; i < clients; ++i)
                get_port_ar[i] = tmp_port;
        }

        ardx_channel_options_t copt_s2c;
        copt_s2c.ring_buffer_length = 1;
        copt_s2c.packet_heap_length = 1 + clients + 1;

        put_port = ardx_connect_channel(&cid_s2c, ARDX_CHANNEL_MODE_PUT, cli.pktsize, &copt_s2c, &popts);

        ardx_sync_group_t *sg = NULL;
        if (sg_type == SINGLE_SG)
        {
            if (cli.is_multi_server)
                sg = ardx_create_sync_group(get_port_ar, clients);
            else
                sg = ardx_create_sync_group(get_port_ar, 1);
            if (!sg || ardx_get_errno() != 0)
                ardx_error_exit("create sg");
        }

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 1024*1024);

        for (int i = 0; i < cli.clients; ++i)
            if (pthread_create( &th[i], &attr, client, (void*)i ) != 0)
                close_and_exit("thread");

        msleep(500);
#ifdef BOUND
        set_cpu_affinity(0);
#endif
        set_priority(master_prio);

        int sending_cl_num = sg_type == HIGH_PRIO ? 1 :clients;

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

            for (int j = 0; j < sending_cl_num; ++j)
            {
                ardx_channel_port_t *local_get_port = NULL;
                if (sg)
                    local_get_port = ardx_sync(sg);
                else
                    local_get_port = get_port_ar[j];
                if (!local_get_port)
                    ardx_error_exit("local port");

                ardx_packet_t *recv_pkt = ardx_channel_get(local_get_port);
                if (!recv_pkt)
                    ardx_error_exit("error rec");

                if ( *((int*)ardx_packet_data(recv_pkt)) != i+101 )
                    close_and_exit("recv mismatch");

                int res = ardx_release_packet(local_get_port, recv_pkt);
                if (res != 0)
                    ardx_error_exit("error release");
            }
            timeacc_end_round(acc);

        }

        timeacc_print(acc);


        if (sg && ardx_delete_sync_group(sg) != 0)
            ardx_error_exit("del sg");
    }

    close_and_exit();

    return 0;
}




