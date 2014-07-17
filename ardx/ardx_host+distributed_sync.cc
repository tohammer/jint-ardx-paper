#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>

#include <ardx/ardx.h>
#include <ardx/ardx-compat.h>
#include "helper.h"


ClInfo_t cli;

void close_and_exit(const char* msg = NULL, const char* extra = NULL)
{
    if (msg)
        fprintf(stderr, "exit msg hl (%d): %s  %s\n", (cli.is_server ? -1 : cli.clientnum), msg, extra ? extra : "");
    if (ardx_get_errno() != ARDERR_NO_ERROR)
        ardx_perror("ardx error");
    exit(0);
}


void sig_handler(int)
{
    close_and_exit();
}


void usage_exit(const char* msg = NULL)
{
    if (msg)
        printf("invalid at %s\n", msg);
    usage();
    exit(1);
}

enum {
    S_SG, S_NONE, S_HIGH,
    C_DUMMY, C_SG, C_NONE
};


int main(int argc, char* argv[])
{
    const char* s_info = "[-sg | -none | -high] <port-prio> <put-rb-len> <put-heap-len> <get-rb-len> <get-heap-len> <put-channel-id> <get-channel-id> ...";
    const char* c_info = "[-dummy | -sg | -none] <port-prio> <put-rb-len> <put-heap-len> <get-rb-len> <get-heap-len> <put-channel-id> <get-channel-id>";
    set_extra_usage(s_info, c_info);

    if (!parse_cl(argc, argv, &cli, 8, 8))
        exit(1);

    int clients = cli.clients;
    int psize = cli.pktsize;

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    int first = first_user_argc();

    int type = 0;
    if (cli.is_server)
    {
        if (ARGEQ(first, "-sg")) type = S_SG;
        else if (ARGEQ(first, "-none")) type = S_NONE;
        else if (ARGEQ(first, "-high")) type = S_HIGH;
        else usage_exit("type");
    }
    else
    {
        if (ARGEQ(first, "-dummy")) type = C_DUMMY;
        else if (ARGEQ(first, "-sg")) type = C_SG;
        else if (ARGEQ(first, "-none")) type = C_NONE;
        else usage_exit("type");
    }

    ardx_port_options_t popts = {atoi(argv[first+1]), 1};

    first += 2;

    ardx_channel_options_t put_copt = {atoi(argv[first+0]), atoi(argv[first+1])},
            get_copt = {atoi(argv[first+2]), atoi(argv[first+3])};

    first += 4;

    ardx_channel_port_t **get_port_ar = (ardx_channel_port_t**)malloc(sizeof(ardx_channel_port_t*)*clients);

    if (!cli.is_multi_server && argc != first+2)
        usage_exit("arg count");

    int get_num = 0;
    for (int i = first+1; i < argc; ++i)
    {
        ardx_channel_id_t get_cid;
        if (ardx_compat_parse_channel_id_string(argv[i], &get_cid) != 0)
            usage_exit("get cid");

        get_port_ar[get_num] = ardx_connect_channel(&get_cid, ARDX_CHANNEL_MODE_GET, psize, &get_copt, &popts);
        if (!get_port_ar[get_num])
            close_and_exit("connect get");

        get_num++;
    }

    if (!cli.is_multi_server)
    {
        for (int i = 1; i < clients; ++i)
            get_port_ar[i] = get_port_ar[0];
    }

    ardx_channel_id_t put_cid;
    if (ardx_compat_parse_channel_id_string(argv[first], &put_cid) != 0)
        usage_exit("put cid");

    ardx_channel_port_t *put_port = ardx_connect_channel(&put_cid, ARDX_CHANNEL_MODE_PUT, psize, &put_copt, &popts);
    if (!put_port)
        close_and_exit("connect put");

    // create sg
    ardx_sync_group_t *sg = NULL;
    if (type == S_SG || type == C_SG || type == C_DUMMY)
    {
        sg = ardx_create_sync_group(get_port_ar, get_num);
        if (!sg)
            close_and_exit("create sync");
        fprintf(stderr, "%d: create sync %d\n", cli.is_server ? -1 : cli.clientnum, get_num);
    }


    // run
    if (cli.is_server)
    {
        int cnt = 1;
        int ready_num = 0;
        int sending_clients = (type == S_HIGH ? 1 : cli.clients);

        do
        {
            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            *(int*)ardx_packet_data(send_pkt) = cnt;
            int res_send = ardx_channel_put(put_port, send_pkt);
            if (!send_pkt || res_send != 0)
                close_and_exit("error send");

            ready_num = 0;
            ardx_packet_t *recv_pkt = NULL;

            for (int i = 0; i < sending_clients; ++i)
            {
                recv_pkt = ardx_channel_get(get_port_ar[i]);
                if (recv_pkt)
                {
                    if (*(int*)ardx_packet_data(recv_pkt) == cnt + 1)
                        ready_num++;
                    ardx_release_packet(get_port_ar[i], recv_pkt);
                }
                if (ardx_get_errno() != 0)
                    close_and_exit("error init");
            }
            fprintf(stderr, "%d/%d (%d) clients ready\n", ready_num, sending_clients, cli.clients);

            cnt++;
        } while (ready_num < sending_clients);



        msleep(500);

        // clear
        ardx_packet_t *recv_pkt = NULL;
        for (int i = 0; i < clients; ++i)
            while ((recv_pkt = ardx_channel_tryget(get_port_ar[i], 0.)) != NULL)
            {
                fprintf(stderr, "drop %d\n", *(int*)ardx_packet_data(recv_pkt));
                ardx_release_packet(get_port_ar[i], recv_pkt);
            }


        set_priority(cli.master_prio);
        timeacc_t acc;
        timeacc_init(acc, cli.runs);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);

            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            *(int*)ardx_packet_data(send_pkt) = i;
            int res_send = ardx_channel_put(put_port, send_pkt);
            if (!send_pkt || res_send != 0)
                close_and_exit("error send");

            for (int j = 0; j < sending_clients; ++j)
            {
                ardx_channel_port_t* local_get_port;
                if (sg)
                    local_get_port = ardx_sync(sg);
                else
                    local_get_port = get_port_ar[j];

                ardx_packet_t *recv_pkt = ardx_channel_get(local_get_port);
                if (*(int*)ardx_packet_data(recv_pkt) != i + 1)
                {
                    fprintf(stderr, "%p: %d != %d\n", recv_pkt, *(int*)ardx_packet_data(recv_pkt), i + 1);
                    close_and_exit("data mismatch");
                }

                int res = ardx_release_packet(local_get_port, recv_pkt);
                if (!recv_pkt || res != 0)
                    close_and_exit("error rec");
            }
            timeacc_end_round(acc);
        }        

        timeacc_print(acc);
        ardx_disconnect_channel(put_port);
        for (int i = 0; i < (cli.is_multi_server ? clients : 1); ++i)
            ardx_disconnect_channel(get_port_ar[i]);

    }
    else
    {
        ardx_channel_port_t *get_port = get_port_ar[0];
        set_priority(cli.client_prio);

        if (type != C_DUMMY)
        {
            while (1)
            {
                ardx_channel_port_t* local_get_port;
                if (sg)
                    local_get_port = ardx_sync(sg);
                else
                    local_get_port = get_port;
                ardx_packet_t *rec_pkt = ardx_channel_get(local_get_port);

                ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);

                *(int*)ardx_packet_data(send_pkt) = *(int*)ardx_packet_data(rec_pkt) + 1;

                int res_rel = ardx_release_packet(local_get_port, rec_pkt);

                int res_put = ardx_channel_put(put_port, send_pkt);

                if (!rec_pkt || res_rel != 0 || !send_pkt || res_put != 0)
                    close_and_exit("client error");
            }
        }
        else
        {
            while (1)
            {
                ardx_channel_port_t *p = ardx_sync(sg);
                ardx_packet_t *rec_pkt = ardx_channel_get(p);
                int res_rel = ardx_release_packet(p, rec_pkt);
                if (!rec_pkt || res_rel != 0)
                    close_and_exit("client error");
            }
        }
    }

    close_and_exit();

    return 0;
}



