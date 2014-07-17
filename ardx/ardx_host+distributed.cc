
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
        fprintf(stderr, "exit msg (%d): %s  %s\n", (cli.is_server ? -1 : cli.clientnum), msg, extra ? extra : "");
    if (ardx_get_errno() != ARDERR_NO_ERROR) {
        ardx_perror("from ardx");
        fprintf(stderr, "ardx error %s\n", ardx_error_msg());
    }
    exit(0);
}


void sig_handler(int)
{
    close_and_exit();
}


void usage_exit()
{
    usage();
    exit(1);
}


int main(int argc, char* argv[])
{
    const char* info = "<port-prio> <put-rb-len> <put-heap-len> <get-rb-len> <get-heap-len> <put-channel-id> <get-channel-id>";
    set_extra_usage(info, info);

    if (!parse_cl(argc, argv, &cli, 7, 7))
        exit(1);

    int clients = cli.clients;
    int psize = cli.pktsize;

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    int first = first_user_argc();
    ardx_port_options_t popts = {atoi(argv[first]), 1};
    first++;

    ardx_channel_options_t put_copt = {atoi(argv[first+0]), atoi(argv[first+1])},
            get_copt = {atoi(argv[first+2]), atoi(argv[first+3])};

    first += 4;

    ardx_channel_port_t **get_port_ar = (ardx_channel_port_t**)malloc(sizeof(ardx_channel_port_t*)*clients);

    if (!cli.is_multi_server && argc != first+2)
        usage_exit();

    for (int i = first+1; i < argc; ++i)
    {
        ardx_channel_id_t get_cid;
        if (ardx_compat_parse_channel_id_string(argv[i], &get_cid) != 0)
            usage_exit();

        int idx = i - (first+1);
        get_port_ar[idx] = ardx_connect_channel(&get_cid, ARDX_CHANNEL_MODE_GET, psize, &get_copt, &popts);
        if (!get_port_ar[idx])
            close_and_exit("connect get");
    }

    if (!cli.is_multi_server)
    {
        for (int i = 1; i < clients; ++i)
            get_port_ar[i] = get_port_ar[0];
    }


    ardx_channel_id_t put_cid;
    if (ardx_compat_parse_channel_id_string(argv[first], &put_cid) != 0)
        usage_exit();

    ardx_channel_port_t *put_port = ardx_connect_channel(&put_cid, ARDX_CHANNEL_MODE_PUT, psize, &put_copt, &popts);
    if (!put_port)
        close_and_exit("connect put");

    // -----

    if (cli.is_server)
    {
        int ready_num = 0;
        ardx_packet_t *recv_pkt = NULL;
        int cnt = 0;

        do
        {
            fprintf(stderr, "ini send %d\n", cnt);
            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            *(int*)ardx_packet_data(send_pkt) = cnt;
            int res_send = ardx_channel_put(put_port, send_pkt);
            if (!send_pkt || res_send != 0)
                close_and_exit("error send");

            ready_num = 0;

            for (int i = 0; i < clients; ++i)
            {
                recv_pkt = ardx_channel_get(get_port_ar[i]);
                if (recv_pkt)
                {
                    if (*(int*)ardx_packet_data(recv_pkt) == cnt + 1)
                        ready_num++;
                    fprintf(stderr, "got repl %d %d (i:%d)\n", *(int*)ardx_packet_data(recv_pkt), cnt+1, i);
                    ardx_release_packet(get_port_ar[i], recv_pkt);
                }
                else
                    fprintf(stderr, "timeout i:%d cnd:%d\n", i, cnt);
                if (ardx_get_errno() != 0)
                    close_and_exit("error init");
            }
            fprintf(stderr, "%d/%d clients ready\n", ready_num, cli.clients);

            cnt++;
        } while (ready_num < cli.clients);


        msleep(3000);

        // clear
        for (int i = 0; i < clients; ++i)
            while ((recv_pkt = ardx_channel_tryget(get_port_ar[i], 0.)) != NULL)
            {
                fprintf(stderr, "drop %d\n", *(int*)ardx_packet_data(recv_pkt));
                ardx_release_packet(get_port_ar[i], recv_pkt);
            }

        set_priority(cli.master_prio);

        fprintf(stderr, "run\n");
        timeacc_t acc;
        timeacc_init(acc, cli.runs, 1);

        for (int i = 0; i < cli.runs; ++i)
        {
            timeacc_start_round(acc);

            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            if (!send_pkt)
                close_and_exit("server malloc");

            *(int*)ardx_packet_data(send_pkt) = i;

            int res_send = ardx_channel_put(put_port, send_pkt);
            if (res_send != 0)
                close_and_exit("server put");

            for (int j = 0; j < clients; ++j)
            {
                ardx_packet_t *recv_pkt = ardx_channel_get(get_port_ar[j]);
                if (!recv_pkt)
                    close_and_exit("server get");

                if (*(int*)ardx_packet_data(recv_pkt) != i + 1)
                    close_and_exit("data mismatch");

                int res = ardx_release_packet(get_port_ar[j], recv_pkt);
                if (res != 0)
                    close_and_exit("server release");

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
        set_priority(cli.client_prio);
        ardx_channel_port_t *get_port = get_port_ar[0];

        while (1)
        {
            ardx_packet_t *rec_pkt = ardx_channel_get(get_port);
            if (!rec_pkt)
                close_and_exit("client get");

            ardx_packet_t *send_pkt = ardx_malloc_packet(put_port);
            if (!send_pkt)
                close_and_exit("client malloc");

            *(int*)ardx_packet_data(send_pkt) = *(int*)ardx_packet_data(rec_pkt) + 1;

            int res_rel = ardx_release_packet(get_port, rec_pkt);
            if (res_rel != 0)
                close_and_exit("client release");

            int res_put = ardx_channel_put(put_port, send_pkt);
            if (res_put != 0)
                close_and_exit("client put");
        }
    }

    close_and_exit();

    return 0;
}



