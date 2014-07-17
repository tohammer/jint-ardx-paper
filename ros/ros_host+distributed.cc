
#include "helper.h"
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

#include "normal/packet.h"


timeacc_t acc;
ClInfo_t cli;

ros::Publisher pub, pub_init;
normal::packet msg;

// ---- client
void client_cb(const normal::packet::ConstPtr& in)
{
    if (msg.data.size() != (size_t)cli.pktsize)
    {
        fprintf(stderr, "ERROR\n");
        exit(1);
    }
    msg.data[0] = in->data[0]+1;
    pub.publish(msg);
}


ros::Subscriber sub_init;
int had_init = 0;
void init_request_cb(const normal::packet::ConstPtr& in)
{
    if (!had_init)
    {
        had_init = 1;
        fprintf(stderr, "init req %d\n", cli.clientnum);
    }
    msg.data[0] = cli.clientnum;
    pub_init.publish(msg);

}


// ---- server
uint8_t rec_count = 0;
int cntr = 0;
uint64_t start;
bool *ready_list = NULL;
int ready_num = 0;

void server_cb(const normal::packet::ConstPtr& in)
{
  if (in->data[0] != (uint8_t)(cntr+1))
  {
    fprintf(stderr, "mismatch\n");
    exit(1);
  }
  
    rec_count++;
    if (rec_count == cli.clients)
    {
        timeacc_end_round(acc);

        cntr++;
        if (cntr == cli.runs)
        {
            timeacc_print(acc);
            ros::shutdown();
            return;
        }

        if (msg.data.size() != (size_t)cli.pktsize)
        {
            fprintf(stderr, "ERROR\n");
            exit(1);
        }
#if 0
	    msleep(50);
#endif
	
        rec_count = 0;
        timeacc_start_round(acc);
        msg.data[0] = cntr;
        pub.publish(msg);
    }
}


int last_ready_num = 0;
void client_ready_cb(const normal::packet::ConstPtr& in)
{
    ready_num = 0;
    ready_list[ in->data[0] ] = true;
    for (int i = 0; i < cli.clients; ++i)
        if (ready_list[i])
            ready_num++;

    if (ready_num > last_ready_num)
    {
        fprintf(stderr, "%d/%d clients ready\n", ready_num, cli.clients);
        last_ready_num = ready_num;
    }
}



// ---- main
int main(int argc, char* argv[])
{
    if (!parse_cl(argc, argv, &cli))
        exit(0);

    msg.data.resize(cli.pktsize);

    if (cli.is_server)
    {
        set_priority(cli.master_prio);

        ros::init(argc, argv, "server");
        ros::TransportHints().tcpNoDelay();

        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("client2server", cli.clients+1, server_cb);
        ros::Subscriber sub_init = nh.subscribe("client_ready", 2*cli.clients, client_ready_cb);
        pub = nh.advertise<normal::packet>("server2client", 1);
        pub_init = nh.advertise<normal::packet>("init_request", 1);

        ready_list = new bool[cli.clients];
        for (int i = 0; i < cli.clients; ++i)
            ready_list[i] = false;

        msleep(200);

        // init
        fprintf(stderr, "init  c:%d  pkt:%d\n", cli.clients, cli.pktsize);
        while (ready_num < cli.clients)
        {
            pub_init.publish(msg);
            ros::spinOnce();
            msleep(500);
        }

        fprintf(stderr, "all ready, run\n");
        msleep(200);

        // run
        timeacc_init(acc, cli.runs);
        timeacc_start_round(acc);

        msg.data[0] = 0;
        pub.publish(msg);
        ros::spin();
    }
    else
    {
        set_priority(cli.client_prio);
        msg.data[0] = cli.clientnum;

        char name[100];
        sprintf(name, "client_%d", cli.clientnum);
        ros::init(argc, argv, name);
        ros::TransportHints().tcpNoDelay();

        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe<normal::packet>("server2client", 2, client_cb);
        sub_init = nh.subscribe<normal::packet>("init_request", 2, init_request_cb);
        pub = nh.advertise<normal::packet>("client2server", cli.clients);
        pub_init = nh.advertise<normal::packet>("client_ready", cli.clients);

        fprintf(stderr, "client run %d\n", cli.clientnum);
        ros::spin();
    }

    return 0;
}
