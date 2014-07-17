#ifndef ROSCOMMON_H
#define ROSCOMMON_H


extern ClInfo_t cli;
extern timeacc_t acc;


void close_and_exit(const char* msg = NULL)
{
    if (msg)
        printf("exit msg: %s\n", msg);
    else
        printf("exit\n");
    exit(0);
}



// ---- client
template <typename P>
class TheClient
{
    int num;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::CallbackQueue cbq;
    ros::AsyncSpinner aspin;

public:
    TheClient( int num_ )
        : num(num_),
          aspin(1, &cbq)
    {
        nh.setCallbackQueue(&cbq);
        sub = nh.subscribe<P>("server2client", 1, &TheClient::cb, this);
        pub = nh.advertise<P>("client2server", 1);

        aspin.start();
    }

    void cb(const typename P::ConstPtr &in)
    {
        typename P::Ptr msg(new P);
        msg->data[0] = in->data[0]+1;
        msg->data[1] = num;
        pub.publish(msg);
    }

    void stop()
    {
        aspin.stop();
    }
};



// ---- server
ros::Publisher pub;

uint8_t rec_count = 0;
int cntr = 0;
uint64_t start;

template <typename P>
void server_cb(const typename P::ConstPtr& in)
{
    rec_count++;
    
    if (in->data[0] != (uint8_t)(cntr+1)) {
        printf("cl:%d  %d != %d\n", in->data[1], in->data[0], (uint8_t)(cntr+1));
        close_and_exit("mismatch");
    }

    if (rec_count == cli.clients)
    {
        timeacc_end_round(acc);
        cntr++;
        if (cntr == cli.runs)
        {
            timeacc_print(acc);
            ros::shutdown();
        }

        rec_count = 0;

        timeacc_start_round(acc);
        typename P::Ptr msg(new P);
        msg->data[0] = cntr;
        pub.publish(msg);
    }
}


// ---- main
template <typename P>
void run_template()
{
    ros::NodeHandle nh;
    ros::CallbackQueue my_callback_queue;
    nh.setCallbackQueue(&my_callback_queue);

    // setup clients
    set_priority(cli.client_prio);
    TheClient<P> *cllist[cli.clients];
    for (int i = 0; i < cli.clients; ++i)
        cllist[i] = new TheClient<P>(i);

    set_priority(cli.master_prio);

    // setup server
    ros::Subscriber sub;

    sub = nh.subscribe("client2server", cli.clients, server_cb<P>);
    pub = nh.advertise<P>("server2client", 1);

    msleep(2000);
    fprintf(stderr, "run\n");

    ros::SingleThreadedSpinner s;

    // start it
    timeacc_init(acc, cli.runs);
    timeacc_start_round(acc);

    typename P::Ptr msg(new P);
    msg->data[0] = cntr;
    pub.publish(msg);

    s.spin(&my_callback_queue);

    for (int i = 0; i < cli.clients; ++i)
        cllist[i]->stop();
}




#endif // ROSCOMMON_H
