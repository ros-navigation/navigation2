#include <stdio.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <zmq.hpp>
#include <fstream>
#include "behavior_tree_logger/BT_logger_generated.h"

// http://zguide.zeromq.org/cpp:interrupt
static bool s_interrupted = false;

static void s_signal_handler(int)
{
    s_interrupted = true;
}

static void CatchSignals(void)
{
    struct sigaction action;
    action.sa_handler = s_signal_handler;
    action.sa_flags = 0;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        printf("Wrong number of arguments\nUsage: %s [filename]\n", argv[0]);
        return 1;
    }

    // register CTRL+C signal handler
    CatchSignals();

    zmq::context_t context(1);

    //  Socket to talk to server
    std::cout << "Trying to connect to [tcp://localhost:1666]\n" << std::endl;

    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:1666");

    //  Subscribe to everything
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    printf("----------- Started -----------------\n");

    bool first_message = true;
    std::ofstream file_os;

    while (!s_interrupted)
    {
        zmq::message_t update;
        zmq::message_t msg;
        try
        {
            subscriber.recv(&update);
        }
        catch (zmq::error_t& e)
        {
            if (!s_interrupted)
            {
                std::cout << "subscriber.recv() failed with exception: " << e.what() << std::endl;
                return -1;
            }
        }

        if (!s_interrupted)
        {
            char* data_ptr = static_cast<char*>(update.data());
            const uint32_t header_size = flatbuffers::ReadScalar<uint32_t>(data_ptr);

            if (first_message)
            {
                printf("First message received\n");
                first_message = false;

                file_os.open(argv[1], std::ofstream::binary | std::ofstream::out);
                file_os.write(data_ptr, 4 + header_size);
            }
            data_ptr += 4 + header_size;

            const uint32_t transition_count = flatbuffers::ReadScalar<uint32_t>(data_ptr);
            data_ptr += sizeof(uint32_t);

            file_os.write(data_ptr, 12 * transition_count);
        }
    }

    subscriber.close();

    printf("Results saved to file\n");
    file_os.close();

    return 0;
}
