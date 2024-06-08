#include <unistd.h>
#include <iostream>
#include <vector>
#include <time.h>

#include "lkmotor_controller.hpp"
#include "utils.hpp"
#include <atomic>
#include <signal.h>

std::atomic<bool> quit(false);

void sig_handler(int signum)
{
  if (signum == SIGINT)
    quit.store(true);
}

int main()
{
    signal(SIGINT, &sig_handler);
    while(!quit)
    {
        std::cout<<"loop"<<std::endl;
    }
    std::cout<<"quit"<<std::endl;
    while(1)
    {
        std::cout<<"loop1"<<std::endl;
    }
}