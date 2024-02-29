How to use : \

class TASK_NAME : public TimingTask
{
    public:
        TASK_NAME(uint32_t _rate):rate(_rate){updateTime(millis());}

        virtual void run(uint32_t now){
            function();
            .....
            tick(rate);
        } 
    private:
        uint32_t rate;
};


main : 

TASK_NAME task1 (1000); // every 1 second
Task *tasks[] = {&task1};
Scheduler scheduler(tasks, NUM_TASKS(tasks));
while(1){scheduler.runTasks();