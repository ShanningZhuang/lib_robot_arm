#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

class Motor
{
public:
    int a;
    Motor(int a);
    void test();
};
Motor::Motor(int a)
{
    this->a = a;
}
void Motor::test()
{
    std::cout<<a<<std::endl;
}

class MotorA:public Motor
{
public:
    MotorA(int a):Motor(a){}
};
int main()
{
    Motor* m1 = new Motor(1);
    m1->test();
    MotorA* m2 = new MotorA(2);
    m2->test();
    std::vector<Motor *> motor_list;
    motor_list.push_back(m1);
    motor_list.push_back(m2);
    motor_list[0]->test();
    motor_list[1]->test();
    return 0;
}