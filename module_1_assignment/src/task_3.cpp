#include "lib_task3.h"


int main()
{   
    // sensor with integer data
    Sensors sensor1("Temperature", 2);
    sensor1.getSensorData();

    // sensor with float data
    Sensors sensor2("Distance", 10.0);
    sensor2.getSensorData();

    // sensor with string data ( Null = No Data - just for demonstration)
    Sensors sensor3("Temperature", "Null");
    sensor3.getSensorData();

    // sensor with char data (x = No Data - just for demonstration)
    Sensors sensor4("Distance", 'x');
    sensor4.getSensorData();

    return 0;
}