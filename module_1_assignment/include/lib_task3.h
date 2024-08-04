#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>

using namespace std;

template <typename T, typename D>
class Sensors
{
    public:
        Sensors(T type, D data) : SensorType(type), SensorData(data)
        {
            cout << SensorType <<": Sensor created !!!" << endl;
        }
        void getSensorData();
    private:
        T SensorType;
        D SensorData;
};

template <typename T, typename D>
void Sensors<T, D>::getSensorData()
{
    cout << "Processing Sensor Data: " << SensorData << endl;
}

#endif // SENSOR_H