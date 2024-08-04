#include<iostream>
#include<vector>

using namespace std;

void readRobotData(const vector<int> &temperatures,const vector<int> &distanceToObjects)
{
    size_t dataSize = temperatures.size();

    // Loop through the data and print it
    for (size_t i = 0; i < dataSize; i++)
    {
        cout << "Robot Temperature: " << temperatures[i] << "C Distance: " << distanceToObjects[i] << "cm"<< endl;
    }
}


int main()
{
    // Create two vectors to store the robot data
    vector<int> temperatures = {20, 30, 40, 50, 60, 70, 80, 90, 100};
    vector<int> distanceToObjects = {10, 20, 30, 40, 50, 60, 70, 80, 90};

    // Call the function to read the robot data
    readRobotData(temperatures, distanceToObjects);

    return 0;
}