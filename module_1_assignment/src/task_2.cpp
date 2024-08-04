#include <iostream>
#include <vector>

using namespace std;

class Robot
{
    public:
        Robot(const string name,const float speed,const vector<int> &physical);

        void getRobotData();
        void moveForward(int distance);
        void moveBackward(int distance);
        void stop();

    private:
        string RobotName;
        float RobotSpeed;
        vector<int> Robotphysical;
};

// Constructor with all the values set to const such that they cannot be changed
Robot::Robot(const string name,const float speed,const vector<int> &physical) 
        : RobotName(name), RobotSpeed(speed), Robotphysical(physical)
    {
        cout << RobotName << " created !!!" << endl;
    }

// Get the robot data
void Robot::getRobotData()
{
    cout << "Robot Name: " << RobotName << endl;
    cout << "Robot Speed: " << RobotSpeed << endl;
    cout << "Robot Weight: " << Robotphysical[0] << endl;
    cout << "Robot Size: " << Robotphysical[1] << endl;
    cout << "Number of Sensors: " << Robotphysical[2] << endl;
    cout << endl;
}

// Move the robot forward
void Robot::moveForward(int distance)
{
    cout << RobotName << ": moving forward ..." << distance << "m" << endl;
}

// Move the robot backward
void Robot::moveBackward(int distance)
{
    cout << RobotName << ": moving backward ..."<< distance << "m" << endl;
}

// Stop the robot
void Robot::stop()
{
    cout << RobotName << ": stopped ..." << endl;
    cout << endl;
}


int main()
{
    string name = "Robot1";
    float speed = 10.0;
    int weight = 20;
    int size = 30;
    int num_sensors = 5;
    vector<int> physical = {weight, size, num_sensors}; 
    Robot robot1(name, speed, physical);
    robot1.getRobotData();
    robot1.moveForward(5);
    robot1.moveBackward(2);
    robot1.stop();


    string name_2 = "Robot2";
    float speed_2 = 20.0;
    int weight_2 = 50;
    int size_2 = 40;
    int num_sensors_2 = 2;
    vector<int> physical_2 = {weight_2, size_2, num_sensors_2}; 
    Robot robot2(name_2, speed_2, physical_2);
    robot2.getRobotData();
    robot2.moveForward(10);
    robot2.moveBackward(20);
    robot2.stop();

    return 0;
}