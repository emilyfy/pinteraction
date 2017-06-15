#ifndef Driver_h
#define Driver_h

//all the declarations
class Driver //declare class
{ //declare all variables and functions to be used in .cpp
    private: //to be used only in cpp
        int _direction;
		    int _stepDelay;
		    int _numberOfSteps;
    protected: //constant
        int _dirPin;
        int _stepPin;
    public: //to be called by .ino file
        Driver(int, int, int);
		    void setSpeed(long);
        void step(int);
};

#endif
