#ifndef Driver_h
#define Driver_h

class Driver
{
    private: //to be used only in cpp
        int _direction;
		    int _stepDelay;
        int _lastEdge;
        int _currStep;
    protected: //constant
        int _numberOfSteps;
        int _dirPin;
        int _stepPin;
    public: //to be called by .ino file
        Driver(int, int, int);
		    void setSpeed(long);
        void step(int);
        int currStep(void);
};

#endif
