#ifndef MECHANICS_H
#define MECHANICS_H

#include <string>
#include <iostream>


struct ITimed {
	virtual void start(long freq)=0;
	virtual void tick()=0;
	
	ITimed* m_next;
	
	ITimed() {
		m_next = NULL;
	}
};

struct IProcess {
    std::string Id;

    IProcess(std::string id) {
        Id = id;
    }
};

struct ICompletionListener {
	virtual void complete(IProcess* src)=0;
};

struct TTimer {
	
	long m_freqUs;
	ITimed* m_timed;
	
	static TTimer* m_timer;
	static TTimer* getTimer();

	void start(long freqUs);
	void stop();
	void sleep(long us);
	
	void addTimed(ITimed* timed);

private:
	
	TTimer() {
		m_freqUs = 0;
		m_timed = NULL;
	}

	static void sig_handler(int signo);

};

const double TOP_SPEED = 1.0;

//const double SERVO_MOVE_RATE = 420;
const double SERVO_MOVE_RATE = 380;


struct TServo : public ITimed, IProcess {
    
    int     Pin;
    int     Zero;
    double  Range;
    int     Value;
    
    int     TargetValue;
    double  TargetMaxMove;
    ICompletionListener* Complete;
    long    TimerFreqUs;
        
    TServo(std::string id, int pin, int zero, double range) :
        IProcess(id) {

        Pin = pin;
        Zero = zero;
        Range = range / 90.0;

		TargetValue = Zero;
		Value = Zero;
		Complete = NULL;

		TTimer::getTimer()->addTimed(this);
    }

    void setValueAndTarget(int value) {
        TargetValue = value;
        setValue(value);
    }

    void setTargetValue(int targetValue, double speed, ICompletionListener* complete) {
	    TargetValue = targetValue;
	    Complete = complete;
        TargetMaxMove = speed *  TimerFreqUs / SERVO_MOVE_RATE + 1;
	    //std::cout << "setTargetValue " << Id << "  " << targetValue << " from " << Value << " maxMove=" << TargetMaxMove << std::endl;
    }

    void stopTarget();

    int getTargetDistanceToAngle(double targetAngle) {
        return getTargetDistance(getAngleValue(targetAngle));
    }
    
    int getAngleValue(double angle) {
        return  Zero + (int)(angle * Range);
    }

    double getAngle() {
        return (Value - Zero) / (double) Range;
    }

    void setAngle(double angle) {
        setValueAndTarget(getAngleValue(angle));
    }

    void setTargetAngle(double angle, double speed, ICompletionListener* complete) {
        setTargetValue(getAngleValue(angle), speed, complete);
    }

    void start(long freqUs) {
	    TimerFreqUs = freqUs;
    }

    void tick();

    protected:

    void setValue(int value);

    int getTargetDistance(int targetValue) {
        return abs(targetValue - Value);
    }

};

void initMechanics();
void stopMechanics();

#endif