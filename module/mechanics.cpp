#include "mechanics.h"

#include <stdexcept>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#if __has_include(<pigpio.h>)
#include <pigpio.h>

#else
int gpioInitialise() { return 0; }
void gpioTerminate() {}
void gpioServo(int pin, int value) {
	std::cout << "gpioServo(" << pin << ", " << value << ")" << std::endl;
}

#endif


TTimer* TTimer::m_timer = NULL;

TTimer* TTimer::getTimer() {
	if (m_timer == NULL) {
		m_timer = new TTimer();
	}
	return m_timer;
}

void TTimer::sig_handler(int signo) {
	//std::cout << "tick" << std::endl;
	ITimed* timed = getTimer()->m_timed;
	while(timed != NULL) {
		//std::cout << " timed" << std::endl;
		timed->tick();
		timed = timed->m_next;
	}
}

void TTimer::start(long freqUs) {
	m_freqUs = freqUs;
	signal ( SIGALRM, TTimer::sig_handler );
	
	struct itimerval timer;
	timer.it_interval.tv_usec = freqUs;
	timer.it_interval.tv_sec = 0;
	timer.it_value.tv_usec = freqUs;
	timer.it_value.tv_sec = 0;
	
	ITimed* timed = TTimer::m_timer->m_timed;
	while(timed != NULL) {
		timed->start(freqUs);
		timed = timed->m_next;
	}
	
	setitimer ( ITIMER_REAL, &timer, NULL );
}

void TTimer::stop() {
	
	m_freqUs = 0;
	
	struct itimerval timer;
	timer.it_interval.tv_usec = 0;
	timer.it_interval.tv_sec = 0;
	timer.it_value.tv_usec = 0;
	timer.it_value.tv_sec = 0;
	
	setitimer ( ITIMER_REAL, &timer, NULL );
	
}

void TTimer::sleep(long us) {
	auto start = std::chrono::high_resolution_clock::now();
	while(true) {
		long lasts = (std::chrono::high_resolution_clock::now() -start).count() / 1000;
		if (lasts >= us) {
			break;
		}
		usleep(us / 10);
	}

}

void TTimer::addTimed(ITimed* timed) {
	timed->m_next = m_timed;
	m_timed = timed;
    //std::cout << "addTimed:" << m_timed << std::endl;
}


void TServo::setValue(int value) {
    Value = value;
	//std::cout << "setValue: " << Pin << " <-" << value << std::endl;
    gpioServo(Pin, value);
}

void TServo::tick() {
	//std::cout << Id << " tick v=" << Value << " target=" << TargetValue << std::endl;
	if (Value != TargetValue) {
		if (abs(Value - TargetValue) <= TargetMaxMove) {
			//std::cout << "tick: target " << Id << " tick v=" << Value << " target=" << TargetValue << " maxMove=" << TargetMaxMove << std::endl;
			setValue(TargetValue);
		} else {
			//std::cout << "tick: max " << Id << " tick v=" << Value << " target=" << TargetValue << " maxMove=" << TargetMaxMove << std::endl;
			setValue(Value + (TargetValue > Value ? TargetMaxMove : -TargetMaxMove));
		}
	} else {
		if (Complete != NULL) {
			auto complete = Complete;
            Complete = NULL;
            complete->complete(this);
		}
	}
}

void TServo::stopTarget() {
	TargetValue = Value;
	tick();
}

void initMechanics() {
	std::cout << "init mechanics" << std::endl;
	if (gpioInitialise() < 0) {
		throw std::runtime_error("Error: init error");
	}

}

void stopMechanics(){
	gpioTerminate();
}
