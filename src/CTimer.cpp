#include "CTimer.h"

CTimer::CTimer(int timeout)
{
	reset();
	timeoutInterval = timeout;
	pause();
}

CTimer::~CTimer()
{
}

void CTimer::reset(int timeout)
{
	timeoutInterval = timeout;
	startTime = getRealTime();
	pauseTime = startTime;
}

int CTimer::getRealTime()
{
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return currentTime.tv_sec * 1000 + currentTime.tv_usec / 1000;
}

int CTimer::getTime()
{
	int result;
	if (running)
	{
		result = getRealTime() - startTime;
	}
	else
	{
		result = pauseTime - startTime;
	}
	return result;
}

bool CTimer::timeOut()
{
	return getTime() > timeoutInterval;
}

bool CTimer::paused()
{
	return (running == false);
}

int CTimer::pause()
{
	running = false;
	return pauseTime = getRealTime();
}

int CTimer::start()
{
	startTime += (getRealTime() - pauseTime);
	running = true;
	return getTime();
}
