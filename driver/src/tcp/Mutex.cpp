/**
 * \file Mutex.cpp
 */

#include "sick_scan/tcp/Mutex.hpp"
#include "sick_scan/tcp/errorhandler.hpp"

// ****************************************************************************
//  ScopedLock
// ****************************************************************************

// Konstruktor.
// Lock sperren.
ScopedLock::ScopedLock(Mutex* mutexPtr)
{
	m_mutexPtr = mutexPtr;
	if (m_mutexPtr != NULL)
	{
		m_mutexPtr->lock();
	}
}

// Destruktor.
// Lock wieder freigeben.
ScopedLock::~ScopedLock()
{
	if (m_mutexPtr != NULL)
	{
		m_mutexPtr->unlock();
	}
}



// ****************************************************************************
//  Mutex
// ****************************************************************************
Mutex::Mutex()
{
	//pthread_mutex_init (&m_mutex, NULL);
}

Mutex::~Mutex()
{
}

void Mutex::lock()
{
	m_mutex.lock();
	//pthread_mutex_lock(&m_mutex);
}

void Mutex::unlock()
{
	m_mutex.unlock();
	// pthread_mutex_unlock(&m_mutex);
}
