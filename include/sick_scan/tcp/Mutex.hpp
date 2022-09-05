#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/**
 * \file Mutex.hpp
 */

#ifndef MUTEX_HPP
#define MUTEX_HPP

#include <mutex>
#include <thread>

#include "sick_scan/tcp/BasicDatatypes.hpp"
// #include <pthread.h>


//
// Mutex class
//
class Mutex
{
public:
	Mutex();
	~Mutex();

	void lock();
	void unlock();

private:
    std::mutex m_mutex;
	// pthread_mutex_t m_mutex;
};



//
// Scoped Lock.
// Zerstoert das Mutex automatisch.
//
class ScopedLock
{
public:
	ScopedLock(Mutex* mutexPtr);
	~ScopedLock();
private:
	Mutex* m_mutexPtr;
};


#endif // MUTEX_HPP
