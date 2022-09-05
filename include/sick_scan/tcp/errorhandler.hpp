#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
/**
 * Fehler-Handler.
 *
 */
#ifndef ERRORHANDLER_HPP
#define ERRORHANDLER_HPP


#include "sick_scan/tcp/BasicDatatypes.hpp"
#include <stdexcept>	// for throw


#define printInfoMessage(a, b)  (b ? infoMessage(a, b):doNothing())

// Fehler-"behandlung": Schreibe die Fehlermeldung und beende das Programm.
void dieWithError(std::string errorMessage);

void infoMessage(std::string message, bool print = true);

void printWarning(std::string message);

void printError(std::string message);

void doNothing();

#endif // ERRORHANDLER_HPP
