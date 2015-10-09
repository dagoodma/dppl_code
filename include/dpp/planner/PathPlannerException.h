#ifndef _DPP_PATHPLANNEREXCEPTION_H_
#define _DPP_PATHPLANNEREXCEPTION_H_

#include <exception>

namespace DPP {

/*
 * Exception base class for DPP Exceptions.
 */
class ExceptionBase : public std::exception { };

class PathPlannerException : ExceptionBase { };