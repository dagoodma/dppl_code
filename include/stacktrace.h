// stacktrace.h (c) 2008, Timo Bingmann from http://idlebox.net/
// published under the WTFPL v2.0

#ifndef _STACKTRACE_H_
#define _STACKTRACE_H_

#include <stdio.h>
#include <stdlib.h>
#include <cxxabi.h>

#include <signal.h>
#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <err.h>
#include <execinfo.h>

static char const * icky_global_program_name;

/** Print a demangled stack backtrace of the caller function to FILE* out. */
static inline void print_stacktrace(FILE *out = stderr, unsigned int max_frames = 63)
{
    fprintf(out, "stack trace:\n");

    // storage array for stack trace address data
    void* addrlist[max_frames+1];

    // retrieve current stack addresses
    int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void*));

    if (addrlen == 0) {
    fprintf(out, "  <empty, possibly corrupt>\n");
    return;
    }

    // resolve addresses into strings containing "filename(function+address)",
    // this array must be free()-ed
    char** symbollist = backtrace_symbols(addrlist, addrlen);

    // allocate string which will be filled with the demangled function name
    size_t funcnamesize = 256;
    char* funcname = (char*)malloc(funcnamesize);

    // iterate over the returned symbol lines. skip the first, it is the
    // address of this function.
    for (int i = 1; i < addrlen; i++)
    {
    char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char *p = symbollist[i]; *p; ++p)
    {
        if (*p == '(')
        begin_name = p;
        else if (*p == '+')
        begin_offset = p;
        else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
        }
    }

    if (begin_name && begin_offset && end_offset
        && begin_name < begin_offset)
    {
        *begin_name++ = '\0';
        *begin_offset++ = '\0';
        *end_offset = '\0';

        // mangled name is now in [begin_name, begin_offset) and caller
        // offset in [begin_offset, end_offset). now apply
        // __cxa_demangle():

        int status;
        char* ret = abi::__cxa_demangle(begin_name,
                        funcname, &funcnamesize, &status);
        if (status == 0) {
        funcname = ret; // use possibly realloc()-ed string
        fprintf(out, "  %s : %s+%s\n",
            symbollist[i], funcname, begin_offset);
        }
        else {
        // demangling failed. Output function name as a C function with
        // no arguments.
        fprintf(out, "  %s : %s()+%s\n",
            symbollist[i], begin_name, begin_offset);
        }
    }
    else
    {
        // couldn't parse the line? print the whole line.
        fprintf(out, "  %s\n", symbollist[i]);
    }
    }

    free(funcname);
    free(symbollist);
}

static inline int addr2line(char const * const program_name, void const * const addr)
{
  char addr2line_cmd[512] = {0};

  /* have addr2line map the address to the relent line in the code */
  #ifdef __APPLE__
    /* apple does things differently... */
    sprintf(addr2line_cmd,"atos -o %.256s %p", program_name, addr); 
  #else
    sprintf(addr2line_cmd,"addr2line -f -p -e %.256s %p", program_name, addr); 
  #endif

  return system(addr2line_cmd);
}

inline void posix_signal_handler(int sig, siginfo_t *siginfo, void *context)
  {
    (void)context;
    switch(sig)
    {
      case SIGSEGV:
        fputs("Caught SIGSEGV: Segmentation Fault\n", stderr);
        break;
      case SIGINT:
        fputs("Caught SIGINT: Interactive attention signal, (usually ctrl+c)\n", stderr);
        break;
      case SIGFPE:
        switch(siginfo->si_code)
        {
          case FPE_INTDIV:
            fputs("Caught SIGFPE: (integer divide by zero)\n", stderr);
            break;
          case FPE_INTOVF:
            fputs("Caught SIGFPE: (integer overflow)\n", stderr);
            break;
          case FPE_FLTDIV:
            fputs("Caught SIGFPE: (floating-point divide by zero)\n", stderr);
            break;
          case FPE_FLTOVF:
            fputs("Caught SIGFPE: (floating-point overflow)\n", stderr);
            break;
          case FPE_FLTUND:
            fputs("Caught SIGFPE: (floating-point underflow)\n", stderr);
            break;
          case FPE_FLTRES:
            fputs("Caught SIGFPE: (floating-point inexact result)\n", stderr);
            break;
          case FPE_FLTINV:
            fputs("Caught SIGFPE: (floating-point invalid operation)\n", stderr);
            break;
          case FPE_FLTSUB:
            fputs("Caught SIGFPE: (subscript out of range)\n", stderr);
            break;
          default:
            fputs("Caught SIGFPE: Arithmetic Exception\n", stderr);
            break;
        }
      case SIGILL:
        switch(siginfo->si_code)
        {
          case ILL_ILLOPC:
            fputs("Caught SIGILL: (illegal opcode)\n", stderr);
            break;
          case ILL_ILLOPN:
            fputs("Caught SIGILL: (illegal operand)\n", stderr);
            break;
          case ILL_ILLADR:
            fputs("Caught SIGILL: (illegal addressing mode)\n", stderr);
            break;
          case ILL_ILLTRP:
            fputs("Caught SIGILL: (illegal trap)\n", stderr);
            break;
          case ILL_PRVOPC:
            fputs("Caught SIGILL: (privileged opcode)\n", stderr);
            break;
          case ILL_PRVREG:
            fputs("Caught SIGILL: (privileged register)\n", stderr);
            break;
          case ILL_COPROC:
            fputs("Caught SIGILL: (coprocessor error)\n", stderr);
            break;
          case ILL_BADSTK:
            fputs("Caught SIGILL: (internal stack error)\n", stderr);
            break;
          default:
            fputs("Caught SIGILL: Illegal Instruction\n", stderr);
            break;
        }
        break;
      case SIGTERM:
        fputs("Caught SIGTERM: a termination request was sent to the program\n", stderr);
        break;
      case SIGABRT:
        fputs("Caught SIGABRT: usually caused by an abort() or assert()\n", stderr);
        break;
      default:
        break;
    }
    print_stacktrace();
    _Exit(1);
  }

  static uint8_t alternate_stack[SIGSTKSZ];
  inline void set_signal_handler(char const *program_name)
  {
    icky_global_program_name = program_name;
    /* setup alternate stack */
    {
      stack_t ss = {};
      /* malloc is usually used here, I'm not 100% sure my static allocation
         is valid but it seems to work just fine. */
      ss.ss_sp = (void*)alternate_stack;
      ss.ss_size = SIGSTKSZ;
      ss.ss_flags = 0;

      if (sigaltstack(&ss, NULL) != 0) { err(1, "sigaltstack"); }
    }

    /* register our signal handlers */
    {
      struct sigaction sig_action = {};
      sig_action.sa_sigaction = posix_signal_handler;
      sigemptyset(&sig_action.sa_mask);

      #ifdef __APPLE__
          /* for some reason we backtrace() doesn't work on osx
             when we use an alternate stack */
          sig_action.sa_flags = SA_SIGINFO;
      #else
          sig_action.sa_flags = SA_SIGINFO | SA_ONSTACK;
      #endif

      if (sigaction(SIGSEGV, &sig_action, NULL) != 0) { err(1, "sigaction"); }
      if (sigaction(SIGFPE,  &sig_action, NULL) != 0) { err(1, "sigaction"); }
      if (sigaction(SIGINT,  &sig_action, NULL) != 0) { err(1, "sigaction"); }
      if (sigaction(SIGILL,  &sig_action, NULL) != 0) { err(1, "sigaction"); }
      if (sigaction(SIGTERM, &sig_action, NULL) != 0) { err(1, "sigaction"); }
      if (sigaction(SIGABRT, &sig_action, NULL) != 0) { err(1, "sigaction"); }
    }
  }

#endif // _STACKTRACE_H_