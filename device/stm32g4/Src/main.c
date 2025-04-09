
#include "../../motorlib/system.h"
#include "st_device.h"


void _close_r() {}
void _getpid_r() {}
void _kill_r() {}
void _lseek_r() {}
void _read_r() {}

uint32_t go_to_bootloader = 0;


int main(void)
{
  system_init();
  while (1)
  {
    system_run();
  }
}


#include "stdlib.h"


/*
 * Size of guard space inserted before and after the block.
 * Larger sizes will more likely catch out-of-bound writes
 * but consume more memory. The default is 16.
 * When changed the size must also be supplied to the
 * Data.MallocList command with the /Guard option.
 */

#define T32_GUARD_SIZE		16


/*
 * Define this if you want to trace the memory allocations
 * (by a hardware or software trace).
 */

#define T32_MEMTRACE


/*
 * Define this if you want to use the LOGGER command to make the trace.
 */

//#define T32_MEMLOGGER



/*
 * Structure inserted before each block.
 * The structures form a double linked list.
 * The layout of the structure should not be changed
 * as it is (hardcoded) used in the debugger commands.
 */

typedef struct T32_allocHeader
{
    struct T32_allocHeader * prev;
    struct T32_allocHeader * next;
    void * caller;
    size_t size;
#if T32_GUARD_SIZE
    unsigned char guard[T32_GUARD_SIZE];
#endif
}
T32_allocHeader;


/*
 * First and last element of the double linked list.
 * All allocated control blocks are inserted between them.
 */

extern T32_allocHeader T32_LastNode;

T32_allocHeader T32_FirstNode = { 0, &T32_LastNode };
T32_allocHeader T32_LastNode = { &T32_FirstNode, 0 };

#define T32_INSERTNODE(p)    { (p)->prev = T32_LastNode.prev; (p)->next = &T32_LastNode; T32_LastNode.prev->next = (p); T32_LastNode.prev = (p); }
#define T32_REMOVENODE(p)    { (p)->prev->next = (p)->next; (p)->next->prev = (p)->prev; }


/*
 * Structure for tracing malloc/free calls.
 * The values are only written and only used by the trace tool.
 */

struct
{
    void * caller;
    size_t size;
    void * ptr;

    size_t totalsize;
    size_t maxsize;
}
T32_MemTrace;

static size_t TotalSize, MaxSize;

#ifdef T32_MEMTRACE
#define T32_TRACENODE(p,s,c)	{ \
				    TotalSize += (s); \
				    if (TotalSize >= MaxSize) \
					MaxSize = TotalSize; \
				    T32_MemTrace.caller = (c); \
				    T32_MemTrace.size = (s); \
				    T32_MemTrace.ptr = (void *) ((p) + 1); \
				    T32_MemTrace.totalsize = TotalSize; \
				    T32_MemTrace.maxsize = MaxSize; \
				}
#else
#define T32_TRACENODE(p,size,caller)
#endif

#ifdef T32_MEMLOGGER
#define T32_TRACELOGGER()	{ \
    T32_LoggerData(0x34, &T32_MemTrace.caller, T32_MemTrace.caller); \
    T32_LoggerData(0x34, &T32_MemTrace.size, T32_MemTrace.size); \
    T32_LoggerData(0x34, &T32_MemTrace.ptr, T32_MemTrace.ptr); \
    T32_LoggerData(0x34, &T32_MemTrace.totalsize, T32_MemTrace.totalsize); \
    T32_LoggerData(0x34, &T32_MemTrace.maxsize, T32_MemTrace.maxsize); \
    }
#else
#define T32_TRACELOGGER()
#endif


/*
 * Code to set and check the guard area.
 * The guard area allows the debugger to check for
 * out-of-bounds writes to the heap.
 * The contents of the guard area should not be changed.
 */

static void SetGuard(unsigned char * guard)
{
    int             i;
    for (i = 0; i < T32_GUARD_SIZE; i++)
	guard[i] = (unsigned char) (i + 1);
}

static int CheckGuard(unsigned char * guard)
{
    int             i;
    for (i = 0; i < T32_GUARD_SIZE; i++)
	if (guard[i] != (unsigned char) (i + 1))
	    return 1;
    return 0;
}


/*
 * Breakpoint location. This function is called when an error
 * is detected by the allocation check functions.
 */

void T32_breakpoint()
{
}


/*
 * Hooks for the alloc functions.
 * The code for these functions may needs to be copied
 * if you have different allocation routines (e.g. calloc).
 */
void * T32_malloc(size_t size) __attribute((used));
void * T32_malloc(size_t size)
{
    void * caller;
    T32_allocHeader *tptr;

    caller = (void *) __builtin_return_address(0);

    tptr = malloc(size + sizeof(T32_allocHeader) + T32_GUARD_SIZE);

    if (tptr == (T32_allocHeader *) 0)
	return (void *) 0;

    tptr->caller = caller;
    tptr->size = size;

#if T32_GUARD_SIZE
    SetGuard(tptr->guard);
    SetGuard(((unsigned char *) (tptr + 1)) + size);
#endif

    T32_INSERTNODE(tptr);

    T32_TRACENODE(tptr,size,caller);
    
    T32_TRACELOGGER();

    return (void *) (tptr + 1);
}

void * T32_realloc(void * ptr, size_t size) __attribute((used));
void * T32_realloc(void * ptr, size_t size)
{
    void           *caller;
    T32_allocHeader *tptr;

    caller = (void *) __builtin_return_address(0);

    if (ptr == (void *) 0) {
	tptr = (T32_allocHeader *) 0;
    } else {
	tptr = ((T32_allocHeader *) (ptr)) - 1;
#if T32_GUARD_SIZE
	if (CheckGuard(tptr->guard))
	    T32_breakpoint();
	if (CheckGuard(((unsigned char *) (tptr + 1)) + tptr->size))
	    T32_breakpoint();
#endif
	T32_REMOVENODE(tptr);

	T32_TRACENODE(tptr, ~tptr->size, caller);

	T32_TRACELOGGER();
    }

    if (size == 0) {
	tptr = realloc(tptr, size);
	return (void *) tptr;
    } else {
	tptr = realloc(tptr, size + sizeof(T32_allocHeader) + T32_GUARD_SIZE);

	if (tptr == (T32_allocHeader *) 0)
	    return (void *) 0;

	tptr->caller = caller;
	tptr->size = size;

#if T32_GUARD_SIZE
	SetGuard(((unsigned char *) (tptr + 1)) + size);
	SetGuard(((unsigned char *) (tptr + 1)) + size);
#endif
	T32_INSERTNODE(tptr);

	T32_TRACENODE(tptr, size, caller);

	T32_TRACELOGGER();

	return (void *) (tptr + 1);
    }
}

void T32_free(void * ptr) __attribute((used));
void T32_free(void * ptr)
{
    void           *caller;
    T32_allocHeader *tptr;

    caller = (void *) __builtin_return_address(0);

    if (ptr == (void *) 0) {
	tptr = (T32_allocHeader *) 0;
    } else {
	tptr = ((T32_allocHeader *) (ptr)) - 1;

#if T32_GUARD_SIZE
	if (CheckGuard(tptr->guard))
	    T32_breakpoint();
	if (CheckGuard(((unsigned char *) (tptr + 1)) + tptr->size))
	    T32_breakpoint();
#endif

	T32_REMOVENODE(tptr);

	T32_TRACENODE(tptr, ~tptr->size, caller);

	T32_TRACELOGGER();
    }

    free(tptr);
}
