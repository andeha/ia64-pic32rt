/*  llvm-rt2.cpp | Middle version that manages dynamic memory. */

#include <Twinbeam.h>

void * _NSConcreteStackBlock[32];
void * _NSConcreteGlobalBlock[32];

void * operator new(size_t size) { return Alloc(size); }
void operator delete(void * p) { Fall⒪⒲(p); }
/* Abstractions `Alloc` and `Fall⒪⒲` are non-overloadable in C++. */

