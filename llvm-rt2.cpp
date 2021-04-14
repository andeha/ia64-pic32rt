/*  llvm-rt3.cpp | with blocks, dynamic memory and an uart for output. */

#include <Twinbeam.h>

void * __gxx_personality_v0;
void * _NSConcreteStackBlock[32] = { 0 };
void * _NSConcreteGlobalBlock[32] { 0 };
uint32_t __ℕ₋🅻[4], __ℕ₋🅷[4];
struct { __builtin_int_t board₁, palm₂; } cxaGuard;
jmp_buf2 /* volatile */ singleTaskProgramState;
namespace Scheduler { void * hw₋collection; Necklace *first, *curr, *last; }
Chronology calendricChronology, computationalChronology;
extern "C" int __cxa_guard_acquire() { return 🔒(cxaGuard); }
extern "C" void __cxa_guard_release() { 🔓(cxaGuard); }
extern "C" int __cxa_atexit(void (* fn)(void *), void * arg, void * dso_handle) { return 0; }
extern "C" void __cxa_pure_virtual() { /* print("pure virtual function called\n"); */ /* boot_Reset(PIC32MZDA_KEY1,PIC32MZDA_KEY2); */ /* ⭐️ Sheriff() */ while (1); } /* 'BLURT' unsuitable because one error allowed. */
namespace std { void terminate() { /* ⭐️ Sheriff() */ } }
extern "C" void * __cxa_begin_catch(void * exceptionObject) throw() { /* ⭐️ Sheriff() */ return NULL; }
void * operator new(size_t size) { return Alloc(size); } /* ⬷ e․𝘨 for impl_ when 😐. */
void operator delete(void * ref) throw() { Fall⒪⒲(ref); }
auto Alloc = ^(__builtin_int_t bytes) { return malloc(bytes); };
auto Fall⒪⒲ = ^(void * ref) { free(ref); };
/* ⬷ to access primitive, include 'extern void * (^Alloc)(__builtin_int_t);' inside your .cpp file. */
extern "C" void * memcpy(void * dst, const void * src, size_t bytes)
{ return Copy8Memory(ByteAlignedRef(dst), ByteAlignedRef(src), __builtin_int_t(bytes)); }
extern "C" void * memset(void * base, int value, size_t bytes)
{ return Overwrite8Memory(ByteAlignedRef(base), (uint8_t)value, __builtin_int_t(bytes)); }
/* enum { BLOCK_IS_FLUCTUANT=8, BLOCK_IS_WEAK=16, BLOCK_IS_ANOTHERBLOCK=7 };
void BlockAssign(void * obj, void ** dst) { *dst=obj; }
extern "C" void _Block_object_assign(void * dst, const void * obj, const int flags)
{ BlockAssign(Critic(obj), dst); }
extern "C" void _Block_object_dispose(const void * obj, const int flags)
{ if (flags & BLOCK_FIELD_IS_BYREF) { Fall⒪⒲(object); } }
*/
int stddetailout, stdctrlout;
/* include sys/ioctl.h> */
void InitVt99augments() { /* fd = open("/dev/fd/0", mode); alt. fd=fcntl(0,f_DUPFD,0); */ }
namespace Terminalctrl₋1 { enum { sparkline=7, formfeed=8, vfill=9, 
 augmentlines=11, deductlines=13, character₋tabulation₋set, /* ✝ */
 character₋tabulation₋with₋just, line₋tabulation, /* ✝⁻¹ ⬷ a.k.a bulleted-lists. */
 paragraph₋separator=19, no₋break₋space /* a.k.a - - and - - a.k.a narrow-no-break-space. */,
}; }
namespace Terminalctrl₋2 { enum { deletechar₋key=21, index₋key /* three-tops-swipe-up */,
 soft₋hyphen₋key, plus₋minus₋key }; }
namespace Terminalctrl₋3 { enum { start₋selected₋area, end₋selected₋area, start₋guard, 
 stop₋guard, start₋string }; } /* ⬷ enjoy 'master-detail' and 'anchored master-detail'. */
/* U+2029 U+241e U+2421 U+0018 U+001d U+0083 U+0086 U+0087 U+0096 U+0097 
 U+0098 U+0094 U+0092 U+0091 U+008d U+0088 U+0089 U+0090 U+0084 U+00ac U+00ad 
 U+00b1 U+00d7 */
#ifdef __x86_64__
#include <stdio.h> /* ⬷ for 'fgetc; for wide character note 'wchar.h'. */
#include <dispatch/dispatch.h>
/* extern "C" long write(int fd, const void * s, long unsigned nbyte); */
auto Putₒ = ^(char8_t * u8s, __builtin_int_t bytes) {
 write(/* stdout, i․𝘦 */ 1, (const void *)u8s, bytes); };
auto Trace₁ = ^(char8_t * u8s, __builtin_int_t bytes) {
 write(/* stderr, i․𝘦 */ 2, (const void *)u8s, bytes); };
auto Trace₂ = ^(char8_t * u8s, __builtin_int_t bytes) {
 write(/* stderr, i․𝘦 */ 3, (const void *)u8s, bytes); };
auto Ctrlₒ = ^(int ctrlcmd, Q1615 measure, int unit) {
 /* int ldisc=TTYDISC; ioctl(0,TIOCSETD,&ldisc); */ }; /* ⬷ 'man 4 tty'. */
#elif defined __mips__
/* #include <pic32rt/pic32mx.hpp> */
#include <pic32rt/pic32mzda.hpp>
/* #include <pic32rt/pic32mm.hpp> */
void
#ifdef __MZDAStarterBoard__
InitMZDAStarterBoard()
#elif defined __EasyPICFusionv7__
InitEasyPICFusionv7Board()
#endif
{ /* ... */ }
#endif
